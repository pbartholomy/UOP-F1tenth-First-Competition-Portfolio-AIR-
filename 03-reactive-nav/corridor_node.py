#!/usr/bin/env python3
"""
corridor_node.py — v4: corridor driving + corner handling.

Two modes, one trigger (front_dist):
  STRAIGHT  front_dist > CORNER_MM  → corridor centering (balance L/R walls)
  CORNER    front_dist <= CORNER_MM → gap following (steer toward biggest opening)
  ESTOP     front_dist <= ESTOP_MM  → hard stop

No camera. No SLAM. No neural net. Pure LiDAR.

Joystick (PS4):
  L1 (btn 4 or 9) : kill switch — hold to stop, release to drive
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped

from roboracer.common import (
    BTN_L1, BTN_L1_ALT, LOOP_HZ,
    SERVO_CENTER, SERVO_MIN, SERVO_MAX,
    MIN_EFFECTIVE_DUTY,
    HokuyoLidar,
    _open_vesc, send_current_zero, send_duty, send_servo, stop_car,
    preprocess_gap_lidar, find_best_gap_direction,
)

# ── TUNING ────────────────────────────────────────────────────
DRIVE_DUTY        = 0.065   # cruise speed
START_BOOST_DUTY  = 0.076   # kick from standstill
START_BOOST_LOOPS = 10      # boost duration at 50Hz (~200ms)

# Speed zones (mm)
FULL_SPEED_MM  = 1500
MODERATE_MM    =  900       # 85% speed
CORNER_MM      =  950       # switch to gap-follow + slow for corner
SLOW_MM        =  500       # 70% speed
ESTOP_MM       =  350       # hard stop

# Corner speed
CORNER_DUTY    = 0.050      # speed while turning (raise carefully)

# Corridor centering (STRAIGHT mode)
STEER_GAIN          = 1.0
STEER_DEADBAND      = 0.0
STEER_SMOOTH        = 0.60
STEER_SCALE         = 0.52  # servo throw for straight corrections

STEER_INTEGRAL_GAIN = 0.20  # how strongly accumulated drift is corrected
STEER_INTEGRAL_MAX  = 0.35  # cap to prevent windup
STEER_INTEGRAL_DECAY= 0.97  # per-loop decay (~halves in ~23 loops / 0.46s)

STEER_D_GAIN   = 1.5        # derivative: anticipate wall approach before it's close
STEER_TRIM     = 0.04       # positive = bias right, negative = bias left; fix constant centering offset

SIDE_MIN_MM    = 600        # start proportional push away from wall
WALL_TURN_MM   = 300        # this close → override and actively turn away
WALL_TURN_STEER= 0.80       # how hard to turn away (0–1, raise if still hugging)
WALL_PANIC_MM  = 140        # this close → full opposite steer immediately

# Gap following (CORNER mode)
GAP_SCALE      = 0.70       # servo throw for corners (raise if under-steering)
GAP_SMOOTH     = 0.55       # EMA for gap steer (higher = snappier turn)
CORNER_EXIT_LOOPS = 8       # loops to stay in gap mode after corner clears (~160ms)

PRINT_HZ       = 8
# ─────────────────────────────────────────────────────────────


class CorridorNode(Node):
    def __init__(self):
        super().__init__("corridor_node")
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self._drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.kill_active = True

    def _joy_cb(self, msg: Joy):
        b = list(msg.buttons) + [0] * 16
        self.kill_active = bool(b[BTN_L1] or b[BTN_L1_ALT])

    def publish_drive(self, duty: float, servo: float):
        msg = AckermannDriveStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed          = float(duty)
        msg.drive.steering_angle = float(servo)
        self._drive_pub.publish(msg)


# ── Corridor centering (straight) ─────────────────────────────

def corridor_steer(clearances: dict, prev_steer: float, integral: float,
                   prev_left_d: float = None, prev_right_d: float = None) -> tuple:
    INF = float("inf")
    CAP = 2000.0

    # FAR sectors point directly sideways (±45°–90°) — most accurate lateral reading
    far_l = min(clearances.get("FAR_LEFT",    INF), CAP)
    l1    = min(clearances.get("LEFT",        INF), CAP)
    l2    = min(clearances.get("CENTER_LEFT", INF), CAP)
    far_r = min(clearances.get("FAR_RIGHT",   INF), CAP)
    r1    = min(clearances.get("RIGHT",       INF), CAP)
    r2    = min(clearances.get("CENTER_RIGHT",INF), CAP)

    # Weight FAR (true sideways) 2x over angled sectors
    left_d  = (far_l * 2.0 + l1 + l2) / 4.0
    right_d = (far_r * 2.0 + r1 + r2) / 4.0
    total   = left_d + right_d

    if total < 50.0:
        return prev_steer, integral, left_d, right_d, "NO_WALLS"

    raw = (left_d - right_d) / total

    if abs(raw) < STEER_DEADBAND:
        raw = 0.0

    # Derivative — if left wall is closing in, start correcting early
    d_term = 0.0
    if prev_left_d is not None and prev_right_d is not None:
        rate_l = left_d  - prev_left_d   # negative = left wall approaching
        rate_r = right_d - prev_right_d  # negative = right wall approaching
        d_term = max(-0.5, min(0.5, (rate_l - rate_r) / CAP * STEER_D_GAIN))

    # Integral — accumulates persistent offset and pushes back
    integral = integral * STEER_INTEGRAL_DECAY + raw * 0.02
    integral = max(-STEER_INTEGRAL_MAX, min(STEER_INTEGRAL_MAX, integral))

    # Nearest reading across all left/right sectors
    l_near = min(far_l, l1, l2)
    r_near = min(far_r, r1, r2)

    # Panic override — wall critically close, full opposite steer immediately
    if l_near < WALL_PANIC_MM:
        target   = -1.0
        integral = 0.0
    elif r_near < WALL_PANIC_MM:
        target   = 1.0
        integral = 0.0
    else:
        # Scale centering gain by proximity — always target center, just try harder near walls
        close = min(l_near, r_near)
        if close < WALL_TURN_MM:
            prox_gain = 4.0
        elif close < SIDE_MIN_MM:
            prox_gain = 1.0 + 3.0 * (SIDE_MIN_MM - close) / (SIDE_MIN_MM - WALL_TURN_MM)
        else:
            prox_gain = 1.0

        target = max(-1.0, min(1.0,
            (raw + d_term + integral * STEER_INTEGRAL_GAIN) * STEER_GAIN * prox_gain))

    smooth = STEER_SMOOTH * target + (1.0 - STEER_SMOOTH) * prev_steer
    dbg = (f"L={int(left_d)}({int(far_l)}) R={int(right_d)}({int(far_r)}) "
           f"err={raw:+.2f} d={d_term:+.2f} int={integral:+.2f}")
    return smooth, integral, left_d, right_d, dbg


# ── Gap following (corner) ────────────────────────────────────

def gap_steer(distances: list, prev_steer: float) -> tuple:
    if not distances:
        return prev_steer, "NO_SCAN"
    processed = preprocess_gap_lidar(distances)
    raw = find_best_gap_direction(processed)
    smooth = GAP_SMOOTH * raw + (1.0 - GAP_SMOOTH) * prev_steer
    return smooth, f"gap={raw:+.2f}"


# ── Speed ─────────────────────────────────────────────────────

def compute_duty(front_dist, in_corner: bool) -> tuple:
    if in_corner:
        return CORNER_DUTY, "CORNER"   # hold corner speed for entire gap-follow phase
    if front_dist is None or front_dist >= FULL_SPEED_MM:
        return DRIVE_DUTY, "FULL"
    if front_dist >= MODERATE_MM:
        return DRIVE_DUTY * 0.85, "MODERATE"
    if front_dist >= CORNER_MM:
        return DRIVE_DUTY * 0.80, "PRE-CORNER"
    if front_dist >= SLOW_MM:
        return CORNER_DUTY, "SLOW"
    if front_dist >= ESTOP_MM:
        return MIN_EFFECTIVE_DUTY, "CRAWL"
    return 0.0, "ESTOP"


# ── Main loop ─────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CorridorNode()

    lidar = HokuyoLidar()
    try:
        lidar.connect()
        lidar.start()
    except Exception as e:
        node.get_logger().warn(f"LiDAR not connected: {e}")
        lidar = None

    ser = _open_vesc()
    if ser is None:
        node.get_logger().error("Could not open VESC — aborting")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("=" * 52)
    node.get_logger().info("  corridor_node v4 — corridor + corners")
    node.get_logger().info(f"  Cruise: {DRIVE_DUTY:.3f}  Corner: {CORNER_DUTY:.3f}  ESTOP: {ESTOP_MM}mm")
    node.get_logger().info(f"  Corner trigger: front < {CORNER_MM}mm")
    node.get_logger().info("  Hold L1 to kill | release L1 to drive")
    node.get_logger().info("=" * 52)

    loop_period  = 1.0 / LOOP_HZ
    print_period = 1.0 / PRINT_HZ
    last_print   = 0.0

    steer            = 0.0
    steer_integral   = 0.0
    prev_left_d      = None
    prev_right_d     = None
    boost_count      = 0
    was_stopped      = True
    corner_exit_hold = 0    # counts down after corner clears

    try:
        while rclpy.ok():
            t0 = time.time()
            rclpy.spin_once(node, timeout_sec=0)

            # ── Kill switch ───────────────────────────
            if node.kill_active:
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                node.publish_drive(0.0, SERVO_CENTER)
                steer            = 0.0
                steer_integral   = 0.0
                prev_left_d      = None
                prev_right_d     = None
                boost_count      = 0
                was_stopped      = True
                corner_exit_hold = 0
                elapsed = time.time() - t0
                if elapsed < loop_period:
                    time.sleep(loop_period - elapsed)
                continue

            # ── Sensors ───────────────────────────────
            front_dist = None
            clearances = {}
            distances  = []
            if lidar and lidar.connected:
                front_dist = lidar.front_min()
                clearances = lidar.sector_clearances()
                distances  = lidar.get_distances()

            # ── Mode + steering ───────────────────────
            if front_dist is not None and front_dist <= CORNER_MM:
                # Active corner — pure gap follow
                in_corner        = True
                corner_exit_hold = CORNER_EXIT_LOOPS
                steer_integral   = 0.0  # reset integral through corners
                prev_left_d      = None
                prev_right_d     = None
                gap_s, gap_dbg   = gap_steer(distances, steer)
                steer            = gap_s
                servo = max(SERVO_MIN, min(SERVO_MAX,
                            SERVO_CENTER + steer * GAP_SCALE))
                mode = "CORNER"

            elif corner_exit_hold > 0:
                # Corner exit — blend gap + corridor, shifting toward corridor as hold counts down
                in_corner        = True
                corner_exit_hold -= 1
                t      = corner_exit_hold / CORNER_EXIT_LOOPS   # 1.0→0.0 as hold expires
                gap_w  = t * t                                  # quadratic: corridor dominates quickly
                corr_w = 1.0 - gap_w

                gap_s,  _                                        = gap_steer(distances, steer)
                corr_s, steer_integral, prev_left_d, prev_right_d, corr_dbg = corridor_steer(
                    clearances, steer, steer_integral, prev_left_d, prev_right_d)
                blended                                          = gap_w * gap_s + corr_w * corr_s
                steer                        = 0.75 * blended + 0.25 * steer
                servo = max(SERVO_MIN, min(SERVO_MAX,
                            SERVO_CENTER + steer * GAP_SCALE))
                steer_dbg = f"EXIT blend gap={gap_w:.2f} corr={corr_w:.2f}  {corr_dbg}"
                mode = "EXIT"

            else:
                # Straight — pure corridor centering
                in_corner = False
                steer, steer_integral, prev_left_d, prev_right_d, steer_dbg = corridor_steer(
                    clearances, steer, steer_integral, prev_left_d, prev_right_d)
                servo = max(SERVO_MIN, min(SERVO_MAX,
                            SERVO_CENTER + steer * STEER_SCALE + STEER_TRIM))
                mode = "STRAIGHT"

            # ── Speed ─────────────────────────────────
            duty, zone = compute_duty(front_dist, in_corner)

            # Side wall speed penalty — slow down when drifting close to either wall
            if not in_corner:
                INF = float("inf")
                l_near = min(clearances.get("FAR_LEFT", INF), clearances.get("LEFT", INF), clearances.get("CENTER_LEFT", INF))
                r_near = min(clearances.get("FAR_RIGHT", INF), clearances.get("RIGHT", INF), clearances.get("CENTER_RIGHT", INF))
                side_near = min(l_near, r_near)
                if side_near < WALL_PANIC_MM:
                    duty = MIN_EFFECTIVE_DUTY   # critically close — crawl so steering can catch up
                    zone += "+WALL!"
                elif side_near < SIDE_MIN_MM:
                    ratio = (side_near - WALL_PANIC_MM) / (SIDE_MIN_MM - WALL_PANIC_MM)
                    duty  = MIN_EFFECTIVE_DUTY + ratio * (duty - MIN_EFFECTIVE_DUTY)
                    zone += "+SLOW"

            if duty > 0.001:
                if was_stopped:
                    boost_count = START_BOOST_LOOPS
                    was_stopped = False
                if boost_count > 0:
                    duty = max(duty, START_BOOST_DUTY)
                    boost_count -= 1
                duty = max(duty, MIN_EFFECTIVE_DUTY)
                send_servo(ser, servo)
                send_duty(ser, duty)
            else:
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                duty           = 0.0
                steer          = 0.0
                steer_integral = 0.0
                prev_left_d    = None
                prev_right_d   = None
                boost_count    = 0
                was_stopped    = True

            node.publish_drive(duty, servo)

            # ── Print ─────────────────────────────────
            now = time.time()
            if now - last_print >= print_period:
                fd = f"{front_dist}mm" if front_dist is not None else "----"
                print(
                    f"\r[{mode:<8}|{zone:<10}] Front:{fd:<6} "
                    f"Steer:{steer:+.3f} Servo:{servo:.3f} Duty:{duty:.3f}  {steer_dbg}    ",
                    end=""
                )
                last_print = now

            elapsed = time.time() - t0
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)

    except KeyboardInterrupt:
        print("\n[corridor_node] Shutting down...")
    finally:
        try:
            stop_car(ser)
            ser.close()
        except Exception:
            pass
        if lidar:
            lidar.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[corridor_node] Done")


if __name__ == "__main__":
    main()
