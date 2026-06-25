#!/usr/bin/env python3
"""
corridor_node.py — v8: corridor centering + gap following.

Straight sections: corridor centering keeps car centered, full speed.
Corners / obstacles: gap following steers toward the largest opening, slowed down.
Smooth blend on corner exit back to corridor mode.

Drive modes:
  0 = AUTONOMOUS  : reactive corridor + gap follow (default)
  1 = MANUAL      : joystick via car_node → /manual_drive

Joystick (PS4):
  L1 (btn 4 or 9) : kill switch
  R1 (btn 5)      : toggle MANUAL ↔ AUTONOMOUS
"""

import math
import os
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from std_msgs.msg import Int32, Float32
from ackermann_msgs.msg import AckermannDriveStamped

from roboracer.common import (
    BTN_L1, BTN_L1_ALT, LOOP_HZ,
    SERVO_CENTER, SERVO_MIN, SERVO_MAX,
    MIN_EFFECTIVE_DUTY,
    HokuyoLidar,
    _open_vesc, send_current_zero, send_duty, send_servo, stop_car,
    send_packet, _read_vesc_packet,
    preprocess_gap_lidar, find_best_gap_direction,
)

# Non-inverting duty send for manual relay
def _send_duty_raw(ser, duty):
    duty = max(-0.6, min(0.6, duty))
    send_packet(ser, bytes([5]) + struct.pack(">i", int(duty * 100000)))

# ── TUNING ────────────────────────────────────────────────────

# Speed
DRIVE_DUTY        = 0.240
MAX_DUTY          = 0.500
START_BOOST_DUTY  = 0.250
START_BOOST_LOOPS = 10

# Front distance thresholds (mm)
CORNER_MM      = 1050   # enter gap-follow mode when front is this close
ESTOP_MM       =  300   # hard stop

# ERPM targets
ERPM_FULL         =  80000   # cruise in corridors
ERPM_MODERATE     =  65000   # front getting close but not corner yet
ERPM_CORNER_FAST  =  55000   # wide / sweeping corner
ERPM_CORNER_TIGHT =  38000   # tight hairpin
ERPM_CRAWL        =  26000   # emergency near wall

# PI speed controller
SPEED_KP        = 0.0000010
SPEED_KI        = 0.0000004
SPEED_I_CAP     = 15000.0
ERPM_POLL_EVERY = 3
COMM_GET_VALUES = 4

# Acceleration / deceleration ramp (ERPM per loop at 50 Hz)
ACCEL_RATE = 2000   # ~0 → 95k in ~1.0 s
DECEL_RATE = 5000   # 95k → 0 in ~0.4 s  (brakes faster than it accelerates)

# ── Corridor centering (tuned in v8, do not change carelessly) ─
STEER_GAIN          = 0.42
STEER_SMOOTH        = 0.28
STEER_SCALE         = 0.52
STEER_DEADBAND      = 0.04
STEER_INTEGRAL_GAIN = 0.05
STEER_INTEGRAL_MAX  = 0.10
STEER_INTEGRAL_DECAY= 0.95
STEER_D_GAIN        = 0.15
STEER_TRIM          = 0.11   # positive = bias right

STARTUP_RAMP_LOOPS  = 15
STARTUP_RAMP_STEP   = 0.05

# Wall avoidance overlays
SIDE_MIN_MM     = 1200
WALL_TURN_MM    =  580
WALL_TURN_STEER =  0.78
WALL_PANIC_MM   =  320

# ── Gap following (corners / obstacle avoidance) ───────────────
GAP_FOLLOW_ENABLED = True    # set False to disable gap follow + obstacle mode
GAP_SCALE         = 0.62   # servo throw during normal corners
GAP_SMOOTH        = 0.68   # smoother gap entry
CORNER_EXIT_LOOPS =  8     # shorter exit blend

# Tight U-turn boost: when front wall is very close, ramp up steering authority
UTURN_MM          =  850   # below this distance = U-turn territory, boost kicks in
UTURN_SCALE_MAX   =  0.92  # max GAP_SCALE during a tight U-turn (reached at ESTOP_MM)

# ── Obstacle avoidance (moving cars, sudden objects) ───────────
OBSTACLE_WATCH_MM    = 2200  # start monitoring closing rate
OBSTACLE_RATE_THRESH =   35  # mm/loop to flag fast-approaching obstacle
OBSTACLE_CLOSE_MM    =  800  # flag if anything this close AND not a normal corner
OBSTACLE_ENGAGE_MM   =  400  # extra headroom added to CORNER_MM when obstacle flagged
OBSTACLE_HOLD_LOOPS  =   15  # stay in obstacle mode N loops after threat clears (~0.3 s)
OBSTACLE_GAP_SCALE   = 0.52  # obstacle avoidance steering — kept moderate to avoid over-steer

# ZED blind-zone corridor centering.
# When LiDAR goes blind (black walls), use ZED left/right to actively center the car.
# v11: stronger gain + faster smoothing + lower deadband for pure-black corridors.
ZED_CENTER_GAIN   = 0.30
ZED_CENTER_SMOOTH = 0.10

# ── v11 robust black-corridor handling ───────────────────────────────────────
# The black zone is a STRAIGHT corridor. The safe default is to drive dead
# straight and crawl. ZED is only allowed to nudge centering when BOTH its
# side readings are plausibly close — a black rubber wall returns no stereo
# depth and defaults to ~6 m ("wide open"), which would otherwise make the car
# steer INTO it. So we reject any side reading outside the trusted window.
BLIND_STEER_DECAY    = 0.80     # each blind tick: wash inherited turn toward straight
BLIND_ZED_MIN_MM     = 150.0    # closer than this = noise/too close → distrust
BLIND_ZED_MAX_MM     = 3000.0   # farther than this = likely invalid (no stereo) → distrust
BLIND_CENTER_DEADBAND= 0.15     # ignore small L/R asymmetry (noise)

# ZED blind-zone gap following (corners / U-turns).
# ZED_ENGAGE_MM: ZED front feeds into front_dist below this distance (blind zone only).
# BLIND_CORNER_MM: gap follow trigger distance when LiDAR is confirmed blind —
#   wider than CORNER_MM so ZED-triggered corners fire earlier with more reaction time.
# ZED_TURN_GAIN: side differential → gap_s multiplier (strong, for committed turns).
ZED_ENGAGE_MM   = 1250
BLIND_CORNER_MM = 1200
ZED_TURN_GAIN   = 0.42

PRINT_HZ = 8

MODE_AUTONOMOUS = 0
MODE_MANUAL     = 1

_WS_ROOT    = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
LIDAR_DIR   = os.path.join(_WS_ROOT, "v8LiDARData")

_SCAN_STEP  = 2.0 * math.pi / 1024.0
_ANGLE_MIN  = (44  - 384) * _SCAN_STEP
_ANGLE_MAX  = (667 - 384) * _SCAN_STEP
_N_STEPS    = 667 - 44 + 1
_RANGE_MIN  = 0.020
_RANGE_MAX  = 4.095


# ── VESC ERPM poll ────────────────────────────────────────────

def poll_erpm(ser) -> float | None:
    try:
        ser.reset_input_buffer()
        send_packet(ser, bytes([COMM_GET_VALUES]))
        pkt = _read_vesc_packet(ser, timeout=0.03)
        if pkt is None or len(pkt) < 33 or pkt[0] != COMM_GET_VALUES:
            return None
        return abs(struct.unpack(">f", pkt[29:33])[0])
    except Exception:
        return None


# ── Speed controller ──────────────────────────────────────────

class SpeedController:
    def __init__(self):
        self.duty         = MIN_EFFECTIVE_DUTY
        self.integral_sum = 0.0
        self.erpm         = 0.0

    def update(self, target_erpm: float) -> float:
        if self.erpm < 200:
            self.duty = MIN_EFFECTIVE_DUTY + (target_erpm / ERPM_FULL) * (DRIVE_DUTY - MIN_EFFECTIVE_DUTY)
            return max(MIN_EFFECTIVE_DUTY, min(MAX_DUTY, self.duty))
        error             = target_erpm - self.erpm
        self.integral_sum = max(-SPEED_I_CAP, min(SPEED_I_CAP, self.integral_sum + error))
        correction        = SPEED_KP * error + SPEED_KI * self.integral_sum
        self.duty         = max(MIN_EFFECTIVE_DUTY, min(MAX_DUTY, self.duty + correction))
        return self.duty

    def reset(self):
        self.duty         = MIN_EFFECTIVE_DUTY
        self.integral_sum = 0.0


# ── Corridor centering ────────────────────────────────────────

def corridor_steer(clearances: dict, prev_steer: float, integral: float,
                   prev_left_d: float = None, prev_right_d: float = None) -> tuple:
    INF = float("inf")
    CAP = 2000.0

    far_l = min(clearances.get("FAR_LEFT",    INF), CAP)
    l1    = min(clearances.get("LEFT",        INF), CAP)
    l2    = min(clearances.get("CENTER_LEFT", INF), CAP)
    far_r = min(clearances.get("FAR_RIGHT",   INF), CAP)
    r1    = min(clearances.get("RIGHT",       INF), CAP)
    r2    = min(clearances.get("CENTER_RIGHT",INF), CAP)

    left_d  = (far_l * 2.0 + l1 + l2) / 4.0
    right_d = (far_r * 2.0 + r1 + r2) / 4.0
    total   = left_d + right_d

    if total < 50.0:
        return prev_steer, integral, left_d, right_d, "NO_WALLS"

    raw = (left_d - right_d) / total
    if abs(raw) < STEER_DEADBAND:
        raw = 0.0

    # Blind-wall zone: both sides at cap = LiDAR can't see the walls (black barriers).
    # Hold the current heading — any correction here is pure noise-driven zig-zag.
    if left_d > 1500.0 and right_d > 1500.0:
        return prev_steer, integral, left_d, right_d, "BLIND"

    d_term = 0.0
    if prev_left_d is not None and prev_right_d is not None:
        rate_l = left_d  - prev_left_d
        rate_r = right_d - prev_right_d
        d_term = max(-0.5, min(0.5, (rate_l - rate_r) / CAP * STEER_D_GAIN))

    integral = integral * STEER_INTEGRAL_DECAY + raw * 0.02
    integral = max(-STEER_INTEGRAL_MAX, min(STEER_INTEGRAL_MAX, integral))

    l_near = min(far_l, l1, l2)
    r_near = min(far_r, r1, r2)

    if l_near < WALL_PANIC_MM:
        return -1.0, 0.0, left_d, right_d, f"PANIC_L({int(l_near)}mm)"
    if r_near < WALL_PANIC_MM:
        return  1.0, 0.0, left_d, right_d, f"PANIC_R({int(r_near)}mm)"

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
           f"err={raw:+.2f} d={d_term:+.2f} prox={prox_gain:.1f}")
    return smooth, integral, left_d, right_d, dbg


# ── Gap following ─────────────────────────────────────────────

def gap_steer(distances: list, prev_steer: float) -> tuple:
    if not distances:
        return prev_steer, "NO_SCAN"
    processed = preprocess_gap_lidar(distances)
    raw       = find_best_gap_direction(processed)
    smooth    = GAP_SMOOTH * raw + (1.0 - GAP_SMOOTH) * prev_steer
    return smooth, f"gap={raw:+.2f}"


# ── Speed zones ───────────────────────────────────────────────

def compute_target_erpm(front_dist, in_corner: bool) -> tuple:
    if in_corner:
        # Scale corner speed: wide corner → fast, tight → slow
        if front_dist is None or front_dist >= CORNER_MM:
            return ERPM_CORNER_FAST, "CRN-WIDE"
        ratio  = max(0.0, min(1.0, (front_dist - ESTOP_MM) / (CORNER_MM - ESTOP_MM)))
        target = int(ERPM_CORNER_TIGHT + ratio * (ERPM_CORNER_FAST - ERPM_CORNER_TIGHT))
        label  = "CRN-WIDE" if ratio > 0.66 else ("CRN-MED" if ratio > 0.33 else "CRN-TGHT")
        return target, label

    # Straight — approach speed taper before corner zone
    if front_dist is None or front_dist >= CORNER_MM * 1.4:
        return ERPM_FULL, "FULL"
    if front_dist >= CORNER_MM:
        ratio  = (front_dist - CORNER_MM) / (CORNER_MM * 0.4)
        target = int(ERPM_MODERATE + ratio * (ERPM_FULL - ERPM_MODERATE))
        return target, "APPROACH"
    return ERPM_MODERATE, "MODERATE"


# ── ROS node ──────────────────────────────────────────────────

class CorridorNode(Node):
    def __init__(self):
        super().__init__("corridor_node")
        self.create_subscription(Joy,   "/joy",        self._joy_cb,  10)
        self.create_subscription(Int32, "/drive_mode", self._mode_cb, 10)
        self.create_subscription(AckermannDriveStamped, "/manual_drive",
                                 self._manual_cb, 10)
        # ZED depth obstacle data (fused with LiDAR; ZED sees objects at heights LiDAR may miss)
        self._zed_front: float | None = None
        self._zed_left:  float | None = None
        self._zed_right: float | None = None
        self._zed_ts    = 0.0
        self.create_subscription(Float32, "/zed/obstacle_front", self._zed_front_cb, 10)
        self.create_subscription(Float32, "/zed/obstacle_left",  self._zed_left_cb,  10)
        self.create_subscription(Float32, "/zed/obstacle_right", self._zed_right_cb, 10)

        self._drive_pub    = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self._scan_pub     = self.create_publisher(LaserScan, "/scan", 10)
        self.kill_active   = True
        self._joy_axes     = []
        self.drive_mode    = MODE_AUTONOMOUS
        self._manual_drive = None

    def _joy_cb(self, msg: Joy):
        b = list(msg.buttons) + [0] * 16
        self.kill_active = bool(b[BTN_L1] or b[BTN_L1_ALT])
        self._joy_axes   = list(msg.axes)

    def _mode_cb(self, msg: Int32):
        self.drive_mode = msg.data

    def _manual_cb(self, msg: AckermannDriveStamped):
        self._manual_drive = msg

    # Separate EMA alphas: front needs fast reaction to sudden walls,
    # sides need heavy smoothing to prevent zig-zag in black zones.
    _ZED_FRONT_ALPHA = 0.82   # very fast — reacts to new wall in 2 ZED frames
    _ZED_SIDE_ALPHA  = 0.42   # faster convergence so ZED-TURN fires in time

    def _zed_front_cb(self, msg: Float32):
        new = msg.data * 1000.0
        self._zed_front = (self._ZED_FRONT_ALPHA * new + (1 - self._ZED_FRONT_ALPHA) * self._zed_front
                           if self._zed_front is not None else new)
        self._zed_ts = time.time()

    def _zed_left_cb(self, msg: Float32):
        new = msg.data * 1000.0
        self._zed_left = (self._ZED_SIDE_ALPHA * new + (1 - self._ZED_SIDE_ALPHA) * self._zed_left
                          if self._zed_left is not None else new)

    def _zed_right_cb(self, msg: Float32):
        new = msg.data * 1000.0
        self._zed_right = (self._ZED_SIDE_ALPHA * new + (1 - self._ZED_SIDE_ALPHA) * self._zed_right
                           if self._zed_right is not None else new)

    def publish_scan(self, distances: list):
        ranges = [mm / 1000.0 for mm in distances]
        ranges = [r if _RANGE_MIN <= r <= _RANGE_MAX else float("inf") for r in ranges]
        ranges += [float("inf")] * max(0, _N_STEPS - len(ranges))
        ranges  = ranges[:_N_STEPS]
        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min       = _ANGLE_MIN
        msg.angle_max       = _ANGLE_MAX
        msg.angle_increment = _SCAN_STEP
        msg.scan_time       = 0.02
        msg.range_min       = _RANGE_MIN
        msg.range_max       = _RANGE_MAX
        msg.ranges          = ranges
        self._scan_pub.publish(msg)

    def publish_drive(self, duty: float, servo: float):
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.header.frame_id      = "base_link"
        msg.drive.speed          = float(duty)
        msg.drive.steering_angle = float(servo)
        self._drive_pub.publish(msg)


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

    # ── LiDAR data file setup ─────────────────────────────────
    os.makedirs(LIDAR_DIR, exist_ok=True)
    existing = []
    for fname in os.listdir(LIDAR_DIR):
        if fname.startswith("v8LiDAR") and fname.endswith(".txt"):
            try:
                existing.append(int(fname[len("v8LiDAR"):-4]))
            except ValueError:
                pass
    next_num   = max(existing) + 1 if existing else 1
    lidar_path = os.path.join(LIDAR_DIR, f"v8LiDAR{next_num}.txt")
    lidar_file = open(lidar_path, "w")

    _DEG = 180.0 / math.pi
    _angles_deg = [round((44 + i - 384) * _SCAN_STEP * _DEG, 2) for i in range(_N_STEPS)]
    lidar_file.write("# v8 LiDAR Dataset — Hokuyo URG-04LX\n")
    lidar_file.write("# Unit: mm  |  0 = below range, inf = above range or no return\n")
    lidar_file.write(f"# Angles: {_angles_deg[0]}° to {_angles_deg[-1]}°"
                     f"  |  {_N_STEPS} readings per scan  |  {LOOP_HZ} Hz\n")
    lidar_file.write("# Columns: timestamp_sec  mode  distance_mm...\n")
    lidar_file.write("timestamp_sec mode " + " ".join(f"deg_{a}" for a in _angles_deg) + "\n")

    node.get_logger().info("=" * 55)
    node.get_logger().info("  corridor_node v8 — corridor + gap follow")
    node.get_logger().info(f"  Cruise: {ERPM_FULL} ERPM   Corner trigger: {CORNER_MM}mm")
    node.get_logger().info("  L1=kill  R1=toggle MANUAL/AUTONOMOUS")
    node.get_logger().info(f"  LiDAR data → {lidar_path}")
    node.get_logger().info("=" * 55)

    loop_period  = 1.0 / LOOP_HZ
    print_period = 1.0 / PRINT_HZ
    last_print   = 0.0

    steer            = 0.0
    steer_integral   = 0.0
    prev_left_d      = None
    prev_right_d     = None
    prev_front_d     = None
    boost_count      = 0
    was_stopped      = True
    startup_ramp     = 0
    corner_exit_hold = 0
    obstacle_hold    = 0
    erpm_ramp        = 0.0   # smoothed ERPM target fed to PI controller
    blind_streak      = 0    # consecutive ticks both sides read blind (hysteresis)
    lidar_blind_count = 0    # consecutive ticks LiDAR front sees nothing (gates ZED gap follow)

    speed_ctrl      = SpeedController()
    erpm_poll_count = 0

    try:
        while rclpy.ok():
            t0 = time.time()
            rclpy.spin_once(node, timeout_sec=0)

            # ── Kill switch ───────────────────────────
            if node.kill_active:
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                node.publish_drive(0.0, SERVO_CENTER)
                steer = 0.0; steer_integral = 0.0
                prev_left_d = None; prev_right_d = None; prev_front_d = None
                boost_count = 0; was_stopped = True
                startup_ramp = STARTUP_RAMP_LOOPS
                corner_exit_hold = 0; obstacle_hold = 0; erpm_ramp = 0.0; blind_streak = 0; lidar_blind_count = 0
                speed_ctrl.reset(); erpm_poll_count = 0
                elapsed = time.time() - t0
                if elapsed < loop_period:
                    time.sleep(loop_period - elapsed)
                continue

            # ── Manual mode ───────────────────────────
            if node.drive_mode == MODE_MANUAL:
                md = node._manual_drive
                if md is not None:
                    send_servo(ser, md.drive.steering_angle)
                    if abs(md.drive.speed) > 0.002:
                        _send_duty_raw(ser, md.drive.speed)
                    else:
                        send_current_zero(ser)
                else:
                    send_current_zero(ser)
                    send_servo(ser, SERVO_CENTER)
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
                if distances:
                    node.publish_scan(distances)
                    lidar_file.write(
                        f"{time.time():.6f} {node.drive_mode} "
                        + " ".join(str(d) for d in distances) + "\n"
                    )

            # Track consecutive ticks with no LiDAR front wall — gates ZED gap follow.
            if front_dist is None or front_dist > 3500:
                lidar_blind_count = min(lidar_blind_count + 1, 10)
            else:
                lidar_blind_count = 0
            lidar_confirmed_blind = lidar_blind_count >= 2

            ZED_STALE_S = 0.35
            zed_fresh   = (time.time() - node._zed_ts) < ZED_STALE_S

            # ZED gap follow: only when LiDAR is confirmed blind.
            # Lets ZED trigger corners in black-wall sections without touching normal sections.
            if zed_fresh and lidar_confirmed_blind:
                zf = node._zed_front
                if zf is not None and 50 < zf < ZED_ENGAGE_MM:
                    front_dist = zf

            # ── Obstacle detection ────────────────────
            closing_rate = 0.0
            new_obstacle = False
            if front_dist is not None and prev_front_d is not None:
                closing_rate = prev_front_d - front_dist        # positive = approaching
                # Rate trigger: closing fast (e.g. catching up to another car)
                if closing_rate > OBSTACLE_RATE_THRESH and front_dist < OBSTACLE_WATCH_MM:
                    new_obstacle = True
            # Distance trigger: something unexpectedly close while not in a corner
            if (front_dist is not None
                    and front_dist < OBSTACLE_CLOSE_MM
                    and corner_exit_hold == 0):
                new_obstacle = True
            prev_front_d = front_dist

            # Hold: stay in obstacle mode for a while after threat clears
            if new_obstacle:
                obstacle_hold = OBSTACLE_HOLD_LOOPS
            elif obstacle_hold > 0:
                obstacle_hold -= 1
            obstacle = new_obstacle or obstacle_hold > 0

            # If obstacle active, push gap-follow trigger out to engage sooner
            effective_corner_mm = CORNER_MM
            if obstacle and front_dist is not None and front_dist > CORNER_MM:
                effective_corner_mm = min(front_dist + OBSTACLE_ENGAGE_MM,
                                          OBSTACLE_WATCH_MM)
            # Blind zone: widen gap follow trigger so ZED-sourced corners fire earlier
            if lidar_confirmed_blind:
                effective_corner_mm = max(effective_corner_mm, BLIND_CORNER_MM)

            # ── Steering mode ─────────────────────────
            steer_prev = steer
            in_corner  = False

            if GAP_FOLLOW_ENABLED and front_dist is not None and front_dist <= effective_corner_mm:
                # Active corner or obstacle — gap follow steers toward largest opening
                in_corner        = True
                corner_exit_hold = CORNER_EXIT_LOOPS
                steer_integral   = 0.0
                prev_left_d      = None
                prev_right_d     = None
                gap_s, steer_dbg = gap_steer(distances, steer)

                # ZED gap follow: when LiDAR is blind and gap_s is weak, use ZED sides
                # to determine turn direction (LiDAR sees uniform open space → gap_s ≈ 0).
                if lidar_confirmed_blind and abs(gap_s) < 0.40 and zed_fresh:
                    zl = node._zed_left
                    zr = node._zed_right
                    if zl is not None and zr is not None:
                        # (zr-zl) positive: left wall close → steer right (+)
                        # (zr-zl) negative: right wall close → steer left (−)
                        side_diff = (zr - zl) / max(zl + zr, 1.0)
                        if abs(side_diff) > 0.05:
                            gap_s = max(-1.0, min(1.0, side_diff * ZED_TURN_GAIN))
                            steer_dbg += f" ZED-TURN({side_diff:+.2f})"

                # U-turn boost: ramp up steering authority as front wall gets very close
                if front_dist is not None and front_dist < UTURN_MM:
                    t = max(0.0, min(1.0, (UTURN_MM - front_dist) / (UTURN_MM - ESTOP_MM)))
                    effective_gap_scale = GAP_SCALE + t * (UTURN_SCALE_MAX - GAP_SCALE)
                    gap_s = max(-1.0, min(1.0, gap_s * (effective_gap_scale / GAP_SCALE)))
                    steer_dbg += f" UTURN({int(front_dist)}mm)"

                # Side-wall awareness during corner: if turning INTO a wall, blend
                # in corridor centering so the car maintains safe distance from
                # the inside corner wall while still following the gap.
                if clearances:
                    INF    = float("inf")
                    l_near = min(clearances.get("FAR_LEFT",    INF),
                                 clearances.get("LEFT",        INF),
                                 clearances.get("CENTER_LEFT", INF))
                    r_near = min(clearances.get("FAR_RIGHT",    INF),
                                 clearances.get("RIGHT",        INF),
                                 clearances.get("CENTER_RIGHT", INF))
                    close_side = min(l_near, r_near)
                    if close_side < WALL_TURN_MM:
                        corr_s, _, _, _, _ = corridor_steer(clearances, steer, 0.0)
                        wall_w = max(0.0, min(0.65,
                            (WALL_TURN_MM - close_side) / (WALL_TURN_MM - WALL_PANIC_MM)))
                        gap_s  = (1.0 - wall_w) * gap_s + wall_w * corr_s
                        steer_dbg += f" +WALL({wall_w:.2f})"

                steer       = gap_s
                scale, trim = (OBSTACLE_GAP_SCALE if obstacle else GAP_SCALE), STEER_TRIM
                mode        = "OBSTACLE" if obstacle else "CORNER"

            elif GAP_FOLLOW_ENABLED and corner_exit_hold > 0:
                # Blending gap → corridor as corner clears
                in_corner        = True
                corner_exit_hold -= 1
                t      = corner_exit_hold / CORNER_EXIT_LOOPS   # 1.0 → 0.0
                gap_w  = t * t                                  # quadratic: corridor takes over fast
                corr_w = 1.0 - gap_w

                gap_s,  _                                           = gap_steer(distances, steer)
                corr_s, steer_integral, prev_left_d, prev_right_d, corr_dbg = corridor_steer(
                    clearances, steer, steer_integral, prev_left_d, prev_right_d)

                steer     = gap_w * gap_s + corr_w * corr_s
                scale     = GAP_SCALE * gap_w + STEER_SCALE * corr_w
                trim      = STEER_TRIM
                steer_dbg = f"EXIT gap={gap_w:.2f}  {corr_dbg}"
                mode      = "EXIT"

            else:
                # Straight — pure corridor centering
                in_corner = False
                steer, steer_integral, prev_left_d, prev_right_d, steer_dbg = corridor_steer(
                    clearances, steer, steer_integral, prev_left_d, prev_right_d)
                scale, trim = STEER_SCALE, STEER_TRIM
                mode        = "CORRIDOR"

                # BLIND zone: LiDAR can't see the black walls. The zone is a
                # STRAIGHT corridor, so the crash-proof default is: straighten
                # out and crawl. Any inherited turn angle (e.g. from exiting the
                # previous corner) is washed toward straight every tick — this
                # alone stops the car driving into a wall on a held heading.
                #
                # ZED is allowed to add a gentle centering nudge ONLY when BOTH
                # of its side readings fall in the trusted window. A black wall
                # gives no stereo depth and reports ~6 m ("open"); steering on
                # that drives INTO the wall, so we ignore it and stay straight.
                if steer_dbg == "BLIND":
                    blind_streak += 1
                    steer *= BLIND_STEER_DECAY            # baseline: go straight

                    if blind_streak < 3:
                        steer_dbg = f"BLIND-WAIT({blind_streak})"
                    else:
                        zl = node._zed_left
                        zr = node._zed_right
                        both_trusted = (
                            zed_fresh and zl is not None and zr is not None
                            and BLIND_ZED_MIN_MM < zl < BLIND_ZED_MAX_MM
                            and BLIND_ZED_MIN_MM < zr < BLIND_ZED_MAX_MM
                        )
                        if both_trusted:
                            side_diff = (zr - zl) / max(zl + zr, 1.0)
                            if abs(side_diff) > BLIND_CENTER_DEADBAND:
                                zed_target = max(-1.0, min(1.0, side_diff * ZED_CENTER_GAIN))
                                steer = ((1.0 - ZED_CENTER_SMOOTH) * steer
                                         + ZED_CENTER_SMOOTH * zed_target)
                                steer_dbg = f"BLIND-ZED({side_diff:+.2f})"
                            else:
                                steer_dbg = f"BLIND-CENTERED({side_diff:+.2f})"
                        else:
                            # ZED unreliable here (black wall / no depth) → straight
                            steer_dbg = "BLIND-STRAIGHT"
                else:
                    blind_streak = 0

            # Startup ramp
            panic = steer_dbg.startswith("PANIC")
            if startup_ramp > 0:
                if not panic:
                    steer = max(steer_prev - STARTUP_RAMP_STEP,
                                min(steer_prev + STARTUP_RAMP_STEP, steer))
                startup_ramp -= 1

            servo = max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + steer * scale + trim))

            # ── Speed ─────────────────────────────────
            target_erpm, zone = compute_target_erpm(front_dist, in_corner)

            # Obstacle speed penalty — brake based on distance AND closing rate
            if obstacle and front_dist is not None and front_dist > ESTOP_MM:
                dist_ratio  = max(0.0, min(1.0,
                    (front_dist - ESTOP_MM) / (OBSTACLE_WATCH_MM - ESTOP_MM)))
                # Closing fast → brake harder (up to 40% extra reduction)
                rate_boost  = max(0.0, min(0.4,
                    closing_rate / (OBSTACLE_RATE_THRESH * 4)))
                combined    = dist_ratio * (1.0 - rate_boost)
                obs_erpm    = int(ERPM_CRAWL + combined * (ERPM_CORNER_TIGHT - ERPM_CRAWL))
                if obs_erpm < target_erpm:
                    target_erpm = obs_erpm
                    zone += "+OBS"

            # Side wall speed penalty
            if clearances:
                INF    = float("inf")
                l_near = min(clearances.get("FAR_LEFT", INF),
                             clearances.get("LEFT", INF),
                             clearances.get("CENTER_LEFT", INF))
                r_near = min(clearances.get("FAR_RIGHT", INF),
                             clearances.get("RIGHT", INF),
                             clearances.get("CENTER_RIGHT", INF))
                side   = min(l_near, r_near)
                if side < WALL_PANIC_MM:
                    target_erpm = ERPM_CRAWL
                    zone += "+WALL!"
                elif side < SIDE_MIN_MM:
                    ratio       = (side - WALL_PANIC_MM) / (SIDE_MIN_MM - WALL_PANIC_MM)
                    target_erpm = int(ERPM_CRAWL + ratio * (target_erpm - ERPM_CRAWL))
                    zone += "+SLOW"

            # Blind zone speed cap — slower so ZED has time to make decisions
            if lidar_confirmed_blind and target_erpm > ERPM_CORNER_TIGHT:
                target_erpm = ERPM_CORNER_TIGHT
                zone += "+BLIND-SLOW"

            # Poll VESC ERPM
            erpm_poll_count += 1
            if erpm_poll_count >= ERPM_POLL_EVERY:
                erpm_poll_count = 0
                measured = poll_erpm(ser)
                if measured is not None:
                    speed_ctrl.erpm = measured

            if target_erpm > 0:
                if was_stopped:
                    boost_count = START_BOOST_LOOPS
                    was_stopped = False

                # Ramp erpm_ramp toward target_erpm at accel/decel rates
                if target_erpm > erpm_ramp:
                    erpm_ramp = min(target_erpm, erpm_ramp + ACCEL_RATE)
                else:
                    erpm_ramp = max(target_erpm, erpm_ramp - DECEL_RATE)

                duty = speed_ctrl.update(erpm_ramp)
                if boost_count > 0:
                    duty = max(duty, START_BOOST_DUTY)
                    boost_count -= 1
                send_servo(ser, servo)
                send_duty(ser, duty)
            else:
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                duty = 0.0; steer = 0.0; steer_integral = 0.0
                prev_left_d = None; prev_right_d = None; prev_front_d = None
                boost_count = 0; was_stopped = True
                corner_exit_hold = 0; obstacle_hold = 0; erpm_ramp = 0.0; blind_streak = 0
                speed_ctrl.reset()

            node.publish_drive(duty, servo)

            # ── Print ─────────────────────────────────
            now = time.time()
            if now - last_print >= print_period:
                fd = f"{front_dist}mm" if front_dist is not None else "----"
                print(
                    f"\r[{mode:<8}|{zone:<10}] Front:{fd:<6} "
                    f"Steer:{steer:+.3f} Servo:{servo:.3f} Duty:{duty:.3f} "
                    f"ERPM:{int(speed_ctrl.erpm):<6}  {steer_dbg[:45]}    ",
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
        try:
            lidar_file.close()
            print(f"[corridor_node] LiDAR data saved → {lidar_path}")
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[corridor_node] Done")


if __name__ == "__main__":
    main()
