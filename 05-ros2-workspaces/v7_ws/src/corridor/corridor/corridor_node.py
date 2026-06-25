#!/usr/bin/env python3
"""
corridor_node.py — v7: corridor + corners + ERPM ctrl + manual/autonomous toggle.

Drive modes (from /drive_mode, published by mode_manager_node):
  0 = AUTONOMOUS   : reactive corridor centering + gap follow (default)
  1 = MANUAL       : joystick controls steering and throttle directly
  3 = PURE_PURSUIT : follow recorded waypoints; reactive safety overrides

Joystick (PS4):
  L1 (btn 4 or 9) : kill switch — hold to stop, release to drive
  R1 (btn 5)      : toggle MANUAL ↔ AUTONOMOUS  (via mode_manager_node)
"""

import math
import os
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32, Float32
import tf2_ros
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

# Non-inverting duty send for manual relay — car_node computes duty with its
# own inversion already applied, so we must not invert again here.
def _send_duty_raw(ser, duty):
    duty = max(-0.6, min(0.6, duty))
    send_packet(ser, bytes([5]) + struct.pack(">i", int(duty * 100000)))

# ── TUNING ────────────────────────────────────────────────────
DRIVE_DUTY        = 0.240   # cruise speed
MAX_DUTY          = 0.500   # PI controller ceiling (hard cap on duty output)
START_BOOST_DUTY  = 0.250   # kick from standstill
START_BOOST_LOOPS = 10      # boost duration at 50Hz (~200ms)

# Speed zones (mm)
FULL_SPEED_MM  =  700
MODERATE_MM    =  500       # 85% speed
CORNER_MM      = 1150       # switch to gap-follow + slow for corner
SLOW_MM        =  450       # 70% speed
ESTOP_MM       =  350       # hard stop

# Corner speed
CORNER_DUTY    = 0.050      # speed while turning (raise carefully)

# Corridor centering (STRAIGHT mode)
STEER_GAIN          = 1.30
STEER_DEADBAND      = 0.0
STEER_SMOOTH        = 0.72
STEER_SCALE         = 0.64  # servo throw for straight corrections

STEER_INTEGRAL_GAIN = 0.45  # how strongly accumulated drift is corrected
STEER_INTEGRAL_MAX  = 0.40  # cap to prevent windup
STEER_INTEGRAL_DECAY= 0.95  # per-loop decay (~halves in ~23 loops / 0.46s)

STEER_D_GAIN   = 3.8        # derivative: anticipate wall approach before it's close
STEER_TRIM     = 0.09       # positive = bias right, negative = bias left; fix constant centering offset

STARTUP_RAMP_LOOPS = 15      # loops to ramp steering after kill release (~300ms @ 50Hz)
STARTUP_RAMP_STEP  = 0.05    # max |Δsteer| per loop during ramp window — softens the initial correction

SIDE_MIN_MM    = 1300       # start proportional push away from wall
WALL_TURN_MM   = 520        # this close → override and actively turn away
WALL_TURN_STEER= 0.75       # how hard to turn away (0–1, raise if still hugging)
WALL_PANIC_MM  = 260        # this close → full opposite steer immediately

# Gap following (CORNER mode)
GAP_SCALE      = 0.72       # servo throw for corners (raise if under-steering)
GAP_SMOOTH     = 0.65       # EMA for gap steer (higher = snappier turn)
CORNER_EXIT_LOOPS = 16      # loops to stay in gap mode after corner clears (~320ms)

PRINT_HZ       = 8

# ── ERPM Speed Controller ─────────────────────────────────────
# Target ERPMs — tune by running the car and observing printed ERPM values
# at your current working duty cycles, then set these to match desired speeds.
ERPM_FULL        =  95000  # cruise (replaces DRIVE_DUTY)
ERPM_MODERATE    =  78000  # 85% zone
ERPM_CORNER_PRE  =  66000  # 80% zone (pre-corner slowdown)
ERPM_CORNER_FAST =  75000  # sweeping/wide corner (front dist near CORNER_MM)
ERPM_CORNER_TIGHT=  52000  # hairpin (front dist near ESTOP_MM)
ERPM_SLOW        =  45000  # tight slow zone
ERPM_CRAWL       =  34000  # near-stop crawl

# PI gains — increase Kp if response is too slow, decrease if duty oscillates
SPEED_KP        = 0.0000010   # duty change per ERPM of error each loop
SPEED_KI        = 0.0000004   # duty change per accumulated ERPM error
SPEED_I_CAP     = 15000.0     # integral sum cap (prevents windup)

ERPM_POLL_EVERY = 3           # poll VESC every N loops (~17 Hz at 50 Hz loop)
COMM_GET_VALUES = 4

# ── Manual drive + odometry ───────────────────────────────────
SERVO_THROW         = SERVO_MAX - SERVO_CENTER   # 0.35
MANUAL_THROTTLE_AX  = 1     # left stick Y  (push up = forward after inversion)
MANUAL_STEER_AX     = 2     # right stick X (push right = steer right)
MANUAL_STEER_SIGN   = 1.0   # flip to -1.0 if steering is reversed
MANUAL_DEADZONE     = 0.10
MANUAL_RAMP_STEP    = 0.003
MANUAL_MAX_DUTY      = 0.6  # full hand-driving authority (matches CAR.py) — independent of autonomous MAX_DUTY

WHEELBASE     = 0.32        # m — Traxxas Fiesta ST Rally
ERPM_TO_MS    = 0.000103    # m/s per ERPM — calibrate if needed

_SCAN_STEP    = 2.0 * math.pi / 1024.0
_ANGLE_MIN    = (44  - 384) * _SCAN_STEP
_ANGLE_MAX    = (667 - 384) * _SCAN_STEP
_N_STEPS      = 667 - 44 + 1
_RANGE_MIN    = 0.020
_RANGE_MAX    = 4.095

MODE_AUTONOMOUS   = 0
MODE_MANUAL       = 1
MODE_PURE_PURSUIT = 3

_WS_ROOT    = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
DATASET_DIR = os.path.join(_WS_ROOT, "v7_LiDAR_DataSet")
# ─────────────────────────────────────────────────────────────



def poll_erpm(ser) -> float | None:
    """Request COMM_GET_VALUES from VESC and return absolute ERPM, or None on failure."""
    try:
        ser.reset_input_buffer()
        send_packet(ser, bytes([COMM_GET_VALUES]))
        pkt = _read_vesc_packet(ser, timeout=0.03)
        if pkt is None or len(pkt) < 33 or pkt[0] != COMM_GET_VALUES:
            return None
        erpm = struct.unpack(">f", pkt[29:33])[0]
        return abs(erpm)
    except Exception:
        return None


class SpeedController:
    """PI controller that adjusts VESC duty to hit a target ERPM."""

    def __init__(self):
        self.duty          = MIN_EFFECTIVE_DUTY
        self.integral_sum  = 0.0
        self.erpm          = 0.0   # last measured ERPM

    def update(self, target_erpm: float) -> float:
        if self.erpm < 200:
            # No valid measurement yet — use feedforward duty proportional to target
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


class CorridorNode(Node):
    def __init__(self):
        super().__init__("corridor_node")
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self._drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.kill_active  = True
        self._joy_axes    = []

        # v7 additions
        self._scan_pub  = self.create_publisher(LaserScan, "/scan", 10)
        self._odom_pub  = self.create_publisher(Odometry,  "/odom", 10)
        self._tf_br     = tf2_ros.TransformBroadcaster(self)
        self.drive_mode = MODE_AUTONOMOUS
        self._pp_drive  = None
        self._ox = 0.0; self._oy = 0.0; self._oyaw = 0.0
        self.create_subscription(Int32, "/drive_mode", self._mode_cb, 10)
        self.create_subscription(AckermannDriveStamped, "/pure_pursuit/drive",
                                 self._pp_cb, 10)

        # Manual drive commands from car_node
        self._manual_drive: AckermannDriveStamped | None = None
        self.create_subscription(
            AckermannDriveStamped, "/manual_drive", self._manual_cb, 10)

        # ZED 2i depth obstacle fuser
        self._zed_front: float | None = None
        self._zed_left:  float | None = None
        self._zed_right: float | None = None
        self.create_subscription(Float32, "/zed/obstacle_front", self._zed_front_cb, 10)
        self.create_subscription(Float32, "/zed/obstacle_left",  self._zed_left_cb,  10)
        self.create_subscription(Float32, "/zed/obstacle_right", self._zed_right_cb, 10)

    def _joy_cb(self, msg: Joy):
        b = list(msg.buttons) + [0] * 16
        self.kill_active = bool(b[BTN_L1] or b[BTN_L1_ALT])
        self._joy_axes   = list(msg.axes)

    def _mode_cb(self, msg: Int32):
        self.drive_mode = msg.data

    def _pp_cb(self, msg: AckermannDriveStamped):
        self._pp_drive = msg

    def _manual_cb(self, msg: AckermannDriveStamped): self._manual_drive = msg
    def _zed_front_cb(self, msg: Float32): self._zed_front = msg.data
    def _zed_left_cb(self,  msg: Float32): self._zed_left  = msg.data
    def _zed_right_cb(self, msg: Float32): self._zed_right = msg.data

    def publish_drive(self, duty: float, servo: float):
        msg = AckermannDriveStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.drive.speed          = float(duty)
        msg.drive.steering_angle = float(servo)
        self._drive_pub.publish(msg)

    def publish_scan(self, distances: list):
        ranges = [mm / 1000.0 for mm in distances]
        ranges = [m if _RANGE_MIN <= m <= _RANGE_MAX else float("inf") for m in ranges]
        ranges += [float("inf")] * max(0, _N_STEPS - len(ranges))
        ranges  = ranges[:_N_STEPS]
        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min       = _ANGLE_MIN
        msg.angle_max       = _ANGLE_MAX
        msg.angle_increment = _SCAN_STEP
        msg.scan_time       = 0.1
        msg.range_min       = _RANGE_MIN
        msg.range_max       = _RANGE_MAX
        msg.ranges          = ranges
        self._scan_pub.publish(msg)

    def update_and_publish_odom(self, erpm: float, servo: float, dt: float):
        v           = erpm * ERPM_TO_MS
        steer_angle = (servo - SERVO_CENTER) / SERVO_THROW * 0.50
        omega       = (v * math.tan(steer_angle) / WHEELBASE) if abs(steer_angle) > 0.001 else 0.0
        self._oyaw += omega * dt
        self._ox   += v * math.cos(self._oyaw) * dt
        self._oy   += v * math.sin(self._oyaw) * dt
        cy = math.cos(self._oyaw * 0.5)
        sy = math.sin(self._oyaw * 0.5)
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = "odom"
        t.child_frame_id  = "base_link"
        t.transform.translation.x = self._ox
        t.transform.translation.y = self._oy
        t.transform.rotation.w    = cy
        t.transform.rotation.z    = sy
        self._tf_br.sendTransform(t)
        odom = Odometry()
        odom.header.stamp            = now
        odom.header.frame_id         = "odom"
        odom.child_frame_id          = "base_link"
        odom.pose.pose.position.x    = self._ox
        odom.pose.pose.position.y    = self._oy
        odom.pose.pose.orientation.w = cy
        odom.pose.pose.orientation.z = sy
        odom.twist.twist.linear.x    = v
        self._odom_pub.publish(odom)


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

    # Panic override — wall critically close; bypass smoother for instant full deflection
    if l_near < WALL_PANIC_MM:
        dbg = f"PANIC_L({int(l_near)}mm)"
        return -1.0, 0.0, left_d, right_d, dbg
    if r_near < WALL_PANIC_MM:
        dbg = f"PANIC_R({int(r_near)}mm)"
        return 1.0, 0.0, left_d, right_d, dbg

    # Normal corridor centering — scale gain by proximity to wall
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

def compute_target_erpm(front_dist, in_corner: bool) -> tuple:
    if in_corner:
        # Scale corner speed by how open the corner is:
        # front_dist near CORNER_MM → wide/sweeping → ERPM_CORNER_FAST
        # front_dist near ESTOP_MM  → hairpin        → ERPM_CORNER_TIGHT
        if front_dist is None or front_dist >= CORNER_MM:
            return ERPM_CORNER_FAST, "CRN-WIDE"
        ratio  = max(0.0, min(1.0, (front_dist - ESTOP_MM) / (CORNER_MM - ESTOP_MM)))
        target = int(ERPM_CORNER_TIGHT + ratio * (ERPM_CORNER_FAST - ERPM_CORNER_TIGHT))
        label  = "CRN-WIDE" if ratio > 0.66 else ("CRN-MED" if ratio > 0.33 else "CRN-TGHT")
        return target, label
    if front_dist is None or front_dist >= FULL_SPEED_MM:
        return ERPM_FULL, "FULL"
    if front_dist >= MODERATE_MM:
        return ERPM_MODERATE, "MODERATE"
    if front_dist >= CORNER_MM:
        return ERPM_CORNER_PRE, "PRE-CORNER"
    if front_dist >= SLOW_MM:
        return ERPM_SLOW, "SLOW"
    if front_dist >= ESTOP_MM:
        return ERPM_CRAWL, "CRAWL"
    return 0, "ESTOP"


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

    # ── LiDAR dataset file setup ──────────────────────────────
    os.makedirs(DATASET_DIR, exist_ok=True)
    existing_nums = []
    for fname in os.listdir(DATASET_DIR):
        if fname.startswith("LiDARData") and fname.endswith(".txt"):
            try:
                existing_nums.append(int(fname[len("LiDARData"):-4]))
            except ValueError:
                pass
    next_num        = max(existing_nums) + 1 if existing_nums else 1
    lidar_file_path = os.path.join(DATASET_DIR, f"LiDARData{next_num}.txt")
    lidar_file      = open(lidar_file_path, "w")

    _DEG = 180.0 / math.pi
    _scan_angles_deg = [
        round((44 + i - 384) * _SCAN_STEP * _DEG, 2)
        for i in range(_N_STEPS)
    ]
    lidar_file.write("# LiDAR Dataset — Hokuyo (v7_ws corridor_node)\n")
    lidar_file.write("# Unit: mm  |  0 = below range, inf = above range or no return\n")
    lidar_file.write(f"# Angle range: {_scan_angles_deg[0]}° to {_scan_angles_deg[-1]}°"
                     f"  |  {_N_STEPS} readings per scan  |  ~{LOOP_HZ} Hz\n")
    lidar_file.write("# Columns: timestamp_sec (Unix epoch), drive_mode, then distance_mm at each angle\n")
    lidar_file.write("timestamp_sec mode " + " ".join(f"deg_{a}" for a in _scan_angles_deg) + "\n")

    node.get_logger().info("=" * 58)
    node.get_logger().info("  corridor_node v7 — reactive + manual + pure pursuit")
    node.get_logger().info(f"  Cruise: {ERPM_FULL} ERPM  Corner: {ERPM_CORNER_TIGHT}–{ERPM_CORNER_FAST} ERPM")
    node.get_logger().info(f"  Corner trigger: front < {CORNER_MM}mm  ESTOP: {ESTOP_MM}mm")
    node.get_logger().info("  L1=kill  R1=toggle MANUAL/AUTONOMOUS")
    node.get_logger().info(f"  LiDAR data → {lidar_file_path}")
    node.get_logger().info("=" * 58)

    loop_period  = 1.0 / LOOP_HZ
    print_period = 1.0 / PRINT_HZ
    last_print   = 0.0

    steer            = 0.0
    steer_integral   = 0.0
    prev_left_d      = None
    prev_right_d     = None
    boost_count      = 0
    was_stopped      = True
    corner_exit_hold = 0
    manual_duty      = 0.0
    startup_ramp     = 0

    speed_ctrl       = SpeedController()
    erpm_poll_count  = 0

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
                manual_duty      = 0.0
                startup_ramp     = STARTUP_RAMP_LOOPS
                speed_ctrl.reset()
                erpm_poll_count  = 0
                node._ox = 0.0; node._oy = 0.0; node._oyaw = 0.0
                elapsed = time.time() - t0
                if elapsed < loop_period:
                    time.sleep(loop_period - elapsed)
                continue

            # ── Manual mode ───────────────────────────
            # car_node computes duty/servo and publishes /manual_drive.
            # corridor_node owns the VESC and relays those values here so
            # there is no extra topic hop in the autonomous path.
            if node.drive_mode == MODE_MANUAL:
                md = node._manual_drive
                if md is not None:
                    m_duty  = md.drive.speed           # pre-computed, inversion already applied
                    m_servo = md.drive.steering_angle
                    send_servo(ser, m_servo)
                    if abs(m_duty) > 0.002:
                        _send_duty_raw(ser, m_duty)    # relay without re-inverting
                    else:
                        send_current_zero(ser)
                else:
                    send_current_zero(ser)
                    send_servo(ser, SERVO_CENTER)
                if lidar and lidar.connected:
                    m_distances = lidar.get_distances()
                    if m_distances:
                        node.publish_scan(m_distances)
                        lidar_file.write(f"{time.time():.6f} MANUAL " + " ".join(str(d) for d in m_distances) + "\n")
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

            # ZED data is received but intentionally not applied — LiDAR owns all decisions

            # ── Mode + steering ───────────────────────
            steer_prev_loop = steer  # pre-update value, used by the startup ramp limiter below
            if front_dist is not None and front_dist <= CORNER_MM:
                # Active corner — pure gap follow
                in_corner        = True
                corner_exit_hold = CORNER_EXIT_LOOPS
                steer_integral   = 0.0  # reset integral through corners
                prev_left_d      = None
                prev_right_d     = None
                gap_s, gap_dbg   = gap_steer(distances, steer)
                steer            = gap_s
                scale, trim      = GAP_SCALE, 0.0
                panic            = False
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
                # taper servo throw from GAP_SCALE → STEER_SCALE as corridor takes over
                scale     = GAP_SCALE * gap_w + STEER_SCALE * corr_w
                trim      = STEER_TRIM * corr_w
                panic     = corr_dbg.startswith("PANIC")
                steer_dbg = f"EXIT blend gap={gap_w:.2f} corr={corr_w:.2f}  {corr_dbg}"
                mode = "EXIT"

            else:
                # Straight — pure corridor centering
                in_corner = False
                steer, steer_integral, prev_left_d, prev_right_d, steer_dbg = corridor_steer(
                    clearances, steer, steer_integral, prev_left_d, prev_right_d)
                scale, trim = STEER_SCALE, STEER_TRIM
                panic       = steer_dbg.startswith("PANIC")
                mode = "STRAIGHT"

            # Startup ramp limiter — caps |Δsteer| per loop for a short window after kill
            # release so the first correction can't snap straight to full deflection.
            # Panic override (wall < WALL_PANIC_MM) always bypasses this.
            if startup_ramp > 0:
                if not panic:
                    steer = max(steer_prev_loop - STARTUP_RAMP_STEP,
                                min(steer_prev_loop + STARTUP_RAMP_STEP, steer))
                startup_ramp -= 1

            servo = max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + steer * scale + trim))

            # ── Speed ─────────────────────────────────
            target_erpm, zone = compute_target_erpm(front_dist, in_corner)

            # Side wall speed penalty
            if not in_corner:
                INF       = float("inf")
                l_near    = min(clearances.get("FAR_LEFT", INF), clearances.get("LEFT", INF), clearances.get("CENTER_LEFT", INF))
                r_near    = min(clearances.get("FAR_RIGHT", INF), clearances.get("RIGHT", INF), clearances.get("CENTER_RIGHT", INF))
                side_near = min(l_near, r_near)
                if side_near < WALL_PANIC_MM:
                    target_erpm = ERPM_CRAWL
                    zone += "+WALL!"
                elif side_near < SIDE_MIN_MM:
                    ratio       = (side_near - WALL_PANIC_MM) / (SIDE_MIN_MM - WALL_PANIC_MM)
                    target_erpm = int(ERPM_CRAWL + ratio * (target_erpm - ERPM_CRAWL))
                    zone += "+SLOW"

            # Poll ERPM from VESC every N loops
            erpm_poll_count += 1
            if erpm_poll_count >= ERPM_POLL_EVERY:
                erpm_poll_count = 0
                measured = poll_erpm(ser)
                if measured is not None:
                    speed_ctrl.erpm = measured

            # ── Pure pursuit override ─────────────────
            # PP steers + sets speed; reactive safety stops always win.
            if (node.drive_mode == MODE_PURE_PURSUIT
                    and node._pp_drive is not None
                    and target_erpm > ERPM_CRAWL):
                pp      = node._pp_drive
                pp_erpm = min(int(pp.drive.speed / ERPM_TO_MS), ERPM_FULL)
                if pp_erpm > 0:
                    servo       = max(SERVO_MIN, min(SERVO_MAX,
                                      SERVO_CENTER + pp.drive.steering_angle
                                      / 0.50 * SERVO_THROW))
                    target_erpm = max(ERPM_SLOW, pp_erpm)
                    mode        = "PP"
                    zone        = "PP"

            if target_erpm > 0:
                if was_stopped:
                    boost_count = START_BOOST_LOOPS
                    was_stopped = False
                duty = speed_ctrl.update(target_erpm)
                if boost_count > 0:
                    duty = max(duty, START_BOOST_DUTY)
                    boost_count -= 1
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
                speed_ctrl.reset()

            node.publish_drive(duty, servo)
            node.update_and_publish_odom(speed_ctrl.erpm, servo, loop_period)

            if distances:
                lidar_file.write(f"{time.time():.6f} {mode} " + " ".join(str(d) for d in distances) + "\n")

            # ── Print ─────────────────────────────────
            now = time.time()
            if now - last_print >= print_period:
                fd = f"{front_dist}mm" if front_dist is not None else "----"
                print(
                    f"\r[{mode:<8}|{zone:<10}] Front:{fd:<6} "
                    f"Steer:{steer:+.3f} Servo:{servo:.3f} Duty:{duty:.3f} "
                    f"ERPM:{int(speed_ctrl.erpm):<6}    ",
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
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[corridor_node] Done")


if __name__ == "__main__":
    main()
