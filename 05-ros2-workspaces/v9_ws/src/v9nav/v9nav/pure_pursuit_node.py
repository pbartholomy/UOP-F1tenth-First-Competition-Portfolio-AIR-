#!/usr/bin/env python3
"""
pure_pursuit_node.py — v9: pure pursuit path following.

On startup, tries to load ~/Desktop/v9_ws/track_path.json.
Also reloads when /mapping_save is received (new path just saved).

Only runs when drive_mode == 0 (AUTONOMOUS).

Publishes /pp_drive (AckermannDriveStamped):
  steering_angle = servo position (0.0–1.0)
  speed          = duty cycle

corridor_node's reactive obstacle avoidance acts as a safety overlay —
if an obstacle appears the obstacle/gap logic in corridor_node takes over.
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Int32, Bool
from ackermann_msgs.msg import AckermannDriveStamped

import tf2_ros

# ── Pure Pursuit Parameters ───────────────────────────────────
LOOKAHEAD_DIST  = 0.7    # meters
WHEELBASE       = 0.29   # meters (Traxxas Ford Fiesta)
SERVO_CENTER    = 0.50
SERVO_MIN       = 0.15
SERVO_MAX       = 0.85
STEER_TRIM      = 0.11
PP_SCALE        = 0.55
PP_SPEED_DUTY   = 0.20
PP_CORNER_DUTY  = 0.15   # used when abs(steer_norm) > 0.45
MAX_STEER_RAD   = 0.52

MODE_AUTONOMOUS = 0
PP_HZ           = 50.0

PATH_FILE = os.path.expanduser("~/Desktop/v9_ws/track_path.json")

# Search window around closest waypoint (±N indices, wrapping)
SEARCH_WINDOW = 50


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _angle_wrap(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while a >  math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        self._drive_mode = 1   # start assumed manual
        self._waypoints  = []  # list of [x, y, yaw]
        self._closest_idx = 0

        # TF listener
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscriptions
        self.create_subscription(Int32, "/drive_mode",   self._mode_cb, 10)
        self.create_subscription(Bool,  "/mapping_save", self._save_cb, 10)

        # Publisher
        self._pp_pub = self.create_publisher(
            AckermannDriveStamped, "/pp_drive", 10)

        # Control timer
        self.create_timer(1.0 / PP_HZ, self._tick)

        # Load path on startup
        self._load_path()

    def _load_path(self):
        if not os.path.exists(PATH_FILE):
            self.get_logger().info(
                f"No track_path.json found at {PATH_FILE} — will load when saved")
            return
        try:
            with open(PATH_FILE, "r") as f:
                self._waypoints = json.load(f)
            self.get_logger().info(
                f"Loaded path: {len(self._waypoints)} waypoints from {PATH_FILE}")
        except Exception as e:
            self.get_logger().error(f"Failed to load path: {e}")
            self._waypoints = []

    def _mode_cb(self, msg: Int32):
        self._drive_mode = msg.data

    def _save_cb(self, msg: Bool):
        """Reload path when mapping_node saves a new one."""
        if msg.data:
            self.get_logger().info("mapping_save received — reloading path")
            self._load_path()
            self._closest_idx = 0

    def _tick(self):
        """Pure pursuit control loop at PP_HZ."""
        if self._drive_mode != MODE_AUTONOMOUS:
            return
        if len(self._waypoints) < 2:
            return

        # Get current pose from TF
        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        cx   = tf.transform.translation.x
        cy   = tf.transform.translation.y
        cyaw = _quat_to_yaw(tf.transform.rotation)

        n = len(self._waypoints)

        # ── 1. Find closest waypoint (search window ± SEARCH_WINDOW, wrapping) ──
        best_dist = float("inf")
        best_idx  = self._closest_idx
        for di in range(-SEARCH_WINDOW, SEARCH_WINDOW + 1):
            idx  = (self._closest_idx + di) % n
            wx, wy = self._waypoints[idx][0], self._waypoints[idx][1]
            d    = math.hypot(cx - wx, cy - wy)
            if d < best_dist:
                best_dist = d
                best_idx  = idx
        self._closest_idx = best_idx

        # ── 2. Walk forward from closest_idx until >= LOOKAHEAD_DIST ──
        acc_dist  = 0.0
        prev_idx  = best_idx
        lookahead_idx = best_idx
        for _ in range(n):
            next_idx = (prev_idx + 1) % n
            wx0, wy0 = self._waypoints[prev_idx][0],  self._waypoints[prev_idx][1]
            wx1, wy1 = self._waypoints[next_idx][0],  self._waypoints[next_idx][1]
            seg_len   = math.hypot(wx1 - wx0, wy1 - wy0)
            acc_dist += seg_len
            lookahead_idx = next_idx
            prev_idx      = next_idx
            if acc_dist >= LOOKAHEAD_DIST:
                break

        # ── 3. Compute alpha = bearing to lookahead - current yaw ──
        lx = self._waypoints[lookahead_idx][0]
        ly = self._waypoints[lookahead_idx][1]
        dx = lx - cx
        dy = ly - cy
        bearing = math.atan2(dy, dx)
        alpha   = _angle_wrap(bearing - cyaw)

        actual_dist = math.hypot(dx, dy)
        effective_L = max(LOOKAHEAD_DIST, actual_dist)

        # ── 4-7. Pure pursuit geometry ──
        curvature  = 2.0 * math.sin(alpha) / effective_L
        steer_rad  = math.atan(curvature * WHEELBASE)
        steer_norm = _clamp(steer_rad / MAX_STEER_RAD, -1.0, 1.0)
        servo      = _clamp(
            SERVO_CENTER + steer_norm * PP_SCALE + STEER_TRIM,
            SERVO_MIN, SERVO_MAX
        )

        # ── 8. Speed ──
        duty = PP_CORNER_DUTY if abs(steer_norm) > 0.45 else PP_SPEED_DUTY

        # ── Publish ──
        msg = AckermannDriveStamped()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.header.frame_id      = "base_link"
        msg.drive.steering_angle = float(servo)
        msg.drive.speed          = float(duty)
        self._pp_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
