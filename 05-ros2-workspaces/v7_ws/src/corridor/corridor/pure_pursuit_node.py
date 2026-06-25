#!/usr/bin/env python3
"""
Pure Pursuit path follower — v6.

Reads waypoints from a CSV (x, y, speed_m/s) recorded by waypoint_logger_node.
Publishes AckermannDriveStamped to /pure_pursuit/drive.
Lookahead scales with speed: L = k_dd * v, clamped to [L_min, L_max].
"""

import math
import csv
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32

MODE_PURE_PURSUIT = 3


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__("pure_pursuit_node")

        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("wheelbase",      0.32)
        self.declare_parameter("k_dd",           0.5)
        self.declare_parameter("lookahead_min",  0.4)
        self.declare_parameter("lookahead_max",  2.0)
        self.declare_parameter("speed_scale",    1.0)
        self.declare_parameter("max_speed",      3.5)
        self.declare_parameter("min_speed",      0.5)
        self.declare_parameter("steering_gain",  1.0)
        self.declare_parameter("use_zed_odom",   True)

        self.wb           = self.get_parameter("wheelbase").value
        self.k_dd         = self.get_parameter("k_dd").value
        self.L_min        = self.get_parameter("lookahead_min").value
        self.L_max        = self.get_parameter("lookahead_max").value
        self.speed_scale  = self.get_parameter("speed_scale").value
        self.max_speed    = self.get_parameter("max_speed").value
        self.min_speed    = self.get_parameter("min_speed").value
        self.steer_gain   = self.get_parameter("steering_gain").value
        self._use_zed     = self.get_parameter("use_zed_odom").value

        self._mode        = 0
        self._waypoints   = []
        self._wp_arr      = None
        self._closest_idx = 0
        self._cur_speed   = self.min_speed
        self._loaded      = False
        self._zed_odom_ok = False   # True once ZED publishes at least one message

        wp_file = self.get_parameter("waypoints_file").value
        if wp_file and os.path.exists(wp_file):
            self._load_waypoints(wp_file)
        else:
            self.get_logger().warn(
                "No waypoints_file set — won't publish until loaded. "
                "Pass: --ros-args -p waypoints_file:=<path>"
            )

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Always keep the dead-reckoning odom as fallback
        self.create_subscription(Odometry, "/odom", self._odom_fallback_cb, sensor_qos)
        self.create_subscription(Int32, "/drive_mode", self._mode_cb, 10)

        if self._use_zed:
            self.create_subscription(
                Odometry, "/zed/zed_node/odom", self._odom_cb, sensor_qos)
            self.get_logger().info("PurePursuitNode: using ZED VIO odom (fallback: /odom)")
        else:
            self.create_subscription(Odometry, "/odom", self._odom_cb, sensor_qos)
            self.get_logger().info("PurePursuitNode: using dead-reckoning /odom")

        self._drive_pub = self.create_publisher(
            AckermannDriveStamped, "/pure_pursuit/drive", 10)

        self.get_logger().info("PurePursuitNode ready")

    def _load_waypoints(self, path: str):
        wps = []
        with open(path, "r") as f:
            for row in csv.reader(f):
                if len(row) >= 2:
                    x, y = float(row[0]), float(row[1])
                    spd  = float(row[2]) if len(row) >= 3 else 1.5
                    wps.append((x, y, spd))
        self._waypoints   = wps
        self._wp_arr      = np.array([(w[0], w[1]) for w in wps], dtype=np.float64)
        self._closest_idx = 0
        self._loaded      = True
        self.get_logger().info(f"Loaded {len(wps)} waypoints from {path}")

    def _mode_cb(self, msg: Int32):
        self._mode = msg.data

    def _odom_fallback_cb(self, msg: Odometry):
        # Only use dead-reckoning when ZED VIO hasn't published yet
        if self._use_zed and self._zed_odom_ok:
            return
        self._odom_cb(msg)

    def _odom_cb(self, msg: Odometry):
        self._zed_odom_ok = True
        if self._mode != MODE_PURE_PURSUIT or not self._loaded:
            return

        px  = msg.pose.pose.position.x
        py  = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        vx  = msg.twist.twist.linear.x
        self._cur_speed = abs(vx) if abs(vx) > 0.1 else self.min_speed

        L  = min(self.L_max, max(self.L_min, self.k_dd * self._cur_speed))
        n  = len(self._waypoints)
        window     = min(n, 80)
        candidates = np.arange(self._closest_idx - 5, self._closest_idx + window) % n
        diffs      = self._wp_arr[candidates] - np.array([px, py])
        dists      = np.linalg.norm(diffs, axis=1)
        self._closest_idx = int(candidates[int(np.argmin(dists))])

        target_wp = None
        for i in range(self._closest_idx, self._closest_idx + n):
            idx = i % n
            d   = math.hypot(self._waypoints[idx][0] - px, self._waypoints[idx][1] - py)
            if d >= L:
                target_wp = self._waypoints[idx]
                break
        if target_wp is None:
            target_wp = self._waypoints[self._closest_idx]

        tx, ty, t_spd = target_wp
        dx     = tx - px
        dy     = ty - py
        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        if abs(local_x) < 0.01:
            steering = 0.0
        else:
            curvature = 2.0 * local_y / (L * L)
            steering  = math.atan(curvature * self.wb) * self.steer_gain
            steering  = max(-0.4, min(0.4, steering))

        speed = max(self.min_speed, min(self.max_speed, t_spd * self.speed_scale))

        cmd = AckermannDriveStamped()
        cmd.header.stamp         = self.get_clock().now().to_msg()
        cmd.drive.speed          = speed
        cmd.drive.steering_angle = steering
        self._drive_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
