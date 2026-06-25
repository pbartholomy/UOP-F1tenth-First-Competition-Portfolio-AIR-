#!/usr/bin/env python3
"""
mapping_node.py — v9: SLAM-based path recording.

In MAPPING mode (drive_mode == 2), reads TF map→base_link at 10 Hz and
records (x, y, yaw) waypoints every 0.08 m of travel.

When /mapping_save is received (Bool True), saves waypoints to
~/Desktop/v9_ws/track_path.json and publishes /mapping_path for rviz.

Publishes:
  /mapping_path    (nav_msgs/Path)    — live path for rviz visualization
  /waypoint_count  (std_msgs/Int32)   — current number of recorded waypoints
"""

import json
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Int32, Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import tf2_ros

MODE_MANUAL  = 1   # path is recorded while driving manually

RECORD_DIST  = 0.08   # meters between waypoints
RECORD_HZ    = 10.0   # TF polling rate

SAVE_PATH = os.path.expanduser("~/Desktop/v9_ws/track_path.json")


def _quat_to_yaw(q) -> float:
    """Convert quaternion to yaw angle."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")

        self._drive_mode = 1     # start assumed manual
        self._armed      = False  # True after Square pressed — gates recording
        self._waypoints  = []
        self._last_x     = None
        self._last_y     = None

        # TF listener
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Subscriptions
        self.create_subscription(Int32, "/drive_mode",    self._mode_cb,  10)
        self.create_subscription(Bool,  "/mapping_save",  self._save_cb,  10)
        self.create_subscription(Bool,  "/mapping_reset", self._reset_cb, 10)
        self.create_subscription(Bool,  "/mapping_arm",   self._arm_cb,   10)

        # Publishers
        self._path_pub  = self.create_publisher(Path,  "/mapping_path",   10)
        self._count_pub = self.create_publisher(Int32, "/waypoint_count",  10)

        # Recording timer
        self.create_timer(1.0 / RECORD_HZ, self._tick)

        # Check for existing path on startup
        if os.path.exists(SAVE_PATH):
            try:
                with open(SAVE_PATH, "r") as f:
                    existing = json.load(f)
                self.get_logger().info(
                    f"Existing path found: {len(existing)} waypoints at {SAVE_PATH}")
            except Exception as e:
                self.get_logger().warn(f"Could not read existing path: {e}")
        else:
            self.get_logger().info("No existing track_path.json found — will create on save")

        self.get_logger().info(
            "mapping_node ready. Press Square to arm recording, then drive the track.")

    def _mode_cb(self, msg: Int32):
        prev = self._drive_mode
        self._drive_mode = msg.data
        if prev != msg.data:
            mode_names = {0: "AUTONOMOUS", 1: "MANUAL"}
            self.get_logger().info(
                f"mapping_node: mode → {mode_names.get(msg.data, str(msg.data))}")
            if msg.data == MODE_MANUAL:
                self._last_x = None
                self._last_y = None
                self.get_logger().info(
                    f"MANUAL mode — path recording active "
                    f"({len(self._waypoints)} waypoints so far)")

    def _arm_cb(self, msg: Bool):
        if msg.data and not self._armed:
            self._armed  = True
            self._last_x = None
            self._last_y = None
            self.get_logger().info(
                f"Recording ARMED — {len(self._waypoints)} waypoints so far")

    def _tick(self):
        """Poll TF and record waypoint while armed in MANUAL mode."""
        if self._drive_mode != MODE_MANUAL or not self._armed:
            return

        try:
            tf = self._tf_buffer.lookup_transform(
                "map", "base_link",
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        x   = tf.transform.translation.x
        y   = tf.transform.translation.y
        yaw = _quat_to_yaw(tf.transform.rotation)

        # Check travel distance from last recorded waypoint
        if self._last_x is not None:
            dist = math.hypot(x - self._last_x, y - self._last_y)
            if dist < RECORD_DIST:
                return

        self._waypoints.append([x, y, yaw])
        self._last_x = x
        self._last_y = y

        # Publish updated path and count
        self._publish_path()
        count_msg      = Int32()
        count_msg.data = len(self._waypoints)
        self._count_pub.publish(count_msg)

    def _publish_path(self):
        path              = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"

        for wp in self._waypoints:
            pose               = PoseStamped()
            pose.header        = path.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            # Encode yaw as quaternion (z-axis rotation only)
            half_yaw = wp[2] / 2.0
            pose.pose.orientation.z = math.sin(half_yaw)
            pose.pose.orientation.w = math.cos(half_yaw)
            path.poses.append(pose)

        self._path_pub.publish(path)

    def _reset_cb(self, msg: Bool):
        if not msg.data:
            return
        count = len(self._waypoints)
        self._waypoints = []
        self._last_x    = None
        self._last_y    = None
        self._armed     = False   # require Square press again before re-recording
        # Publish empty path to clear rviz visualization
        self._publish_path()
        count_msg      = Int32()
        count_msg.data = 0
        self._count_pub.publish(count_msg)
        self.get_logger().info(
            f"Mapping reset — cleared {count} waypoints, recording fresh")

    def _save_cb(self, msg: Bool):
        if not msg.data:
            return
        if not self._waypoints:
            self.get_logger().warn("mapping_save received but no waypoints to save!")
            return

        os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
        try:
            with open(SAVE_PATH, "w") as f:
                json.dump(self._waypoints, f, indent=2)
            self.get_logger().info(
                f"Path saved: {len(self._waypoints)} waypoints → {SAVE_PATH}")
        except Exception as e:
            self.get_logger().error(f"Failed to save path: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
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
