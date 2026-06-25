#!/usr/bin/env python3
"""
Waypoint logger — records automatically from launch until Ctrl+C.

Recording starts as soon as the car moves (odometry > 0).
CSV saved to ~/f1tenth_waypoints/waypoints_TIMESTAMP.csv on shutdown.
L1 is still the kill switch in corridor_node — hold to stop the car,
release to resume. Recording continues regardless (stationary points
are filtered by min_dist).
"""

import os
import csv
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry


class WaypointLoggerNode(Node):
    def __init__(self):
        super().__init__("waypoint_logger_node")

        self.declare_parameter("min_dist", 0.10)   # m between saved points
        self._min_dist = self.get_parameter("min_dist").value

        self._waypoints = []
        self._last_pos  = None

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(Odometry, "/odom", self._odom_cb, sensor_qos)
        self.get_logger().info("WaypointLogger recording — Ctrl+C to save and exit.")

    def _odom_cb(self, msg: Odometry):
        x     = msg.pose.pose.position.x
        y     = msg.pose.pose.position.y
        speed = abs(msg.twist.twist.linear.x)

        if self._last_pos is not None:
            dx = x - self._last_pos[0]
            dy = y - self._last_pos[1]
            if (dx * dx + dy * dy) ** 0.5 < self._min_dist:
                return

        self._waypoints.append((x, y, round(speed, 3)))
        self._last_pos = (x, y)

    def save(self):
        if not self._waypoints:
            self.get_logger().warn("No waypoints recorded — nothing saved.")
            return
        out_dir = os.path.expanduser("~/f1tenth_waypoints")
        os.makedirs(out_dir, exist_ok=True)
        fname = os.path.join(out_dir, f"waypoints_{int(time.time())}.csv")
        with open(fname, "w", newline="") as f:
            csv.writer(f).writerows(self._waypoints)
        self.get_logger().info(
            f"Saved {len(self._waypoints)} waypoints → {fname}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
