#!/usr/bin/env python3
"""
Mode manager — broadcasts /drive_mode at 2 Hz.
Mode is set by the 'mode' launch parameter: 'reactive' or 'pure_pursuit'.
No joystick input — L1 kill switch in corridor_node is the only controller input.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

MODE_REACTIVE     = 0
MODE_PURE_PURSUIT = 3


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__("mode_manager_node")
        self.declare_parameter("mode", "reactive")
        mode_str   = self.get_parameter("mode").value
        self._mode = MODE_PURE_PURSUIT if mode_str == "pure_pursuit" else MODE_REACTIVE

        self._pub = self.create_publisher(Int32, "/drive_mode", 10)
        self.create_timer(0.5, self._publish)

        label = "PURE PURSUIT" if self._mode == MODE_PURE_PURSUIT else "REACTIVE"
        self.get_logger().info(f"Mode manager: {label}")

    def _publish(self):
        msg      = Int32()
        msg.data = self._mode
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
