#!/usr/bin/env python3
"""
mode_manager_node — publishes /drive_mode at 2 Hz.

R1 (btn 5) toggles MANUAL ↔ AUTONOMOUS at runtime.
L1 remains the kill switch in corridor_node and is not affected here.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

MODE_AUTONOMOUS   = 0
MODE_MANUAL       = 1
MODE_PURE_PURSUIT = 3

LABELS = {MODE_AUTONOMOUS: "AUTONOMOUS", MODE_MANUAL: "MANUAL",
          MODE_PURE_PURSUIT: "PURE PURSUIT"}


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__("mode_manager_node")
        self.declare_parameter("initial_mode", "manual")
        init         = self.get_parameter("initial_mode").value
        self._mode   = MODE_AUTONOMOUS if init == "autonomous" else MODE_MANUAL
        self._r1_was = False

        self._pub = self.create_publisher(Int32, "/drive_mode", 10)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self.create_timer(0.5, self._publish)

        self.get_logger().info(
            f"Mode manager: {LABELS[self._mode]}. R1 (btn 5) = toggle MANUAL/AUTONOMOUS.")

    def _joy_cb(self, msg: Joy):
        r1 = len(msg.buttons) > 5 and bool(msg.buttons[5])
        if r1 and not self._r1_was:
            self._mode = MODE_MANUAL if self._mode != MODE_MANUAL else MODE_AUTONOMOUS
            self.get_logger().info(f"→ {LABELS[self._mode]}")
        self._r1_was = r1

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
