#!/usr/bin/env python3
"""
mode_manager_node — publishes /drive_mode at 2 Hz.

R1 (btn 5) toggles: MANUAL(1) ↔ AUTONOMOUS(0)
  MANUAL → AUTONOMOUS: saves the path recorded during manual driving
  AUTONOMOUS → MANUAL: returns to manual (path recording continues/accumulates)

Triangle (btn 3): resets recorded waypoints while in MANUAL so you can re-drive
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32, Bool

MODE_AUTONOMOUS = 0
MODE_MANUAL     = 1

BTN_SQUARE   = 2   # arms path recording
BTN_TRIANGLE = 3   # resets path recording

LABELS = {MODE_AUTONOMOUS: "AUTONOMOUS", MODE_MANUAL: "MANUAL"}


class ModeManagerNode(Node):
    def __init__(self):
        super().__init__("mode_manager_node")
        self.declare_parameter("initial_mode", "manual")
        init = self.get_parameter("initial_mode").value
        self._mode = MODE_AUTONOMOUS if init == "autonomous" else MODE_MANUAL

        self._r1_was       = False
        self._square_was   = False
        self._triangle_was = False

        self._pub        = self.create_publisher(Int32, "/drive_mode",      10)
        self._save_pub   = self.create_publisher(Bool,  "/mapping_save",    10)
        self._reset_pub  = self.create_publisher(Bool,  "/mapping_reset",   10)
        self._arm_pub    = self.create_publisher(Bool,  "/mapping_arm",     10)
        self.create_subscription(Joy, "/joy", self._joy_cb, 10)
        self.create_timer(0.5, self._publish)

        self.get_logger().info(
            f"Mode manager: starting in {LABELS[self._mode]}. "
            f"Square = start recording | R1 = save + go autonomous | Triangle = clear path")

    def _joy_cb(self, msg: Joy):
        buttons = list(msg.buttons) + [0] * 16

        # ── R1: toggle MANUAL ↔ AUTONOMOUS ──────────────────────
        r1 = bool(buttons[5])
        if r1 and not self._r1_was:
            prev       = self._mode
            self._mode = MODE_AUTONOMOUS if prev == MODE_MANUAL else MODE_MANUAL
            self.get_logger().info(f"Mode: {LABELS[prev]} → {LABELS[self._mode]}")

            if self._mode == MODE_AUTONOMOUS:
                save_msg      = Bool()
                save_msg.data = True
                self._save_pub.publish(save_msg)
                self.get_logger().info("Saving path → switching to AUTONOMOUS")

        self._r1_was = r1

        # ── Square: arm recording (MANUAL only) ─────────────────
        square = bool(buttons[BTN_SQUARE])
        if square and not self._square_was:
            if self._mode == MODE_MANUAL:
                arm_msg      = Bool()
                arm_msg.data = True
                self._arm_pub.publish(arm_msg)
                self.get_logger().info("Square — mapping ARMED, recording started")
            else:
                self.get_logger().info("Square ignored (not in MANUAL mode)")

        self._square_was = square

        # ── Triangle: reset path recording (MANUAL only) ────────
        triangle = bool(buttons[BTN_TRIANGLE])
        if triangle and not self._triangle_was:
            if self._mode == MODE_MANUAL:
                reset_msg      = Bool()
                reset_msg.data = True
                self._reset_pub.publish(reset_msg)
                self.get_logger().info("Triangle — path cleared, press Square to re-arm")
            else:
                self.get_logger().info("Triangle ignored (not in MANUAL mode)")

        self._triangle_was = triangle

    def _publish(self):
        msg      = Int32()
        msg.data = self._mode
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManagerNode()
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
