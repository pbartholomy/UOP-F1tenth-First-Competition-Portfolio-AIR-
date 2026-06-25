#!/usr/bin/env python3
"""
joy_node.py — PS4 controller -> /joy (sensor_msgs/Joy)

Self-contained controller reader for the corridor package, ported from
~/Desktop/CAR.py's connect/settle conventions so manual driving behaves the
same as that reference script. Publishes raw axes/buttons — deadzone and
scaling are applied downstream in corridor_node's manual branch.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

import pygame

LOOP_HZ      = 50
SETTLE_LOOPS = 30   # pump-only loops after connect, before first publish (CAR.py: ~600ms)


class JoyNode(Node):
    def __init__(self):
        super().__init__("joy_node")
        self._pub      = self.create_publisher(Joy, "/joy", 10)
        self._joystick = None
        self._settle   = 0

        pygame.init()
        pygame.joystick.init()
        self.get_logger().info("joy_node: waiting for controller...")
        self.create_timer(1.0 / LOOP_HZ, self._loop)

    def _connect(self):
        pygame.event.pump()
        if pygame.joystick.get_count() == 0:
            return
        self._joystick = pygame.joystick.Joystick(0)
        self._joystick.init()
        self._settle = SETTLE_LOOPS
        self.get_logger().info(
            f"joy_node: connected '{self._joystick.get_name()}' "
            f"({self._joystick.get_numaxes()} axes, {self._joystick.get_numbuttons()} buttons) "
            f"-- settling..."
        )

    def _loop(self):
        if self._joystick is None:
            self._connect()
            return

        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEREMOVED:
                self.get_logger().warn("joy_node: controller disconnected")
                self._joystick = None
                return
        pygame.event.pump()

        if self._settle > 0:
            self._settle -= 1
            return

        js  = self._joystick
        msg = Joy()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "joystick"
        msg.axes    = [js.get_axis(i)   for i in range(js.get_numaxes())]
        msg.buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
        self._pub.publish(msg)

    def destroy_node(self):
        pygame.quit()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = JoyNode()
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
