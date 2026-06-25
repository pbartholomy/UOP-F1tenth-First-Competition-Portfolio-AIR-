#!/usr/bin/env python3
"""
car_node.py — manual input processor for v7_ws.

Reads /joy and converts stick input to drive commands using the same
logic as Desktop/CAR.py, then publishes /manual_drive.

corridor_node owns the VESC and relays /manual_drive to hardware in
manual mode — this keeps the autonomous path latency-free (no topic hop).

R1 (btn 5) toggle is managed by mode_manager_node.
L1 kill switch is handled by corridor_node (it owns the VESC).
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32
from ackermann_msgs.msg import AckermannDriveStamped

# ── Settings (matching Desktop CAR.py) ────────────────────────

AXIS_DRIVE      = 1
AXIS_STEERING   = 2
DEADZONE        = 0.10
MAX_DUTY        = 0.14   # capped for slow manual mapping speed
DUTY_RAMP_STEP  = 0.003
INVERT_STEERING = False
INVERT_DRIVE    = True    # confirmed: negative duty → forward on this car

SERVO_CENTER = 0.50
SERVO_MIN    = 0.15
SERVO_MAX    = 0.85

MODE_MANUAL  = 1
LOOP_HZ      = 50
WARMUP_LOOPS = 10


def _deadzone(v):
    return 0.0 if abs(v) < DEADZONE else v


def _steering_to_servo(x):
    x = _deadzone(x)
    if INVERT_STEERING:
        x = -x
    return max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + x * 0.35))


def _drive_from_stick(y):
    drive = -y
    if INVERT_DRIVE:
        drive = -drive
    return max(-1.0, min(1.0, _deadzone(drive)))


def _ramp(current, target, step):
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


def _reverse_guard(current, target):
    if current >  0.01 and target < -0.01:
        return 0.0
    if current < -0.01 and target >  0.01:
        return 0.0
    return target


class CarNode(Node):
    def __init__(self):
        super().__init__("car_node")
        self._joy_axes    = [0.0] * 8
        self._joy_buttons = [0]   * 16
        self._drive_mode  = MODE_MANUAL

        self.create_subscription(Joy,   "/joy",        self._joy_cb,  10)
        self.create_subscription(Int32, "/drive_mode", self._mode_cb, 10)
        self._manual_pub = self.create_publisher(
            AckermannDriveStamped, "/manual_drive", 10)

    def _joy_cb(self, msg: Joy):
        self._joy_axes    = list(msg.axes)    + [0.0] * max(0, 8  - len(msg.axes))
        self._joy_buttons = list(msg.buttons) + [0]   * max(0, 16 - len(msg.buttons))

    def _mode_cb(self, msg: Int32):
        prev = self._drive_mode
        self._drive_mode = msg.data
        if prev != msg.data:
            self.get_logger().info(
                "→ MANUAL" if msg.data == MODE_MANUAL else "→ AUTONOMOUS")


def main(args=None):
    rclpy.init(args=args)
    node = CarNode()
    node.get_logger().info("car_node ready — publishing /manual_drive")

    loop_period    = 1.0 / LOOP_HZ
    current_duty   = 0.0
    warmup_counter = 0

    try:
        while rclpy.ok():
            t0 = time.time()
            rclpy.spin_once(node, timeout_sec=0)

            if node._drive_mode == MODE_MANUAL:
                axes      = node._joy_axes
                drive_raw = _drive_from_stick(axes[AXIS_DRIVE])
                steer_raw = axes[AXIS_STEERING]
                servo_pos = _steering_to_servo(steer_raw)

                if warmup_counter < WARMUP_LOOPS:
                    warmup_counter += 1
                    current_duty = drive_raw * MAX_DUTY
                    duty_out     = 0.0
                    servo_out    = SERVO_CENTER
                else:
                    target_duty  = drive_raw * MAX_DUTY
                    target_duty  = _reverse_guard(current_duty, target_duty)
                    current_duty = _ramp(current_duty, target_duty, DUTY_RAMP_STEP)
                    duty_out     = current_duty
                    servo_out    = servo_pos

                msg = AckermannDriveStamped()
                msg.header.stamp         = node.get_clock().now().to_msg()
                msg.drive.speed          = float(duty_out)
                msg.drive.steering_angle = float(servo_out)
                node._manual_pub.publish(msg)

            else:
                current_duty   = 0.0
                warmup_counter = 0

            elapsed = time.time() - t0
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
