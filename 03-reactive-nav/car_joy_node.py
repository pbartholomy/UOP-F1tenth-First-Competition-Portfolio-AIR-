#!/usr/bin/env python3
"""
car_joy_node.py — ROS 2 joystick node for Roboracer / F1TENTH

Reads PS4 controller via pygame and publishes:
  /joy    (sensor_msgs/msg/Joy)             — raw axis / button values
  /drive  (ackermann_msgs/msg/AckermannDriveStamped) — computed drive cmd

Also writes directly to VESC over serial when USE_VESC_DIRECT = True.

Controls:
  Left  stick up/down      forward / reverse
  Right stick left/right   steering
  Hold X                   emergency stop (publishes zero, holds VESC)
  Circle                   quit node cleanly

Run:
  source /opt/ros/humble/setup.bash
  python3 ~/Desktop/car_joy_node.py
"""

import math
import struct
import sys
import time

import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

try:
    from ackermann_msgs.msg import AckermannDriveStamped
    HAS_ACKERMANN = True
except ImportError:
    HAS_ACKERMANN = False
    print("[WARN] ackermann_msgs not found — /drive will not be published")
    print("       Install: sudo apt install ros-humble-ackermann-msgs")

try:
    import serial as pyserial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# ============================================================
# TUNABLE PARAMETERS
# ============================================================

USE_VESC_DIRECT = True        # False = publish-only, no serial
VESC_PORT       = "/dev/ttyACM0"
VESC_BAUDRATE   = 115200

MAX_DUTY        = 0.6
MAX_CURRENT_A   = 65
DUTY_RAMP_STEP  = 0.003
DEADZONE        = 0.10
LOOP_HZ         = 50
WARMUP_LOOPS    = 10

# PS4 axis / button indices (Jetson / ds4drv mapping)
AXIS_DRIVE    = 1   # left stick Y  (up = negative on most SDL backends)
AXIS_STEERING = 2   # right stick X

BTN_ESTOP  = 0      # X      — hold for emergency stop
BTN_QUIT   = 1      # Circle — clean shutdown

SERVO_CENTER = 0.50
SERVO_MIN    = 0.15
SERVO_MAX    = 0.85
SERVO_RANGE  = 0.35   # half-range from center

INVERT_STEERING = False
INVERT_DRIVE    = False

# AckermannDriveStamped unit conversions
MAX_SPEED_MPS    = 3.0        # duty 1.0 → this speed (tune to match car)
MAX_STEER_RAD    = 0.40       # servo full deflection → this angle (~23 deg)

# VESC COMM IDs
COMM_SET_DUTY      = 5
COMM_SET_CURRENT   = 6
COMM_SET_SERVO_POS = 12

# ============================================================
# VESC SERIAL HELPERS
# ============================================================

def _crc16(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def _build_packet(payload: bytes) -> bytes:
    crc = _crc16(payload)
    return bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])


def _send(ser, payload: bytes):
    ser.write(_build_packet(payload))
    ser.flush()


def vesc_duty(ser, duty: float):
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    _send(ser, bytes([COMM_SET_DUTY]) + struct.pack(">i", int(duty * 100000)))


def vesc_current_zero(ser):
    _send(ser, bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0))


def vesc_servo(ser, pos: float):
    pos = max(SERVO_MIN, min(SERVO_MAX, pos))
    _send(ser, bytes([COMM_SET_SERVO_POS]) + struct.pack(">h", int(pos * 1000)))


def vesc_stop(ser):
    vesc_current_zero(ser)
    vesc_servo(ser, SERVO_CENTER)

# ============================================================
# SIGNAL PROCESSING
# ============================================================

def apply_deadzone(v: float) -> float:
    return 0.0 if abs(v) < DEADZONE else v


def stick_to_drive(y: float) -> float:
    drive = -y
    if INVERT_DRIVE:
        drive = -drive
    return max(-1.0, min(1.0, apply_deadzone(drive)))


def stick_to_servo(x: float) -> float:
    if INVERT_STEERING:
        x = -x
    return max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + apply_deadzone(x) * SERVO_RANGE))


def ramp(current: float, target: float, step: float) -> float:
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


def safe_reverse_guard(current: float, target: float) -> float:
    if current > 0.01 and target < -0.01:
        return 0.0
    if current < -0.01 and target > 0.01:
        return 0.0
    return target


def servo_to_steer_rad(servo: float) -> float:
    deflection = servo - SERVO_CENTER          # −0.35 … +0.35
    return -(deflection / SERVO_RANGE) * MAX_STEER_RAD

# ============================================================
# ROS 2 NODE
# ============================================================

class CarJoyNode(Node):
    def __init__(self):
        super().__init__("car_joy_node")

        self._joy_pub = self.create_publisher(Joy, "/joy", 10)
        self._drive_pub = (
            self.create_publisher(AckermannDriveStamped, "/drive", 10)
            if HAS_ACKERMANN else None
        )

        self._ser = None
        if USE_VESC_DIRECT and HAS_SERIAL:
            try:
                self._ser = pyserial.Serial(
                    VESC_PORT, VESC_BAUDRATE, timeout=0.05, write_timeout=0.05
                )
                vesc_stop(self._ser)
                self.get_logger().info(f"VESC opened on {VESC_PORT}")
            except Exception as e:
                self.get_logger().warn(f"VESC serial failed: {e} — running publish-only")
                self._ser = None

        pygame.init()
        pygame.joystick.init()
        self._joystick = None
        self._estop = False
        self._current_duty = 0.0
        self._warmup = 0
        self._last_print = 0.0

        self.get_logger().info("Waiting for PS4 controller...")
        self._timer = self.create_timer(1.0 / LOOP_HZ, self._loop)

    # ── MAIN LOOP ──────────────────────────────────────────────

    def _loop(self):
        # Controller connect / reconnect
        if self._joystick is None:
            pygame.event.pump()
            if pygame.joystick.get_count() > 0:
                self._joystick = pygame.joystick.Joystick(0)
                self._joystick.init()
                self._warmup = 0
                name = self._joystick.get_name()
                self.get_logger().info(f"Controller connected: {name}")
            else:
                return

        # Drain pygame event queue
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == BTN_ESTOP:
                    self._estop = True
                    self._current_duty = 0.0
                    self._send_stop()
                    self.get_logger().info("[ESTOP] Active")
                if event.button == BTN_QUIT:
                    self.get_logger().info("[QUIT] Circle pressed — shutting down")
                    rclpy.shutdown()
                    return

            if event.type == pygame.JOYBUTTONUP:
                if event.button == BTN_ESTOP:
                    self._estop = False
                    self.get_logger().info("[ESTOP] Released")

            if event.type == pygame.JOYDEVICEREMOVED:
                self.get_logger().warn("Controller disconnected")
                self._send_stop()
                self._joystick = None
                return

        if self._estop:
            self._current_duty = 0.0
            self._send_stop()
            self._publish(0.0, SERVO_CENTER)
            return

        # Warmup: settle axes before sending any drive command
        if self._warmup < WARMUP_LOOPS:
            self._warmup += 1
            drive_raw = self._joystick.get_axis(AXIS_DRIVE)
            self._current_duty = stick_to_drive(drive_raw) * MAX_DUTY
            if self._ser:
                vesc_stop(self._ser)
            return

        drive_raw = self._joystick.get_axis(AXIS_DRIVE)
        steer_raw = self._joystick.get_axis(AXIS_STEERING)

        drive     = stick_to_drive(drive_raw)
        servo_pos = stick_to_servo(steer_raw)

        target_duty         = drive * MAX_DUTY
        target_duty         = safe_reverse_guard(self._current_duty, target_duty)
        self._current_duty  = ramp(self._current_duty, target_duty, DUTY_RAMP_STEP)

        if self._ser:
            vesc_servo(self._ser, servo_pos)
            if abs(self._current_duty) > 0.002:
                vesc_duty(self._ser, self._current_duty)
            else:
                self._current_duty = 0.0
                vesc_current_zero(self._ser)

        self._publish(self._current_duty, servo_pos)
        self._print_status(drive_raw, steer_raw, servo_pos)

    # ── PUBLISHERS ─────────────────────────────────────────────

    def _publish(self, duty: float, servo_pos: float):
        stamp = self.get_clock().now().to_msg()
        js = self._joystick

        # --- /joy ---
        joy_msg = Joy()
        joy_msg.header.stamp = stamp
        joy_msg.header.frame_id = "joystick"
        if js:
            joy_msg.axes    = [js.get_axis(i) for i in range(js.get_numaxes())]
            joy_msg.buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
        self._joy_pub.publish(joy_msg)

        # --- /drive ---
        if self._drive_pub is None:
            return
        msg = AckermannDriveStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = "base_link"
        msg.drive.speed           = duty * MAX_SPEED_MPS
        msg.drive.steering_angle  = servo_to_steer_rad(servo_pos)
        self._drive_pub.publish(msg)

    def _send_stop(self):
        if self._ser:
            vesc_stop(self._ser)
        self._publish(0.0, SERVO_CENTER)

    # ── STATUS LINE ────────────────────────────────────────────

    def _print_status(self, drive_raw: float, steer_raw: float, servo: float):
        now = time.time()
        if now - self._last_print < 0.125:   # ~8 Hz
            return
        self._last_print = now
        mode = "DRIVE" if abs(self._current_duty) > 0.002 else "IDLE "
        print(
            f"\r[{mode}] "
            f"LeftY:{drive_raw:+.2f}  RightX:{steer_raw:+.2f}  "
            f"Servo:{servo:.2f}  Duty:{self._current_duty:+.3f}     ",
            end="", flush=True,
        )

    # ── CLEANUP ────────────────────────────────────────────────

    def destroy_node(self):
        self._send_stop()
        if self._ser:
            self._ser.close()
        pygame.quit()
        print("\n[INFO] Shutdown complete")
        super().destroy_node()


# ============================================================
# ENTRY POINT
# ============================================================

def main():
    rclpy.init(args=sys.argv)
    node = CarJoyNode()
    print("[INFO] car_joy_node running")
    print("  /joy   → sensor_msgs/Joy")
    if HAS_ACKERMANN:
        print("  /drive → AckermannDriveStamped")
    print("  Hold X = estop  |  Circle = quit\n")
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
