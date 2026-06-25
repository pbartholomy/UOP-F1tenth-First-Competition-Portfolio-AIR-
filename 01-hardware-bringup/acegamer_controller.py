#!/usr/bin/env python3
"""
AceGamer Wireless Controller -> VESC Driver
Roboracer / F1TENTH Project

Controls:
  Left Stick UP    : Forward
  Left Stick DOWN  : Reverse
  D-Pad LEFT       : Steer Left
  D-Pad RIGHT      : Steer Right
  X Button         : Emergency Stop (hold)
  PS Button        : Quit

Requirements:
  pip3 install pygame pyserial

Run:
  python3 acegamer_controller.py
"""

import sys
import struct
import time
import pygame
import serial

# ─── Configuration ────────────────────────────────────────────────────────────

VESC_PORT           = "/dev/vesc"
VESC_BAUDRATE       = 115200

MAX_DUTY            = 0.20        # Maximum duty cycle (20%) — direct power control
MAX_CURRENT_A       = 3.0         # Motor current limit (A)
BRAKE_CURRENT_A     = 3.0         # Braking current (HB = 3A)
BATTERY_CURRENT_A   = 3.0         # Battery current limit (Ib = 3A)
MAX_RPM             = 5000        # Omega — max RPM limit

DEADZONE            = 0.10        # Stick deadzone — ignore small movements near center
LOOP_HZ             = 50          # Control loop rate (50 times per second)

# Left stick axis — forward/reverse only
AXIS_LEFT_Y         = 1           # Left stick vertical: -1.0 = up (forward), +1.0 = down (reverse)

# D-Pad hat — steering
DPAD_HAT            = 0           # Hat index 0 = D-pad. get_hat(0) returns (x, y)
                                  # x: -1 = left, 0 = center, 1 = right

# Servo / steering limits
SERVO_CENTER        = 0.5         # Straight ahead
SERVO_MIN           = 0.15        # Full left turn
SERVO_MAX           = 0.85        # Full right turn

# Button indices
BTN_X               = 0           # Emergency stop
BTN_PS              = 10          # Quit (Home / PS button)

# ─── VESC Protocol ────────────────────────────────────────────────────────────

COMM_SET_DUTY           = 5
COMM_SET_CURRENT        = 6
COMM_SET_CURRENT_BRAKE  = 7
COMM_SET_RPM            = 8
COMM_SET_SERVO_POS      = 12

def _crc16(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc

def _packet(payload: bytes) -> bytes:
    crc = _crc16(payload)
    return bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])

def _send_duty(ser, duty: float):
    # duty: -1.0 to +1.0 — clamped to MAX_DUTY
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    value = int(duty * 100000)
    ser.write(_packet(bytes([COMM_SET_DUTY]) + struct.pack(">i", value)))

def _send_current(ser, amps: float):
    ma = int(max(-MAX_CURRENT_A, min(MAX_CURRENT_A, amps)) * 1000)
    ser.write(_packet(bytes([COMM_SET_CURRENT]) + struct.pack(">i", ma)))

def _send_brake(ser, amps: float):
    ma = int(max(0.0, min(MAX_CURRENT_A, amps)) * 1000)
    ser.write(_packet(bytes([COMM_SET_CURRENT_BRAKE]) + struct.pack(">i", ma)))

def _send_servo(ser, pos: float):
    pos = max(SERVO_MIN, min(SERVO_MAX, pos))
    ser.write(_packet(bytes([COMM_SET_SERVO_POS]) + struct.pack(">H", int(pos * 1000))))

def _stop(ser):
    _send_current(ser, 0.0)
    _send_servo(ser, SERVO_CENTER)

def open_vesc(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
        print(f"[VESC] Connected on {port} at {baud} baud")
        return ser
    except serial.SerialException as e:
        print(f"[VESC] ERROR: {e}")
        sys.exit(1)

# ─── Stick Helpers ────────────────────────────────────────────────────────────

def apply_deadzone(val: float) -> float:
    if abs(val) < DEADZONE:
        return 0.0
    # Rescale so output starts from 0 outside deadzone edge
    sign = 1.0 if val > 0 else -1.0
    return sign * (abs(val) - DEADZONE) / (1.0 - DEADZONE)

# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    print("=" * 50)
    print("  Roboracer AceGamer Controller")
    print("=" * 50)

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("[CTRL] No controller found. Connect controller and retry.")
        sys.exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"[CTRL] Connected: {joy.get_name()}")
    print(f"       Axes: {joy.get_numaxes()}  Buttons: {joy.get_numbuttons()}  Hats: {joy.get_numhats()}")

    ser = open_vesc(VESC_PORT, VESC_BAUDRATE)

    print()
    print("  Controls:")
    print("    Left Stick UP/DOWN   = Forward / Reverse")
    print("    D-Pad LEFT / RIGHT   = Steering")
    print("    X Button             = Emergency Stop")
    print("    PS Button            = Quit")
    print()

    loop_period = 1.0 / LOOP_HZ
    estop = False

    try:
        while True:
            loop_start = time.monotonic()

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_PS:
                        print("\n[CTRL] Quitting.")
                        raise KeyboardInterrupt
                    if event.button == BTN_X:
                        estop = True
                        _stop(ser)
                        print("\n[ESTOP] Emergency stop! Hold X to keep stopped.")

                if event.type == pygame.JOYBUTTONUP:
                    if event.button == BTN_X:
                        estop = False
                        print("\n[ESTOP] Released.")

                if event.type == pygame.JOYDEVICEREMOVED:
                    print("\n[CTRL] Controller disconnected!")
                    _stop(ser)
                    raise KeyboardInterrupt

            if estop:
                time.sleep(loop_period)
                continue

            # Left stick Y — forward / reverse
            raw_y = joy.get_axis(AXIS_LEFT_Y)
            drive = apply_deadzone(raw_y)

            # D-Pad — steering (hat 0: x=-1 left, x=0 center, x=1 right)
            dpad_x, _ = joy.get_hat(DPAD_HAT)
            if dpad_x == -1:
                servo_pos   = SERVO_MIN
                steer_label = "LEFT"
            elif dpad_x == 1:
                servo_pos   = SERVO_MAX
                steer_label = "RIGHT"
            else:
                servo_pos   = SERVO_CENTER
                steer_label = "CENTER"
            _send_servo(ser, servo_pos)

            # Drive — stick up (negative Y) = forward, stick down (positive Y) = reverse
            if drive < 0:
                duty = abs(drive) * MAX_DUTY
                _send_duty(ser, duty)
                label = f"FWD  Duty: {duty:.2f} ({duty*100:.0f}%)"
            elif drive > 0:
                duty = drive * (MAX_DUTY / 2.0)
                _send_duty(ser, -duty)
                label = f"REV  Duty: -{duty:.2f} ({duty*100:.0f}%)"
            else:
                _send_current(ser, 0.0)
                label = "IDLE"

            print(f"\r[{label:<20}]  Steer: {steer_label:<12}  Servo: {servo_pos:.2f}    ", end="")
            sys.stdout.flush()

            elapsed = time.monotonic() - loop_start
            remaining = loop_period - elapsed
            if remaining > 0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("\n\n[INFO] Shutting down...")
    finally:
        _stop(ser)
        ser.close()
        pygame.quit()
        print("[INFO] Stopped. Goodbye.")

if __name__ == "__main__":
    main()
