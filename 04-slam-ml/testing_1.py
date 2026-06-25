#!/usr/bin/env python3
"""
testing_1.py — PS4 controller + VESC manual diagnostic test

Purpose:
  1) Prove the PS4 controller is being read.
  2) Prove Python is sending real duty/servo commands to the VESC.
  3) Keep the test safer than the earlier 0.6 duty script.

Controls:
  Left stick up/down    = forward / reverse duty
  Right stick left/right= steering servo
  Hold X               = emergency stop / inhibit motion
  Circle               = quit safely
  Triangle             = run a tiny direct motor pulse test
  L1                   = force centered steering while held

IMPORTANT:
  - PUT THE CAR ON A STAND FIRST.
  - CLOSE VESC Tool before running this.
  - Start with MAX_DUTY = 0.05 or 0.06.
  - If the servo moves but the motor does not, the controller is NOT the problem.
"""

import argparse
import glob
import os
import struct
import sys
import time

import pygame
import serial

# ---------------- USER SETTINGS ----------------
DEFAULT_VESC_PORT = "/dev/ttyACM0"
VESC_BAUDRATE = 115200
LOOP_HZ = 50
PRINT_HZ = 8

# Safer than the old 0.6 max duty. Increase only after bench testing.
MAX_DUTY = 0.06
DUTY_RAMP_STEP = 0.003

SERVO_CENTER = 0.50
SERVO_MIN = 0.15
SERVO_MAX = 0.85
STEER_SCALE = 0.35          # center +/- 0.35 => 0.15 to 0.85 range
STEER_RAMP_STEP = 0.03
INVERT_STEERING = False

# Common pygame DualShock 4 axis mapping on Linux.
# Your log proves LeftY and RightX are changing, so these are the main ones to verify.
AXIS_LEFT_Y = 1
AXIS_RIGHT_X = 2

# Common pygame DualShock 4 button mapping.
BTN_X = 0          # hold = emergency stop / inhibit
BTN_CIRCLE = 1     # quit
BTN_TRIANGLE = 2   # tiny pulse test
BTN_L1 = 4         # center steering while held

DEADZONE_DRIVE = 0.12
DEADZONE_STEER = 0.08

COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_SERVO_POS = 12


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def apply_deadzone(x, dz):
    if abs(x) < dz:
        return 0.0
    # rescale after deadzone so control still reaches +/-1
    sign = 1.0 if x > 0 else -1.0
    return sign * ((abs(x) - dz) / (1.0 - dz))


def ramp_value(current, target, step):
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


# ---------------- VESC PACKETS ----------------
def crc16(data):
    crc = 0x0000
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def build_packet(payload):
    crc = crc16(payload)
    return bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])


def send_packet(ser, payload):
    ser.write(build_packet(payload))
    ser.flush()


def send_duty(ser, duty):
    duty = clamp(duty, -MAX_DUTY, MAX_DUTY)
    send_packet(ser, bytes([COMM_SET_DUTY]) + struct.pack(">i", int(duty * 100000)))


def send_current_zero(ser):
    send_packet(ser, bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0))


def send_servo(ser, pos):
    pos = clamp(pos, SERVO_MIN, SERVO_MAX)
    send_packet(ser, bytes([COMM_SET_SERVO_POS]) + struct.pack(">h", int(pos * 1000)))


def stop_car(ser):
    try:
        send_current_zero(ser)
        send_servo(ser, SERVO_CENTER)
    except Exception as e:
        print(f"\n[WARN] stop_car failed: {e}")


def direct_pulse_test(ser):
    """Small direct VESC pulse. If this fails, the issue is VESC/port/config, not PS4."""
    print("\n[TEST] Tiny direct motor pulse: +0.030 duty for 0.6 sec, then stop")
    print("       Car must be on a stand.")
    t_end = time.time() + 0.6
    while time.time() < t_end:
        send_duty(ser, 0.03)
        send_servo(ser, SERVO_CENTER)
        time.sleep(0.02)
    stop_car(ser)
    print("[TEST] Pulse complete. If wheels did not move, check VESC port/config/faults.\n")


# ---------------- SETUP ----------------
def list_serial_ports():
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return ports


def wait_for_controller():
    pygame.init()
    pygame.joystick.init()

    print("[INFO] Waiting for PS4 controller...")
    joystick = None

    while joystick is None:
        pygame.event.pump()
        count = pygame.joystick.get_count()
        if count > 0:
            joystick = pygame.joystick.Joystick(0)
            joystick.init()
            break
        time.sleep(0.25)

    print("[PS4] Connected")
    print(f"Name: {joystick.get_name()}")
    print(f"Axes: {joystick.get_numaxes()}")
    print(f"Buttons: {joystick.get_numbuttons()}")
    print(f"Hats: {joystick.get_numhats()}")
    print(f"[MAP] Drive axis LEFT_Y={AXIS_LEFT_Y}, steer axis RIGHT_X={AXIS_RIGHT_X}")
    print(f"[MAP] X={BTN_X} emergency hold, Circle={BTN_CIRCLE} quit, Triangle={BTN_TRIANGLE} pulse test, L1={BTN_L1} center steering")

    print("\n[INFO] Settling controller axes...")
    time.sleep(1.0)
    pygame.event.pump()
    return joystick


def get_button(joy, idx):
    return idx < joy.get_numbuttons() and joy.get_button(idx)


def get_axis(joy, idx):
    if idx >= joy.get_numaxes():
        return 0.0
    return float(joy.get_axis(idx))


# ---------------- MAIN ----------------
def main():
    global MAX_DUTY
    parser = argparse.ArgumentParser(description="PS4 controller + VESC diagnostic manual drive test")
    parser.add_argument("--port", default=DEFAULT_VESC_PORT, help="VESC serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--max-duty", type=float, default=MAX_DUTY, help="temporary max duty cap")
    args = parser.parse_args()

    MAX_DUTY = abs(float(args.max_duty))

    print("=" * 70)
    print("testing_1.py — PS4 + VESC diagnostic test")
    print("=" * 70)
    print("PUT THE CAR ON A STAND FIRST. CLOSE VESC TOOL.")
    print(f"Serial ports currently visible: {list_serial_ports() or 'NONE'}")
    print(f"Using VESC_PORT={args.port}")
    print(f"MAX_DUTY={MAX_DUTY:.3f}")
    print("=" * 70)

    joy = wait_for_controller()

    print(f"\n[VESC] Opening {args.port}...")
    try:
        ser = serial.Serial(args.port, VESC_BAUDRATE, timeout=0.05, write_timeout=0.1)
    except Exception as e:
        print(f"[ERROR] Could not open {args.port}: {e}")
        print("Try: ls -l /dev/ttyACM* and confirm which port disappears when you unplug the VESC.")
        pygame.quit()
        return 1

    time.sleep(1.0)
    stop_car(ser)

    print("\n[READY] Manual duty control active")
    print("Controls:")
    print("  Left stick UP/DOWN     = forward/reverse")
    print("  Right stick LEFT/RIGHT = steering")
    print("  Hold X                 = emergency stop / inhibit")
    print("  Triangle               = tiny direct pulse test")
    print("  Hold L1                = center steering")
    print("  Circle                 = quit")
    print("\nWatch TargetDuty. If TargetDuty changes but wheels do not move, check VESC port/config/faults.\n")

    loop_period = 1.0 / LOOP_HZ
    print_period = 1.0 / PRINT_HZ
    last_print = 0.0
    last_triangle = False

    current_duty = 0.0
    current_servo = SERVO_CENTER
    target_duty = 0.0
    mode = "IDLE"

    try:
        while True:
            loop_start = time.time()

            # Pump events so pygame keeps controller state updated.
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                if event.type == pygame.JOYDEVICEREMOVED:
                    print("\n[PS4] Controller disconnected. Stopping car.")
                    stop_car(ser)
                    raise KeyboardInterrupt

            pygame.event.pump()

            # Buttons
            x_held = bool(get_button(joy, BTN_X))
            circle = bool(get_button(joy, BTN_CIRCLE))
            triangle = bool(get_button(joy, BTN_TRIANGLE))
            l1_held = bool(get_button(joy, BTN_L1))

            if circle:
                raise KeyboardInterrupt

            if triangle and not last_triangle:
                # Debounced tiny direct VESC test.
                current_duty = 0.0
                target_duty = 0.0
                stop_car(ser)
                direct_pulse_test(ser)
            last_triangle = triangle

            # Axes
            left_y_raw = get_axis(joy, AXIS_LEFT_Y)
            right_x_raw = get_axis(joy, AXIS_RIGHT_X)

            # On DS4, pushing left stick UP usually gives negative Y.
            drive_cmd = -apply_deadzone(left_y_raw, DEADZONE_DRIVE)
            steer_cmd = apply_deadzone(right_x_raw, DEADZONE_STEER)

            if x_held:
                target_duty = 0.0
                current_duty = 0.0
                current_servo = SERVO_CENTER
                stop_car(ser)
                mode = "ESTOP_X_HELD"
            else:
                target_duty = clamp(drive_cmd * MAX_DUTY, -MAX_DUTY, MAX_DUTY)
                current_duty = ramp_value(current_duty, target_duty, DUTY_RAMP_STEP)

                if l1_held:
                    target_servo = SERVO_CENTER
                else:
                    steer_for_servo = -steer_cmd if INVERT_STEERING else steer_cmd
                    target_servo = clamp(SERVO_CENTER + steer_for_servo * STEER_SCALE, SERVO_MIN, SERVO_MAX)
                current_servo = ramp_value(current_servo, target_servo, STEER_RAMP_STEP)

                send_servo(ser, current_servo)
                if abs(current_duty) > 0.002:
                    send_duty(ser, current_duty)
                    mode = "DRIVE_DUTY"
                else:
                    send_current_zero(ser)
                    mode = "IDLE"

            now = time.time()
            if now - last_print >= print_period:
                print(
                    f"\r[{mode}] "
                    f"LeftY/DriveRaw:{left_y_raw:+.2f} "
                    f"RightX/SteerRaw:{right_x_raw:+.2f} "
                    f"DriveCmd:{drive_cmd:+.2f} "
                    f"TargetDuty:{target_duty:+.3f} "
                    f"DutySent:{current_duty:+.3f} "
                    f"Servo:{current_servo:.2f} "
                    f"X:{int(x_held)} L1:{int(l1_held)} Tri:{int(triangle)}     ",
                    end="",
                    flush=True,
                )
                last_print = now

            elapsed = time.time() - loop_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)

    except KeyboardInterrupt:
        print("\n[INFO] Quitting...")

    finally:
        stop_car(ser)
        ser.close()
        pygame.quit()
        print("[VESC] STOPPED motor and centered steering")
        print("[INFO] Closed safely")

    return 0


if __name__ == "__main__":
    sys.exit(main())
