#!/usr/bin/env python3
"""
Roboracer Controller
Left Stick  UP/DOWN  = Forward / Reverse
Right Stick LEFT/RIGHT = Steering
X Button = Emergency Stop (hold)
PS Button = Quit
"""

import sys, struct, time, pygame, serial

VESC_PORT     = "/dev/vesc"
VESC_BAUDRATE = 115200

MAX_DUTY      = 0.20
MAX_DUTY_REV  = 0.10
DEADZONE      = 0.10
LOOP_HZ       = 50

AXIS_LEFT_Y   = 1     # Left stick vertical  (-1 = up/forward, +1 = down/reverse)
AXIS_RIGHT_X  = 2     # Right stick horizontal (-1 = left, +1 = right)

SERVO_CENTER  = 0.5
SERVO_MIN     = 0.15
SERVO_MAX     = 0.85

BTN_X         = 0
BTN_PS        = 10

COMM_SET_DUTY       = 5
COMM_SET_CURRENT    = 6
COMM_SET_SERVO_POS  = 12

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc

def packet(payload):
    crc = crc16(payload)
    return bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])

def send_duty(ser, duty):
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    ser.write(packet(bytes([COMM_SET_DUTY]) + struct.pack(">i", int(duty * 100000))))

def send_current(ser, amps):
    ser.write(packet(bytes([COMM_SET_CURRENT]) + struct.pack(">i", int(amps * 1000))))

def send_servo(ser, pos):
    pos = max(SERVO_MIN, min(SERVO_MAX, pos))
    ser.write(packet(bytes([COMM_SET_SERVO_POS]) + struct.pack(">H", int(pos * 1000))))

def deadzone(val):
    if abs(val) < DEADZONE:
        return 0.0
    sign = 1.0 if val > 0 else -1.0
    return sign * (abs(val) - DEADZONE) / (1.0 - DEADZONE)

def main():
    print("=" * 45)
    print("  Roboracer Controller")
    print("  Left Stick  = Drive    Right Stick = Steer")
    print("=" * 45)

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controller found.")
        sys.exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print(f"Controller: {joy.get_name()}")

    try:
        ser = serial.Serial(VESC_PORT, VESC_BAUDRATE, timeout=0.05)
        print(f"VESC: connected on {VESC_PORT}")
    except serial.SerialException as e:
        print(f"VESC ERROR: {e}")
        sys.exit(1)

    loop_period = 1.0 / LOOP_HZ
    estop = False

    try:
        while True:
            t0 = time.monotonic()

            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_PS:
                        raise KeyboardInterrupt
                    if event.button == BTN_X:
                        estop = True
                        send_current(ser, 0.0)
                        send_servo(ser, SERVO_CENTER)
                        print("\n[ESTOP] Hold X to keep stopped.")

                if event.type == pygame.JOYBUTTONUP:
                    if event.button == BTN_X:
                        estop = False
                        print("\n[ESTOP] Released.")

                if event.type == pygame.JOYDEVICEREMOVED:
                    print("\nController disconnected.")
                    raise KeyboardInterrupt

            if estop:
                time.sleep(loop_period)
                continue

            # Left stick Y — forward/reverse
            left_y = deadzone(joy.get_axis(AXIS_LEFT_Y))

            # Right stick X — steering
            right_x = deadzone(joy.get_axis(AXIS_RIGHT_X))
            servo_pos = SERVO_CENTER + right_x * (SERVO_MAX - SERVO_CENTER)
            send_servo(ser, servo_pos)

            # Drive
            if left_y < 0:
                duty = abs(left_y) * MAX_DUTY
                send_duty(ser, duty)
                drive_label = f"FWD {duty*100:.0f}%"
            elif left_y > 0:
                duty = left_y * MAX_DUTY_REV
                send_duty(ser, -duty)
                drive_label = f"REV {duty*100:.0f}%"
            else:
                send_current(ser, 0.0)
                drive_label = "IDLE"

            steer_label = f"{right_x:+.2f}"
            print(f"\r[{drive_label:<10}]  Steer: {steer_label}  Servo: {servo_pos:.2f}   ", end="")
            sys.stdout.flush()

            elapsed = time.monotonic() - t0
            rem = loop_period - elapsed
            if rem > 0:
                time.sleep(rem)

    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        send_current(ser, 0.0)
        send_servo(ser, SERVO_CENTER)
        ser.close()
        pygame.quit()
        print("Stopped.")

if __name__ == "__main__":
    main()
