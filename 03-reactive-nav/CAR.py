import struct
import time
import serial
import pygame

# ============================================================
# CAR.py — PS4 → VESC DRIVER (Roboracer / F1TENTH)
# Fixed: no startup jerk, removed copy-paste corruption
#
# Controls:
#   Left stick up/down       = forward / reverse
#   Right stick left/right   = steering
#   Hold X                   = emergency stop
#   Circle                   = quit
#
# Run:
#   python3 ~/Desktop/CAR.py
# ============================================================

# =========================
# VESC SETTINGS
# =========================

VESC_PORT = "/dev/ttyACM1"
# If /dev/ttyACM1 ever fails, try:
# VESC_PORT = "/dev/vesc"
# VESC_PORT = "/dev/ttyACM1"

VESC_BAUDRATE = 115200

MAX_RPM = 50000
MAX_CURRENT_A = 65
MAX_DUTY = 0.6

LOOP_HZ = 50
DEADZONE = 0.10

# Smaller ramp = smoother acceleration
DUTY_RAMP_STEP = 0.003

# =========================
# PS4 CONTROLLER MAPPING ON JETSON
# =========================

AXIS_DRIVE = 1         # left stick up/down
AXIS_STEERING = 2      # right stick left/right

BTN_X = 0              # emergency stop while held
BTN_CIRCLE = 1         # quit

# =========================
# VESC COMMAND IDS
# =========================

COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_RPM = 8
COMM_SET_SERVO_POS = 12   # IMPORTANT: confirmed working

# =========================
# SERVO LIMITS
# =========================

SERVO_CENTER = 0.50
SERVO_MIN = 0.15
SERVO_MAX = 0.85

# Change these if directions are backwards
INVERT_STEERING = False
INVERT_DRIVE = True

# Print only a few times per second so terminal does not lag
PRINT_HZ = 8

# How many loops to wait at startup before sending any drive command
# Gives pygame time to settle axis values so first read is not junk
WARMUP_LOOPS = 10


def crc16(data):
    crc = 0x0000

    for byte in data:
        crc ^= byte << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1

            crc &= 0xFFFF

    return crc


def build_packet(payload):
    crc = crc16(payload)

    return (
        bytes([0x02, len(payload)])
        + payload
        + bytes([crc >> 8, crc & 0xFF, 0x03])
    )


def send_packet(ser, payload):
    ser.write(build_packet(payload))
    ser.flush()


def send_duty(ser, duty):
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    value = int(duty * 100000)

    payload = bytes([COMM_SET_DUTY]) + struct.pack(">i", value)
    send_packet(ser, payload)


def send_current(ser, amps):
    amps = max(-MAX_CURRENT_A, min(MAX_CURRENT_A, amps))
    milliamps = int(amps * 1000)

    payload = bytes([COMM_SET_CURRENT]) + struct.pack(">i", milliamps)
    send_packet(ser, payload)


def send_current_zero(ser):
    payload = bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0)
    send_packet(ser, payload)


def send_servo(ser, position):
    position = max(SERVO_MIN, min(SERVO_MAX, position))
    value = int(position * 1000)

    payload = bytes([COMM_SET_SERVO_POS]) + struct.pack(">h", value)
    send_packet(ser, payload)


def stop_car(ser):
    send_current_zero(ser)
    send_servo(ser, SERVO_CENTER)
    print("\n[VESC] STOPPED motor and centered steering")


def apply_deadzone(value):
    if abs(value) < DEADZONE:
        return 0.0

    return value


def steering_to_servo(x):
    x = apply_deadzone(x)

    if INVERT_STEERING:
        x = -x

    # Center position + proportional adjust (0.35 scales full range to 0.15 - 0.85)
    servo = SERVO_CENTER + (x * 0.35)

    return max(SERVO_MIN, min(SERVO_MAX, servo))


def drive_input_from_stick(y):
    drive = -y

    if INVERT_DRIVE:
        drive = -drive

    drive = apply_deadzone(drive)

    return max(-1.0, min(1.0, drive))


def ramp_value(current, target, step):
    if target > current + step:
        return current + step

    if target < current - step:
        return current - step

    return target


def safe_reverse_guard(current_duty, target_duty):
    if current_duty > 0.01 and target_duty < -0.01:
        return 0.0

    if current_duty < -0.01 and target_duty > 0.01:
        return 0.0

    return target_duty


def main():
    pygame.init()
    pygame.joystick.init()

    print("[INFO] Waiting for PS4 controller...")

    while pygame.joystick.get_count() == 0:
        pygame.event.pump()
        time.sleep(0.5)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print("[PS4] Connected")
    print(f"Name: {joystick.get_name()}")
    print(f"Axes: {joystick.get_numaxes()}")
    print(f"Buttons: {joystick.get_numbuttons()}")
    print(f"Hats: {joystick.get_numhats()}")
    print()

    # Pump events for a moment so pygame axis values settle before we read them
    print("[INFO] Settling controller axes...")
    for _ in range(30):
        pygame.event.pump()
        time.sleep(0.02)

    print(f"[VESC] Opening {VESC_PORT}...")

    ser = serial.Serial(
        VESC_PORT,
        VESC_BAUDRATE,
        timeout=0.05,
        write_timeout=0.05
    )

    # Send zero current immediately on connect so VESC starts in a known safe state
    send_current_zero(ser)
    send_servo(ser, SERVO_CENTER)

    print("[READY] FULL DRIVE ENABLED - DUTY MODE")
    print()
    print("Controls:")
    print("  Left stick UP        = forward")
    print("  Left stick DOWN      = reverse")
    print("  Right stick LEFT     = steer left")
    print("  Right stick RIGHT    = steer right")
    print("  Hold X               = emergency stop")
    print("  Circle               = quit")
    print()
    print("Limits:")
    print(f"  MAX_DUTY      = {MAX_DUTY}")
    print(f"  MAX_CURRENT_A = {MAX_CURRENT_A}")
    print(f"  MAX_RPM       = {MAX_RPM}")
    print()
    print("PUT THE CAR ON A STAND FIRST.")
    print("MAKE SURE VESC TOOL IS CLOSED.")
    print()

    loop_period = 1.0 / LOOP_HZ
    print_period = 1.0 / PRINT_HZ
    last_print = 0.0

    estop = False
    warmup_counter = 0

    # Seed current_duty from the stick's actual position at startup
    # so the ramp never has to jump from 0 to wherever the stick already is
    initial_drive = drive_input_from_stick(joystick.get_axis(AXIS_DRIVE))
    current_duty = initial_drive * MAX_DUTY

    try:
        while True:
            loop_start = time.time()

            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_X:
                        estop = True
                        current_duty = 0.0
                        stop_car(ser)

                    if event.button == BTN_CIRCLE:
                        raise KeyboardInterrupt

                if event.type == pygame.JOYBUTTONUP:
                    if event.button == BTN_X:
                        estop = False
                        print("[ESTOP] Released")

                if event.type == pygame.JOYDEVICEREMOVED:
                    current_duty = 0.0
                    stop_car(ser)
                    raise KeyboardInterrupt

            if estop:
                current_duty = 0.0
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                time.sleep(loop_period)
                continue

            # Warmup: read axes but send nothing for the first N loops
            # Re-seed current_duty each warmup loop so it always matches the stick
            if warmup_counter < WARMUP_LOOPS:
                warmup_counter += 1
                drive_raw = joystick.get_axis(AXIS_DRIVE)
                current_duty = drive_input_from_stick(drive_raw) * MAX_DUTY
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                time.sleep(loop_period)
                continue

            drive_raw = joystick.get_axis(AXIS_DRIVE)
            steer_raw = joystick.get_axis(AXIS_STEERING)

            drive = drive_input_from_stick(drive_raw)
            servo_pos = steering_to_servo(steer_raw)

            target_duty = drive * MAX_DUTY
            target_duty = safe_reverse_guard(current_duty, target_duty)
            current_duty = ramp_value(current_duty, target_duty, DUTY_RAMP_STEP)

            send_servo(ser, servo_pos)

            if abs(current_duty) > 0.002:
                send_duty(ser, current_duty)
                mode = "DRIVE_DUTY"
            else:
                current_duty = 0.0
                send_current_zero(ser)
                mode = "IDLE"

            now = time.time()
            if now - last_print >= print_period:
                print(
                    f"\r[{mode}] "
                    f"LeftY/DriveRaw:{drive_raw:+.2f} "
                    f"RightX/SteerRaw:{steer_raw:+.2f} "
                    f"Drive:{drive:+.2f} "
                    f"Servo:{servo_pos:.2f} "
                    f"TargetDuty:{target_duty:+.3f} "
                    f"Duty:{current_duty:+.3f}     ",
                    end=""
                )
                last_print = now

            elapsed = time.time() - loop_start
            sleep_time = loop_period - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[INFO] Quitting...")

    finally:
        current_duty = 0.0
        stop_car(ser)
        ser.close()
        pygame.quit()
        print("[INFO] Closed safely")


if __name__ == "__main__":
    main()
