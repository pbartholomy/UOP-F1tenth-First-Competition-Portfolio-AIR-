import struct
import time
import serial
import pygame
import threading

# ============================================================
# AutonomousSpeedControl.py — PS4 → VESC DRIVER + HOKUYO URG-04LX LIDAR
# (Roboracer / F1TENTH)
#
# LiDAR-based 4-zone speed control (forward motion only):
#   >= 2000 mm   → FULL speed  (MAX_DUTY)
#   1000–2000 mm → MODERATE speed (MODERATE_DUTY_CAP)
#   500–1000 mm  → SLOW speed  (SLOW_DUTY_CAP)
#   < 500 mm     → EMERGENCY STOP
#
# Controls:
#   Left stick up/down       = forward / reverse
#   Right stick left/right   = steering
#   Hold X                   = emergency stop
#   Circle                   = quit
#
# Run:
#   python3 ~/Desktop/AutonomousSpeedControl.py
# ============================================================

# =========================
# VESC SETTINGS
# =========================

VESC_PORT = "/dev/ttyACM0"
VESC_BAUDRATE = 115200

MAX_RPM = 50000
MAX_CURRENT_A = 30
MAX_DUTY = 0.20

LOOP_HZ = 50
DEADZONE = 0.10
DUTY_RAMP_STEP = 0.010

# =========================
# PS4 CONTROLLER MAPPING
# =========================

AXIS_DRIVE = 1
AXIS_STEERING = 2
BTN_X = 0
BTN_CIRCLE = 1

# =========================
# VESC COMMAND IDS
# =========================

COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_RPM = 8
COMM_SET_SERVO_POS = 12

# =========================
# SERVO LIMITS
# =========================

SERVO_CENTER = 0.50
SERVO_MIN = 0.15
SERVO_MAX = 0.85

INVERT_STEERING = False
INVERT_DRIVE = False

PRINT_HZ = 8
WARMUP_LOOPS = 10

# =========================
# HOKUYO URG-04LX SETTINGS
# =========================

LIDAR_PORT = "/dev/ttyACM1"
LIDAR_BAUDRATE = 19200

LIDAR_STEP_MIN = 44
LIDAR_STEP_MAX = 725
LIDAR_STEP_FRONT = 384
LIDAR_FRONT_WINDOW = 50

# 4-zone speed control distances (mm)
LIDAR_FULL_SPEED_MM    = 2500   # >= 2500 mm: full speed
LIDAR_MODERATE_MM      = 1500   # 1500–2500 mm: moderate speed
LIDAR_SLOW_MM          = 800    # 800–1500 mm: slow speed
LIDAR_ESTOP_MM         = 800    # < 800 mm: emergency stop

# Duty caps for each zone (forward only)
MODERATE_DUTY_CAP = 0.14        # ~70 % of MAX_DUTY
SLOW_DUTY_CAP     = 0.07        # ~35 % of MAX_DUTY

LIDAR_ENABLED = True


# =========================
# HOKUYO URG-04LX CLASS
# =========================

class HokuyoLidar:
    def __init__(self, port=LIDAR_PORT, baud=LIDAR_BAUDRATE):
        self.port = port
        self.baud = baud
        self._ser = None
        self._lock = threading.Lock()
        self._distances = []
        self._running = False
        self._thread = None
        self.connected = False

    def connect(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=1.0)
        time.sleep(0.2)
        self._ser.write(b"SCIP2.0\n")
        time.sleep(0.2)
        self._ser.reset_input_buffer()
        self._ser.write(b"BM\n")
        time.sleep(0.2)
        self._ser.reset_input_buffer()
        self.connected = True
        print(f"[LIDAR] Connected on {self.port}")

    def _readline(self):
        return self._ser.readline().rstrip(b"\n")

    def _get_scan(self):
        cmd = f"GD{LIDAR_STEP_MIN:04d}{LIDAR_STEP_MAX:04d}01\n".encode()
        self._ser.write(cmd)
        self._readline()
        status = self._readline()
        if not status.startswith(b"00"):
            return None
        self._readline()
        raw = b""
        while True:
            line = self._readline()
            if not line:
                break
            raw += line[:-1]
        return self._decode(raw)

    @staticmethod
    def _decode(raw):
        out = []
        for i in range(0, len(raw) - 2, 3):
            v = ((raw[i] - 0x30) << 12) | ((raw[i + 1] - 0x30) << 6) | (raw[i + 2] - 0x30)
            out.append(v)
        return out

    def _run(self):
        while self._running:
            try:
                scan = self._get_scan()
                if scan:
                    with self._lock:
                        self._distances = scan
            except Exception:
                pass

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get_distances(self):
        with self._lock:
            return list(self._distances)

    def front_min(self):
        d = self.get_distances()
        if not d:
            return None
        center = LIDAR_STEP_FRONT - LIDAR_STEP_MIN
        lo = max(0, center - LIDAR_FRONT_WINDOW)
        hi = min(len(d) - 1, center + LIDAR_FRONT_WINDOW)
        zone = [x for x in d[lo:hi + 1] if x > 20]
        return min(zone) if zone else None

    def stop(self):
        self._running = False
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(b"QT\n")
            except Exception:
                pass
            self._ser.close()
        self.connected = False
        print("[LIDAR] Disconnected")


# =========================
# VESC HELPERS
# =========================

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


# =========================
# LIDAR SPEED CONTROL
# =========================

def lidar_speed_zone(front_dist):
    """Return (duty_cap, zone_label) based on distance reading."""
    if front_dist is None:
        return MAX_DUTY, "FULL"
    if front_dist >= LIDAR_FULL_SPEED_MM:
        return MAX_DUTY, "FULL"
    if front_dist >= LIDAR_MODERATE_MM:
        return MODERATE_DUTY_CAP, "MODERATE"
    if front_dist >= LIDAR_SLOW_MM:
        return SLOW_DUTY_CAP, "SLOW"
    return 0.0, "ESTOP"


# =========================
# MAIN
# =========================

def main():
    pygame.init()
    pygame.joystick.init()

    lidar = None
    if LIDAR_ENABLED:
        lidar = HokuyoLidar()
        try:
            lidar.connect()
            lidar.start()
        except Exception as e:
            print(f"[LIDAR] WARNING: could not connect — {e}")
            print("[LIDAR] Continuing without LiDAR. Set LIDAR_ENABLED=False to suppress.")
            lidar = None

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

    send_current_zero(ser)
    send_servo(ser, SERVO_CENTER)

    print("[READY] AUTONOMOUS SPEED CONTROL ENABLED — 4-ZONE LIDAR")
    print()
    print("Controls:")
    print("  Left stick UP        = forward")
    print("  Left stick DOWN      = reverse")
    print("  Right stick LEFT     = steer left")
    print("  Right stick RIGHT    = steer right")
    print("  Hold X               = emergency stop")
    print("  Circle               = quit")
    print()
    print("LiDAR Speed Zones (forward only):")
    print(f"  >= {LIDAR_FULL_SPEED_MM} mm   → FULL     (duty cap {MAX_DUTY:.2f})")
    print(f"  {LIDAR_MODERATE_MM}–{LIDAR_FULL_SPEED_MM} mm → MODERATE (duty cap {MODERATE_DUTY_CAP:.2f})")
    print(f"  {LIDAR_SLOW_MM}–{LIDAR_MODERATE_MM} mm   → SLOW     (duty cap {SLOW_DUTY_CAP:.2f})")
    print(f"  < {LIDAR_ESTOP_MM} mm     → EMERGENCY STOP")
    print()
    print("Limits:")
    print(f"  MAX_DUTY      = {MAX_DUTY}")
    print(f"  MAX_CURRENT_A = {MAX_CURRENT_A}")
    print(f"  MAX_RPM       = {MAX_RPM}")
    print()
    if lidar and lidar.connected:
        print(f"LiDAR: {LIDAR_PORT}  active")
    else:
        print("LiDAR: NOT connected")
    print()
    print("PUT THE CAR ON A STAND FIRST.")
    print("MAKE SURE VESC TOOL IS CLOSED.")
    print()

    loop_period = 1.0 / LOOP_HZ
    print_period = 1.0 / PRINT_HZ
    last_print = 0.0

    estop = False
    lidar_estop = False
    warmup_counter = 0

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

            # --- LiDAR distance check ---
            front_dist = None
            speed_cap = MAX_DUTY
            zone_label = "FULL"

            if lidar and lidar.connected:
                front_dist = lidar.front_min()
                speed_cap, zone_label = lidar_speed_zone(front_dist)

                if zone_label == "ESTOP":
                    if not lidar_estop:
                        lidar_estop = True
                        current_duty = 0.0
                        stop_car(ser)
                        print(f"\n[LIDAR] EMERGENCY STOP — obstacle at {front_dist} mm")
                elif lidar_estop:
                    lidar_estop = False
                    print(f"[LIDAR] Path clear ({front_dist} mm) — estop released")

            if estop:
                current_duty = 0.0
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                time.sleep(loop_period)
                continue

            if lidar_estop:
                # Obstacle ahead — allow reverse and steering only until LiDAR clears
                drive_raw = joystick.get_axis(AXIS_DRIVE)
                steer_raw = joystick.get_axis(AXIS_STEERING)
                drive = drive_input_from_stick(drive_raw)
                servo_pos = steering_to_servo(steer_raw)

                target_duty = drive * MAX_DUTY if drive < 0 else 0.0
                current_duty = ramp_value(current_duty, target_duty, DUTY_RAMP_STEP)
                send_servo(ser, servo_pos)

                if abs(current_duty) > 0.002:
                    send_duty(ser, current_duty)
                else:
                    current_duty = 0.0
                    send_current_zero(ser)

                now = time.time()
                if now - last_print >= print_period:
                    front_str = f"{front_dist}mm" if front_dist is not None else "--"
                    print(
                        f"\r[LIDAR-ESTOP | REVERSE ONLY] "
                        f"LeftY:{drive_raw:+.2f} "
                        f"Duty:{current_duty:+.3f} "
                        f"Front:{front_str} — back up to clear     ",
                        end=""
                    )
                    last_print = now

                elapsed = time.time() - loop_start
                sleep_time = loop_period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue

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

            # Apply LiDAR speed cap to forward motion only
            if target_duty > speed_cap:
                target_duty = speed_cap

            target_duty = safe_reverse_guard(current_duty, target_duty)
            current_duty = ramp_value(current_duty, target_duty, DUTY_RAMP_STEP)

            send_servo(ser, servo_pos)

            if abs(current_duty) > 0.002:
                send_duty(ser, current_duty)
                mode = f"DRIVE[{zone_label}]"
            else:
                current_duty = 0.0
                send_current_zero(ser)
                mode = "IDLE"

            now = time.time()
            if now - last_print >= print_period:
                if front_dist is not None:
                    lidar_str = f"Front:{front_dist}mm Zone:{zone_label}"
                else:
                    lidar_str = "Front:--"

                print(
                    f"\r[{mode}] "
                    f"LeftY:{drive_raw:+.2f} "
                    f"RightX:{steer_raw:+.2f} "
                    f"Drive:{drive:+.2f} "
                    f"Servo:{servo_pos:.2f} "
                    f"Duty:{current_duty:+.3f} "
                    f"Cap:{speed_cap:.2f} "
                    f"{lidar_str}     ",
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
        if lidar:
            lidar.stop()
        pygame.quit()
        print("[INFO] Closed safely")


if __name__ == "__main__":
    main()
