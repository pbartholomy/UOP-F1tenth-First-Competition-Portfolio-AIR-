import struct
import time
import serial
import pygame
import threading
import math
import os

import cv2
import numpy as np

try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
except Exception:
    sl = None
    ZED_AVAILABLE = False

# ============================================================
# zed_lidar_track_nav.py — LiDAR + ZED orange-track navigation
# VESC 6 MkVI + Hokuyo URG-04LX + ZED 2i
#
# Base: Reactive_ftg.py
#
# Purpose:
#   Make the car loop smoothly around an orange-tube track by using:
#     1. LiDAR for hard obstacle safety and corridor clearance
#     2. ZED camera for orange tube / track-center guidance
#
# Important:
#   - Use /dev/serial/by-id paths so USB port swaps do not break the car.
#   - Start on a stand.
#   - Start slow.
#   - Tune SPEED and STEERING parameters only after the car is stable.
# ============================================================

# ── STABLE PORTS ─────────────────────────────────────────────
# These are better than /dev/ttyACM0 and /dev/ttyACM1 because Linux can swap ttyACM numbers.
VESC_PORT = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00"
LIDAR_PORT = "/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00"

VESC_BAUDRATE = 115200
LIDAR_BAUDRATE = 19200

# ── VESC / SPEED ─────────────────────────────────────────────
# Slower than your previous 0.15/0.18 because your track has tight walls.
MAX_DUTY = 0.10
AUTO_DRIVE_DUTY = 0.075
MODERATE_DUTY_CAP = 0.060
SLOW_DUTY_CAP = 0.045
CRAWL_DUTY_CAP = 0.035

# Use CURRENT control instead of duty for better torque at low speeds.
# Current control (mA) provides more consistent torque than duty cycle.
USE_CURRENT_CONTROL = True
CURRENT_DRIVE_MA = 3000      # 3A drive current - increase if still stalling
CURRENT_CRAWL_MA = 1500     # 1.5A crawl current
CURRENT_BRAKE_MA = 0        # braking is done with duty cycle

LOOP_HZ = 50
DUTY_RAMP_STEP = 0.003

# Curve speed reduction.
CURVE_SLOW_STEER = 0.45       # start slowing once abs(steer) passes this
CURVE_MIN_SCALE = 0.55        # at full steering, duty is scaled down to this fraction

# ── SERVO ────────────────────────────────────────────────────
SERVO_CENTER = 0.50
SERVO_MIN = 0.15
SERVO_MAX = 0.85
INVERT_STEERING = False
STEER_RAMP_STEP = 0.025       # smoother than before

# ── PS4 CONTROLLER ───────────────────────────────────────────
BTN_X = 0
BTN_CIRCLE = 1
BTN_TRIANGLE = 2              # common DualShock mapping
BTN_TRIANGLE_ALT = 3          # fallback for some Linux/pygame mappings
BTN_L1 = 4                    # hold = kill/stop fallback
BTN_L1_ALT = 9

# ── VESC COMMAND IDS ─────────────────────────────────────────
COMM_SET_DUTY = 5
COMM_SET_CURRENT = 6
COMM_SET_RPM = 8
COMM_SET_SERVO_POS = 12

# ── HOKUYO URG-04LX ──────────────────────────────────────────
LIDAR_STEP_MIN = 44
LIDAR_STEP_MAX = 725
LIDAR_STEP_FRONT = 384

# Wider front safety window because orange tubes are close and the car turns.
LIDAR_FRONT_WINDOW = 65

# Speed zones.
LIDAR_FULL_SPEED_MM = 1200
LIDAR_MODERATE_MM = 850
LIDAR_SLOW_MM = 550
LIDAR_CRAWL_MM = 350
LIDAR_ESTOP_MM = 230
LIDAR_ESTOP_CLEAR_MM = 500

BRAKE_LOOPS = 18
REVERSE_LOOPS = 25
SCAN_LOOPS = 15
HOLD_TIMEOUT_LOOPS = 80

# ── FOLLOW-THE-GAP / SAFETY ──────────────────────────────────
# Make the car "feel wider" so it does not aim at gaps that are technically visible
# but too tight for the vehicle to pass smoothly.
CAR_WIDTH_MM = 240
FTG_DISPARITY_THRESH_MM = 380
FTG_EXTRA_SAMPLES = 8
FTG_SAFETY_FACTOR = 2.0

# Side panic thresholds. If one tube is too close, override steering away.
SIDE_PANIC_MM = 260
SIDE_WARN_MM = 380
FRONT_CORNER_WARN_MM = 450

# Corridor target. In a tube track, being too close to either side causes scraping/crashing.
WALL_TARGET_MM = 650

# ── ZED CAMERA ───────────────────────────────────────────────
ZED_ENABLED = True
ZED_RESOLUTION = sl.RESOLUTION.HD720 if ZED_AVAILABLE else None
ZED_FPS = 30
ZED_DEPTH_MODE = sl.DEPTH_MODE.PERFORMANCE if ZED_AVAILABLE else None

# Orange HSV thresholds. These are broad because the tube is bright and lighting varies.
ORANGE_LO_1 = np.array([0, 100, 70], dtype=np.uint8)
ORANGE_HI_1 = np.array([24, 255, 255], dtype=np.uint8)
ORANGE_LO_2 = np.array([165, 100, 70], dtype=np.uint8)
ORANGE_HI_2 = np.array([180, 255, 255], dtype=np.uint8)

# Use the lower/middle part of the image because the tube boundaries near the car matter most.
CAM_ROI_TOP = 0.38
CAM_ROI_BOTTOM = 0.92
ORANGE_MIN_AREA = 1200
ORANGE_NEAR_DEPTH_M = 0.35

# Fusion weights. LiDAR remains dominant for safety.
LIDAR_WEIGHT = 0.62
CAMERA_WEIGHT = 0.38
CAMERA_MAX_STEER = 0.55

# If camera sees both left and right orange walls, use image centerline.
# If camera sees only one wall, nudge away from that wall.
SINGLE_WALL_NUDGE = 0.28

# ── BEHAVIORS ────────────────────────────────────────────────
BEHAVIOR_TRACK_FUSION = 0
BEHAVIOR_LIDAR_GAP = 1
BEHAVIOR_LIDAR_CORRIDOR = 2
BEHAVIOR_RIGHT_WALL = 3
BEHAVIOR_NAMES = ["TRACK_FUSION", "LIDAR_GAP", "LIDAR_CORRIDOR", "RIGHT_WALL"]
BEHAVIOR_DEFAULT = BEHAVIOR_TRACK_FUSION

# ── DISPLAY / DEBUG ──────────────────────────────────────────
MAP_ENABLED = True
ZED_DISPLAY = True
PRINT_HZ = 8
MAP_W = 600
MAP_H = 600
MAP_SCALE = 0.10
WARMUP_LOOPS = 20

# ── PRECOMPUTED TABLES ──────────────────────────────────────
_N_STEPS = LIDAR_STEP_MAX - LIDAR_STEP_MIN + 1
_CENTER_IDX = LIDAR_STEP_FRONT - LIDAR_STEP_MIN
_STEPS_PER_90 = int(90.0 * 1024 / 360)
_SCAN_INTERVAL_RAD = 2.0 * math.pi / 1024

SECTORS = [
    ("FAR_LEFT", -90, -45),
    ("LEFT", -45, -15),
    ("CENTER_LEFT", -15, -5),
    ("CENTER", -5, 5),
    ("CENTER_RIGHT", 5, 15),
    ("RIGHT", 15, 45),
    ("FAR_RIGHT", 45, 90),
]


def _build_angle_tables():
    sins, coss = [], []
    for idx in range(_N_STEPS):
        step = idx + LIDAR_STEP_MIN
        a = (step - LIDAR_STEP_FRONT) * _SCAN_INTERVAL_RAD
        sins.append(math.sin(a))
        coss.append(math.cos(a))
    return sins, coss


def _build_sector_masks():
    masks = {name: [] for name, _, _ in SECTORS}
    for idx in range(_N_STEPS):
        step = idx + LIDAR_STEP_MIN
        angle_deg = (step - LIDAR_STEP_FRONT) * (360.0 / 1024)
        for name, lo, hi in SECTORS:
            if lo <= angle_deg <= hi:
                masks[name].append(idx)
    return masks


_SIN, _COS = _build_angle_tables()
_SECTOR_MASKS = _build_sector_masks()


def safe_percentile(vals, pct, default=float("inf")):
    vals = [v for v in vals if v > 20 and v < 5600]
    if not vals:
        return default
    vals.sort()
    i = max(0, min(len(vals) - 1, int(len(vals) * pct)))
    return vals[i]


# ── HOKUYO LIDAR ─────────────────────────────────────────────

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
                if scan and len(scan) > 100:
                    with self._lock:
                        self._distances = scan
            except Exception:
                time.sleep(0.01)

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
        lo = max(0, _CENTER_IDX - LIDAR_FRONT_WINDOW)
        hi = min(len(d) - 1, _CENTER_IDX + LIDAR_FRONT_WINDOW)
        return safe_percentile(d[lo:hi + 1], 0.10, default=None)

    def sector_clearances(self):
        d = self.get_distances()
        result = {}
        for name, indices in _SECTOR_MASKS.items():
            vals = [d[i] for i in indices if i < len(d)]
            # Use 15th percentile instead of raw min so one noisy ray does not jerk the car.
            result[name] = safe_percentile(vals, 0.15)
        return result

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


# ── ZED CAMERA ───────────────────────────────────────────────

class ZEDOrangeTracker:
    def __init__(self):
        self.connected = False
        self._running = False
        self._thread = None
        self._lock = threading.Lock()

        self._cam = None
        self._steer = 0.0
        self._confidence = 0.0
        self._area = 0
        self._left_area = 0
        self._right_area = 0
        self._nearest = float("nan")
        self._display = None
        self._state = "NO_ZED"

    def connect(self):
        if not ZED_AVAILABLE:
            raise RuntimeError("pyzed is not available on this system")

        self._cam = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = ZED_RESOLUTION
        init.camera_fps = ZED_FPS
        init.depth_mode = ZED_DEPTH_MODE
        init.coordinate_units = sl.UNIT.METER

        status = self._cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED open failed: {status}")

        info = self._cam.get_camera_information()
        print(f"[ZED] Connected: {info.camera_model} S/N {info.serial_number}")
        self.connected = True

    def start(self):
        if not self.connected:
            return
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        runtime = sl.RuntimeParameters()
        image_mat = sl.Mat()
        depth_mat = sl.Mat()

        while self._running:
            if self._cam.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.005)
                continue

            self._cam.retrieve_image(image_mat, sl.VIEW.LEFT)
            self._cam.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

            bgr = cv2.cvtColor(image_mat.get_data(), cv2.COLOR_BGRA2BGR)
            depth = depth_mat.get_data()

            try:
                steer, conf, area, la, ra, near, disp, state = self._detect(bgr, depth)
                with self._lock:
                    self._steer = steer
                    self._confidence = conf
                    self._area = area
                    self._left_area = la
                    self._right_area = ra
                    self._nearest = near
                    self._display = disp
                    self._state = state
            except Exception:
                time.sleep(0.005)

    def _detect(self, bgr, depth_raw):
        h, w = bgr.shape[:2]
        y0 = int(h * CAM_ROI_TOP)
        y1 = int(h * CAM_ROI_BOTTOM)

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_LO_1, ORANGE_HI_1) | cv2.inRange(hsv, ORANGE_LO_2, ORANGE_HI_2)

        mask[:y0, :] = 0
        mask[y1:, :] = 0

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        total = int(np.count_nonzero(mask))
        display = bgr.copy()
        overlay = np.zeros_like(display)
        overlay[mask > 0] = (0, 140, 255)
        display = cv2.addWeighted(display, 0.72, overlay, 0.28, 0)
        cv2.rectangle(display, (0, y0), (w - 1, y1), (255, 255, 255), 1)
        cv2.line(display, (w // 2, y0), (w // 2, y1), (255, 255, 255), 1)

        if total < ORANGE_MIN_AREA:
            cv2.putText(display, "Orange track: no strong detection", (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 160, 255), 2, cv2.LINE_AA)
            return 0.0, 0.0, total, 0, 0, float("nan"), display, "NO_ORANGE"

        roi = mask[y0:y1, :]
        left_area = int(np.count_nonzero(roi[:, :w // 2]))
        right_area = int(np.count_nonzero(roi[:, w // 2:]))

        ys, xs = np.where(roi > 0)
        xs_global = xs.astype(np.float32)

        # Depth near orange.
        orange_depths = depth_raw[mask > 0]
        valid_depths = orange_depths[np.isfinite(orange_depths)]
        nearest = float(np.min(valid_depths)) if valid_depths.size else float("nan")

        # Find contours and keep only meaningful orange blobs.
        contours, _ = cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blobs = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 250:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            cx = x + bw / 2.0
            blobs.append((area, cx, x, y + y0, bw, bh))

        # Draw contour boxes.
        for area, cx, x, y, bw, bh in blobs:
            cv2.rectangle(display, (x, y), (x + bw, y + bh), (0, 220, 255), 2)

        left_seen = left_area > ORANGE_MIN_AREA * 0.35
        right_seen = right_area > ORANGE_MIN_AREA * 0.35

        if left_seen and right_seen:
            # Estimate left wall center and right wall center separately.
            left_x = float(np.mean(xs_global[xs_global < w / 2])) if np.any(xs_global < w / 2) else 0.0
            right_x = float(np.mean(xs_global[xs_global >= w / 2])) if np.any(xs_global >= w / 2) else float(w)
            corridor_center = (left_x + right_x) / 2.0

            # If the track center appears to the right of image center, steer right.
            image_error = (corridor_center - (w / 2.0)) / (w / 2.0)
            steer = -image_error * CAMERA_MAX_STEER
            state = "BOTH_WALLS"
            cv2.line(display, (int(corridor_center), y0), (int(corridor_center), y1), (0, 255, 0), 3)

        elif left_seen:
            # Orange wall/tube is mostly on the left, so steer right.
            steer = -SINGLE_WALL_NUDGE
            state = "LEFT_ONLY"

        elif right_seen:
            # Orange wall/tube is mostly on the right, so steer left.
            steer = SINGLE_WALL_NUDGE
            state = "RIGHT_ONLY"

        else:
            # Orange exists but is ambiguous. Use centroid balance.
            centroid_x = float(np.mean(xs_global))
            image_error = (centroid_x - (w / 2.0)) / (w / 2.0)
            steer = -image_error * 0.25
            state = "AMBIGUOUS"

        # If a tube is extremely close in depth, increase confidence but do not exceed camera max steer.
        steer = max(-CAMERA_MAX_STEER, min(CAMERA_MAX_STEER, steer))

        conf = min(1.0, total / 25000.0)
        if np.isfinite(nearest) and nearest < ORANGE_NEAR_DEPTH_M:
            conf = min(1.0, conf + 0.25)

        near_str = f"{nearest:.2f}m" if np.isfinite(nearest) else "--"
        cv2.putText(display, f"Orange: {state} steer={steer:+.2f} conf={conf:.2f} near={near_str}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2, cv2.LINE_AA)
        cv2.putText(display, f"L={left_area} R={right_area} total={total}",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 220, 255), 2, cv2.LINE_AA)

        return steer, conf, total, left_area, right_area, nearest, display, state

    def get(self):
        with self._lock:
            return {
                "steer": self._steer,
                "confidence": self._confidence,
                "area": self._area,
                "left_area": self._left_area,
                "right_area": self._right_area,
                "nearest": self._nearest,
                "state": self._state,
            }

    def get_display_frame(self):
        with self._lock:
            return self._display.copy() if self._display is not None else None

    def stop(self):
        self._running = False
        if self._cam is not None:
            self._cam.close()
        self.connected = False
        print("[ZED] Disconnected")


# ── VESC HELPERS ─────────────────────────────────────────────

def crc16(data):
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def build_packet(payload):
    crc = crc16(payload)
    return bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])


def send_packet(ser, payload):
    ser.write(build_packet(payload))
    ser.flush()


def send_duty(ser, duty):
    duty = max(-MAX_DUTY, min(MAX_DUTY, duty))
    send_packet(ser, bytes([COMM_SET_DUTY]) + struct.pack(">i", int(duty * 100000)))


def send_current_zero(ser):
    send_packet(ser, bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0))


def send_servo(ser, position):
    position = max(SERVO_MIN, min(SERVO_MAX, position))
    send_packet(ser, bytes([COMM_SET_SERVO_POS]) + struct.pack(">h", int(position * 1000)))


def stop_car(ser):
    send_current_zero(ser)
    send_servo(ser, SERVO_CENTER)
    print("\n[VESC] Motor stopped, steering centred")


def ramp_value(current, target, step):
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


# ── SPEED ZONE ───────────────────────────────────────────────

def lidar_speed_zone(front_dist, steer_abs=0.0):
    if front_dist is None:
        cap, label = AUTO_DRIVE_DUTY, "FULL"
    elif front_dist >= LIDAR_FULL_SPEED_MM:
        cap, label = AUTO_DRIVE_DUTY, "FULL"
    elif front_dist >= LIDAR_MODERATE_MM:
        cap, label = MODERATE_DUTY_CAP, "MODERATE"
    elif front_dist >= LIDAR_SLOW_MM:
        cap, label = SLOW_DUTY_CAP, "SLOW"
    elif front_dist >= LIDAR_ESTOP_MM:
        cap, label = CRAWL_DUTY_CAP, "CRAWL"
    else:
        cap, label = 0.0, "ESTOP"

    # Slow down on turns.
    if label != "ESTOP":
        if steer_abs > CURVE_SLOW_STEER:
            t = min(1.0, (steer_abs - CURVE_SLOW_STEER) / (1.0 - CURVE_SLOW_STEER))
            scale = 1.0 - t * (1.0 - CURVE_MIN_SCALE)
            cap *= scale

    return cap, label


# ── LIDAR STEERING LOGIC ─────────────────────────────────────

def preprocess_gap_lidar(distances):
    if not distances:
        return []

    ranges = list(distances)
    last_r = 0
    skip_iter = False
    num_excluded = 0

    for idx, r in enumerate(ranges):
        if idx == 0:
            last_r = r
            continue

        if skip_iter:
            last_r = r
            num_excluded -= 1
            if num_excluded == -1:
                skip_iter = False
            continue

        if r <= 20 or last_r <= 20:
            last_r = r
            continue

        r_m = r / 1000.0
        last_r_m = last_r / 1000.0

        if abs(r_m - last_r_m) > FTG_DISPARITY_THRESH_MM / 1000.0:
            try:
                half_angle = math.asin(
                    min(1.0, (CAR_WIDTH_MM / 1000.0 * FTG_SAFETY_FACTOR) / min(r_m, last_r_m))
                )
                num_excluded = round(half_angle / _SCAN_INTERVAL_RAD) + FTG_EXTRA_SAMPLES
            except (ValueError, ZeroDivisionError):
                num_excluded = FTG_EXTRA_SAMPLES

            closer = min(r, last_r)
            if r > last_r:
                samples = range(idx, min(len(ranges), idx + num_excluded + 1))
            else:
                samples = range(max(0, idx - num_excluded), idx)

            for j in samples:
                if ranges[j] > closer:
                    ranges[j] = closer

            skip_iter = True

        last_r = r

    return ranges


def find_best_gap_direction(distances):
    lo = max(0, _CENTER_IDX - _STEPS_PER_90)
    hi = min(len(distances) - 1, _CENTER_IDX + _STEPS_PER_90)
    sub = distances[lo:hi + 1]
    if not sub:
        return 0.0

    # Instead of aiming at one noisy farthest ray, score a window around each ray.
    best_score = -1
    best_idx = _CENTER_IDX
    window = 10

    for i in range(lo, hi + 1):
        a = max(lo, i - window)
        b = min(hi, i + window)
        vals = [distances[j] for j in range(a, b + 1) if distances[j] > 20]
        if not vals:
            continue

        avg_clearance = sum(vals) / len(vals)
        offset_norm = abs(i - _CENTER_IDX) / float(_STEPS_PER_90)

        # Prefer open space, but penalize extreme angles to avoid late wall crashes.
        score = avg_clearance * (1.0 - 0.28 * offset_norm)

        if score > best_score:
            best_score = score
            best_idx = i

    offset = best_idx - _CENTER_IDX
    steer = -offset / float(_STEPS_PER_90)
    return max(-1.0, min(1.0, steer))


def compute_gap_following(distances, clearances):
    if not distances:
        return 0.0
    processed = preprocess_gap_lidar(distances)
    return find_best_gap_direction(processed)


def compute_corridor_centering(clearances):
    left_d = min(
        clearances.get("LEFT", float("inf")),
        clearances.get("CENTER_LEFT", float("inf")),
    )
    right_d = min(
        clearances.get("RIGHT", float("inf")),
        clearances.get("CENTER_RIGHT", float("inf")),
    )

    cap = float(LIDAR_FULL_SPEED_MM)
    left_d = min(left_d, cap) if left_d < float("inf") else cap
    right_d = min(right_d, cap) if right_d < float("inf") else cap

    denom = left_d + right_d
    if denom < 1.0:
        return 0.0

    # Positive steer means left. If right wall is closer, left_d > right_d -> steer left.
    return max(-1.0, min(1.0, (left_d - right_d) / denom))


def compute_wall_following(clearances):
    right_d = min(clearances.get("RIGHT", float("inf")), clearances.get("FAR_RIGHT", float("inf")))
    if right_d == float("inf"):
        right_d = WALL_TARGET_MM * 2.0
    error = right_d - WALL_TARGET_MM
    return max(-1.0, min(1.0, -(error / float(WALL_TARGET_MM))))


def lidar_panic_override(clearances, front_dist):
    left = min(clearances.get("FAR_LEFT", float("inf")), clearances.get("LEFT", float("inf")))
    right = min(clearances.get("FAR_RIGHT", float("inf")), clearances.get("RIGHT", float("inf")))
    front_left = clearances.get("CENTER_LEFT", float("inf"))
    front_right = clearances.get("CENTER_RIGHT", float("inf"))

    # Hard side panic.
    if left < SIDE_PANIC_MM and right < SIDE_PANIC_MM:
        return 0.0, "BOTH_SIDE_PANIC"
    if left < SIDE_PANIC_MM:
        return -0.65, "LEFT_PANIC"     # left wall close -> steer right
    if right < SIDE_PANIC_MM:
        return 0.65, "RIGHT_PANIC"    # right wall close -> steer left

    # Front-corner warning. If a front corner is closing, steer away early.
    if front_left < FRONT_CORNER_WARN_MM and front_right >= front_left:
        return -0.45, "FRONT_LEFT_WARN"
    if front_right < FRONT_CORNER_WARN_MM and front_left > front_right:
        return 0.45, "FRONT_RIGHT_WARN"

    if front_dist is not None and front_dist < LIDAR_CRAWL_MM:
        # Choose the more open side.
        if left > right:
            return 0.55, "FRONT_CLOSE_LEFT_OPEN"
        else:
            return -0.55, "FRONT_CLOSE_RIGHT_OPEN"

    return None, "OK"


def fuse_track_steer(lidar_gap, lidar_corridor, camera_data, clearances, front_dist):
    panic, reason = lidar_panic_override(clearances, front_dist)
    if panic is not None:
        return panic, f"LIDAR_{reason}"

    # LiDAR base: mix gap following with corridor centering.
    # On a tube track, corridor centering helps prevent wall scraping.
    lidar_base = 0.42 * lidar_gap + 0.58 * lidar_corridor
    lidar_base = max(-1.0, min(1.0, lidar_base))

    cam_steer = camera_data["steer"]
    cam_conf = camera_data["confidence"]

    if cam_conf <= 0.05:
        return lidar_base, "LIDAR_ONLY"

    # Camera gets more influence only when confidence is strong.
    cw = CAMERA_WEIGHT * cam_conf
    lw = 1.0 - cw
    fused = lw * lidar_base + cw * cam_steer
    fused = max(-1.0, min(1.0, fused))
    return fused, f"FUSED_{camera_data['state']}"


def select_reactive_steer(mode, distances, clearances, camera_data, front_dist):
    gap = compute_gap_following(distances, clearances)
    corridor = compute_corridor_centering(clearances)

    if mode == BEHAVIOR_LIDAR_GAP:
        panic, reason = lidar_panic_override(clearances, front_dist)
        return (panic if panic is not None else gap), f"GAP_{reason}"

    if mode == BEHAVIOR_LIDAR_CORRIDOR:
        panic, reason = lidar_panic_override(clearances, front_dist)
        return (panic if panic is not None else corridor), f"CORRIDOR_{reason}"

    if mode == BEHAVIOR_RIGHT_WALL:
        panic, reason = lidar_panic_override(clearances, front_dist)
        return (panic if panic is not None else compute_wall_following(clearances)), f"WALL_{reason}"

    return fuse_track_steer(gap, corridor, camera_data, clearances, front_dist)


def reactive_steer_to_servo(steer):
    x = -steer if INVERT_STEERING else steer
    return max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + x * 0.50))


# ── MAP DISPLAY ──────────────────────────────────────────────

class LidarMapDisplay:
    _CX = MAP_W // 2
    _CY = MAP_H // 2

    def __init__(self):
        self._surface = pygame.display.set_mode((MAP_W, MAP_H))
        pygame.display.set_caption("LiDAR + ZED Track Nav")
        self._font_sm = pygame.font.SysFont("monospace", 12)
        self._font_md = pygame.font.SysFont("monospace", 14, bold=True)

    @staticmethod
    def _w2s(x_mm, y_mm):
        return LidarMapDisplay._CX + int(x_mm * MAP_SCALE), LidarMapDisplay._CY - int(y_mm * MAP_SCALE)

    @staticmethod
    def _dist_color(d):
        if d >= LIDAR_FULL_SPEED_MM:
            return (0, 210, 60)
        if d >= LIDAR_MODERATE_MM:
            return (220, 220, 0)
        if d >= LIDAR_SLOW_MM:
            return (255, 130, 0)
        return (255, 50, 50)

    def render(self, distances, clearances, steer, zone, front_dist, behavior, source, cam):
        s = self._surface
        s.fill((12, 12, 22))
        self._draw_grid(s)
        self._draw_rings(s)
        self._draw_scan(s, distances)
        self._draw_car(s)
        self._draw_steer_arrow(s, steer)
        self._draw_hud(s, zone, front_dist, behavior, source, steer, clearances, cam)
        pygame.display.flip()

    @staticmethod
    def _draw_grid(s):
        step_px = max(1, int(500 * MAP_SCALE))
        for x in range(0, MAP_W, step_px):
            pygame.draw.line(s, (28, 28, 45), (x, 0), (x, MAP_H))
        for y in range(0, MAP_H, step_px):
            pygame.draw.line(s, (28, 28, 45), (0, y), (MAP_W, y))
        cx, cy = LidarMapDisplay._CX, LidarMapDisplay._CY
        pygame.draw.line(s, (55, 55, 85), (cx, 0), (cx, MAP_H))
        pygame.draw.line(s, (55, 55, 85), (0, cy), (MAP_W, cy))

    @staticmethod
    def _draw_rings(s):
        cx, cy = LidarMapDisplay._CX, LidarMapDisplay._CY
        for mm, col in [(400, (70, 30, 30)), (800, (90, 55, 20)), (1500, (30, 75, 30))]:
            pygame.draw.circle(s, col, (cx, cy), int(mm * MAP_SCALE), 1)

    @staticmethod
    def _draw_scan(s, distances):
        for idx, dist in enumerate(distances):
            if idx >= len(_SIN):
                break
            if dist <= 20 or dist > 5500:
                continue
            sx, sy = LidarMapDisplay._w2s(dist * _SIN[idx], dist * _COS[idx])
            if 0 <= sx < MAP_W and 0 <= sy < MAP_H:
                pygame.draw.circle(s, LidarMapDisplay._dist_color(dist), (sx, sy), 2)

    @staticmethod
    def _draw_car(s):
        cx, cy = LidarMapDisplay._CX, LidarMapDisplay._CY
        pygame.draw.rect(s, (150, 150, 255), (cx - 5, cy - 9, 10, 18))
        pygame.draw.polygon(s, (255, 255, 100), [(cx, cy - 19), (cx - 5, cy - 9), (cx + 5, cy - 9)])

    @staticmethod
    def _draw_steer_arrow(s, steer):
        if abs(steer) < 0.04:
            return
        cx, cy = LidarMapDisplay._CX, LidarMapDisplay._CY
        base_y = cy + 38
        arrow_px = int(steer * 90)
        ex = cx + arrow_px
        dx = 1 if arrow_px > 0 else -1
        pygame.draw.line(s, (80, 200, 255), (cx, base_y), (ex, base_y), 3)
        pygame.draw.polygon(s, (80, 200, 255), [(ex, base_y), (ex - dx * 10, base_y - 5), (ex - dx * 10, base_y + 5)])

    def _draw_hud(self, s, zone, front_dist, behavior, source, steer, clearances, cam):
        fd = f"{front_dist} mm" if front_dist is not None else "-- mm"
        near = cam.get("nearest", float("nan"))
        near_s = f"{near:.2f}m" if np.isfinite(near) else "--"

        lines = [
            (f"Zone:   {zone}", (220, 220, 0) if zone != "FULL" else (0, 210, 60)),
            (f"Front:  {fd}", (200, 200, 200)),
            (f"Mode:   {behavior}", (80, 200, 255)),
            (f"Source: {source}", (255, 160, 80)),
            (f"Steer:  {steer:+.2f}", (160, 200, 255)),
            (f"ZED:    {cam.get('state','--')} conf={cam.get('confidence',0.0):.2f} near={near_s}", (255, 140, 0)),
        ]
        y = 5
        for text, col in lines:
            s.blit(self._font_md.render(text, True, col), (5, y))
            y += 18

        y = MAP_H - 5 - len(SECTORS) * 14
        for name, _, _ in SECTORS:
            d = clearances.get(name, float("inf"))
            d_str = f"{int(d):5d} mm" if d < float("inf") else "  inf  "
            col = self._dist_color(d) if d < float("inf") else (70, 70, 70)
            s.blit(self._font_sm.render(f"{name:<15}{d_str}", True, col), (5, y))
            y += 14


def maybe_render_map(map_display, lidar, clearances, steer, zone, front, behavior, source, cam, last_map, period):
    now = time.time()
    if map_display and lidar and (now - last_map) >= period:
        map_display.render(lidar.get_distances(), clearances, steer, zone, front, behavior, source, cam)
        return now
    return last_map


# ── MAIN ─────────────────────────────────────────────────────

def main():
    pygame.init()
    pygame.joystick.init()

    # LiDAR
    lidar = HokuyoLidar()
    try:
        lidar.connect()
        lidar.start()
    except Exception as e:
        print(f"[LIDAR] WARNING: could not connect — {e}")
        lidar = None

    # ZED
    cam = None
    if ZED_ENABLED:
        cam = ZEDOrangeTracker()
        try:
            cam.connect()
            cam.start()
            if ZED_DISPLAY:
                cv2.namedWindow("ZED Orange Track", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("ZED Orange Track", 960, 540)
        except Exception as e:
            print(f"[ZED] WARNING: could not connect — {e}")
            cam = None

    # Map
    map_display = None
    if MAP_ENABLED:
        try:
            map_display = LidarMapDisplay()
            print("[MAP] 2D LiDAR display active")
        except Exception as e:
            print(f"[MAP] WARNING: display unavailable — {e}")

    # Controller optional
    joystick = None
    pygame.event.pump()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[PS4] Connected: {joystick.get_name()}")
        print("       Triangle = cycle mode | X/Circle = quit | L1 = hold stop")
    else:
        print("[PS4] No controller detected — autonomous only")

    # VESC
    print(f"[VESC] Opening {VESC_PORT}...")
    ser = serial.Serial(VESC_PORT, VESC_BAUDRATE, timeout=0.05, write_timeout=0.05)
    time.sleep(0.5)
    send_current_zero(ser)
    send_servo(ser, SERVO_CENTER)

    print()
    print("=" * 72)
    print("  ZED + LiDAR ORANGE TRACK NAV — safer smooth loop test")
    print("=" * 72)
    print(f"  VESC : {VESC_PORT}")
    print(f"  LiDAR: {LIDAR_PORT}")
    print(f"  Speed: auto={AUTO_DRIVE_DUTY:.3f} max={MAX_DUTY:.3f}")
    print(f"  Default mode: {BEHAVIOR_NAMES[BEHAVIOR_DEFAULT]}")
    print("  Start on a stand first. Then test slowly on the ground.")
    print("=" * 72)
    print()

    loop_period = 1.0 / LOOP_HZ
    print_period = 1.0 / PRINT_HZ
    map_period = 1.0 / 15

    last_print = 0.0
    last_map = 0.0

    behavior_mode = BEHAVIOR_DEFAULT
    warmup_counter = 0

    lidar_estop = False
    reverse_counter = 0
    hold_counter = 0
    kill_active = False

    current_duty = 0.0
    current_steer = 0.0
    target_steer = 0.0
    steer_source = "INIT"

    clearances = {name: float("inf") for name, _, _ in SECTORS}
    front_dist = None
    zone_label = "FULL"

    try:
        while True:
            loop_start = time.time()

            # Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button in (BTN_X, BTN_CIRCLE):
                        raise KeyboardInterrupt
                    if event.button in (BTN_TRIANGLE, BTN_TRIANGLE_ALT):
                        behavior_mode = (behavior_mode + 1) % len(BEHAVIOR_NAMES)
                        print(f"\n[MODE] {BEHAVIOR_NAMES[behavior_mode]}")
                    if event.button in (BTN_L1, BTN_L1_ALT):
                        kill_active = True
                        current_duty = 0.0
                        stop_car(ser)
                        print("\n[KILL] L1 held")

                if event.type == pygame.JOYBUTTONUP:
                    if event.button in (BTN_L1, BTN_L1_ALT):
                        kill_active = False
                        print("[KILL] Released")

            # Poll L1 too, in case button events are unreliable.
            if joystick is not None:
                pygame.event.pump()
                try:
                    kill_active = bool(
                        (joystick.get_numbuttons() > BTN_L1 and joystick.get_button(BTN_L1)) or
                        (joystick.get_numbuttons() > BTN_L1_ALT and joystick.get_button(BTN_L1_ALT))
                    )
                except Exception:
                    pass

            if kill_active:
                current_duty = 0.0
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                if cam and cam.connected and ZED_DISPLAY:
                    frame = cam.get_display_frame()
                    if frame is not None:
                        cv2.imshow("ZED Orange Track", frame)
                    cv2.waitKey(1)
                time.sleep(loop_period)
                continue

            # Sensor processing
            distances = []
            camera_data = {
                "steer": 0.0,
                "confidence": 0.0,
                "area": 0,
                "left_area": 0,
                "right_area": 0,
                "nearest": float("nan"),
                "state": "NO_CAMERA",
            }

            if cam and cam.connected:
                camera_data = cam.get()

            if lidar and lidar.connected:
                distances = lidar.get_distances()
                front_dist = lidar.front_min()
                clearances = lidar.sector_clearances()
                target_steer, steer_source = select_reactive_steer(
                    behavior_mode, distances, clearances, camera_data, front_dist
                )
            else:
                front_dist = None
                target_steer = camera_data["steer"]
                steer_source = "CAMERA_ONLY"

            # Smooth steering
            current_steer = ramp_value(current_steer, target_steer, STEER_RAMP_STEP)
            servo_pos = reactive_steer_to_servo(current_steer)

            # Speed cap after steering is known
            speed_cap, zone_label = lidar_speed_zone(front_dist, abs(current_steer))

            # ESTOP logic
            if zone_label == "ESTOP" and not lidar_estop:
                lidar_estop = True
                reverse_counter = BRAKE_LOOPS + REVERSE_LOOPS + SCAN_LOOPS
                hold_counter = 0
                current_duty = 0.0
                print(f"\n[LIDAR] ESTOP — obstacle at {front_dist} mm")

            elif lidar_estop and zone_label != "ESTOP":
                path_clear = front_dist is None or front_dist >= LIDAR_ESTOP_CLEAR_MM
                if reverse_counter == 0 and (path_clear or hold_counter >= HOLD_TIMEOUT_LOOPS):
                    lidar_estop = False
                    hold_counter = 0
                    print(f"\n[LIDAR] Path clear ({front_dist} mm) — resuming")

            # Warmup
            if warmup_counter < WARMUP_LOOPS:
                warmup_counter += 1
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                if cam and cam.connected and ZED_DISPLAY:
                    frame = cam.get_display_frame()
                    if frame is not None:
                        cv2.imshow("ZED Orange Track", frame)
                    cv2.waitKey(1)
                time.sleep(loop_period)
                continue

            # ESTOP sequence
            if lidar_estop:
                if reverse_counter > REVERSE_LOOPS + SCAN_LOOPS:
                    current_duty = 0.0
                    send_current_zero(ser)
                    send_servo(ser, SERVO_CENTER)
                    reverse_counter -= 1
                    estop_state = "BRAKE"

                elif reverse_counter > SCAN_LOOPS:
                    send_duty(ser, -CRAWL_DUTY_CAP)
                    send_servo(ser, SERVO_CENTER)
                    reverse_counter -= 1
                    estop_state = "REVERSE"

                elif reverse_counter > 0:
                    current_duty = 0.0
                    send_current_zero(ser)
                    send_servo(ser, SERVO_CENTER)
                    reverse_counter -= 1
                    estop_state = "SCAN"

                else:
                    hold_counter += 1
                    if hold_counter >= HOLD_TIMEOUT_LOOPS:
                        lidar_estop = False
                        hold_counter = 0
                        print("\n[ESTOP] Hold timeout — forcing resume")
                    current_duty = 0.0
                    send_current_zero(ser)
                    send_servo(ser, servo_pos)
                    estop_state = f"HOLD({hold_counter})"

                now = time.time()
                if now - last_print >= print_period:
                    fd = f"{front_dist}mm" if front_dist is not None else "--"
                    print(f"\r[ESTOP/{estop_state}] Front:{fd} Steer:{current_steer:+.2f} Source:{steer_source}     ", end="")
                    last_print = now

            else:
                target_duty = min(AUTO_DRIVE_DUTY, speed_cap)
                current_duty = ramp_value(current_duty, target_duty, DUTY_RAMP_STEP)

                send_servo(ser, servo_pos)
                if abs(current_duty) > 0.002:
                    send_duty(ser, current_duty)
                    drive_state = f"DRIVE[{zone_label}]"
                else:
                    send_current_zero(ser)
                    drive_state = "IDLE"

                now = time.time()
                if now - last_print >= print_period:
                    fd = f"{front_dist}mm" if front_dist is not None else "--"
                    near = camera_data["nearest"]
                    near_s = f"{near:.2f}m" if np.isfinite(near) else "--"
                    print(
                        f"\r[{drive_state}] "
                        f"Mode:{BEHAVIOR_NAMES[behavior_mode]} "
                        f"Src:{steer_source} "
                        f"Steer:{current_steer:+.2f} Servo:{servo_pos:.2f} "
                        f"Duty:{current_duty:+.3f} Cap:{speed_cap:.3f} "
                        f"Front:{fd} ZED:{camera_data['state']} c={camera_data['confidence']:.2f} near={near_s}     ",
                        end=""
                    )
                    last_print = now

            # Render debug windows
            last_map = maybe_render_map(
                map_display, lidar, clearances, current_steer, zone_label, front_dist,
                BEHAVIOR_NAMES[behavior_mode], steer_source, camera_data, last_map, map_period
            )

            if cam and cam.connected and ZED_DISPLAY:
                frame = cam.get_display_frame()
                if frame is not None:
                    cv2.imshow("ZED Orange Track", frame)
                cv2.waitKey(1)

            elapsed = time.time() - loop_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)

    except KeyboardInterrupt:
        print("\n[INFO] Quitting...")

    finally:
        try:
            stop_car(ser)
            ser.close()
        except Exception:
            pass

        if lidar:
            lidar.stop()
        if cam:
            cam.stop()

        cv2.destroyAllWindows()
        pygame.quit()
        print("[INFO] Closed safely")


if __name__ == "__main__":
    main()
