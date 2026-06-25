import struct
import time
import serial
import pygame
import threading
import math
import cv2
import numpy as np

# ============================================================
# Reactive_ZED_gap.py — Autonomous reactive navigation
# VESC 6 MkVI + Hokuyo URG-04LX + ZED Camera (Roboracer / F1TENTH)
#
# Extends Reactive_option2_gap.py with ZED camera fusion:
#   - Detects orange walls/cones in real time
#   - Blends a parallel-alignment correction into the LiDAR steering
#   - LiDAR still owns obstacle avoidance and speed zones
#   - Camera adds a gentle wall-balance nudge on top
#
# Three selectable behaviors (Triangle to cycle):
#   GAP_FOLLOW   — steer toward the widest contiguous open gap
#   CORRIDOR_CTR — stay centred between left and right walls
#   WALL_FOLLOW  — maintain target distance from the right wall
#
# Speed zones (forward only):
#   >= 1000 mm  FULL      (MAX_DUTY)
#    500–1000   MODERATE  (MODERATE_DUTY_CAP)
#    300– 500   SLOW      (SLOW_DUTY_CAP)
#    < 200 mm   ESTOP     (stop until path clears)
#
# Controls:
#   Triangle   = cycle behavior mode (optional joystick)
#   Circle     = quit               (optional joystick)
#   Ctrl-C     = quit
# ============================================================

# ── VESC ────────────────────────────────────────────────────
VESC_PORT       = "/dev/ttyACM1"
VESC_BAUDRATE   = 115200
MAX_DUTY        = 0.10
AUTO_DRIVE_DUTY = 0.08
LOOP_HZ         = 50
DUTY_RAMP_STEP  = 0.005

# ── PS4 CONTROLLER (optional) ───────────────────────────────
BTN_X        = 0
BTN_CIRCLE   = 1
BTN_TRIANGLE = 3

# ── VESC COMMAND IDs ────────────────────────────────────────
COMM_SET_DUTY      = 5
COMM_SET_CURRENT   = 6
COMM_SET_RPM       = 8
COMM_SET_SERVO_POS = 12

# ── SERVO ───────────────────────────────────────────────────
SERVO_CENTER    = 0.50
SERVO_MIN       = 0.15
SERVO_MAX       = 0.85
INVERT_STEERING = False

PRINT_HZ     = 8
WARMUP_LOOPS = 15

# ── HOKUYO URG-04LX ─────────────────────────────────────────
LIDAR_PORT         = "/dev/ttyACM0"
LIDAR_BAUDRATE     = 19200
LIDAR_STEP_MIN     = 44
LIDAR_STEP_MAX     = 725
LIDAR_STEP_FRONT   = 384
LIDAR_FRONT_WINDOW = 50

LIDAR_FULL_SPEED_MM  = 1000
LIDAR_MODERATE_MM    = 500
LIDAR_SLOW_MM        = 300
LIDAR_ESTOP_MM       = 200
LIDAR_ESTOP_CLEAR_MM = 400

MODERATE_DUTY_CAP = 0.09
SLOW_DUTY_CAP     = 0.10

BRAKE_LOOPS   = 20
REVERSE_LOOPS = 40
SCAN_LOOPS    = 20

LIDAR_ENABLED = True

# ── ZED CAMERA ──────────────────────────────────────────────
CAMERA_ENABLED = True

# Device index — ZED usually appears as /dev/video0.
# Change to 1 or 2 if another camera is already on index 0.
ZED_DEVICE = 0

# The ZED outputs a side-by-side stereo frame (left eye | right eye).
# Set ZED_STEREO = True so the class takes only the left half.
# If your setup exposes the left eye alone, set to False.
ZED_STEREO         = True
ZED_CAPTURE_WIDTH  = 2560   # stereo frame width (2× eye width at 720p)
ZED_CAPTURE_HEIGHT = 720

# Orange detection — HSV thresholds.
# Hue in OpenCV is 0–180 (not 0–360).  Orange sits around H 8–20.
ORANGE_HSV_LO = np.array([ 8, 150, 100], dtype=np.uint8)
ORANGE_HSV_HI = np.array([20, 255, 255], dtype=np.uint8)

# Minimum orange pixel count to trust a detection (filters out tiny glints)
ORANGE_MIN_AREA = 600

# How strongly the camera correction nudges the LiDAR steer.
# 0.0 = camera ignored; 1.0 = camera fully overrides.
# 0.30 means camera can shift the steer by up to ±0.30 on top of LiDAR.
CAMERA_STEER_WEIGHT = 0.30

# ── REACTIVE BEHAVIORS ───────────────────────────────────────
BEHAVIOR_GAP_FOLLOW   = 0
BEHAVIOR_CORRIDOR_CTR = 1
BEHAVIOR_WALL_FOLLOW  = 2
BEHAVIOR_NAMES        = ["GAP_FOLLOW", "CORRIDOR_CTR", "WALL_FOLLOW"]
BEHAVIOR_DEFAULT      = BEHAVIOR_GAP_FOLLOW

GAP_MIN_DIST_MM        = 400
DISPARITY_THRESHOLD_MM = 350
DISPARITY_EXTRA_SAMPLES = 6
WALL_TARGET_MM         = 500
HOLD_TIMEOUT_LOOPS     = 100
STEER_RAMP_STEP        = 0.03

SECTORS = [
    ("FAR_LEFT",     -90, -45),
    ("LEFT",         -45, -15),
    ("CENTER_LEFT",  -15,  -5),
    ("CENTER",        -5,   5),
    ("CENTER_RIGHT",   5,  15),
    ("RIGHT",         15,  45),
    ("FAR_RIGHT",     45,  90),
]

# ── 2D MAP DISPLAY ──────────────────────────────────────────
MAP_ENABLED = True
MAP_W       = 600
MAP_H       = 600
MAP_SCALE   = 0.10

# ── PRECOMPUTED LOOK-UP TABLES ───────────────────────────────

_N_STEPS      = LIDAR_STEP_MAX - LIDAR_STEP_MIN + 1
_CENTER_IDX   = LIDAR_STEP_FRONT - LIDAR_STEP_MIN   # 340
_STEPS_PER_90 = int(90.0 * 1024 / 360)              # ≈ 256


def _build_angle_tables():
    sins, coss = [], []
    for idx in range(_N_STEPS):
        step = idx + LIDAR_STEP_MIN
        a = (step - LIDAR_STEP_FRONT) * (2.0 * math.pi / 1024)
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


_SIN, _COS    = _build_angle_tables()
_SECTOR_MASKS = _build_sector_masks()


# ── HOKUYO LIDAR CLASS ───────────────────────────────────────

class HokuyoLidar:
    def __init__(self, port=LIDAR_PORT, baud=LIDAR_BAUDRATE):
        self.port       = port
        self.baud       = baud
        self._ser       = None
        self._lock      = threading.Lock()
        self._distances = []
        self._running   = False
        self._thread    = None
        self.connected  = False

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
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get_distances(self):
        with self._lock:
            return list(self._distances)

    def front_min(self):
        d = self.get_distances()
        if not d:
            return None
        lo   = max(0, _CENTER_IDX - LIDAR_FRONT_WINDOW)
        hi   = min(len(d) - 1, _CENTER_IDX + LIDAR_FRONT_WINDOW)
        zone = sorted(x for x in d[lo:hi + 1] if x > 20)
        if not zone:
            return None
        return zone[max(0, len(zone) // 10)]

    def sector_clearances(self):
        d      = self.get_distances()
        result = {}
        for name, indices in _SECTOR_MASKS.items():
            vals = [d[i] for i in indices if i < len(d) and d[i] > 20]
            result[name] = min(vals) if vals else float("inf")
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


# ── ZED CAMERA CLASS ─────────────────────────────────────────

class ZedCamera:
    """
    Captures frames from the ZED stereo camera via OpenCV VideoCapture,
    detects orange regions in a background thread, and exposes:
      orange_balance  — signed [-1, +1]
                        negative = more orange on LEFT  (left wall dominant)
                        positive = more orange on RIGHT (right wall dominant)
      orange_area     — total orange pixel count (0 if below ORANGE_MIN_AREA)
    """

    def __init__(self, device=ZED_DEVICE):
        self.device          = device
        self._cap            = None
        self._lock           = threading.Lock()
        self._running        = False
        self._thread         = None
        self.connected       = False
        self._orange_balance = 0.0
        self._orange_area    = 0
        self._display_frame  = None

    def connect(self):
        self._cap = cv2.VideoCapture(self.device)
        if ZED_STEREO:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  ZED_CAPTURE_WIDTH)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ZED_CAPTURE_HEIGHT)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera device {self.device}")
        self.connected = True
        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"[ZED]   Connected on /dev/video{self.device}  ({actual_w}×{actual_h})")

    def _process(self, frame):
        """Detect orange and update balance/area under the lock."""
        # For stereo mode take only the left eye (left half of the wide frame)
        if ZED_STEREO and frame.shape[1] > frame.shape[0] * 1.5:
            frame = frame[:, :frame.shape[1] // 2]

        h, w = frame.shape[:2]

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_HSV_LO, ORANGE_HSV_HI)

        # Remove noise with a small morphological open
        kernel = np.ones((5, 5), np.uint8)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # Ignore top 15 % (far distance / sky) and bottom 20 % (floor)
        mask[:int(h * 0.15), :]  = 0
        mask[int(h * 0.80):, :]  = 0

        total = int(np.count_nonzero(mask))

        if total < ORANGE_MIN_AREA:
            balance = 0.0
            total   = 0
        else:
            left_area  = int(np.count_nonzero(mask[:, :w // 2]))
            right_area = int(np.count_nonzero(mask[:, w // 2:]))
            # positive = more orange on right; negative = more orange on left
            balance = float(right_area - left_area) / float(total)

        # Build annotated display frame
        display = frame.copy()
        overlay = np.zeros_like(display)
        overlay[mask > 0] = (0, 140, 255)          # highlight orange regions
        display = cv2.addWeighted(display, 0.75, overlay, 0.25, 0)
        cv2.line(display, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        side = "RIGHT" if balance > 0.1 else ("LEFT" if balance < -0.1 else "CTR")
        label = f"Orange: {balance:+.2f} {side}  ({total}px)"
        cv2.putText(display, label, (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2, cv2.LINE_AA)

        with self._lock:
            self._orange_balance = balance
            self._orange_area    = total
            self._display_frame  = display

    def _run(self):
        while self._running:
            ret, frame = self._cap.read()
            if ret and frame is not None:
                try:
                    self._process(frame)
                except Exception:
                    pass

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get_orange(self):
        """Returns (orange_balance, orange_area) — safe to call from any thread."""
        with self._lock:
            return self._orange_balance, self._orange_area

    def get_display_frame(self):
        """Returns the latest annotated frame, or None if not yet available."""
        with self._lock:
            return self._display_frame.copy() if self._display_frame is not None else None

    def stop(self):
        self._running = False
        if self._cap:
            self._cap.release()
        self.connected = False
        print("[ZED]   Disconnected")


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


def send_rpm(ser, rpm):
    send_packet(ser, bytes([COMM_SET_RPM]) + struct.pack(">i", int(rpm)))


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


# ── LIDAR SPEED ZONE ─────────────────────────────────────────

def lidar_speed_zone(front_dist):
    if front_dist is None:
        return MAX_DUTY, "FULL"
    if front_dist >= LIDAR_FULL_SPEED_MM:
        return MAX_DUTY,          "FULL"
    if front_dist >= LIDAR_MODERATE_MM:
        return MODERATE_DUTY_CAP, "MODERATE"
    if front_dist >= LIDAR_SLOW_MM:
        return SLOW_DUTY_CAP,     "SLOW"
    if front_dist >= LIDAR_ESTOP_MM:
        return SLOW_DUTY_CAP,     "SLOW"
    return 0.0, "ESTOP"


# ── REACTIVE BEHAVIOR FUNCTIONS ──────────────────────────────

def preprocess_gap_lidar(distances):
    if not distances:
        return []
    ranges = list(distances)
    for i in range(1, len(ranges)):
        prev_d = ranges[i - 1]
        curr_d = ranges[i]
        if prev_d <= 20 or curr_d <= 20:
            continue
        if abs(curr_d - prev_d) > DISPARITY_THRESHOLD_MM:
            closer = min(prev_d, curr_d)
            if curr_d > prev_d:
                start = i
                end   = min(len(ranges), i + DISPARITY_EXTRA_SAMPLES)
            else:
                start = max(0, i - DISPARITY_EXTRA_SAMPLES)
                end   = i
            for j in range(start, end):
                if ranges[j] > closer:
                    ranges[j] = closer
    return ranges


def compute_gap_following(distances):
    if not distances:
        return 0.0
    distances  = preprocess_gap_lidar(distances)
    lo         = max(0, _CENTER_IDX - _STEPS_PER_90)
    hi         = min(len(distances) - 1, _CENTER_IDX + _STEPS_PER_90)
    best_start = _CENTER_IDX
    best_len   = 0
    run_start  = None
    for i in range(lo, hi + 1):
        d = distances[i] if i < len(distances) else 0
        if d > GAP_MIN_DIST_MM:
            if run_start is None:
                run_start = i
        else:
            if run_start is not None:
                run_len = i - run_start
                if run_len > best_len:
                    best_len   = run_len
                    best_start = run_start
                run_start = None
    if run_start is not None:
        run_len = (hi + 1) - run_start
        if run_len > best_len:
            best_len   = run_len
            best_start = run_start
    gap_center = best_start + best_len // 2
    offset     = gap_center - _CENTER_IDX
    steer      = -offset / float(_STEPS_PER_90)
    return max(-1.0, min(1.0, steer))


def compute_corridor_centering(clearances):
    left_d = min(
        clearances.get("LEFT",        float("inf")),
        clearances.get("CENTER_LEFT", float("inf")),
    )
    right_d = min(
        clearances.get("RIGHT",        float("inf")),
        clearances.get("CENTER_RIGHT", float("inf")),
    )
    cap     = float(LIDAR_MODERATE_MM)
    left_d  = min(left_d,  cap) if left_d  < float("inf") else cap
    right_d = min(right_d, cap) if right_d < float("inf") else cap
    denom   = left_d + right_d
    if denom < 1.0:
        return 0.0
    return max(-1.0, min(1.0, (left_d - right_d) / denom))


def compute_wall_following(clearances):
    right_d = min(
        clearances.get("RIGHT",     float("inf")),
        clearances.get("FAR_RIGHT", float("inf")),
    )
    if right_d == float("inf"):
        right_d = WALL_TARGET_MM * 2.5
    error = right_d - WALL_TARGET_MM
    steer = -(error / float(WALL_TARGET_MM))
    return max(-1.0, min(1.0, steer))


def select_reactive_steer(behavior_mode, distances, clearances):
    if behavior_mode == BEHAVIOR_GAP_FOLLOW:
        return compute_gap_following(distances)
    if behavior_mode == BEHAVIOR_CORRIDOR_CTR:
        return compute_corridor_centering(clearances)
    if behavior_mode == BEHAVIOR_WALL_FOLLOW:
        return compute_wall_following(clearances)
    return 0.0


def compute_parallel_escape(clearances):
    left_open  = clearances.get("FAR_LEFT", 0) + clearances.get("LEFT", 0)
    right_open = clearances.get("FAR_RIGHT", 0) + clearances.get("RIGHT", 0)
    if left_open >= right_open:
        direction, steer = "LEFT", 1.0
    else:
        direction, steer = "RIGHT", -1.0
    print(f"\n[SCAN] Corridor opens {direction}  "
          f"(L={int(left_open)} R={int(right_open)}) — reversing to align parallel")
    return steer


def reactive_steer_to_servo(steer):
    x = -steer if INVERT_STEERING else steer
    return max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + x * 0.50))


# ── CAMERA–LIDAR STEERING FUSION ────────────────────────────

def fuse_camera_steer(lidar_steer, cam):
    """
    Blend a wall-parallel correction from the ZED camera into the LiDAR steer.

    How it works:
      orange_balance > 0  →  more orange on the RIGHT  →  right wall is dominant
                          →  nudge LEFT  (negative correction)
      orange_balance < 0  →  more orange on the LEFT   →  left wall is dominant
                          →  nudge RIGHT (positive correction)
      orange_balance ≈ 0  →  balanced / nothing detected  →  no correction

    The correction is proportional to the imbalance and scaled by
    CAMERA_STEER_WEIGHT so LiDAR always has the primary say.
    """
    if cam is None or not cam.connected:
        return lidar_steer, 0.0, 0

    orange_balance, orange_area = cam.get_orange()

    if orange_area < ORANGE_MIN_AREA:
        return lidar_steer, 0.0, 0

    correction   = -orange_balance * CAMERA_STEER_WEIGHT
    fused        = max(-1.0, min(1.0, lidar_steer + correction))
    return fused, orange_balance, orange_area


# ── 2D LIDAR MAP DISPLAY ─────────────────────────────────────

class LidarMapDisplay:
    _CX = MAP_W // 2
    _CY = MAP_H // 2

    def __init__(self):
        self._surface = pygame.display.set_mode((MAP_W, MAP_H))
        pygame.display.set_caption("LiDAR + ZED Map  —  Reactive")
        self._font_sm = pygame.font.SysFont("monospace", 12)
        self._font_md = pygame.font.SysFont("monospace", 14, bold=True)

    @staticmethod
    def _w2s(x_mm, y_mm):
        sx = LidarMapDisplay._CX + int(x_mm * MAP_SCALE)
        sy = LidarMapDisplay._CY - int(y_mm * MAP_SCALE)
        return sx, sy

    @staticmethod
    def _dist_color(d):
        if d >= LIDAR_FULL_SPEED_MM: return (  0, 210,  60)
        if d >= LIDAR_MODERATE_MM:   return (220, 220,   0)
        if d >= LIDAR_SLOW_MM:       return (255, 130,   0)
        return (255,  50,  50)

    def render(self, distances, clearances, reactive_steer,
               zone_label, front_dist, behavior_name,
               orange_balance=0.0, orange_area=0):
        s = self._surface
        s.fill((12, 12, 22))
        self._draw_grid(s)
        self._draw_rings(s)
        self._draw_sector_tints(s, clearances)
        self._draw_scan(s, distances)
        self._draw_car(s)
        self._draw_steer_arrow(s, reactive_steer)
        self._draw_orange_bar(s, orange_balance, orange_area)
        self._draw_hud(s, zone_label, front_dist, behavior_name,
                       reactive_steer, clearances, orange_balance, orange_area)
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
        for mm, col in [
            ( 400, ( 70,  30,  30)),
            ( 800, ( 90,  55,  20)),
            (1500, ( 30,  75,  30)),
        ]:
            pygame.draw.circle(s, col, (cx, cy), int(mm * MAP_SCALE), 1)

    @staticmethod
    def _draw_sector_tints(s, clearances):
        cx, cy   = LidarMapDisplay._CX, LidarMapDisplay._CY
        ARC_SEGS = 10
        for name, deg_lo, deg_hi in SECTORS:
            dist = clearances.get(name, float("inf"))
            if dist >= LIDAR_FULL_SPEED_MM:
                continue
            ratio = 1.0 - min(dist, float(LIDAR_FULL_SPEED_MM)) / float(LIDAR_FULL_SPEED_MM)
            glow  = int(ratio * 55)
            if dist < LIDAR_SLOW_MM:
                col = (glow * 4, 0, 0)
            elif dist < LIDAR_MODERATE_MM:
                col = (glow * 3, glow * 2, 0)
            else:
                col = (0, glow * 2, 0)
            max_r = max(5, int(min(dist, float(LIDAR_FULL_SPEED_MM)) * MAP_SCALE))
            pts   = [(cx, cy)]
            for k in range(ARC_SEGS + 1):
                deg = deg_lo + (deg_hi - deg_lo) * k / ARC_SEGS
                rad = math.radians(deg)
                pts.append((
                    cx + int(math.sin(rad) * max_r),
                    cy - int(math.cos(rad) * max_r),
                ))
            if len(pts) >= 3:
                pygame.draw.polygon(s, col, pts)

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
        cw, ch = 10, 18
        pygame.draw.rect(s, (150, 150, 255), (cx - cw // 2, cy - ch // 2, cw, ch))
        pygame.draw.polygon(s, (255, 255, 100), [
            (cx,      cy - ch // 2 - 10),
            (cx - 5,  cy - ch // 2),
            (cx + 5,  cy - ch // 2),
        ])

    @staticmethod
    def _draw_steer_arrow(s, steer):
        if abs(steer) < 0.04:
            return
        cx, cy   = LidarMapDisplay._CX, LidarMapDisplay._CY
        base_y   = cy + 38
        arrow_px = int(steer * 90)
        ex       = cx + arrow_px
        dx       = 1 if arrow_px > 0 else -1
        pygame.draw.line(s, (80, 200, 255), (cx, base_y), (ex, base_y), 3)
        pygame.draw.polygon(s, (80, 200, 255), [
            (ex,           base_y),
            (ex - dx * 10, base_y - 5),
            (ex - dx * 10, base_y + 5),
        ])

    @staticmethod
    def _draw_orange_bar(s, balance, area):
        """Horizontal balance bar: left = left wall dominant, right = right wall dominant."""
        if area < ORANGE_MIN_AREA:
            return
        bar_w  = 160
        bar_h  = 10
        bar_x  = MAP_W // 2 - bar_w // 2
        bar_y  = MAP_H - 30
        # background
        pygame.draw.rect(s, (40, 20, 0), (bar_x, bar_y, bar_w, bar_h))
        # fill proportional to balance
        fill   = int((balance + 1.0) / 2.0 * bar_w)
        fill   = max(0, min(bar_w, fill))
        pygame.draw.rect(s, (255, 140, 0), (bar_x, bar_y, fill, bar_h))
        # centre tick
        pygame.draw.line(s, (255, 255, 255),
                         (bar_x + bar_w // 2, bar_y),
                         (bar_x + bar_w // 2, bar_y + bar_h), 1)

    def _draw_hud(self, s, zone_label, front_dist, behavior_name,
                  reactive_steer, clearances, orange_balance, orange_area):
        zone_col = {
            "FULL":     (  0, 210,  60),
            "MODERATE": (220, 220,   0),
            "SLOW":     (255, 130,   0),
            "ESTOP":    (255,  50,  50),
        }.get(zone_label, (200, 200, 200))

        fd_str = f"{front_dist} mm" if front_dist else "-- mm"

        if orange_area >= ORANGE_MIN_AREA:
            side    = "RIGHT" if orange_balance > 0.1 else ("LEFT" if orange_balance < -0.1 else "CTR")
            org_str = f"{orange_balance:+.2f} {side} ({orange_area}px)"
            org_col = (255, 160, 30)
        else:
            org_str = "no detection"
            org_col = (80, 60, 30)

        y = 5
        for text, col in [
            (f"Zone:   {zone_label}",           zone_col),
            (f"Front:  {fd_str}",               (200, 200, 200)),
            (f"Mode:   {behavior_name}",         (80, 200, 255)),
            (f"Steer:  {reactive_steer:+.2f}",   (160, 200, 255)),
            (f"Orange: {org_str}",               org_col),
        ]:
            s.blit(self._font_md.render(text, True, col), (5, y))
            y += 18

        y = MAP_H - 50 - len(SECTORS) * 14
        for name, _, _ in SECTORS:
            d     = clearances.get(name, float("inf"))
            d_str = f"{int(d):5d} mm" if d < float("inf") else "  inf  "
            col   = self._dist_color(d) if d < float("inf") else (70, 70, 70)
            s.blit(self._font_sm.render(f"{name:<15}{d_str}", True, col), (5, y))
            y += 14


# ── MAP RENDER HELPER ────────────────────────────────────────

def maybe_render_map(map_display, lidar, clearances, reactive_steer,
                     zone_label, front_dist, behavior_name,
                     last_map, map_period,
                     orange_balance=0.0, orange_area=0):
    now = time.time()
    if map_display and lidar and (now - last_map) >= map_period:
        distances = lidar.get_distances()
        map_display.render(distances, clearances, reactive_steer,
                           zone_label, front_dist, behavior_name,
                           orange_balance, orange_area)
        return now
    return last_map


# ── MAIN ─────────────────────────────────────────────────────

def main():
    pygame.init()
    pygame.joystick.init()

    # LiDAR ──────────────────────────────────────────────────
    lidar = None
    if LIDAR_ENABLED:
        lidar = HokuyoLidar()
        try:
            lidar.connect()
            lidar.start()
        except Exception as e:
            print(f"[LIDAR] WARNING: could not connect — {e}")
            lidar = None

    # ZED camera ─────────────────────────────────────────────
    cam = None
    if CAMERA_ENABLED:
        cam = ZedCamera()
        try:
            cam.connect()
            cam.start()
        except Exception as e:
            print(f"[ZED]   WARNING: could not connect — {e}")
            cam = None

    # Map display ────────────────────────────────────────────
    map_display = None
    if MAP_ENABLED:
        try:
            map_display = LidarMapDisplay()
            print("[MAP]   2D display active (600×600)")
        except Exception as e:
            print(f"[MAP]   WARNING: display unavailable — {e}")

    # Joystick ───────────────────────────────────────────────
    joystick = None
    pygame.event.pump()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[PS4]   Connected: {joystick.get_name()}")
        print("         X / Circle = quit   Triangle = cycle mode")
    else:
        print("[PS4]   No controller — running fully autonomous")
        print("         Close window or Ctrl-C to quit")

    # VESC ───────────────────────────────────────────────────
    print(f"[VESC]  Opening {VESC_PORT}...")
    ser = serial.Serial(VESC_PORT, VESC_BAUDRATE, timeout=0.05, write_timeout=0.05)
    send_current_zero(ser)
    send_servo(ser, SERVO_CENTER)

    # Banner ─────────────────────────────────────────────────
    print()
    print("=" * 62)
    print("   REACTIVE AUTONOMOUS + ZED CAMERA — F1TENTH / ROBORACER")
    print("=" * 62)
    print(f"  Auto duty:  {AUTO_DRIVE_DUTY:.3f}  (max {MAX_DUTY:.3f})")
    print(f"  Zones: FULL>={LIDAR_FULL_SPEED_MM}mm  "
          f"MOD>={LIDAR_MODERATE_MM}mm  "
          f"SLOW>={LIDAR_SLOW_MM}mm  "
          f"ESTOP<{LIDAR_ESTOP_MM}mm")
    print(f"  Default behavior: {BEHAVIOR_NAMES[BEHAVIOR_DEFAULT]}")
    print()
    print(f"  LiDAR:  {'active  ' + LIDAR_PORT if lidar else 'NOT connected'}")
    print(f"  Camera: {'active  /dev/video' + str(ZED_DEVICE) if cam else 'NOT connected'}")
    print(f"  Map:    {'active (600×600)' if map_display else 'unavailable'}")
    print(f"  Orange weight: {CAMERA_STEER_WEIGHT:.2f}")
    print()
    print("  PLACE CAR IN A CLEAR SPACE.  VESC TOOL MUST BE CLOSED.")
    print("=" * 62)
    print()

    # State ──────────────────────────────────────────────────
    loop_period  = 1.0 / LOOP_HZ
    print_period = 1.0 / PRINT_HZ
    map_period   = 1.0 / 15
    last_print   = 0.0
    last_map     = 0.0

    lidar_estop     = False
    reverse_counter = 0
    hold_counter    = 0
    behavior_mode   = BEHAVIOR_DEFAULT
    warmup_counter  = 0

    current_duty   = 0.0
    current_steer  = 0.0
    reactive_steer = 0.0
    clearances     = {name: float("inf") for name, _, _ in SECTORS}
    zone_label     = "FULL"
    front_dist     = None
    speed_cap      = MAX_DUTY
    orange_balance = 0.0
    orange_area    = 0

    try:
        while True:
            loop_start = time.time()

            # Event handling ─────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button in (BTN_X, BTN_CIRCLE):
                        raise KeyboardInterrupt
                    elif event.button == BTN_TRIANGLE:
                        behavior_mode = (behavior_mode + 1) % len(BEHAVIOR_NAMES)
                        print(f"\n[MODE] Switched to {BEHAVIOR_NAMES[behavior_mode]}")
                if event.type == pygame.JOYDEVICEREMOVED:
                    joystick = None
                    print("[PS4]   Controller disconnected — continuing autonomous")

            # LiDAR processing ───────────────────────────────
            front_dist = None
            speed_cap  = MAX_DUTY
            zone_label = "FULL"
            distances  = []

            if lidar and lidar.connected:
                distances             = lidar.get_distances()
                front_dist            = lidar.front_min()
                speed_cap, zone_label = lidar_speed_zone(front_dist)
                clearances            = lidar.sector_clearances()

                lidar_steer = select_reactive_steer(
                    behavior_mode, distances, clearances)

                # Camera fusion: blend orange wall balance into steering ──
                reactive_steer, orange_balance, orange_area = fuse_camera_steer(
                    lidar_steer, cam)

                if zone_label == "ESTOP":
                    if not lidar_estop:
                        lidar_estop     = True
                        reverse_counter = BRAKE_LOOPS + REVERSE_LOOPS + SCAN_LOOPS
                        hold_counter    = 0
                        current_duty    = 0.0
                        print(f"\n[LIDAR] ESTOP — obstacle at {front_dist} mm — braking")
                elif lidar_estop:
                    path_clear = front_dist is None or front_dist >= LIDAR_ESTOP_CLEAR_MM
                    timed_out  = hold_counter >= HOLD_TIMEOUT_LOOPS
                    if reverse_counter == 0 and (path_clear or timed_out):
                        lidar_estop  = False
                        hold_counter = 0
                        reason = "path clear" if path_clear else "hold timeout — resuming on oval"
                        print(f"[LIDAR] {reason} ({front_dist} mm)")

            # Smooth the reactive steering output ────────────
            current_steer = ramp_value(current_steer, reactive_steer, STEER_RAMP_STEP)
            servo_pos     = reactive_steer_to_servo(current_steer)

            # Warmup: hold still while sensors settle ────────
            if warmup_counter < WARMUP_LOOPS:
                warmup_counter += 1
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                time.sleep(loop_period)
                continue

            # ESTOP sequence ─────────────────────────────────
            if lidar_estop:
                if reverse_counter > REVERSE_LOOPS + SCAN_LOOPS:
                    current_duty = 0.0
                    send_current_zero(ser)
                    send_servo(ser, SERVO_CENTER)
                    reverse_counter -= 1
                    estop_state_str = "BRAKE"

                elif reverse_counter > SCAN_LOOPS:
                    if reverse_counter == REVERSE_LOOPS + SCAN_LOOPS:
                        print(f"\n[ESTOP] Reversing straight ~2 ft")
                    send_duty(ser, -AUTO_DRIVE_DUTY)
                    send_servo(ser, SERVO_CENTER)
                    reverse_counter -= 1
                    estop_state_str = "REVERSE"

                elif reverse_counter > 0:
                    if reverse_counter == 1:
                        current_duty = 0.0
                        compute_parallel_escape(clearances)
                    send_current_zero(ser)
                    send_servo(ser, SERVO_CENTER)
                    reverse_counter -= 1
                    estop_state_str = "SCAN"

                else:
                    hold_counter += 1
                    if hold_counter >= HOLD_TIMEOUT_LOOPS:
                        lidar_estop  = False
                        hold_counter = 0
                        print(f"\n[ESTOP] Hold timeout — forcing resume")
                    current_duty = 0.0
                    send_current_zero(ser)
                    send_servo(ser, servo_pos)
                    estop_state_str = f"HOLD({hold_counter}/{HOLD_TIMEOUT_LOOPS})"

                now = time.time()
                if now - last_print >= print_period:
                    fd_str = f"{front_dist}mm" if front_dist else "--"
                    print(
                        f"\r[ESTOP/{estop_state_str}] Front:{fd_str}  "
                        f"Mode:{BEHAVIOR_NAMES[behavior_mode]}  "
                        f"Steer:{current_steer:+.2f}  "
                        f"Org:{orange_balance:+.2f}({orange_area}px)     ",
                        end=""
                    )
                    last_print = now

                last_map = maybe_render_map(
                    map_display, lidar, clearances, current_steer,
                    zone_label, front_dist, BEHAVIOR_NAMES[behavior_mode],
                    last_map, map_period, orange_balance, orange_area)

                elapsed = time.time() - loop_start
                if elapsed < loop_period:
                    time.sleep(loop_period - elapsed)
                continue

            # Autonomous forward drive ────────────────────────
            target_duty  = min(AUTO_DRIVE_DUTY, speed_cap)
            current_duty = ramp_value(current_duty, target_duty, DUTY_RAMP_STEP)

            send_servo(ser, servo_pos)
            if abs(current_duty) > 0.002:
                send_duty(ser, current_duty)
                mode_str = f"DRIVE[{zone_label}]"
            else:
                send_current_zero(ser)
                mode_str = "IDLE"

            now = time.time()
            if now - last_print >= print_period:
                fd_str = f"{front_dist}mm" if front_dist else "--"
                print(
                    f"\r[{mode_str}] "
                    f"Mode:{BEHAVIOR_NAMES[behavior_mode]}  "
                    f"Steer:{current_steer:+.2f}  "
                    f"Servo:{servo_pos:.2f}  "
                    f"Duty:{current_duty:+.3f}  "
                    f"Front:{fd_str}  "
                    f"Org:{orange_balance:+.2f}({orange_area}px)     ",
                    end=""
                )
                last_print = now

            last_map = maybe_render_map(
                map_display, lidar, clearances, current_steer,
                zone_label, front_dist, BEHAVIOR_NAMES[behavior_mode],
                last_map, map_period, orange_balance, orange_area)

            # ZED live feed window ────────────────────────────
            if cam and cam.connected:
                disp = cam.get_display_frame()
                if disp is not None:
                    cv2.imshow("ZED Camera — Orange Detection", disp)
                cv2.waitKey(1)

            elapsed = time.time() - loop_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)

    except KeyboardInterrupt:
        print("\n[INFO] Quitting...")

    finally:
        stop_car(ser)
        ser.close()
        if lidar:
            lidar.stop()
        if cam:
            cam.stop()
        cv2.destroyAllWindows()
        pygame.quit()
        print("[INFO] Closed safely")


if __name__ == "__main__":
    main()
