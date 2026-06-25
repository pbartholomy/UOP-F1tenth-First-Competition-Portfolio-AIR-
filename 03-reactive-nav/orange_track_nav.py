import struct
import time
import serial
import pygame
import threading
import math
import cv2
import numpy as np
import pyzed.sl as sl

# ============================================================
# orange_track_nav.py — LiDAR + ZED 2i Autonomous Track Navigation
# Roboracer / F1TENTH  —  Hokuyo URG-04LX + ZED 2i + VESC 6 MkVI
#
# Navigation strategy:
#   LiDAR  — Follow-the-Gap (primary obstacle avoidance + speed zones)
#   ZED 2i — Orange border detection via pyzed SDK
#             Depth-aware left/right balance nudges the LiDAR steer
#             to keep the car centered between orange track borders
#
# Controls (PS4):
#   L1          = KILL SWITCH  (hold to stop, release to resume)
#   Triangle    = cycle behavior mode
#   Circle / X  = quit program
#   Ctrl-C      = quit program
#
# Speed zones (forward LiDAR):
#   >= 1000 mm  FULL      (AUTO_DRIVE_DUTY)
#    500–1000   MODERATE
#    300– 500   SLOW
#    < 200 mm   ESTOP     (brake → reverse → scan → hold)
# ============================================================

# ── PORTS ───────────────────────────────────────────────────
# Verify with: ls -l /dev/ttyACM*  before running
VESC_PORT      = "/dev/ttyACM1"
LIDAR_PORT     = "/dev/ttyACM0"
VESC_BAUDRATE  = 115200
LIDAR_BAUDRATE = 115200 # 19200 intially

# ── VESC ────────────────────────────────────────────────────
MAX_DUTY        = 0.12
AUTO_DRIVE_DUTY = 0.10
LOOP_HZ         = 50
DUTY_RAMP_STEP  = 0.005
MODERATE_DUTY   = 0.09
SLOW_DUTY       = 0.07

# ── SERVO ───────────────────────────────────────────────────
SERVO_CENTER    = 0.50
SERVO_MIN       = 0.15
SERVO_MAX       = 0.85
INVERT_STEERING = False
STEER_RAMP_STEP = 0.03

# ── PS4 BUTTON MAP ──────────────────────────────────────────
BTN_X        = 0    # quit
BTN_CIRCLE   = 1    # quit
BTN_TRIANGLE = 3    # cycle mode
BTN_L1       = 9    # KILL SWITCH — hold to stop

# ── VESC COMMAND IDs ────────────────────────────────────────
COMM_SET_DUTY      = 5
COMM_SET_CURRENT   = 6
COMM_SET_RPM       = 8
COMM_SET_SERVO_POS = 12

# ── HOKUYO URG-04LX ─────────────────────────────────────────
LIDAR_STEP_MIN     = 44
LIDAR_STEP_MAX     = 725
LIDAR_STEP_FRONT   = 384
LIDAR_FRONT_WINDOW = 50

LIDAR_FULL_SPEED_MM  = 1000
LIDAR_MODERATE_MM    = 500
LIDAR_SLOW_MM        = 300
LIDAR_ESTOP_MM       = 200
LIDAR_ESTOP_CLEAR_MM = 400

BRAKE_LOOPS   = 20
REVERSE_LOOPS = 40
SCAN_LOOPS    = 20
HOLD_TIMEOUT  = 100

# ── FOLLOW-THE-GAP ──────────────────────────────────────────
CAR_WIDTH_MM         = 220
FTG_DISPARITY_MM     = 500
FTG_EXTRA_SAMPLES    = 5
FTG_SAFETY_FACTOR    = 1.5
SIDE_SAFETY_MM       = 200

# ── ZED 2i ──────────────────────────────────────────────────
# Serial 36476709 — Bus 2 port 1.3 — /dev/video0
ZED_RESOLUTION = sl.RESOLUTION.HD720
ZED_FPS        = 30
ZED_DEPTH_MODE = sl.DEPTH_MODE.NEURAL

# Orange HSV thresholds (two bands — orange wraps near hue=0)
ORANGE_LO_1 = np.array([  0, 150,  80], dtype=np.uint8)
ORANGE_HI_1 = np.array([ 15, 255, 255], dtype=np.uint8)
ORANGE_LO_2 = np.array([160, 150,  80], dtype=np.uint8)
ORANGE_HI_2 = np.array([180, 255, 255], dtype=np.uint8)

ORANGE_MIN_AREA = 800    # px² — ignore tiny glints
# Ignore top 15 % (sky/distance) and bottom 20 % (floor reflection)
ORANGE_ROI_TOP    = 0.15
ORANGE_ROI_BOTTOM = 0.80

# How much camera balance nudges the LiDAR steer (0 = off, 1 = full override)
CAMERA_STEER_WEIGHT = 0.30

# ── REACTIVE BEHAVIORS ──────────────────────────────────────
BEHAVIOR_GAP_FOLLOW   = 0
BEHAVIOR_CORRIDOR_CTR = 1
BEHAVIOR_WALL_FOLLOW  = 2
BEHAVIOR_NAMES        = ["GAP_FOLLOW", "CORRIDOR_CTR", "WALL_FOLLOW"]

WALL_TARGET_MM = 500

# ── 2D LIDAR MAP ────────────────────────────────────────────
MAP_W     = 600
MAP_H     = 600
MAP_SCALE = 0.10

PRINT_HZ    = 8
WARMUP_LOOPS = 15

# ── PRECOMPUTED TABLES ──────────────────────────────────────
_N_STEPS      = LIDAR_STEP_MAX - LIDAR_STEP_MIN + 1
_CENTER_IDX   = LIDAR_STEP_FRONT - LIDAR_STEP_MIN
_STEPS_PER_90 = int(90.0 * 1024 / 360)
_SCAN_RAD     = 2.0 * math.pi / 1024

SECTORS = [
    ("FAR_LEFT",     -90, -45),
    ("LEFT",         -45, -15),
    ("CENTER_LEFT",  -15,  -5),
    ("CENTER",        -5,   5),
    ("CENTER_RIGHT",   5,  15),
    ("RIGHT",         15,  45),
    ("FAR_RIGHT",     45,  90),
]


def _build_tables():
    sins, coss, masks = [], [], {n: [] for n, _, _ in SECTORS}
    for idx in range(_N_STEPS):
        step = idx + LIDAR_STEP_MIN
        a = (step - LIDAR_STEP_FRONT) * _SCAN_RAD
        sins.append(math.sin(a))
        coss.append(math.cos(a))
        adeg = (step - LIDAR_STEP_FRONT) * (360.0 / 1024)
        for name, lo, hi in SECTORS:
            if lo <= adeg <= hi:
                masks[name].append(idx)
    return sins, coss, masks


_SIN, _COS, _SECTOR_MASKS = _build_tables()


# ── HOKUYO LIDAR ────────────────────────────────────────────

class HokuyoLidar:
    def __init__(self):
        self._ser       = None
        self._lock      = threading.Lock()
        self._distances = []
        self._running   = False
        self._thread    = None
        self.connected  = False

    def connect(self):
        self._ser = serial.Serial(LIDAR_PORT, LIDAR_BAUDRATE, timeout=1.0)
        time.sleep(0.2)
        self._ser.write(b"SCIP2.0\n"); time.sleep(0.2)
        self._ser.reset_input_buffer()
        self._ser.write(b"BM\n");      time.sleep(0.2)
        self._ser.reset_input_buffer()
        self.connected = True
        print(f"[LIDAR] Connected on {LIDAR_PORT}")

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
        out = []
        for i in range(0, len(raw) - 2, 3):
            v = ((raw[i] - 0x30) << 12) | ((raw[i+1] - 0x30) << 6) | (raw[i+2] - 0x30)
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
        zone = sorted(x for x in d[lo:hi+1] if x > 20)
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


# ── ZED 2i CAMERA ───────────────────────────────────────────

class ZED2iCamera:
    """
    Runs pyzed in a background thread.
    Exposes:
      orange_balance  — [-1, +1]  negative=left dominant, positive=right dominant
      orange_area     — total orange px (0 if below threshold)
      nearest_orange  — depth in meters to the closest orange region (nan if none)
      display_frame   — annotated left BGR image for the live window
    """

    def __init__(self):
        self._cam     = sl.Camera()
        self._lock    = threading.Lock()
        self._running = False
        self._thread  = None
        self.connected = False

        self._orange_balance = 0.0
        self._orange_area    = 0
        self._nearest_orange = float("nan")
        self._display_frame  = None

    def connect(self):
        init = sl.InitParameters()
        init.camera_resolution = ZED_RESOLUTION
        init.camera_fps        = ZED_FPS
        init.depth_mode        = ZED_DEPTH_MODE
        init.coordinate_units  = sl.UNIT.METER

        status = self._cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED open failed: {status}")

        info = self._cam.get_camera_information()
        print(f"[ZED2i] {info.camera_model}  S/N {info.serial_number}  "
              f"FW {info.camera_configuration.firmware_version}")
        self.connected = True

    def _detect_orange(self, bgr, depth_raw):
        h, w = bgr.shape[:2]

        hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, ORANGE_LO_1, ORANGE_HI_1) | \
               cv2.inRange(hsv, ORANGE_LO_2, ORANGE_HI_2)

        # Blank out sky/floor rows
        mask[:int(h * ORANGE_ROI_TOP),    :] = 0
        mask[int(h * ORANGE_ROI_BOTTOM):, :] = 0

        # Clean up noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        total = int(np.count_nonzero(mask))

        if total < ORANGE_MIN_AREA:
            balance = 0.0
            total   = 0
            nearest = float("nan")
        else:
            left_px  = int(np.count_nonzero(mask[:, :w // 2]))
            right_px = int(np.count_nonzero(mask[:, w // 2:]))
            balance  = float(right_px - left_px) / float(total)

            # Nearest depth in the orange mask
            orange_depths = depth_raw[mask > 0]
            valid = orange_depths[np.isfinite(orange_depths)]
            nearest = float(np.min(valid)) if valid.size > 0 else float("nan")

        # Annotated display frame
        display  = bgr.copy()
        overlay  = np.zeros_like(display)
        overlay[mask > 0] = (0, 140, 255)
        display  = cv2.addWeighted(display, 0.75, overlay, 0.25, 0)
        cv2.line(display, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)
        side  = "RIGHT" if balance > 0.1 else ("LEFT" if balance < -0.1 else "CTR")
        near_str = f"{nearest:.2f}m" if np.isfinite(nearest) else "--"
        cv2.putText(display,
                    f"Orange: {balance:+.2f} {side}  nearest={near_str}  ({total}px)",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2, cv2.LINE_AA)

        return balance, total, nearest, display

    def _run(self):
        runtime   = sl.RuntimeParameters()
        image_mat = sl.Mat()
        depth_mat = sl.Mat()

        while self._running:
            if self._cam.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            self._cam.retrieve_image(image_mat,   sl.VIEW.LEFT)
            self._cam.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

            bgr       = cv2.cvtColor(image_mat.get_data(), cv2.COLOR_BGRA2BGR)
            depth_raw = depth_mat.get_data()

            try:
                balance, area, nearest, disp = self._detect_orange(bgr, depth_raw)
            except Exception:
                continue

            with self._lock:
                self._orange_balance = balance
                self._orange_area    = area
                self._nearest_orange = nearest
                self._display_frame  = disp

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def get_orange(self):
        with self._lock:
            return self._orange_balance, self._orange_area, self._nearest_orange

    def get_display_frame(self):
        with self._lock:
            return self._display_frame.copy() if self._display_frame is not None else None

    def stop(self):
        self._running = False
        if self._cam:
            self._cam.close()
        self.connected = False
        print("[ZED2i] Disconnected")


# ── VESC HELPERS ────────────────────────────────────────────

def crc16(data):
    crc = 0x0000
    for b in data:
        crc ^= b << 8
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


def send_servo(ser, pos):
    pos = max(SERVO_MIN, min(SERVO_MAX, pos))
    send_packet(ser, bytes([COMM_SET_SERVO_POS]) + struct.pack(">h", int(pos * 1000)))


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


# ── SPEED ZONE ──────────────────────────────────────────────

def lidar_speed_zone(front_dist):
    if front_dist is None:
        return AUTO_DRIVE_DUTY, "FULL"
    if front_dist >= LIDAR_FULL_SPEED_MM:
        return AUTO_DRIVE_DUTY, "FULL"
    if front_dist >= LIDAR_MODERATE_MM:
        return MODERATE_DUTY,   "MODERATE"
    if front_dist >= LIDAR_SLOW_MM:
        return SLOW_DUTY,       "SLOW"
    return 0.0, "ESTOP"


# ── FOLLOW-THE-GAP ──────────────────────────────────────────

def preprocess_ftg(distances):
    if not distances:
        return []
    ranges = list(distances)
    last_r = 0
    skip   = False
    skip_n = 0

    for idx, r in enumerate(ranges):
        if idx == 0:
            last_r = r
            continue

        if skip:
            last_r = r
            skip_n -= 1
            if skip_n == -1:
                skip = False
            continue

        if r <= 20 or last_r <= 20:
            last_r = r
            continue

        r_m, l_m = r / 1000.0, last_r / 1000.0
        if abs(r_m - l_m) > FTG_DISPARITY_MM / 1000.0:
            try:
                half = math.asin(min(1.0, (CAR_WIDTH_MM / 1000.0 * FTG_SAFETY_FACTOR)
                                          / min(r_m, l_m)))
                skip_n = round(half / _SCAN_RAD) + FTG_EXTRA_SAMPLES
            except (ValueError, ZeroDivisionError):
                skip_n = FTG_EXTRA_SAMPLES

            closer  = min(r, last_r)
            samples = (range(idx, min(len(ranges), idx + skip_n + 1))
                       if r > last_r else
                       range(max(0, idx - skip_n), idx))
            for j in samples:
                if ranges[j] > closer:
                    ranges[j] = closer
            skip = True

        last_r = r
    return ranges


def compute_gap_follow(distances, clearances):
    if not distances:
        return 0.0
    processed = preprocess_ftg(distances)
    lo  = max(0, _CENTER_IDX - _STEPS_PER_90)
    hi  = min(len(processed) - 1, _CENTER_IDX + _STEPS_PER_90)
    sub = processed[lo:hi + 1]
    if not sub:
        return 0.0
    gap_idx = sub.index(max(sub)) + lo
    steer   = max(-1.0, min(1.0, -(gap_idx - _CENTER_IDX) / float(_STEPS_PER_90)))

    # Side safety override
    if clearances.get("FAR_LEFT",  float("inf")) < SIDE_SAFETY_MM:
        steer = -0.25
    elif clearances.get("FAR_RIGHT", float("inf")) < SIDE_SAFETY_MM:
        steer =  0.25
    return steer


def compute_corridor_ctr(clearances):
    left_d  = min(clearances.get("LEFT",        float("inf")),
                  clearances.get("CENTER_LEFT",  float("inf")))
    right_d = min(clearances.get("RIGHT",        float("inf")),
                  clearances.get("CENTER_RIGHT", float("inf")))
    cap     = float(LIDAR_MODERATE_MM)
    left_d  = min(left_d,  cap) if left_d  < float("inf") else cap
    right_d = min(right_d, cap) if right_d < float("inf") else cap
    denom   = left_d + right_d
    return max(-1.0, min(1.0, (left_d - right_d) / denom)) if denom > 1 else 0.0


def compute_wall_follow(clearances):
    right_d = min(clearances.get("RIGHT",     float("inf")),
                  clearances.get("FAR_RIGHT", float("inf")))
    if right_d == float("inf"):
        right_d = WALL_TARGET_MM * 2.5
    return max(-1.0, min(1.0, -(right_d - WALL_TARGET_MM) / float(WALL_TARGET_MM)))


def select_steer(mode, distances, clearances):
    if mode == BEHAVIOR_GAP_FOLLOW:
        return compute_gap_follow(distances, clearances)
    if mode == BEHAVIOR_CORRIDOR_CTR:
        return compute_corridor_ctr(clearances)
    return compute_wall_follow(clearances)


def steer_to_servo(steer):
    x = -steer if INVERT_STEERING else steer
    return max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + x * 0.50))


# ── CAMERA + LIDAR FUSION ───────────────────────────────────

def fuse_steer(lidar_steer, cam):
    """
    Blend ZED orange balance into the LiDAR steer.
    More orange on the right  → car drifting right → nudge left (negative).
    More orange on the left   → car drifting left  → nudge right (positive).
    """
    if cam is None or not cam.connected:
        return lidar_steer, 0.0, 0, float("nan")

    balance, area, nearest = cam.get_orange()
    if area < ORANGE_MIN_AREA:
        return lidar_steer, 0.0, 0, float("nan")

    correction = -balance * CAMERA_STEER_WEIGHT
    fused      = max(-1.0, min(1.0, lidar_steer + correction))
    return fused, balance, area, nearest


# ── 2D LIDAR MAP ────────────────────────────────────────────

class LidarMap:
    _CX = MAP_W // 2
    _CY = MAP_H // 2

    def __init__(self):
        self._surf   = pygame.display.set_mode((MAP_W, MAP_H))
        self._font_s = pygame.font.SysFont("monospace", 12)
        self._font_m = pygame.font.SysFont("monospace", 14, bold=True)
        pygame.display.set_caption("LiDAR + ZED 2i  —  Orange Track Nav")

    @staticmethod
    def _w2s(x, y):
        return (LidarMap._CX + int(x * MAP_SCALE),
                LidarMap._CY - int(y * MAP_SCALE))

    @staticmethod
    def _dist_col(d):
        if d >= LIDAR_FULL_SPEED_MM: return (  0, 210,  60)
        if d >= LIDAR_MODERATE_MM:   return (220, 220,   0)
        if d >= LIDAR_SLOW_MM:       return (255, 130,   0)
        return (255, 50, 50)

    def render(self, distances, clearances, steer, zone,
               front_dist, mode_name, kill_active,
               orange_balance, orange_area, nearest_orange):
        s = self._surf
        s.fill((12, 12, 22))
        self._grid(s)
        self._rings(s)
        self._sector_tints(s, clearances)
        self._scan(s, distances)
        self._car(s)
        self._steer_arrow(s, steer)
        self._orange_bar(s, orange_balance, orange_area)
        self._hud(s, zone, front_dist, mode_name, steer,
                  clearances, kill_active,
                  orange_balance, orange_area, nearest_orange)
        pygame.display.flip()

    @staticmethod
    def _grid(s):
        step = max(1, int(500 * MAP_SCALE))
        for x in range(0, MAP_W, step):
            pygame.draw.line(s, (28, 28, 45), (x, 0), (x, MAP_H))
        for y in range(0, MAP_H, step):
            pygame.draw.line(s, (28, 28, 45), (0, y), (MAP_W, y))
        cx, cy = LidarMap._CX, LidarMap._CY
        pygame.draw.line(s, (55, 55, 85), (cx, 0), (cx, MAP_H))
        pygame.draw.line(s, (55, 55, 85), (0, cy), (MAP_W, cy))

    @staticmethod
    def _rings(s):
        cx, cy = LidarMap._CX, LidarMap._CY
        for mm, col in [(400, (70,30,30)), (800, (90,55,20)), (1500, (30,75,30))]:
            pygame.draw.circle(s, col, (cx, cy), int(mm * MAP_SCALE), 1)

    @staticmethod
    def _sector_tints(s, clearances):
        cx, cy = LidarMap._CX, LidarMap._CY
        for name, d_lo, d_hi in SECTORS:
            dist = clearances.get(name, float("inf"))
            if dist >= LIDAR_FULL_SPEED_MM:
                continue
            ratio = 1.0 - min(dist, float(LIDAR_FULL_SPEED_MM)) / float(LIDAR_FULL_SPEED_MM)
            glow  = int(ratio * 55)
            col   = ((glow*4, 0, 0) if dist < LIDAR_SLOW_MM else
                     (glow*3, glow*2, 0) if dist < LIDAR_MODERATE_MM else
                     (0, glow*2, 0))
            max_r = max(5, int(min(dist, float(LIDAR_FULL_SPEED_MM)) * MAP_SCALE))
            pts   = [(cx, cy)]
            for k in range(11):
                deg = d_lo + (d_hi - d_lo) * k / 10
                rad = math.radians(deg)
                pts.append((cx + int(math.sin(rad)*max_r),
                             cy - int(math.cos(rad)*max_r)))
            if len(pts) >= 3:
                pygame.draw.polygon(s, col, pts)

    @staticmethod
    def _scan(s, distances):
        for idx, dist in enumerate(distances):
            if idx >= len(_SIN) or dist <= 20 or dist > 5500:
                continue
            sx, sy = LidarMap._w2s(dist * _SIN[idx], dist * _COS[idx])
            if 0 <= sx < MAP_W and 0 <= sy < MAP_H:
                pygame.draw.circle(s, LidarMap._dist_col(dist), (sx, sy), 2)

    @staticmethod
    def _car(s):
        cx, cy = LidarMap._CX, LidarMap._CY
        pygame.draw.rect(s, (150, 150, 255), (cx-5, cy-9, 10, 18))
        pygame.draw.polygon(s, (255, 255, 100),
                            [(cx, cy-19), (cx-5, cy-9), (cx+5, cy-9)])

    @staticmethod
    def _steer_arrow(s, steer):
        if abs(steer) < 0.04:
            return
        cx, cy   = LidarMap._CX, LidarMap._CY
        base_y   = cy + 38
        arrow_px = int(steer * 90)
        ex       = cx + arrow_px
        dx       = 1 if arrow_px > 0 else -1
        pygame.draw.line(s, (80, 200, 255), (cx, base_y), (ex, base_y), 3)
        pygame.draw.polygon(s, (80, 200, 255),
                            [(ex, base_y),
                             (ex - dx*10, base_y - 5),
                             (ex - dx*10, base_y + 5)])

    @staticmethod
    def _orange_bar(s, balance, area):
        if area < ORANGE_MIN_AREA:
            return
        bw, bh = 160, 10
        bx = MAP_W // 2 - bw // 2
        by = MAP_H - 30
        pygame.draw.rect(s, (40, 20, 0), (bx, by, bw, bh))
        fill = max(0, min(bw, int((balance + 1.0) / 2.0 * bw)))
        pygame.draw.rect(s, (255, 140, 0), (bx, by, fill, bh))
        pygame.draw.line(s, (255,255,255),
                         (bx + bw//2, by), (bx + bw//2, by + bh), 1)

    def _hud(self, s, zone, front_dist, mode_name, steer,
             clearances, kill_active,
             orange_balance, orange_area, nearest_orange):
        zone_col = {"FULL":(0,210,60), "MODERATE":(220,220,0),
                    "SLOW":(255,130,0), "ESTOP":(255,50,50)}.get(zone, (200,200,200))
        kill_col = (255, 50, 50) if kill_active else (60, 60, 60)
        fd_str   = f"{front_dist} mm" if front_dist else "-- mm"

        if orange_area >= ORANGE_MIN_AREA:
            side    = "RIGHT" if orange_balance > 0.1 else ("LEFT" if orange_balance < -0.1 else "CTR")
            near_s  = f"{nearest_orange:.2f}m" if np.isfinite(nearest_orange) else "--"
            org_str = f"{orange_balance:+.2f} {side}  d={near_s}"
            org_col = (255, 160, 30)
        else:
            org_str = "no detection"
            org_col = (80, 60, 30)

        y = 5
        for text, col in [
            (f"Zone:   {zone}",              zone_col),
            (f"Front:  {fd_str}",            (200, 200, 200)),
            (f"Mode:   {mode_name}",          (80,  200, 255)),
            (f"Steer:  {steer:+.2f}",         (160, 200, 255)),
            (f"Orange: {org_str}",            org_col),
            (f"KILL:   {'ACTIVE' if kill_active else 'off'}", kill_col),
        ]:
            s.blit(self._font_m.render(text, True, col), (5, y))
            y += 18

        y = MAP_H - 50 - len(SECTORS) * 14
        for name, _, _ in SECTORS:
            d     = clearances.get(name, float("inf"))
            d_str = f"{int(d):5d} mm" if d < float("inf") else "  inf  "
            col   = self._dist_col(d) if d < float("inf") else (70, 70, 70)
            s.blit(self._font_s.render(f"{name:<15}{d_str}", True, col), (5, y))
            y += 14


# ── MAIN ─────────────────────────────────────────────────────

def main():
    pygame.init()
    pygame.joystick.init()

    # LiDAR ──────────────────────────────────────────────────
    lidar = HokuyoLidar()
    try:
        lidar.connect()
        lidar.start()
    except Exception as e:
        print(f"[LIDAR] WARNING: could not connect — {e}")
        lidar = None

    # ZED 2i ─────────────────────────────────────────────────
    cam = ZED2iCamera()
    try:
        cam.connect()
        cam.start()
    except Exception as e:
        print(f"[ZED2i] WARNING: could not connect — {e}")
        cam = None

    # LiDAR map ──────────────────────────────────────────────
    lidar_map = None
    try:
        lidar_map = LidarMap()
        print("[MAP]   2D LiDAR display active")
    except Exception as e:
        print(f"[MAP]   WARNING: display unavailable — {e}")

    # PS4 controller ─────────────────────────────────────────
    joystick = None
    pygame.event.pump()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[PS4]   Connected: {joystick.get_name()}")
        print("         L1=KILL SWITCH  Triangle=cycle mode  X/Circle=quit")
    else:
        print("[PS4]   No controller — running fully autonomous")
        print("         Ctrl-C to quit")

    # VESC ───────────────────────────────────────────────────
    print(f"[VESC]  Opening {VESC_PORT}...")
    ser = serial.Serial(VESC_PORT, VESC_BAUDRATE, timeout=0.05, write_timeout=0.1)
    time.sleep(1.0)   # VESC needs a moment to be ready after port opens
    send_current_zero(ser)
    send_servo(ser, SERVO_CENTER)

    print()
    print("=" * 64)
    print("   ORANGE TRACK NAV — LiDAR + ZED 2i  (F1TENTH / ROBORACER)")
    print("=" * 64)
    print(f"  Auto duty : {AUTO_DRIVE_DUTY:.3f}  (max {MAX_DUTY:.3f})")
    print(f"  Zones     : FULL>={LIDAR_FULL_SPEED_MM}mm  MOD>={LIDAR_MODERATE_MM}mm  "
          f"SLOW>={LIDAR_SLOW_MM}mm  ESTOP<{LIDAR_ESTOP_MM}mm")
    print(f"  Camera wt : {CAMERA_STEER_WEIGHT:.2f}")
    print(f"  LiDAR     : {'active  ' + LIDAR_PORT if lidar else 'NOT connected'}")
    print(f"  ZED 2i    : {'active' if cam and cam.connected else 'NOT connected'}")
    print()
    print("  PLACE CAR IN A CLEAR SPACE.  VESC TOOL MUST BE CLOSED.")
    print("=" * 64)
    print()

    loop_period  = 1.0 / LOOP_HZ
    print_period = 1.0 / PRINT_HZ
    map_period   = 1.0 / 15
    last_print   = 0.0
    last_map     = 0.0

    kill_active     = False
    lidar_estop     = False
    reverse_counter = 0
    hold_counter    = 0
    behavior_mode   = BEHAVIOR_GAP_FOLLOW
    warmup_counter  = 0

    current_duty   = 0.0
    current_steer  = 0.0
    reactive_steer = 0.0
    clearances     = {n: float("inf") for n, _, _ in SECTORS}
    zone_label     = "FULL"
    front_dist     = None
    speed_cap      = AUTO_DRIVE_DUTY
    orange_balance = 0.0
    orange_area    = 0
    nearest_orange = float("nan")

    # Pre-create the ZED window so it appears immediately
    if cam and cam.connected:
        cv2.namedWindow("ZED 2i — Orange Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("ZED 2i — Orange Detection", 960, 540)

    try:
        while True:
            loop_start = time.time()

            # Events ─────────────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button in (BTN_X, BTN_CIRCLE):
                        raise KeyboardInterrupt
                    elif event.button == BTN_TRIANGLE:
                        behavior_mode = (behavior_mode + 1) % len(BEHAVIOR_NAMES)
                        print(f"\n[MODE] → {BEHAVIOR_NAMES[behavior_mode]}")
                    elif event.button == BTN_L1:
                        kill_active  = True
                        current_duty = 0.0
                        stop_car(ser)
                        print("\n[KILL] L1 held — car stopped")

                if event.type == pygame.JOYBUTTONUP:
                    if event.button == BTN_L1:
                        kill_active = False
                        print("[KILL] L1 released — resuming")

                if event.type == pygame.JOYDEVICEREMOVED:
                    joystick    = None
                    kill_active = False
                    print("[PS4]  Disconnected — continuing autonomous")

            # Kill switch active ──────────────────────────────
            if kill_active:
                current_duty = 0.0
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                if cam and cam.connected:
                    frame = cam.get_display_frame()
                    if frame is not None:
                        cv2.imshow("ZED 2i — Orange Detection", frame)
                    cv2.waitKey(1)
                time.sleep(loop_period)
                continue

            # LiDAR ───────────────────────────────────────────
            distances  = []
            front_dist = None
            speed_cap  = AUTO_DRIVE_DUTY
            zone_label = "FULL"

            if lidar and lidar.connected:
                distances             = lidar.get_distances()
                front_dist            = lidar.front_min()
                speed_cap, zone_label = lidar_speed_zone(front_dist)
                clearances            = lidar.sector_clearances()

                lidar_steer = select_steer(behavior_mode, distances, clearances)

                # ZED camera fusion ──────────────────────────
                reactive_steer, orange_balance, orange_area, nearest_orange = \
                    fuse_steer(lidar_steer, cam)

                if zone_label == "ESTOP" and not lidar_estop:
                    lidar_estop     = True
                    reverse_counter = BRAKE_LOOPS + REVERSE_LOOPS + SCAN_LOOPS
                    hold_counter    = 0
                    current_duty    = 0.0
                    print(f"\n[LIDAR] ESTOP — obstacle at {front_dist} mm")
                elif lidar_estop and zone_label != "ESTOP":
                    path_clear = front_dist is None or front_dist >= LIDAR_ESTOP_CLEAR_MM
                    if reverse_counter == 0 and (path_clear or hold_counter >= HOLD_TIMEOUT):
                        lidar_estop  = False
                        hold_counter = 0
                        print(f"[LIDAR] Path clear ({front_dist} mm) — resuming")

            # Smooth steer ────────────────────────────────────
            current_steer = ramp_value(current_steer, reactive_steer, STEER_RAMP_STEP)
            servo_pos     = steer_to_servo(current_steer)

            # Warmup ──────────────────────────────────────────
            if warmup_counter < WARMUP_LOOPS:
                warmup_counter += 1
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                if cam and cam.connected:
                    frame = cam.get_display_frame()
                    if frame is not None:
                        cv2.imshow("ZED 2i — Orange Detection", frame)
                    cv2.waitKey(1)
                time.sleep(loop_period)
                continue

            # ESTOP sequence: brake → reverse → scan → hold ──
            if lidar_estop:
                if reverse_counter > REVERSE_LOOPS + SCAN_LOOPS:
                    send_current_zero(ser)
                    send_servo(ser, SERVO_CENTER)
                    current_duty    = 0.0
                    reverse_counter -= 1
                    estop_str = "BRAKE"

                elif reverse_counter > SCAN_LOOPS:
                    send_duty(ser, -AUTO_DRIVE_DUTY)
                    send_servo(ser, SERVO_CENTER)
                    reverse_counter -= 1
                    estop_str = "REVERSE"

                elif reverse_counter > 0:
                    send_current_zero(ser)
                    send_servo(ser, SERVO_CENTER)
                    reverse_counter -= 1
                    estop_str = "SCAN"

                else:
                    hold_counter += 1
                    if hold_counter >= HOLD_TIMEOUT:
                        lidar_estop  = False
                        hold_counter = 0
                        print("\n[ESTOP] Hold timeout — forcing resume")
                    send_current_zero(ser)
                    send_servo(ser, servo_pos)
                    estop_str = f"HOLD({hold_counter}/{HOLD_TIMEOUT})"

                now = time.time()
                if now - last_print >= print_period:
                    fd = f"{front_dist}mm" if front_dist else "--"
                    print(f"\r[ESTOP/{estop_str}] Front:{fd}  "
                          f"Mode:{BEHAVIOR_NAMES[behavior_mode]}  "
                          f"Steer:{current_steer:+.2f}  "
                          f"Org:{orange_balance:+.2f}({orange_area}px)     ", end="")
                    last_print = now

            else:
                # Normal autonomous drive ─────────────────────
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
                    fd   = f"{front_dist}mm" if front_dist else "--"
                    near = f"{nearest_orange:.2f}m" if np.isfinite(nearest_orange) else "--"
                    print(f"\r[{mode_str}] "
                          f"Mode:{BEHAVIOR_NAMES[behavior_mode]}  "
                          f"Steer:{current_steer:+.2f}  "
                          f"Servo:{servo_pos:.2f}  "
                          f"Duty:{current_duty:+.3f}  "
                          f"Front:{fd}  "
                          f"Org:{orange_balance:+.2f}({orange_area}px) near={near}     ",
                          end="")
                    last_print = now

            # LiDAR map ──────────────────────────────────────
            now = time.time()
            if lidar_map and lidar and (now - last_map) >= map_period:
                lidar_map.render(
                    lidar.get_distances(), clearances, current_steer,
                    zone_label, front_dist, BEHAVIOR_NAMES[behavior_mode],
                    kill_active, orange_balance, orange_area, nearest_orange)
                last_map = now

            # ZED live window ────────────────────────────────
            if cam and cam.connected:
                frame = cam.get_display_frame()
                if frame is not None:
                    cv2.imshow("ZED 2i — Orange Detection", frame)
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
