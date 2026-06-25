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
    from scipy.spatial import KDTree
    SCIPY_AVAILABLE = True
except ImportError:
    KDTree = None
    SCIPY_AVAILABLE = False

try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
except Exception:
    sl = None
    ZED_AVAILABLE = False

# ── STABLE PORTS ─────────────────────────────────────────────
VESC_PORT  = "/dev/serial/by-id/usb-STMicroelectronics_ChibiOS_RT_Virtual_COM_Port_304-if00"
LIDAR_PORT = "/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00"
VESC_BAUDRATE  = 115200
LIDAR_BAUDRATE = 19200

# ── VESC / SPEED ─────────────────────────────────────────────
INVERT_MOTOR       = True   # set True if positive duty drives backward
MAX_DUTY           = 0.11
AUTO_DRIVE_DUTY    = 0.072   # was 0.065 — testrun26 smooth, raising speed ~10%
# All caps strictly below AUTO_DRIVE_DUTY so speed zones actually slow the car
MODERATE_DUTY_CAP  = 0.069   # was 0.063
SLOW_DUTY_CAP      = 0.065   # was 0.060
CRAWL_DUTY_CAP     = 0.061   # was 0.057
MIN_EFFECTIVE_DUTY = 0.058   # was 0.056 — floor below which motor stalls
START_BOOST_DUTY   = 0.082   # was 0.074 — proportional to new base speed
START_BOOST_LOOPS  = 10      # was 14
REVERSE_DUTY       = 0.08    # was 0.09 — gentler reverse
LOOP_HZ            = 50
DUTY_RAMP_STEP     = 0.006   # was 0.005 — faster acceleration back to speed after ESTOP
CURVE_SLOW_STEER   = 0.50    # was 0.45 — steering is smoother now, start slowing slightly later
CURVE_MIN_SCALE    = 0.56    # was 0.50 — less speed reduction at full steer

# ── SERVO ────────────────────────────────────────────────────
SERVO_CENTER    = 0.50
SERVO_MIN       = 0.15
SERVO_MAX       = 0.85
INVERT_STEERING = False
STEER_RAMP_STEP = 0.10

# ── STEERING PID ─────────────────────────────────────────────
STEER_KP = 1.75   # was 1.6 — more responsive; testrun25 stable enough to raise without zig-zag
STEER_KI = 0.0    # was 0.012 — REMOVED: integral winds up in corners and biases straights → left crashes
STEER_KD = 0.22   # was 0.18 — heavier damping kills residual oscillation

# ── STEER SMOOTHING ──────────────────────────────────────────
# EMA applied to raw sensor steer BEFORE the PID so the PID tracks a
# smooth target instead of noisy per-scan readings. Lower = more smoothing.
STEER_SMOOTH_ALPHA = 0.45   # was 0.40 — slightly less smoothing, quicker target tracking in corners

# ── VESC IMU (BMI160 built into VESC 6 MkVI) ─────────────────
IMU_ENABLED       = True
IMU_POLL_EVERY    = 3       # poll every N main-loop iterations (~17 Hz)
COMM_GET_IMU_DATA = 65      # VESC firmware command ID
IMU_GYRO_MASK     = 0x0004  # bit 2 = gyroscope (x, y, z) in deg/s
IMU_HDG_CORRECT_DEG = 15.0  # heading drift above this triggers correction

# ── PS4 CONTROLLER ───────────────────────────────────────────
BTN_X           = 0
BTN_CIRCLE      = 1
BTN_SQUARE      = 2   # reset SLAM map
BTN_TRIANGLE    = 3
BTN_TRIANGLE_ALT = 3
BTN_L1          = 4
BTN_L1_ALT      = 9

# ── VESC COMMAND IDS ─────────────────────────────────────────
COMM_SET_DUTY     = 5
COMM_SET_CURRENT  = 6
COMM_SET_RPM      = 8
COMM_SET_SERVO_POS = 12

# ── HOKUYO URG-04LX ──────────────────────────────────────────
LIDAR_STEP_MIN   = 44
LIDAR_STEP_MAX   = 667
LIDAR_STEP_FRONT = 384
LIDAR_FRONT_WINDOW = 65
LIDAR_FULL_SPEED_MM = 2000   # was 1800 — need more clearance for full speed
LIDAR_MODERATE_MM   = 1400   # was 1200 — start slowing sooner
LIDAR_SLOW_MM       = 1050   # was 900
LIDAR_CRAWL_MM      = 700    # was 600
LIDAR_ESTOP_MM      = 420    # was 390 — trigger sooner; Hokuyo beam passes over low tubes at distance
LIDAR_ESTOP_CLEAR_MM = 495   # was 455 — need more clearance before resuming at higher trigger threshold
BRAKE_LOOPS      = 2         # was 3
REVERSE_LOOPS    = 3         # was 4 — brief reverse; car steers out, not backs out
SCAN_LOOPS       = 3         # was 4
HOLD_TIMEOUT_LOOPS = 22      # was 35 — 0.44s hold then force-resume; 0.7s was too much dead time

# ── FOLLOW-THE-GAP / SAFETY ──────────────────────────────────
CAR_WIDTH_MM           = 240
FTG_DISPARITY_THRESH_MM = 380
FTG_EXTRA_SAMPLES      = 8
FTG_SAFETY_FACTOR      = 2.2   # was 2.0 — wider safety bubble around disparities
SIDE_PANIC_MM          = 340   # was 300 — panic sooner; low tubes passed over by LiDAR until very close
SIDE_WARN_MM           = 520   # was 460 — start warning earlier so the corner-warn steer fires before contact
FRONT_CORNER_WARN_MM   = 1050  # was 950 — higher speed needs even earlier corner commitment
WALL_TARGET_MM         = 620   # was 560 — stay further from walls; 560 tolerated too much wall proximity

# ── ZED CAMERA ───────────────────────────────────────────────
CAM_ESTOP_M      = 0.38   # was 0.42 — further reduced; gap-only nearest makes this reliable now
ZED_ENABLED      = True
ZED_RESOLUTION   = sl.RESOLUTION.HD720 if ZED_AVAILABLE else None
ZED_FPS          = 30
ZED_DEPTH_MODE   = sl.DEPTH_MODE.PERFORMANCE if ZED_AVAILABLE else None
CORRIDOR_SCAN_Y_TOP    = 0.38
CORRIDOR_SCAN_Y_BOT    = 0.72
CORRIDOR_SCAN_NCOLS    = 32
CORRIDOR_WALL_M        = 0.85
CORRIDOR_MIN_OPEN_COLS = 3
FLOOR_SCAN_Y_TOP  = 0.70
FLOOR_SCAN_Y_BOT  = 0.93
FLOOR_SCAN_NCOLS  = 24
FLOOR_BLOCKED_M   = 0.42
FLOOR_OPEN_FAR_M  = 3.0
LIDAR_WEIGHT      = 0.62
CAMERA_WEIGHT     = 0.14   # raised: 0.08 gave camera only 5-8% of fused steer, ignored at corners
CAMERA_MAX_STEER  = 0.90

# ── ZED POINT CLOUD ──────────────────────────────────────────
ZED_PC_ENABLED       = True
ZED_PC_DEPTH_MIN_M   = 0.25
ZED_PC_DEPTH_MAX_M   = 3.5
ZED_FWD_OFFSET_M     = 0.10
ZED_PC_MAP_WEIGHT    = 0.45

# ── SLAM PARAMETERS ──────────────────────────────────────────
SLAM_ENABLED     = True
SLAM_CELL_M      = 0.05
SLAM_MAP_M       = 24.0
SLAM_N           = int(SLAM_MAP_M / SLAM_CELL_M)   # 480 cells
SLAM_HIT         = 0.70
SLAM_FREE        = -0.35
SLAM_MAX_LOG     = 4.0
SLAM_MIN_LOG     = -4.0
SLAM_OCC_THRESH  = 0.50
SLAM_MIN_DIST_M  = 0.10
SLAM_MAX_DIST_M  = 3.80
SLAM_TRUST_CELLS      = 700      # v3: raised back up — eager SLAM with drift caused left bias
SLAM_AUTO_RESET_CELLS = 3200     # reset map when this many unique cells observed (≈one lap); prevents ICP drift reversing the car
SLAM_MAP_PATH    = os.path.expanduser("~/Desktop/slam_map_save.npz")

ICP_ITERS       = 20             # was 15 — more iterations for better pose accuracy
ICP_CONV_M      = 0.001
ICP_MAX_DIST_M  = 0.25
ICP_MAX_DX_M    = 0.30           # was 0.40 — reject larger spurious jumps
ICP_MAX_DTHETA  = 0.35

# Map lookahead
LOOKAHEAD_M       = 3.0                        # was 2.5 — see farther down the track
LOOKAHEAD_STEP_M  = SLAM_CELL_M * 2            # 10 cm steps
LOOKAHEAD_ANGLES  = list(range(-65, 66, 5))    # -65..+65 in 5° steps
MAP_STEER_WEIGHT  = 0.12                       # was 0.10 — more map guidance in lap 2+ when confidence is high; still below 0.15 drift threshold
MAP_CONF_CELLS    = 3000

# Corner prediction
CORNER_DETECT_M    = 3.0   # was 2.5 — look farther ahead so SLAM warns earlier
CORNER_FWD_THRESH  = 0.52  # lowered: triggers at 1.56m ahead, earlier braking before corner entry
CORNER_SIDE_RATIO  = 1.15  # was 1.3 — less asymmetry needed; catches tighter corners
CORNER_BRAKE       = 0.53  # lowered: more aggressive speed reduction on SLAM corner detect
CORNER_PRE_STEER   = 0.33  # was 0.28 — more corner commitment now that ESTOP false positives are reduced

# ── BEHAVIORS ────────────────────────────────────────────────
BEHAVIOR_TRACK_FUSION = 0
BEHAVIOR_LIDAR_GAP    = 1
BEHAVIOR_LIDAR_CORRIDOR = 2
BEHAVIOR_RIGHT_WALL   = 3
BEHAVIOR_NAMES = ["TRACK_FUSION", "LIDAR_GAP", "LIDAR_CORRIDOR", "RIGHT_WALL"]
BEHAVIOR_DEFAULT = BEHAVIOR_TRACK_FUSION

# ── DISPLAY ──────────────────────────────────────────────────
MAP_ENABLED      = True
ZED_DISPLAY      = True
SLAM_DISPLAY     = True
SLAM_DISPLAY_SZ  = 400
ZED_RECORD_VIDEO = True
ZED_RECORD_FPS   = 20.0
ZED_RECORD_DIR   = os.path.expanduser("~/Desktop/zed_recordings")
PRINT_HZ         = 8
MAP_W = 600
MAP_H = 600
MAP_SCALE = 0.10
WARMUP_LOOPS = 10

# ── PRECOMPUTED TABLES ───────────────────────────────────────
_N_STEPS       = LIDAR_STEP_MAX - LIDAR_STEP_MIN + 1
_CENTER_IDX    = LIDAR_STEP_FRONT - LIDAR_STEP_MIN
_STEPS_PER_90  = int(90.0 * 1024 / 360)
_SCAN_INTERVAL_RAD = 2.0 * math.pi / 1024

SECTORS = [
    ("FAR_LEFT",     -90, -45),
    ("LEFT",         -45, -15),
    ("CENTER_LEFT",  -15,  -5),
    ("CENTER",        -5,   5),
    ("CENTER_RIGHT",   5,  15),
    ("RIGHT",         15,  45),
    ("FAR_RIGHT",     45,  90),
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
    vals = [v for v in vals if 20 < v < 5600]
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


# ── ZED CAMERA + 3D POINT CLOUD ──────────────────────────────

class ZEDDepthNavigator:
    def __init__(self):
        self.connected = False
        self._running  = False
        self._thread   = None
        self._lock     = threading.Lock()
        self._cam      = None
        self._steer       = 0.0
        self._confidence  = 0.0
        self._nearest     = float("nan")
        self._display     = None
        self._state       = "NO_ZED"
        self._floor_steer = 0.0
        self._floor_conf  = 0.0
        self._obstacle_pts = np.zeros((0, 2), dtype=np.float32)

    def connect(self):
        if not ZED_AVAILABLE:
            raise RuntimeError("pyzed not available")
        self._cam = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = ZED_RESOLUTION
        init.camera_fps        = ZED_FPS
        init.depth_mode        = ZED_DEPTH_MODE
        init.coordinate_units  = sl.UNIT.METER
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
        runtime   = sl.RuntimeParameters()
        image_mat = sl.Mat()
        depth_mat = sl.Mat()
        pc_mat    = sl.Mat()

        while self._running:
            if self._cam.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.005)
                continue

            self._cam.retrieve_image(image_mat, sl.VIEW.LEFT)
            self._cam.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

            bgr       = cv2.cvtColor(image_mat.get_data(), cv2.COLOR_BGRA2BGR)
            depth_arr = np.asarray(depth_mat.get_data(), dtype=np.float32)
            h, w      = bgr.shape[:2]

            obs_pts = np.zeros((0, 2), dtype=np.float32)
            if ZED_PC_ENABLED:
                try:
                    self._cam.retrieve_measure(pc_mat, sl.MEASURE.XYZ)
                    pc_arr  = np.asarray(pc_mat.get_data(), dtype=np.float32)
                    obs_pts = self._extract_obstacle_pts(pc_arr)
                except Exception:
                    pass

            try:
                steer, conf, state, nearest, disp = self._scan_corridor(bgr, depth_arr, h, w)
                floor_steer, floor_conf = self._detect_floor_gap(depth_arr, disp, h, w)
                with self._lock:
                    self._steer        = steer
                    self._confidence   = conf
                    self._state        = state
                    self._nearest      = nearest
                    self._display      = disp
                    self._floor_steer  = floor_steer
                    self._floor_conf   = floor_conf
                    self._obstacle_pts = obs_pts
            except Exception:
                time.sleep(0.005)

    def _extract_obstacle_pts(self, pc_arr):
        """
        Extract 2D robot-frame (X-right, Y-forward) obstacle positions from point cloud.
        ZED coord: X=right, Y=down, Z=forward → robot: x=ZED_X, y=ZED_Z+offset
        """
        h, w = pc_arr.shape[:2]
        y0, y1 = h // 3, 2 * h // 3
        band = pc_arr[y0:y1:4, ::4, :3].reshape(-1, 3)

        valid = (
            np.isfinite(band[:, 2]) &
            (band[:, 2] > ZED_PC_DEPTH_MIN_M) &
            (band[:, 2] < ZED_PC_DEPTH_MAX_M) &
            np.isfinite(band[:, 0])
        )
        pts = band[valid]
        if len(pts) == 0:
            return np.zeros((0, 2), dtype=np.float32)

        robot_x = pts[:, 0]
        robot_y = pts[:, 2] + ZED_FWD_OFFSET_M
        return np.column_stack([robot_x, robot_y]).astype(np.float32)

    def _scan_corridor(self, bgr, depth_arr, h, w):
        y0 = int(h * CORRIDOR_SCAN_Y_TOP)
        y1 = int(h * CORRIDOR_SCAN_Y_BOT)
        col_w = max(1, w // CORRIDOR_SCAN_NCOLS)
        col_depth = []
        for i in range(CORRIDOR_SCAN_NCOLS):
            xc0 = i * col_w
            xc1 = min(w, xc0 + col_w)
            col_data = depth_arr[y0:y1, xc0:xc1].ravel()
            valid = col_data[np.isfinite(col_data) & (col_data > 0.05) & (col_data < 6.0)]
            col_depth.append(float(np.percentile(valid, 15)) if valid.size >= 5 else 0.0)
        open_mask = [d >= CORRIDOR_WALL_M for d in col_depth]
        best_start, best_len = 0, 0
        cur_start, cur_len   = 0, 0
        for i, is_open in enumerate(open_mask):
            if is_open:
                if cur_len == 0:
                    cur_start = i
                cur_len += 1
                if cur_len > best_len:
                    best_len  = cur_len
                    best_start = cur_start
            else:
                cur_len = 0
        # Compute nearest from gap columns only when a gap is found, so that corner
        # side-walls don't trigger a false camera ESTOP via a spuriously small nearest.
        if best_len >= CORRIDOR_MIN_OPEN_COLS:
            near_vals = [col_depth[i] for i in range(best_start, best_start + best_len) if col_depth[i] > 0.05]
        else:
            near_vals = [d for d in col_depth if d > 0.05]
        nearest   = float(min(near_vals)) if near_vals else float("nan")
        display   = bgr.copy()
        for i, d in enumerate(col_depth):
            xc0 = i * col_w
            xc1 = min(w, xc0 + col_w) - 1
            if open_mask[i]:
                frac  = min(1.0, (d - CORRIDOR_WALL_M) / (2.5 - CORRIDOR_WALL_M))
                color = (0, int(80 + 140 * frac), 0)
            else:
                color = (0, 0, 140)
            cv2.rectangle(display, (xc0, y1 - 14), (xc1, y1), color, -1)
        cv2.rectangle(display, (0, y0), (w - 1, y1), (160, 160, 160), 1)
        near_s = f"{nearest:.2f}m" if np.isfinite(nearest) else "--"
        if best_len < CORRIDOR_MIN_OPEN_COLS:
            cv2.putText(display, f"Depth corridor: no gap  near={near_s}",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 80, 255), 2, cv2.LINE_AA)
            return 0.0, 0.0, "NO_GAP", nearest, display
        gap_center_col = best_start + best_len / 2.0
        steer_raw = -((gap_center_col - CORRIDOR_SCAN_NCOLS / 2.0) / (CORRIDOR_SCAN_NCOLS / 2.0))
        steer     = max(-CAMERA_MAX_STEER, min(CAMERA_MAX_STEER, steer_raw * CAMERA_MAX_STEER))
        open_x0 = best_start * col_w
        open_x1 = min(w, (best_start + best_len) * col_w)
        cx_px   = int(gap_center_col * col_w)
        cv2.rectangle(display, (open_x0, y0), (open_x1, y1 - 14), (0, 220, 0), 2)
        cv2.line(display, (cx_px, y0), (cx_px, y1 - 14), (0, 255, 0), 2)
        open_depths = [col_depth[i] for i in range(best_start, best_start + best_len) if col_depth[i] > 0]
        avg_depth   = sum(open_depths) / len(open_depths) if open_depths else 0.0
        conf        = min(0.90, (best_len / CORRIDOR_SCAN_NCOLS) * 2.0) * min(1.0, avg_depth / 1.5)
        cv2.putText(display,
                    f"Depth corridor: steer={steer:+.2f} conf={conf:.2f} "
                    f"gap={best_len}/{CORRIDOR_SCAN_NCOLS} near={near_s}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 230, 255), 2, cv2.LINE_AA)
        return steer, conf, "DEPTH_GAP", nearest, display

    def _detect_floor_gap(self, depth_arr, display, h, w):
        y0    = int(h * FLOOR_SCAN_Y_TOP)
        y1    = int(h * FLOOR_SCAN_Y_BOT)
        col_w = max(1, w // FLOOR_SCAN_NCOLS)
        clearance = []
        for i in range(FLOOR_SCAN_NCOLS):
            xc0 = i * col_w
            xc1 = min(w, xc0 + col_w)
            col_data = depth_arr[y0:y1, xc0:xc1].ravel()
            valid = col_data[np.isfinite(col_data) & (col_data > 0.10) & (col_data < FLOOR_OPEN_FAR_M)]
            if valid.size < 5:
                clearance.append(0.0)
            else:
                med = float(np.median(valid))
                clearance.append(0.0 if med < FLOOR_BLOCKED_M else med)
        smooth = list(clearance)
        for i in range(1, FLOOR_SCAN_NCOLS - 1):
            smooth[i] = (clearance[i - 1] + clearance[i] + clearance[i + 1]) / 3.0
        max_clear = max(smooth) if smooth else 0.0
        bar_y0, bar_y1 = h - 20, h - 4
        for i, val in enumerate(smooth):
            xc0 = i * col_w
            xc1 = min(w, xc0 + col_w) - 1
            frac = min(1.0, val / max(0.01, max_clear)) if max_clear > 0 else 0.0
            g = int(180 * frac)
            r = int(160 * (1.0 - frac))
            cv2.rectangle(display, (xc0, bar_y0), (xc1, bar_y1), (0, g, r), -1)
        if max_clear < FLOOR_BLOCKED_M:
            cv2.putText(display, "Floor gap: all blocked",
                        (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 80, 255), 2, cv2.LINE_AA)
            return 0.0, 0.0
        total_w = sum(smooth)
        best_x  = sum(smooth[i] * (i + 0.5) for i in range(FLOOR_SCAN_NCOLS)) / max(0.001, total_w)
        steer_raw = -((best_x - FLOOR_SCAN_NCOLS / 2.0) / (FLOOR_SCAN_NCOLS / 2.0))
        steer     = max(-CAMERA_MAX_STEER, min(CAMERA_MAX_STEER, steer_raw * CAMERA_MAX_STEER))
        sorted_c  = sorted(smooth, reverse=True)
        contrast  = (sorted_c[0] - sorted_c[-1]) / max(0.01, sorted_c[0])
        conf      = min(1.0, contrast * 2.0) * min(1.0, sorted_c[0] / 1.5)
        cv2.putText(display, f"Floor gap: steer={steer:+.2f} conf={conf:.2f}",
                    (10, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (80, 255, 80), 2, cv2.LINE_AA)
        return steer, conf

    def get(self):
        with self._lock:
            return {
                "steer":       self._steer,
                "confidence":  self._confidence,
                "nearest":     self._nearest,
                "state":       self._state,
                "floor_steer": self._floor_steer,
                "floor_conf":  self._floor_conf,
            }

    def get_display_frame(self):
        with self._lock:
            return self._display.copy() if self._display is not None else None

    def get_obstacle_pts(self):
        with self._lock:
            return self._obstacle_pts.copy()

    def stop(self):
        self._running = False
        if self._cam is not None:
            self._cam.close()
        self.connected = False
        print("[ZED] Disconnected")


# ── SLAM MAPPER ───────────────────────────────────────────────

class SlamMapper:
    """
    Lightweight 2D SLAM: scan-to-scan ICP for odometry +
    log-odds occupancy grid for map building.
    Runs in its own thread; navigation reads cached lookahead/corner results.
    """

    def __init__(self):
        n = SLAM_N
        self._n    = n
        self._grid = np.zeros((n, n), dtype=np.float32)
        self._cx   = n // 2
        self._cy   = n // 2

        self._pose     = np.array([0.0, 0.0, 0.0])  # x, y, theta (m, m, rad)
        self._prev_pts = None

        self._lock    = threading.Lock()
        self._q_lock  = threading.Lock()
        self._queue   = []
        self._running = False
        self._thread  = None

        self._c_lookahead = {a: LOOKAHEAD_M for a in LOOKAHEAD_ANGLES}
        self._c_corner    = (False, 1.0, 0.0)
        self._observed    = 0

    def _scan_to_pts(self, distances):
        pts = []
        for idx, d in enumerate(distances):
            d_m = d / 1000.0
            if d_m < SLAM_MIN_DIST_M or d_m > SLAM_MAX_DIST_M:
                continue
            angle = (idx + LIDAR_STEP_MIN - LIDAR_STEP_FRONT) * _SCAN_INTERVAL_RAD
            pts.append([d_m * math.sin(angle), d_m * math.cos(angle)])
        return np.array(pts, dtype=np.float32) if pts else np.zeros((0, 2), dtype=np.float32)

    def _icp(self, src, dst):
        """2D point-to-point ICP. Returns (dx, dy, dtheta) or None if failed."""
        if len(src) < 20 or len(dst) < 20:
            return None

        src_h = src.copy()

        for _ in range(ICP_ITERS):
            if SCIPY_AVAILABLE:
                tree       = KDTree(dst)
                dists, idx = tree.query(src_h, k=1)
            else:
                diff  = src_h[:, None, :] - dst[None, :, :]
                dists2 = (diff ** 2).sum(axis=2)
                idx   = dists2.argmin(axis=1)
                dists = np.sqrt(dists2[np.arange(len(src_h)), idx])

            valid = dists < ICP_MAX_DIST_M
            if valid.sum() < 15:
                return None

            sv = src_h[valid]
            dv = dst[idx[valid]]
            sc = sv.mean(axis=0)
            dc = dv.mean(axis=0)

            H = (sv - sc).T @ (dv - dc)
            try:
                U, _, Vt = np.linalg.svd(H)
            except np.linalg.LinAlgError:
                return None

            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T

            t = dc - R @ sc
            prev  = src_h.copy()
            src_h = (R @ src_h.T).T + t
            if np.linalg.norm(src_h - prev) < ICP_CONV_M:
                break

        dtheta = math.atan2(R[1, 0], R[0, 0])
        return float(t[0]), float(t[1]), float(dtheta)

    def _update_grid(self, pts_robot):
        """Vectorised log-odds occupancy update from LiDAR scan in robot frame."""
        px, py, pth = self._pose
        ct, st = math.cos(pth), math.sin(pth)
        n = self._n

        rx = int(self._cx + px / SLAM_CELL_M)
        ry = int(self._cy - py / SLAM_CELL_M)
        if not (0 <= rx < n and 0 <= ry < n):
            return

        mx = px + ct * pts_robot[:, 0] - st * pts_robot[:, 1]
        my = py + st * pts_robot[:, 0] + ct * pts_robot[:, 1]

        ex = (self._cx + mx / SLAM_CELL_M).astype(int)
        ey = (self._cy - my / SLAM_CELL_M).astype(int)
        valid = (ex >= 0) & (ex < n) & (ey >= 0) & (ey < n)

        np.add.at(self._grid, (ey[valid], ex[valid]), SLAM_HIT)
        np.clip(self._grid, SLAM_MIN_LOG, SLAM_MAX_LOG, out=self._grid)

        prev_occ = (self._grid[ey[valid], ex[valid]] - SLAM_HIT > SLAM_OCC_THRESH)
        now_occ  = (self._grid[ey[valid], ex[valid]] > SLAM_OCC_THRESH)
        self._observed += int((now_occ & ~prev_occ).sum())

        for frac in np.linspace(0.10, 0.88, 7):
            fx  = (rx + (ex - rx) * frac).astype(int)
            fy  = (ry + (ey - ry) * frac).astype(int)
            fv  = valid & (fx >= 0) & (fx < n) & (fy >= 0) & (fy < n)
            np.add.at(self._grid, (fy[fv], fx[fv]), SLAM_FREE)
            np.clip(self._grid, SLAM_MIN_LOG, SLAM_MAX_LOG, out=self._grid)

    def _update_grid_zed(self, pts_robot):
        """Add ZED obstacle points (Nx2 robot-frame XY) to the grid with reduced weight."""
        if len(pts_robot) == 0:
            return
        n = self._n
        px, py, pth = self._pose
        ct, st = math.cos(pth), math.sin(pth)

        mx = px + ct * pts_robot[:, 0] - st * pts_robot[:, 1]
        my = py + st * pts_robot[:, 0] + ct * pts_robot[:, 1]
        cx = (self._cx + mx / SLAM_CELL_M).astype(int)
        cy = (self._cy - my / SLAM_CELL_M).astype(int)
        valid = (cx >= 0) & (cx < n) & (cy >= 0) & (cy < n)
        np.add.at(self._grid, (cy[valid], cx[valid]), SLAM_HIT * ZED_PC_MAP_WEIGHT)
        np.clip(self._grid, SLAM_MIN_LOG, SLAM_MAX_LOG, out=self._grid)

    def _compute_lookahead(self):
        """Vectorised ray-casting in the occupancy grid for all lookahead angles."""
        px, py, pth = self._pose
        n = self._n
        n_steps = max(2, int(LOOKAHEAD_M / LOOKAHEAD_STEP_M))
        steps   = np.arange(2, n_steps) * LOOKAHEAD_STEP_M
        result  = {}

        for angle_deg in LOOKAHEAD_ANGLES:
            heading = pth + math.radians(angle_deg)
            sin_h, cos_h = math.sin(heading), math.cos(heading)

            wx = px + sin_h * steps
            wy = py + cos_h * steps
            ci = (self._cx + wx / SLAM_CELL_M).astype(int)
            cj = (self._cy - wy / SLAM_CELL_M).astype(int)

            in_bounds = (ci >= 0) & (ci < n) & (cj >= 0) & (cj < n)
            safe_ci   = np.clip(ci, 0, n - 1)
            safe_cj   = np.clip(cj, 0, n - 1)
            hits      = (~in_bounds) | (self._grid[safe_cj, safe_ci] > SLAM_OCC_THRESH)
            idx       = np.where(hits)[0]
            result[angle_deg] = float(idx[0] * LOOKAHEAD_STEP_M) if len(idx) > 0 else LOOKAHEAD_M

        return result

    def _compute_corner(self, lookahead):
        """Detect a corner ahead using the cached lookahead distances."""
        fwd   = lookahead.get(0,   LOOKAHEAD_M)
        left  = lookahead.get(-30, LOOKAHEAD_M)
        right = lookahead.get(30,  LOOKAHEAD_M)

        if fwd < CORNER_DETECT_M * CORNER_FWD_THRESH and max(left, right) > fwd * CORNER_SIDE_RATIO:
            if left > right:
                pre = CORNER_PRE_STEER
            else:
                pre = -CORNER_PRE_STEER
            brake = max(CORNER_BRAKE, fwd / CORNER_DETECT_M)
            return True, float(brake), float(pre)

        return False, 1.0, 0.0

    def _run(self):
        while self._running:
            item = None
            with self._q_lock:
                if self._queue:
                    item = self._queue.pop(0)
            if item is None:
                time.sleep(0.005)
                continue

            distances, zed_pts = item
            pts = self._scan_to_pts(distances)

            with self._lock:
                if self._prev_pts is not None and len(pts) >= 20:
                    res = self._icp(pts, self._prev_pts)
                    if res is not None:
                        dx, dy, dtheta = res
                        if abs(dx) < ICP_MAX_DX_M and abs(dy) < ICP_MAX_DX_M and abs(dtheta) < ICP_MAX_DTHETA:
                            ct = math.cos(self._pose[2])
                            st = math.sin(self._pose[2])
                            self._pose[0] += ct * dx - st * dy
                            self._pose[1] += st * dx + ct * dy
                            self._pose[2]  = (self._pose[2] + dtheta) % (2 * math.pi)

                if len(pts) > 0:
                    self._prev_pts = pts.copy()

                if len(pts) >= 10:
                    self._update_grid(pts)

                if zed_pts is not None and len(zed_pts) > 0:
                    self._update_grid_zed(zed_pts)

                lookahead          = self._compute_lookahead()
                self._c_lookahead  = lookahead
                self._c_corner     = self._compute_corner(lookahead)

    def add_scan(self, distances, zed_pts=None):
        with self._q_lock:
            self._queue.append((distances, zed_pts))
            if len(self._queue) > 3:
                self._queue = self._queue[-2:]

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def reset(self):
        with self._lock:
            self._grid[:] = 0.0
            self._pose[:] = 0.0
            self._prev_pts = None
            self._observed = 0
            self._c_lookahead = {a: LOOKAHEAD_M for a in LOOKAHEAD_ANGLES}
            self._c_corner    = (False, 1.0, 0.0)
        print("[SLAM] Map reset")

    def save(self, path):
        with self._lock:
            np.savez_compressed(path,
                                grid=self._grid,
                                observed=np.array([self._observed], dtype=np.int32))
        print(f"[SLAM] Map saved → {path}")

    def load(self, path):
        data = np.load(path)
        grid = data["grid"].astype(np.float32)
        if grid.shape != (self._n, self._n):
            raise ValueError(f"Saved map shape {grid.shape} != expected ({self._n},{self._n})")
        obs = int(data["observed"][0]) if "observed" in data else int((grid > SLAM_OCC_THRESH).sum())
        with self._lock:
            self._grid     = grid
            # Clamp so SLAM guidance is active immediately but auto-reset doesn't fire at once.
            self._observed = max(SLAM_TRUST_CELLS, min(obs, SLAM_AUTO_RESET_CELLS - 1))
            self._pose[:]  = 0.0
            self._prev_pts = None
            self._c_lookahead = {a: LOOKAHEAD_M for a in LOOKAHEAD_ANGLES}
            self._c_corner    = (False, 1.0, 0.0)
        print(f"[SLAM] Map loaded ← {path}  ({self._observed} cells active)")

    def get_lookahead(self):
        with self._lock:
            return dict(self._c_lookahead)

    def get_corner(self):
        with self._lock:
            return tuple(self._c_corner)

    def get_pose(self):
        with self._lock:
            return tuple(self._pose)

    def get_observed(self):
        with self._lock:
            return self._observed

    def get_map_image(self):
        with self._lock:
            px, py, pth = self._pose
            ci_c = int(self._cx + px / SLAM_CELL_M)
            cj_c = int(self._cy - py / SLAM_CELL_M)
            half = SLAM_DISPLAY_SZ // 2

            x0 = max(0, ci_c - half);  x1 = min(self._n, ci_c + half)
            y0 = max(0, cj_c - half);  y1 = min(self._n, cj_c + half)

            region = self._grid[y0:y1, x0:x1]
            img    = np.full((y1 - y0, x1 - x0, 3), 128, dtype=np.uint8)
            img[region >  SLAM_OCC_THRESH] = [30,  30,  30]
            img[region < -SLAM_OCC_THRESH] = [220, 220, 220]

            rc, rr = ci_c - x0, cj_c - y0
            if 0 <= rc < img.shape[1] and 0 <= rr < img.shape[0]:
                cv2.circle(img, (rc, rr), 5, (0, 60, 255), -1)
                ax = int(rc + 18 * math.sin(pth))
                ay = int(rr - 18 * math.cos(pth))
                cv2.arrowedLine(img, (rc, rr), (ax, ay), (0, 200, 255), 2, tipLength=0.4)

            for angle_deg in range(-60, 61, 15):
                d   = self._c_lookahead.get(angle_deg, LOOKAHEAD_M)
                hdg = pth + math.radians(angle_deg)
                ex_  = int(rc + (d / SLAM_CELL_M) * math.sin(hdg))
                ey_  = int(rr - (d / SLAM_CELL_M) * math.cos(hdg))
                ex_  = max(0, min(img.shape[1] - 1, ex_))
                ey_  = max(0, min(img.shape[0] - 1, ey_))
                col  = (0, 200, 80) if d >= LOOKAHEAD_M * 0.8 else (0, 100, 220)
                cv2.line(img, (rc, rr), (ex_, ey_), col, 1)

            corner_det, _, _ = self._c_corner
            obs_label = f"cells:{self._observed}"
            label_col = (0, 60, 220) if corner_det else (120, 220, 120)
            corner_str = "  CORNER!" if corner_det else ""
            cv2.putText(img, f"SLAM {obs_label}{corner_str}",
                        (4, img.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.40, label_col, 1)
            return img


# ── STEERING PID ─────────────────────────────────────────────

class SteerPID:
    def __init__(self):
        self.reset()

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0

    def update(self, target, current, dt):
        error            = target - current
        self._integral   = max(-0.40, min(0.40, self._integral + error * dt))
        derivative       = (error - self._prev_error) / max(dt, 1e-4)
        self._prev_error = error
        delta = (STEER_KP * error + STEER_KI * self._integral + STEER_KD * derivative) * dt
        return max(-1.0, min(1.0, current + delta))


# ── VESC IMU HELPERS ─────────────────────────────────────────

def _read_vesc_packet(ser, timeout=0.045):
    """Read one complete VESC packet from the serial port, returns payload or None."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] == 0x02:
            lb = ser.read(1)
            if not lb:
                return None
            length = lb[0]
            rest = ser.read(length + 3)
            if len(rest) >= length + 3 and rest[-1] == 0x03:
                return bytes(rest[:length])
    return None


def poll_imu_gyro_z(ser):
    """
    Request gyroscope data from the VESC built-in BMI160.
    Returns yaw-rate in deg/s (Z axis), or None if unavailable.
    """
    try:
        payload = bytes([COMM_GET_IMU_DATA]) + struct.pack(">H", IMU_GYRO_MASK)
        ser.reset_input_buffer()
        send_packet(ser, payload)
        pkt = _read_vesc_packet(ser)
        if pkt is None or len(pkt) < 13 or pkt[0] != COMM_GET_IMU_DATA:
            return None
        return struct.unpack(">f", pkt[9:13])[0]
    except Exception:
        return None


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
    if INVERT_MOTOR:
        duty = -duty
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

def lidar_speed_zone(front_dist, steer_abs=0.0, corner_brake=1.0):
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

    if label != "ESTOP":
        if steer_abs > CURVE_SLOW_STEER:
            t     = min(1.0, (steer_abs - CURVE_SLOW_STEER) / (1.0 - CURVE_SLOW_STEER))
            scale = 1.0 - t * (1.0 - CURVE_MIN_SCALE)
            cap  *= scale
        cap *= corner_brake

    return cap, label


# ── LIDAR STEERING LOGIC ─────────────────────────────────────

def preprocess_gap_lidar(distances):
    if not distances:
        return []
    ranges     = list(distances)
    last_r     = 0
    skip_iter  = False
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
        r_m      = r / 1000.0
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
            samples = range(idx, min(len(ranges), idx + num_excluded + 1)) if r > last_r \
                      else range(max(0, idx - num_excluded), idx)
            for j in samples:
                if ranges[j] > closer:
                    ranges[j] = closer
            skip_iter = True
        last_r = r
    return ranges


def find_best_gap_direction(distances):
    lo  = max(0, _CENTER_IDX - _STEPS_PER_90)
    hi  = min(len(distances) - 1, _CENTER_IDX + _STEPS_PER_90)
    if not distances[lo:hi + 1]:
        return 0.0
    best_score, best_idx = -1, _CENTER_IDX
    window = 25
    for i in range(lo, hi + 1):
        a    = max(lo, i - window)
        b    = min(hi, i + window)
        vals = [distances[j] for j in range(a, b + 1) if distances[j] > 20]
        if not vals:
            continue
        avg_clearance = sum(vals) / len(vals)
        offset_norm   = abs(i - _CENTER_IDX) / float(_STEPS_PER_90)
        # Moderate center preference: off-center gaps (corners) score better than before
        score         = avg_clearance * (1.0 - 0.38 * offset_norm)
        if score > best_score:
            best_score = score
            best_idx   = i
    offset = best_idx - _CENTER_IDX
    return max(-1.0, min(1.0, -offset / float(_STEPS_PER_90)))


def compute_gap_following(distances, clearances):
    if not distances:
        return 0.0
    gap = find_best_gap_direction(preprocess_gap_lidar(distances))
    # Deadband: small gap offsets in a straight corridor → return 0, not a micro-correction
    return 0.0 if abs(gap) < 0.08 else gap


def compute_corridor_centering(clearances):
    cap    = float(LIDAR_FULL_SPEED_MM)
    l1     = min(clearances.get("LEFT",         cap), cap)
    l2     = min(clearances.get("CENTER_LEFT",  cap), cap)
    r1     = min(clearances.get("RIGHT",        cap), cap)
    r2     = min(clearances.get("CENTER_RIGHT", cap), cap)
    left_d  = (l1 + l2) / 2.0
    right_d = (r1 + r2) / 2.0
    denom   = left_d + right_d
    if denom < 1.0:
        return 0.0
    raw = max(-1.0, min(1.0, (left_d - right_d) / denom))
    # Deadband: ignore small imbalances in corridors; only steer when clearly off-centre.
    # Without this, sub-10% noise drives constant micro-corrections → zig-zag.
    if abs(raw) < 0.09:
        return 0.0
    return raw


def compute_wall_following(clearances):
    # v3: two-sided wall avoidance — original only watched the RIGHT wall, giving zero
    # restoring force when the car drifted toward the LEFT wall → left crashes.
    # Now pushes away from whichever wall is closer than WALL_TARGET_MM.
    right_d = min(clearances.get("RIGHT",    float("inf")),
                  clearances.get("FAR_RIGHT", float("inf")))
    left_d  = min(clearances.get("LEFT",     float("inf")),
                  clearances.get("FAR_LEFT",  float("inf")))
    if right_d == float("inf"): right_d = WALL_TARGET_MM * 2.0
    if left_d  == float("inf"): left_d  = WALL_TARGET_MM * 2.0

    right_push = max(0.0, (WALL_TARGET_MM - right_d) / float(WALL_TARGET_MM))  # +: steer left
    left_push  = max(0.0, (WALL_TARGET_MM - left_d)  / float(WALL_TARGET_MM))  # +: steer right
    return max(-1.0, min(1.0, right_push - left_push))


def lidar_panic_override(clearances, front_dist):
    left       = min(clearances.get("FAR_LEFT",  float("inf")), clearances.get("LEFT",  float("inf")))
    right      = min(clearances.get("FAR_RIGHT", float("inf")), clearances.get("RIGHT", float("inf")))
    front_left  = clearances.get("CENTER_LEFT",  float("inf"))
    front_right = clearances.get("CENTER_RIGHT", float("inf"))
    if left < SIDE_PANIC_MM and right < SIDE_PANIC_MM:
        if left > right:
            return  0.50, "BOTH_PANIC_L"
        elif right > left:
            return -0.50, "BOTH_PANIC_R"
        else:
            return 0.0, "BOTH_SIDE_PANIC"
    if left < SIDE_PANIC_MM:
        return -0.80, "LEFT_PANIC"
    if right < SIDE_PANIC_MM:
        return  0.80, "RIGHT_PANIC"
    if front_dist is not None and front_dist < FRONT_CORNER_WARN_MM:
        mag = 0.75 if front_dist < LIDAR_CRAWL_MM else 0.62
        return (mag, "CORNER_WARN_L") if left > right else (-mag, "CORNER_WARN_R")
    return None, "OK"


# ── MAP-BASED STEERING HELPERS ────────────────────────────────

def compute_map_steer(lookahead, observed_cells):
    """
    Find best heading from SLAM lookahead.
    Returns (steer [-1..1], confidence [0..1]).
    Positive steer = left, negative = right (same convention as LiDAR code).
    """
    if observed_cells < SLAM_TRUST_CELLS:
        return 0.0, 0.0

    best_steer = 0.0
    best_score = -1.0
    for angle_deg, dist in lookahead.items():
        angle_norm = angle_deg / 65.0
        score      = dist * (1.0 - 0.15 * abs(angle_norm))
        if score > best_score:
            best_score = score
            best_steer = -(angle_deg / 65.0)

    conf = min(0.90, observed_cells / float(MAP_CONF_CELLS))
    return max(-1.0, min(1.0, best_steer)), conf


# ── SENSOR FUSION ─────────────────────────────────────────────

def fuse_track_steer(lidar_gap, lidar_corridor, camera_data, clearances, front_dist, slam=None):
    """
    Returns (steer, source_label, corner_brake_scale).
    corner_brake_scale < 1.0 when SLAM detects a corner ahead.
    """
    panic, reason = lidar_panic_override(clearances, front_dist)
    if panic is not None:
        return panic, f"LIDAR_{reason}", 1.0

    lidar_wall = compute_wall_following(clearances)
    # corridor (with deadband) is quiet in straights; minimize noisy raw gap contribution
    lidar_base = 0.13 * lidar_gap + 0.57 * lidar_corridor + 0.30 * lidar_wall
    lidar_base = max(-1.0, min(1.0, lidar_base))

    cam_steer   = camera_data["steer"]
    cam_conf    = camera_data["confidence"]
    floor_steer = camera_data.get("floor_steer", 0.0)
    floor_conf  = min(camera_data.get("floor_conf", 0.0), 0.65)

    nearest = camera_data.get("nearest", float("nan"))
    if nearest == nearest and nearest < 0.80:
        scale = max(0.10, nearest / 0.80)
        cam_conf   *= scale
        floor_conf *= scale

    # Floor scan sees clear carpet between/under the short orange tubes even when a wall
    # is directly ahead — at close range the floor's high conf dilutes the corridor steer.
    if nearest == nearest and nearest < 0.60:
        floor_conf = min(floor_conf, 0.25)

    if cam_conf > 0.15 and floor_conf > 0.05 and abs(cam_steer) > 0.30:
        if cam_steer * floor_steer < -0.05:
            floor_conf *= 0.20

    total_cam_conf = cam_conf + floor_conf
    if total_cam_conf < 0.05:
        fused  = lidar_base
        source = "LIDAR_ONLY"
    else:
        cam_blend      = (cam_steer * cam_conf + floor_steer * floor_conf) / total_cam_conf
        cam_blend      = max(-1.0, min(1.0, cam_blend))
        effective_conf = max(cam_conf, floor_conf)

        near = camera_data.get("nearest", float("nan"))
        if near == near and near < 0.50:
            effective_conf = max(effective_conf, 0.70)
        if abs(cam_steer) > 0.50 and cam_conf >= 0.25:
            effective_conf = max(effective_conf, 0.68)

        cw    = CAMERA_WEIGHT * effective_conf
        fused = max(-1.0, min(1.0, (1.0 - cw) * lidar_base + cw * cam_blend))
        src   = "FLOOR" if floor_conf > cam_conf else camera_data["state"]
        source = f"FUSED_{src}"

    # ── SLAM overlay ─────────────────────────────────────────
    corner_brake = 1.0
    if slam is not None:
        observed = slam.get_observed()
        map_steer, map_conf = 0.0, 0.0
        if observed >= SLAM_TRUST_CELLS:
            lookahead           = slam.get_lookahead()
            map_steer, map_conf = compute_map_steer(lookahead, observed)

        corner_det, corner_brake, pre_steer = slam.get_corner()

        if map_conf > 0.05:
            mw = MAP_STEER_WEIGHT * map_conf * (4.0 if corner_det else 1.0)
            mw = min(0.55, mw)
            # Sanity check: if the map strongly contradicts reactive sensors the SLAM
            # pose has likely drifted — suppress it so the car doesn't get steered backward.
            if abs(map_steer - fused) > 0.75:
                mw *= 0.15
                source += "+MAP(drift?)"
            else:
                source += "+MAP"
            fused  = max(-1.0, min(1.0, (1.0 - mw) * fused + mw * map_steer))

        if corner_det:
            fused  = max(-1.0, min(1.0, fused + pre_steer))
            source += "+CORNER"

    # Near-wall camera authority: when ZED sees a close obstacle with a strong steer
    # signal, override the lidar-dominated fused steer. The Hokuyo beams can pass
    # over the short (~25cm) orange tubes at distance, so lidar may report all-clear
    # while the camera correctly sees the wall. Authority ramps from 0 at 0.60m to
    # 0.80 at 0.34m; corridor steer must be meaningful (>0.28) and confident (>0.28).
    if (nearest == nearest and nearest < 0.60
            and abs(cam_steer) > 0.28 and cam_conf > 0.28):
        authority = min(0.80, (0.60 - nearest) / 0.26)
        fused = max(-1.0, min(1.0, (1.0 - authority) * fused + authority * cam_steer))
        source += "+CAM_NEAR"

    return fused, source, corner_brake


def select_reactive_steer(mode, distances, clearances, camera_data, front_dist, slam=None):
    """Returns (steer, source_label, corner_brake_scale)."""
    gap      = compute_gap_following(distances, clearances)
    corridor = compute_corridor_centering(clearances)

    if mode == BEHAVIOR_LIDAR_GAP:
        panic, reason = lidar_panic_override(clearances, front_dist)
        return (panic if panic is not None else gap), f"GAP_{reason}", 1.0

    if mode == BEHAVIOR_LIDAR_CORRIDOR:
        panic, reason = lidar_panic_override(clearances, front_dist)
        return (panic if panic is not None else corridor), f"CORRIDOR_{reason}", 1.0

    if mode == BEHAVIOR_RIGHT_WALL:
        panic, reason = lidar_panic_override(clearances, front_dist)
        return (panic if panic is not None else compute_wall_following(clearances)), f"WALL_{reason}", 1.0

    return fuse_track_steer(gap, corridor, camera_data, clearances, front_dist, slam)


def reactive_steer_to_servo(steer):
    x = -steer if INVERT_STEERING else steer
    return max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + x * 0.50))


# ── MAP DISPLAY (pygame) ──────────────────────────────────────

class LidarMapDisplay:
    _CX = MAP_W // 2
    _CY = MAP_H // 2

    def __init__(self):
        self._surface  = pygame.display.set_mode((MAP_W, MAP_H))
        pygame.display.set_caption("LiDAR + ZED + SLAM Nav v2")
        self._font_sm  = pygame.font.SysFont("monospace", 12)
        self._font_md  = pygame.font.SysFont("monospace", 14, bold=True)

    @staticmethod
    def _w2s(x_mm, y_mm):
        return LidarMapDisplay._CX + int(x_mm * MAP_SCALE), LidarMapDisplay._CY - int(y_mm * MAP_SCALE)

    @staticmethod
    def _dist_color(d):
        if d >= LIDAR_FULL_SPEED_MM:  return (0,   210,  60)
        if d >= LIDAR_MODERATE_MM:    return (220, 220,   0)
        if d >= LIDAR_SLOW_MM:        return (255, 130,   0)
        return (255, 50, 50)

    def render(self, distances, clearances, steer, zone, front_dist,
               behavior, source, cam, slam_info):
        s = self._surface
        s.fill((12, 12, 22))
        self._draw_grid(s)
        self._draw_rings(s)
        self._draw_scan(s, distances)
        self._draw_car(s)
        self._draw_steer_arrow(s, steer)
        self._draw_hud(s, zone, front_dist, behavior, source, steer, clearances, cam, slam_info)
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
        cx, cy  = LidarMapDisplay._CX, LidarMapDisplay._CY
        base_y  = cy + 38
        arrow_px = int(steer * 90)
        ex = cx + arrow_px
        dx = 1 if arrow_px > 0 else -1
        pygame.draw.line(s, (80, 200, 255), (cx, base_y), (ex, base_y), 3)
        pygame.draw.polygon(s, (80, 200, 255), [(ex, base_y), (ex - dx * 10, base_y - 5), (ex - dx * 10, base_y + 5)])

    def _draw_hud(self, s, zone, front_dist, behavior, source, steer, clearances, cam, slam_info):
        fd     = f"{front_dist} mm" if front_dist is not None else "-- mm"
        near   = cam.get("nearest", float("nan"))
        near_s = f"{near:.2f}m" if np.isfinite(near) else "--"
        obs, corner_det, corner_brake, pose = slam_info

        lines = [
            (f"Zone:   {zone}",                         (220, 220, 0) if zone != "FULL" else (0, 210, 60)),
            (f"Front:  {fd}",                           (200, 200, 200)),
            (f"Mode:   {behavior}",                     (80,  200, 255)),
            (f"Source: {source}",                       (255, 160,  80)),
            (f"Steer:  {steer:+.2f}",                   (160, 200, 255)),
            (f"Depth:  {cam.get('state','--')} conf={cam.get('confidence',0.0):.2f} near={near_s}",
                                                        (0,   230, 255)),
            (f"Floor:  steer={cam.get('floor_steer',0.0):+.2f} conf={cam.get('floor_conf',0.0):.2f}",
                                                        (80,  255,  80)),
            (f"SLAM:   cells={obs} corner={'YES' if corner_det else 'no'} brake={corner_brake:.2f}",
                                                        (255, 100, 200) if corner_det else (180, 180, 180)),
            (f"Pose:   x={pose[0]:.1f} y={pose[1]:.1f} th={math.degrees(pose[2]):.0f}°",
                                                        (160, 160, 220)),
        ]
        y = 5
        for text, col in lines:
            s.blit(self._font_md.render(text, True, col), (5, y))
            y += 18

        y = MAP_H - 5 - len(SECTORS) * 14
        for name, _, _ in SECTORS:
            d     = clearances.get(name, float("inf"))
            d_str = f"{int(d):5d} mm" if d < float("inf") else "  inf  "
            col   = self._dist_color(d) if d < float("inf") else (70, 70, 70)
            s.blit(self._font_sm.render(f"{name:<15}{d_str}", True, col), (5, y))
            y += 14


def maybe_render_map(map_display, lidar, clearances, steer, zone, front,
                     behavior, source, cam, slam_info, last_map, period):
    now = time.time()
    if map_display and lidar and (now - last_map) >= period:
        map_display.render(lidar.get_distances(), clearances, steer, zone, front,
                           behavior, source, cam, slam_info)
        return now
    return last_map


# ── VIDEO RECORDER ───────────────────────────────────────────

class VideoRecorder:
    def __init__(self, enabled=True):
        self.enabled    = enabled
        self.writer     = None
        self.path       = None
        self.last_write = 0.0

    def write(self, frame):
        if not self.enabled or frame is None:
            return
        now        = time.time()
        min_period = 1.0 / max(1.0, float(ZED_RECORD_FPS))
        if now - self.last_write < min_period:
            return
        if self.writer is None:
            os.makedirs(ZED_RECORD_DIR, exist_ok=True)
            ts       = time.strftime("%Y%m%d_%H%M%S")
            self.path = os.path.join(ZED_RECORD_DIR, f"zed_slam_v2_{ts}.mp4")
            h, w     = frame.shape[:2]
            fourcc   = cv2.VideoWriter_fourcc(*"mp4v")
            self.writer = cv2.VideoWriter(self.path, fourcc, float(ZED_RECORD_FPS), (w, h))
            if not self.writer.isOpened():
                print(f"\n[ZED_REC] WARNING: could not open writer at {self.path}")
                self.enabled = False
                self.writer  = None
                return
            print(f"\n[ZED_REC] Recording to: {self.path}")
        self.writer.write(frame)
        self.last_write = now

    def close(self):
        if self.writer is not None:
            self.writer.release()
            print(f"[ZED_REC] Saved: {self.path}")
            self.writer = None


# ── VESC PORT DETECTION ──────────────────────────────────────

def _find_vesc_port():
    import glob
    candidates = [VESC_PORT] + sorted(glob.glob("/dev/ttyACM*"))
    for port in candidates:
        import os as _os
        if _os.path.exists(port):
            return port
    return None


def _open_vesc():
    port = _find_vesc_port()
    if port is None:
        import glob
        available = (
            glob.glob("/dev/serial/by-id/*") +
            glob.glob("/dev/ttyACM*") +
            glob.glob("/dev/ttyUSB*")
        )
        print("\n[VESC] ERROR: No VESC port found.")
        print("       Configured path: " + VESC_PORT)
        print("       Currently available ports:")
        for p in available:
            print(f"         {p}")
        if not available:
            print("         (none — check USB cable)")
        print("       Plug in the VESC and re-run, or update VESC_PORT at the top of the file.")
        return None

    if port != VESC_PORT:
        print(f"[VESC] Configured path not found; falling back to {port}")

    print(f"[VESC] Opening {port}...")
    try:
        ser = serial.Serial(port, VESC_BAUDRATE, timeout=0.05, write_timeout=0.05)
        time.sleep(0.5)
        send_current_zero(ser)
        send_servo(ser, SERVO_CENTER)
        print(f"[VESC] Connected on {port}")
        return ser
    except Exception as e:
        print(f"[VESC] ERROR opening {port}: {e}")
        return None


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
        print(f"[LIDAR] WARNING: {e}")
        lidar = None

    # ZED
    cam = None
    if ZED_ENABLED:
        cam = ZEDDepthNavigator()
        try:
            cam.connect()
            cam.start()
            if ZED_DISPLAY:
                cv2.namedWindow("ZED Depth Nav", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("ZED Depth Nav", 960, 540)
        except Exception as e:
            print(f"[ZED] WARNING: {e}")
            cam = None

    # SLAM
    slam_mapper = None
    if SLAM_ENABLED:
        slam_mapper = SlamMapper()
        slam_mapper.start()
        if SLAM_DISPLAY:
            cv2.namedWindow("SLAM Map", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("SLAM Map", SLAM_DISPLAY_SZ, SLAM_DISPLAY_SZ)
        print(f"[SLAM] Started — ICP={'scipy KDTree' if SCIPY_AVAILABLE else 'numpy brute-force'}")
        print(f"[SLAM] Grid: {SLAM_N}x{SLAM_N} cells @ {int(SLAM_CELL_M*100)}cm | Map: {SLAM_MAP_M}m x {SLAM_MAP_M}m")
        print(f"[SLAM] Map steer activates after {SLAM_TRUST_CELLS} observed cells")
        if os.path.exists(SLAM_MAP_PATH):
            try:
                slam_mapper.load(SLAM_MAP_PATH)
                print(f"[SLAM] Corner prediction active from lap 1")
            except Exception as e:
                print(f"[SLAM] WARNING: could not load saved map: {e}")
        else:
            print(f"[SLAM] No saved map found — will build from scratch this run")

    # Pygame map
    map_display = None
    if MAP_ENABLED:
        try:
            map_display = LidarMapDisplay()
        except Exception as e:
            print(f"[MAP] WARNING: {e}")

    # Controller
    joystick = None
    pygame.event.pump()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[PS4] {joystick.get_name()}")
        print("       Triangle = cycle mode | Square = reset SLAM | X/Circle = quit | L1 = stop")
    else:
        print("[PS4] No controller — autonomous only")

    # VESC
    ser = _open_vesc()
    if ser is None:
        if lidar:  lidar.stop()
        if cam:    cam.stop()
        if slam_mapper: slam_mapper.stop()
        pygame.quit()
        return

    print()
    print("=" * 72)
    print("  ZED + LiDAR + SLAM NAV  —  v2")
    print("=" * 72)
    print(f"  VESC : {VESC_PORT}")
    print(f"  LiDAR: {LIDAR_PORT}")
    print(f"  Speed: auto={AUTO_DRIVE_DUTY:.3f} max={MAX_DUTY:.3f}")
    print(f"  Speed zones: full≥{LIDAR_FULL_SPEED_MM}mm  mod≥{LIDAR_MODERATE_MM}mm  slow≥{LIDAR_SLOW_MM}mm  crawl≥{LIDAR_CRAWL_MM}mm  estop<{LIDAR_ESTOP_MM}mm")
    print(f"  Default mode: {BEHAVIOR_NAMES[BEHAVIOR_DEFAULT]}")
    print(f"  SLAM: corner brake={CORNER_BRAKE:.2f} pre-steer={CORNER_PRE_STEER:.2f} detect={CORNER_DETECT_M}m")
    print(f"  ZED 3D point cloud: {'ON' if ZED_PC_ENABLED else 'OFF'}")
    print("  Start on a stand. Triangle = cycle mode. Square = reset SLAM map.")
    print("=" * 72)
    print()

    loop_period  = 1.0 / LOOP_HZ
    print_period = 1.0 / PRINT_HZ
    map_period   = 1.0 / 15

    last_print  = 0.0
    last_map    = 0.0
    last_slam_img = 0.0
    slam_img_period = 1.0 / 8

    zed_recorder    = VideoRecorder(ZED_RECORD_VIDEO)
    behavior_mode   = BEHAVIOR_DEFAULT
    warmup_counter  = 0
    lidar_estop     = False
    estop_cooldown  = 0
    reverse_counter = 0
    hold_counter    = 0
    kill_active     = False
    current_duty    = 0.0
    duty_to_send    = 0.0
    start_boost_counter = 0
    current_steer   = 0.0
    target_steer    = 0.0
    steer_source    = "INIT"
    corner_brake    = 1.0
    clearances      = {name: float("inf") for name, _, _ in SECTORS}
    front_dist      = None
    zone_label      = "FULL"

    steer_pid = SteerPID()
    _ema_target_steer    = 0.0         # EMA state for pre-PID target smoothing
    _prev_side_panic     = False       # tracks side-panic (not corner-warn) for exit damp
    estop_reverse_servo  = SERVO_CENTER  # servo position used during ESTOP reverse phase

    imu_heading         = 0.0
    estop_entry_heading = 0.0
    imu_available       = False
    loop_count          = 0

    estop_exit_dampen   = 0
    ESTOP_EXIT_DAMP_LOOPS = 25   # was 40 — shorter suppression; car needs to steer to re-center after recovery
    ESTOP_EXIT_DAMP_CAP   = 0.78 # was 0.65 — allow more steer authority immediately post-ESTOP

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
                    if event.button == BTN_SQUARE and slam_mapper:
                        slam_mapper.reset()
                    if event.button in (BTN_L1, BTN_L1_ALT):
                        kill_active   = True
                        current_duty  = 0.0
                        stop_car(ser)
                        print("\n[KILL] L1 held")

                if event.type == pygame.JOYBUTTONUP:
                    if event.button in (BTN_L1, BTN_L1_ALT):
                        kill_active = False
                        print("[KILL] Released")

            if joystick is not None:
                pygame.event.pump()
                try:
                    kill_active = bool(
                        (joystick.get_numbuttons() > BTN_L1     and joystick.get_button(BTN_L1)) or
                        (joystick.get_numbuttons() > BTN_L1_ALT and joystick.get_button(BTN_L1_ALT))
                    )
                except Exception:
                    pass

            if kill_active:
                current_duty = 0.0
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                if cam and cam.connected:
                    frame = cam.get_display_frame()
                    if frame is not None:
                        if ZED_DISPLAY:
                            cv2.imshow("ZED Depth Nav", frame)
                            cv2.waitKey(1)
                        zed_recorder.write(frame)
                time.sleep(loop_period)
                continue

            # Sensor reads
            distances   = []
            camera_data = {
                "steer": 0.0, "confidence": 0.0,
                "nearest": float("nan"), "state": "NO_CAMERA",
                "floor_steer": 0.0, "floor_conf": 0.0,
            }

            if cam and cam.connected:
                camera_data = cam.get()

            zed_pts = None
            if cam and cam.connected and ZED_PC_ENABLED:
                zed_pts = cam.get_obstacle_pts()
                if len(zed_pts) == 0:
                    zed_pts = None

            if lidar and lidar.connected:
                distances  = lidar.get_distances()
                front_dist = lidar.front_min()
                clearances = lidar.sector_clearances()

                if slam_mapper and distances:
                    slam_mapper.add_scan(distances, zed_pts)
                    # Auto-reset when the map has aged (ICP drift accumulates over a full lap
                    # and can reverse the car's heading). Only reset in a clear straight so
                    # corner guidance isn't lost mid-turn.
                    if (SLAM_AUTO_RESET_CELLS > 0
                            and slam_mapper.get_observed() >= SLAM_AUTO_RESET_CELLS
                            and front_dist is not None
                            and front_dist >= LIDAR_FULL_SPEED_MM):
                        slam_mapper.reset()
                        _ema_target_steer = 0.0
                        steer_pid.reset()
                        print("\n[SLAM] Auto-reset — map refreshed to clear accumulated drift")

                target_steer, steer_source, corner_brake = select_reactive_steer(
                    behavior_mode, distances, clearances, camera_data, front_dist, slam_mapper
                )
            else:
                front_dist   = None
                target_steer = camera_data["steer"]
                steer_source = "CAMERA_ONLY"
                corner_brake = 1.0

            # Apply EMA to target before PID; panic overrides bypass EMA and PID for immediacy
            _panic_kw = ("PANIC", "WARN", "FRONT_CLOSE")
            _is_panic      = any(kw in steer_source for kw in _panic_kw)
            # Side-panic exit damp is separate from corner-warn: damping a corner-warn exit
            # kills the turn steer and causes the car to drive straight into the wall.
            _is_side_panic = any(kw in steer_source for kw in ("LEFT_PANIC", "RIGHT_PANIC", "BOTH_PANIC"))
            if _is_panic:
                _ema_target_steer = target_steer
                current_steer = target_steer
                steer_pid.reset()
            else:
                if _prev_side_panic:
                    # Exiting a SIDE panic only: cut steer to 35% to prevent coasting into
                    # the opposite wall. Corner-warn exits are left undamped so the car
                    # keeps its turn steer through the apex.
                    current_steer     *= 0.35
                    _ema_target_steer *= 0.35
                _ema_target_steer = (STEER_SMOOTH_ALPHA * target_steer
                                     + (1.0 - STEER_SMOOTH_ALPHA) * _ema_target_steer)
                current_steer = steer_pid.update(_ema_target_steer, current_steer, loop_period)
            _prev_side_panic = _is_side_panic

            # Post-ESTOP steer damping
            if estop_exit_dampen > 0:
                estop_exit_dampen -= 1
                current_steer = max(-ESTOP_EXIT_DAMP_CAP, min(ESTOP_EXIT_DAMP_CAP, current_steer))

            # Servo output deadband: when PID wants a tiny correction, snap to centre instead.
            # current_steer (PID state) is preserved so the PID can respond quickly when a
            # real correction is needed; only what goes to the servo hardware is zeroed.
            _panic_active = any(kw in steer_source for kw in ("PANIC", "WARN", "FRONT_CLOSE"))
            _servo_steer  = current_steer if (_panic_active or lidar_estop
                                              or abs(current_steer) >= 0.06) else 0.0
            servo_pos = reactive_steer_to_servo(_servo_steer)

            speed_cap, zone_label = lidar_speed_zone(front_dist, abs(_servo_steer), corner_brake)

            if estop_cooldown > 0:
                estop_cooldown -= 1

            # Camera nearest-depth ESTOP
            _cam_near = camera_data.get("nearest", float("nan"))
            if (cam and cam.connected and _cam_near == _cam_near
                    and _cam_near < CAM_ESTOP_M
                    and not lidar_estop and estop_cooldown <= 0
                    and zone_label != "ESTOP"):
                zone_label = "ESTOP"
                print(f"\n[CAM_ESTOP] Camera near at {_cam_near:.2f}m")

            # ESTOP logic
            if zone_label == "ESTOP" and not lidar_estop and estop_cooldown <= 0:
                lidar_estop         = True
                reverse_counter     = BRAKE_LOOPS + REVERSE_LOOPS + SCAN_LOOPS
                hold_counter        = 0
                current_duty        = 0.0
                estop_entry_heading = imu_heading
                steer_pid.reset()
                # Compute which side is open and set reverse steer to angle the nose
                # into the open corridor during reverse (steering is inverted while going
                # backward: steer right → nose swings left, steer left → nose swings right).
                _ol_e = min(clearances.get("FAR_LEFT",  float("inf")), clearances.get("LEFT",  float("inf")))
                _or_e = min(clearances.get("FAR_RIGHT", float("inf")), clearances.get("RIGHT", float("inf")))
                estop_reverse_servo = reactive_steer_to_servo(-0.30 if _ol_e > _or_e else 0.30)
                print(f"\n[LIDAR] ESTOP — obstacle at {front_dist} mm  reverse={'left-nose' if _ol_e > _or_e else 'right-nose'}")

            elif lidar_estop and zone_label != "ESTOP":
                path_clear = front_dist is None or front_dist >= LIDAR_ESTOP_CLEAR_MM
                if reverse_counter == 0 and (path_clear or hold_counter >= HOLD_TIMEOUT_LOOPS):
                    lidar_estop       = False
                    hold_counter      = 0
                    estop_cooldown    = 30   # was 50 (1s); 0.6s is enough, 1s leaves car unprotected
                    estop_exit_dampen = ESTOP_EXIT_DAMP_LOOPS
                    steer_pid.reset()
                    _ol = min(clearances.get("FAR_LEFT",  float("inf")), clearances.get("LEFT",  float("inf")))
                    _or = min(clearances.get("FAR_RIGHT", float("inf")), clearances.get("RIGHT", float("inf")))
                    _base = 0.28 if _ol > _or else -0.28
                    _hdg_err = imu_heading - estop_entry_heading
                    _hdg_corr = max(-0.65, min(0.65, -_hdg_err / 90.0))
                    current_steer = max(-1.0, min(1.0, _base + _hdg_corr))
                    print(f"\n[LIDAR] Resuming — pre-steer={'L' if _ol>_or else 'R'} hdg_err={_hdg_err:.1f}° corr={_hdg_corr:+.2f}")

            # Warmup
            if warmup_counter < WARMUP_LOOPS:
                warmup_counter += 1
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                if cam and cam.connected:
                    frame = cam.get_display_frame()
                    if frame is not None:
                        if ZED_DISPLAY:
                            cv2.imshow("ZED Depth Nav", frame)
                            cv2.waitKey(1)
                        zed_recorder.write(frame)
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
                    send_duty(ser, -REVERSE_DUTY)
                    send_servo(ser, estop_reverse_servo)  # steer to angle nose into open corridor
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
                        lidar_estop       = False
                        hold_counter      = 0
                        estop_cooldown    = 30   # was 50 (1s)
                        estop_exit_dampen = ESTOP_EXIT_DAMP_LOOPS
                        steer_pid.reset()
                        _ol = min(clearances.get("FAR_LEFT",  float("inf")), clearances.get("LEFT",  float("inf")))
                        _or = min(clearances.get("FAR_RIGHT", float("inf")), clearances.get("RIGHT", float("inf")))
                        _base = 0.28 if _ol > _or else -0.28
                        _hdg_err = imu_heading - estop_entry_heading
                        _hdg_corr = max(-0.65, min(0.65, -_hdg_err / 90.0))
                        current_steer = max(-1.0, min(1.0, _base + _hdg_corr))
                        print(f"\n[ESTOP] Hold timeout — resuming, hdg_err={_hdg_err:.1f}° corr={_hdg_corr:+.2f}")
                    current_duty = 0.0
                    send_current_zero(ser)
                    send_servo(ser, servo_pos)
                    estop_state = f"HOLD({hold_counter})"

                now = time.time()
                if now - last_print >= print_period:
                    fd = f"{front_dist}mm" if front_dist is not None else "--"
                    print(f"\r[ESTOP/{estop_state}] Front:{fd} Steer:{current_steer:+.2f}    ", end="")
                    last_print = now

            else:
                target_duty = min(AUTO_DRIVE_DUTY, speed_cap)

                if target_duty > 0.002:
                    target_duty = max(target_duty, MIN_EFFECTIVE_DUTY)
                    if abs(current_duty) < 0.003 and start_boost_counter <= 0:
                        start_boost_counter = START_BOOST_LOOPS
                    if start_boost_counter > 0:
                        target_duty = max(target_duty, START_BOOST_DUTY)
                        start_boost_counter -= 1
                else:
                    start_boost_counter = 0

                current_duty = ramp_value(current_duty, target_duty, DUTY_RAMP_STEP)
                duty_to_send = current_duty

                if 0.002 < abs(duty_to_send) < MIN_EFFECTIVE_DUTY:
                    duty_to_send = math.copysign(MIN_EFFECTIVE_DUTY, duty_to_send)

                send_servo(ser, servo_pos)
                if abs(duty_to_send) > 0.002:
                    send_duty(ser, duty_to_send)
                    drive_state = f"DRIVE[{zone_label}]"
                else:
                    send_current_zero(ser)
                    duty_to_send = 0.0
                    drive_state  = "IDLE"

                now = time.time()
                if now - last_print >= print_period:
                    fd     = f"{front_dist}mm" if front_dist is not None else "--"
                    near   = camera_data["nearest"]
                    near_s = f"{near:.2f}m" if np.isfinite(near) else "--"
                    obs    = slam_mapper.get_observed() if slam_mapper else 0
                    cdet, cbrak, _ = slam_mapper.get_corner() if slam_mapper else (False, 1.0, 0.0)
                    print(
                        f"\r[{drive_state}] "
                        f"Mode:{BEHAVIOR_NAMES[behavior_mode]} "
                        f"Src:{steer_source} "
                        f"Steer:{current_steer:+.2f} Servo:{servo_pos:.2f} "
                        f"Duty:{duty_to_send:+.3f} Cap:{speed_cap:.3f} "
                        f"Front:{fd} SLAM:{obs}cells{'*CORNER*' if cdet else ''}    ",
                        end=""
                    )
                    last_print = now

            # Pygame LiDAR map
            if slam_mapper:
                obs     = slam_mapper.get_observed()
                cdet, cbrak, _ = slam_mapper.get_corner()
                pose    = slam_mapper.get_pose()
                slam_info = (obs, cdet, cbrak, pose)
            else:
                slam_info = (0, False, 1.0, (0.0, 0.0, 0.0))

            last_map = maybe_render_map(
                map_display, lidar, clearances, current_steer, zone_label, front_dist,
                BEHAVIOR_NAMES[behavior_mode], steer_source, camera_data, slam_info,
                last_map, map_period
            )

            # ZED display
            if cam and cam.connected:
                frame = cam.get_display_frame()
                if frame is not None:
                    if ZED_DISPLAY:
                        cv2.imshow("ZED Depth Nav", frame)
                        cv2.waitKey(1)
                    zed_recorder.write(frame)

            # SLAM map display
            now = time.time()
            if slam_mapper and SLAM_DISPLAY and (now - last_slam_img) >= slam_img_period:
                slam_img = slam_mapper.get_map_image()
                cv2.imshow("SLAM Map", slam_img)
                cv2.waitKey(1)
                last_slam_img = now

            # IMU polling
            loop_count += 1
            if IMU_ENABLED and loop_count % IMU_POLL_EVERY == 0:
                gz = poll_imu_gyro_z(ser)
                if gz is not None:
                    imu_heading += gz * loop_period * IMU_POLL_EVERY
                    if not imu_available:
                        imu_available = True
                        print("[IMU] VESC built-in IMU active")

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
        if slam_mapper:
            try:
                slam_mapper.save(SLAM_MAP_PATH)
            except Exception as e:
                print(f"[SLAM] WARNING: could not save map: {e}")
            slam_mapper.stop()
        zed_recorder.close()
        cv2.destroyAllWindows()
        pygame.quit()
        print("[INFO] Closed safely")


if __name__ == "__main__":
    main()
