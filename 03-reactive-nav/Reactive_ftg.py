import struct
import time
import serial
import pygame
import threading
import math

# ============================================================
# Reactive_ftg.py — Autonomous reactive navigation
# VESC 6 MkVI + Hokuyo URG-04LX  (Roboracer / F1TENTH)
#
# Composite of:
#   Reactive.py         — working VESC/LiDAR/ESTOP infrastructure
#   follow_the_gap.py   — F1TENTH GitHub FTG algorithm logic
#
# Gap-following algorithm (BEHAVIOR_GAP_FOLLOW, default):
#   Step 1 — Disparity extender: detects obstacle edges (large jumps
#             between adjacent rays) and inflates the closer obstacle
#             across the angular width the car occupies at that range.
#             Prevents the car from steering into gaps too narrow to
#             fit through.  Ported from follow_the_gap.py with the
#             car-geometry-aware angular calculation.
#   Step 2 — Steer toward the single farthest ray in the forward
#             hemisphere (after preprocessing).
#   Step 3 — Side safety nudge: if FAR_LEFT or FAR_RIGHT clearance
#             is below SIDE_SAFETY_MM, override with a gentle nudge
#             away from the obstacle.
#
# Two additional behaviors (Triangle to cycle):
#   CORRIDOR_CTR — stay centred between left and right walls
#   WALL_FOLLOW  — maintain target distance from the right wall
#
# Speed zones (forward LiDAR):
#   >= 1000 mm  FULL      (MAX_DUTY)
#    500–1000   MODERATE  (MODERATE_DUTY_CAP)
#    300– 500   SLOW      (SLOW_DUTY_CAP)
#    < 200 mm   ESTOP     (brake → reverse → scan → hold)
#
# Controls:
#   Triangle   = cycle behavior mode (optional joystick)
#   Circle / X = quit
#   Ctrl-C     = quit
# ============================================================

# ── VESC ─────────────────────────────────────────────────────
VESC_PORT       = "/dev/ttyACM1"
VESC_BAUDRATE   = 115200
MAX_DUTY        = 0.18        # hard cap
AUTO_DRIVE_DUTY = 0.15        # forward drive duty
LOOP_HZ         = 50
DUTY_RAMP_STEP  = 0.005       # duty change per loop tick

# ── PS4 CONTROLLER (optional — only used for mode/quit) ──────
BTN_X        = 0
BTN_CIRCLE   = 1
BTN_TRIANGLE = 3

# ── VESC COMMAND IDs ─────────────────────────────────────────
COMM_SET_DUTY      = 5
COMM_SET_CURRENT   = 6
COMM_SET_RPM       = 8
COMM_SET_SERVO_POS = 12

# ── SERVO ────────────────────────────────────────────────────
SERVO_CENTER    = 0.50
SERVO_MIN       = 0.15
SERVO_MAX       = 0.85
INVERT_STEERING = False

PRINT_HZ     = 8
WARMUP_LOOPS = 15

# ── HOKUYO URG-04LX ──────────────────────────────────────────
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

MODERATE_DUTY_CAP = 0.12
SLOW_DUTY_CAP     = 0.10      # must exceed VESC minimum effective duty

BRAKE_LOOPS   = 20      # ~0.4 s at zero current before reversing
REVERSE_LOOPS = 40      # ~0.8 s straight back — tune for ~2 ft
SCAN_LOOPS    = 20      # ~0.4 s hold still so LiDAR settles

LIDAR_ENABLED = True

# ── REACTIVE BEHAVIORS ───────────────────────────────────────
BEHAVIOR_GAP_FOLLOW   = 0
BEHAVIOR_CORRIDOR_CTR = 1
BEHAVIOR_WALL_FOLLOW  = 2
BEHAVIOR_NAMES        = ["GAP_FOLLOW", "CORRIDOR_CTR", "WALL_FOLLOW"]
BEHAVIOR_DEFAULT      = BEHAVIOR_GAP_FOLLOW

# ── FOLLOW-THE-GAP PARAMETERS (ported from follow_the_gap.py) ─
CAR_WIDTH_MM            = 220    # physical car width in mm (Traxxas Fiesta ~220 mm)
FTG_DISPARITY_THRESH_MM = 500    # jump larger than this is an obstacle edge
FTG_EXTRA_SAMPLES       = 5      # extra steps added on top of angular calculation
FTG_SAFETY_FACTOR       = 1.5    # inflate car width by this factor for clearance margin
SIDE_SAFETY_MM          = 200    # FAR_LEFT/FAR_RIGHT below this → nudge away

# Angular resolution of the Hokuyo (radians per encoder step)
_SCAN_INTERVAL_RAD = 2.0 * math.pi / 1024

# ── OTHER BEHAVIOR PARAMETERS ────────────────────────────────
WALL_TARGET_MM     = 500
HOLD_TIMEOUT_LOOPS = 100   # force resume on oval where walls are always close
STEER_RAMP_STEP    = 0.03

# 7 sectors covering the forward hemisphere
SECTORS = [
    ("FAR_LEFT",     -90, -45),
    ("LEFT",         -45, -15),
    ("CENTER_LEFT",  -15,  -5),
    ("CENTER",        -5,   5),
    ("CENTER_RIGHT",   5,  15),
    ("RIGHT",         15,  45),
    ("FAR_RIGHT",     45,  90),
]

# ── 2D MAP DISPLAY ───────────────────────────────────────────
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


_SIN, _COS = _build_angle_tables()
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
        # 10th-percentile so one noisy ray can't trigger ESTOP
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


# ── FOLLOW-THE-GAP ALGORITHM ─────────────────────────────────

def preprocess_gap_lidar(distances):
    """
    Disparity extender ported from follow_the_gap.py (F1TENTH GitHub).

    Scans adjacent ray pairs for large jumps (obstacle edges).  When one
    is found, the closer distance is propagated across the angular span
    the car body occupies at that range, inflated by FTG_SAFETY_FACTOR.
    This makes every narrow gap appear closed so the car never tries to
    squeeze through an opening that's too tight.
    """
    if not distances:
        return []

    ranges       = list(distances)
    last_r       = 0
    skip_iter    = False
    num_excluded = 0

    for idx, r in enumerate(ranges):
        if idx == 0:
            last_r = r
            continue

        # Skip iterations consumed by the previous disparity extension
        if skip_iter:
            last_r = r
            num_excluded -= 1
            if num_excluded == -1:
                skip_iter = False
            continue

        if r <= 20 or last_r <= 20:
            last_r = r
            continue

        r_m      = r      / 1000.0
        last_r_m = last_r / 1000.0

        if abs(r_m - last_r_m) > FTG_DISPARITY_THRESH_MM / 1000.0:
            # Compute how many steps the car's inflated width subtends at this range
            try:
                half_angle   = math.asin(
                    min(1.0, (CAR_WIDTH_MM / 1000.0 * FTG_SAFETY_FACTOR) / min(r_m, last_r_m))
                )
                num_excluded = round(half_angle / _SCAN_INTERVAL_RAD) + FTG_EXTRA_SAMPLES
            except (ValueError, ZeroDivisionError):
                num_excluded = FTG_EXTRA_SAMPLES

            closer = min(r, last_r)
            if r > last_r:
                # Transition from obstacle to open: extend the obstacle rightward
                samples = range(idx, min(len(ranges), idx + num_excluded + 1))
            else:
                # Transition from open to obstacle: extend the obstacle leftward
                samples = range(max(0, idx - num_excluded), idx)

            for j in samples:
                if ranges[j] > closer:
                    ranges[j] = closer

            skip_iter = True

        last_r = r

    return ranges


def find_max_gap(distances):
    """
    Return the index of the single farthest ray in the forward hemisphere.
    Ported from follow_the_gap.py find_max_gap — steering toward the farthest
    point after preprocessing guides the car into the largest open space.
    """
    lo  = max(0, _CENTER_IDX - _STEPS_PER_90)
    hi  = min(len(distances) - 1, _CENTER_IDX + _STEPS_PER_90)
    sub = distances[lo:hi + 1]
    if not sub:
        return _CENTER_IDX
    return sub.index(max(sub)) + lo


def compute_gap_following(distances, clearances):
    """
    Full Follow-the-Gap pipeline:
      1. Preprocess (disparity extender)
      2. Find farthest ray → compute steer
      3. Side-safety override if FAR_LEFT/FAR_RIGHT is dangerously close

    Sign convention:
      negative return → steer right
      positive return → steer left
    """
    if not distances:
        return 0.0

    processed = preprocess_gap_lidar(distances)
    gap_idx   = find_max_gap(processed)

    offset = gap_idx - _CENTER_IDX
    steer  = -offset / float(_STEPS_PER_90)
    steer  = max(-1.0, min(1.0, steer))

    # Side safety nudge (from follow_the_gap.py lidar_callback safety check)
    left_side  = clearances.get("FAR_LEFT",  float("inf"))
    right_side = clearances.get("FAR_RIGHT", float("inf"))
    if left_side < SIDE_SAFETY_MM:
        steer = -0.25   # obstacle on left flank — nudge right
    elif right_side < SIDE_SAFETY_MM:
        steer =  0.25   # obstacle on right flank — nudge left

    return steer


def compute_corridor_centering(clearances):
    """Balance left/right clearance to stay centred in a corridor."""
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
    """Follow the right wall at WALL_TARGET_MM distance."""
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
        return compute_gap_following(distances, clearances)
    if behavior_mode == BEHAVIOR_CORRIDOR_CTR:
        return compute_corridor_centering(clearances)
    if behavior_mode == BEHAVIOR_WALL_FOLLOW:
        return compute_wall_following(clearances)
    return 0.0


def compute_parallel_escape(clearances):
    """
    After an ESTOP reverse, read the environment and report which way
    the corridor opens.  Gap-following resumes automatically on the
    next forward pass and will steer toward the open side.
    """
    left_open  = clearances.get("FAR_LEFT", 0) + clearances.get("LEFT", 0)
    right_open = clearances.get("FAR_RIGHT", 0) + clearances.get("RIGHT", 0)

    if left_open >= right_open:
        direction = "LEFT"
    else:
        direction = "RIGHT"

    print(f"\n[SCAN] Corridor opens {direction}  "
          f"(L={int(left_open)} R={int(right_open)}) — resuming FTG")


def reactive_steer_to_servo(steer):
    x = -steer if INVERT_STEERING else steer
    return max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + x * 0.50))


# ── 2D LIDAR MAP DISPLAY ─────────────────────────────────────

class LidarMapDisplay:
    _CX = MAP_W // 2
    _CY = MAP_H // 2

    def __init__(self):
        self._surface = pygame.display.set_mode((MAP_W, MAP_H))
        pygame.display.set_caption("LiDAR 2D Map  —  FTG Reactive")
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
               zone_label, front_dist, behavior_name):
        s = self._surface
        s.fill((12, 12, 22))
        self._draw_grid(s)
        self._draw_rings(s)
        self._draw_sector_tints(s, clearances)
        self._draw_scan(s, distances)
        self._draw_car(s)
        self._draw_steer_arrow(s, reactive_steer)
        self._draw_hud(s, zone_label, front_dist, behavior_name,
                       reactive_steer, clearances)
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

    def _draw_hud(self, s, zone_label, front_dist, behavior_name,
                  reactive_steer, clearances):
        zone_col = {
            "FULL":     (  0, 210,  60),
            "MODERATE": (220, 220,   0),
            "SLOW":     (255, 130,   0),
            "ESTOP":    (255,  50,  50),
        }.get(zone_label, (200, 200, 200))

        fd_str = f"{front_dist} mm" if front_dist else "-- mm"

        y = 5
        for text, col in [
            (f"Zone:   {zone_label}",           zone_col),
            (f"Front:  {fd_str}",               (200, 200, 200)),
            (f"Mode:   {behavior_name}",         (80, 200, 255)),
            (f"Steer:  {reactive_steer:+.2f}",   (160, 200, 255)),
        ]:
            s.blit(self._font_md.render(text, True, col), (5, y))
            y += 18

        y = MAP_H - 5 - len(SECTORS) * 14
        for name, _, _ in SECTORS:
            d     = clearances.get(name, float("inf"))
            d_str = f"{int(d):5d} mm" if d < float("inf") else "  inf  "
            col   = self._dist_color(d) if d < float("inf") else (70, 70, 70)
            s.blit(self._font_sm.render(f"{name:<15}{d_str}", True, col), (5, y))
            y += 14


# ── MAP RENDER HELPER ────────────────────────────────────────

def maybe_render_map(map_display, lidar, clearances, reactive_steer,
                     zone_label, front_dist, behavior_name, last_map, map_period):
    now = time.time()
    if map_display and lidar and (now - last_map) >= map_period:
        distances = lidar.get_distances()
        map_display.render(distances, clearances, reactive_steer,
                           zone_label, front_dist, behavior_name)
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

    # Map display ────────────────────────────────────────────
    map_display = None
    if MAP_ENABLED:
        try:
            map_display = LidarMapDisplay()
            print("[MAP] 2D LiDAR display active (600×600)")
        except Exception as e:
            print(f"[MAP] WARNING: display unavailable — {e}")

    # Joystick — optional ────────────────────────────────────
    joystick = None
    pygame.event.pump()
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print(f"[PS4] Connected: {joystick.get_name()}")
        print("       X / Circle = quit   Triangle = cycle mode")
    else:
        print("[PS4] No controller detected — running fully autonomous")
        print("       Close window or Ctrl-C to quit")

    # VESC ───────────────────────────────────────────────────
    print(f"[VESC] Opening {VESC_PORT}...")
    ser = serial.Serial(VESC_PORT, VESC_BAUDRATE, timeout=0.05, write_timeout=0.05)
    send_current_zero(ser)
    send_servo(ser, SERVO_CENTER)

    # Banner ─────────────────────────────────────────────────
    print()
    print("=" * 60)
    print("   REACTIVE FTG AUTONOMOUS — F1TENTH / ROBORACER")
    print("=" * 60)
    print(f"  Auto duty:  {AUTO_DRIVE_DUTY:.3f}  (max {MAX_DUTY:.3f})")
    print(f"  Zones: FULL>={LIDAR_FULL_SPEED_MM}mm  "
          f"MOD>={LIDAR_MODERATE_MM}mm  "
          f"SLOW>={LIDAR_SLOW_MM}mm  "
          f"ESTOP<{LIDAR_ESTOP_MM}mm")
    print(f"  Default behavior: {BEHAVIOR_NAMES[BEHAVIOR_DEFAULT]}")
    print(f"  FTG disparity threshold: {FTG_DISPARITY_THRESH_MM} mm")
    print(f"  Car width (inflated): {CAR_WIDTH_MM * FTG_SAFETY_FACTOR:.0f} mm")
    print()
    print(f"  LiDAR: {'active  ' + LIDAR_PORT if lidar else 'NOT connected'}")
    print(f"  Map:   {'active (600x600 window)' if map_display else 'unavailable'}")
    print()
    print("  PLACE CAR IN A CLEAR SPACE.  VESC TOOL MUST BE CLOSED.")
    print("=" * 60)
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
                    print("[PS4] Controller disconnected — continuing autonomous")

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

                reactive_steer = select_reactive_steer(
                    behavior_mode, distances, clearances)

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
                        reason = "path clear" if path_clear else "hold timeout — resuming"
                        print(f"[LIDAR] {reason} ({front_dist} mm)")

            # Smooth the reactive steering output ────────────
            current_steer = ramp_value(current_steer, reactive_steer, STEER_RAMP_STEP)
            servo_pos     = reactive_steer_to_servo(current_steer)

            # Warmup: hold still while LiDAR settles ─────────
            if warmup_counter < WARMUP_LOOPS:
                warmup_counter += 1
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                time.sleep(loop_period)
                continue

            # ESTOP sequence: brake → reverse → scan → hold ──
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
                        f"Steer:{current_steer:+.2f}     ",
                        end=""
                    )
                    last_print = now

                last_map = maybe_render_map(
                    map_display, lidar, clearances, current_steer,
                    zone_label, front_dist, BEHAVIOR_NAMES[behavior_mode],
                    last_map, map_period)

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
                    f"Cap:{speed_cap:.2f}  "
                    f"Front:{fd_str}     ",
                    end=""
                )
                last_print = now

            last_map = maybe_render_map(
                map_display, lidar, clearances, current_steer,
                zone_label, front_dist, BEHAVIOR_NAMES[behavior_mode],
                last_map, map_period)

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
        pygame.quit()
        print("[INFO] Closed safely")


if __name__ == "__main__":
    main()
