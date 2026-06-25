import struct
import time
import serial
import pygame
import threading
import math

# ============================================================
# SteeringAssist.py — PS4 → VESC + HOKUYO URG-04LX
# (Roboracer / F1TENTH)
#
# Extends AssistedAutonomyV1 with:
#   1. STEERING ASSIST — 7-sector LiDAR analysis biases steering
#      toward the clearest path.  Driver input is always the base;
#      the assist adds an additive correction whose strength scales
#      with proximity zone.  Toggle with Triangle button.
#
#   2. 2D LIDAR MAP — live 600×600 pygame bird's-eye view.
#      Scan points coloured by distance zone.  Range rings,
#      sector tints, car marker, and assist-direction arrow.
#
# Speed zones (forward only):
#   >= 2500 mm  FULL      (MAX_DUTY)
#   1500–2500   MODERATE  (MODERATE_DUTY_CAP)
#    800–1500   SLOW      (SLOW_DUTY_CAP)
#    < 800 mm   ESTOP     (reverse + steering assist allowed)
#
# Controls:
#   Left stick  up/down    = forward / reverse
#   Right stick left/right = steering (blended with assist)
#   Hold X                 = manual emergency stop
#   Circle                 = quit
#   Triangle               = toggle steering assist on/off
# ============================================================

# ── VESC ────────────────────────────────────────────────────
VESC_PORT      = "/dev/ttyACM1"
VESC_BAUDRATE  = 115200
MAX_RPM        = 50000
MAX_CURRENT_A  = 30
MAX_DUTY       = 0.20
LOOP_HZ        = 50
DEADZONE       = 0.10
DUTY_RAMP_STEP = 0.010

# ── PS4 CONTROLLER ──────────────────────────────────────────
AXIS_DRIVE    = 1
AXIS_STEERING = 2
BTN_X         = 0   # hold = emergency stop
BTN_CIRCLE    = 1   # quit
BTN_TRIANGLE  = 3   # toggle steering assist

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
INVERT_DRIVE    = False

PRINT_HZ     = 8
WARMUP_LOOPS = 10

# ── HOKUYO URG-04LX ─────────────────────────────────────────
LIDAR_PORT         = "/dev/ttyACM0"
LIDAR_BAUDRATE     = 19200
LIDAR_STEP_MIN     = 44
LIDAR_STEP_MAX     = 725
LIDAR_STEP_FRONT   = 384
LIDAR_FRONT_WINDOW = 50

LIDAR_FULL_SPEED_MM = 2500
LIDAR_MODERATE_MM   = 1500
LIDAR_SLOW_MM       = 800
LIDAR_ESTOP_MM      = 800

MODERATE_DUTY_CAP = 0.14
SLOW_DUTY_CAP     = 0.07

LIDAR_ENABLED = True

# ── STEERING ASSIST ─────────────────────────────────────────
ASSIST_ENABLED_DEFAULT = True

# Additive strength per zone — how much the assist can nudge
# the steering beyond driver input (0 = off, 1 = full-scale nudge)
ASSIST_STRENGTH_FULL     = 0.00
ASSIST_STRENGTH_MODERATE = 0.40
ASSIST_STRENGTH_SLOW     = 0.70
ASSIST_STRENGTH_ESTOP    = 1.00

# 7 sectors covering the forward hemisphere.
# Angles in degrees from straight ahead; negative = left, positive = right.
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
MAP_W       = 600       # window width  (px)
MAP_H       = 600       # window height (px)
MAP_SCALE   = 0.10      # px per mm  →  1 px = 10 mm; visible range ±3 000 mm

# ── PRECOMPUTED LOOK-UP TABLES ───────────────────────────────

_N_STEPS = LIDAR_STEP_MAX - LIDAR_STEP_MIN + 1


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
        center = LIDAR_STEP_FRONT - LIDAR_STEP_MIN
        lo     = max(0, center - LIDAR_FRONT_WINDOW)
        hi     = min(len(d) - 1, center + LIDAR_FRONT_WINDOW)
        zone   = [x for x in d[lo:hi + 1] if x > 20]
        return min(zone) if zone else None

    def sector_clearances(self):
        """Return min valid distance (mm) for each named sector."""
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


def send_current_zero(ser):
    send_packet(ser, bytes([COMM_SET_CURRENT]) + struct.pack(">i", 0))


def send_servo(ser, position):
    position = max(SERVO_MIN, min(SERVO_MAX, position))
    send_packet(ser, bytes([COMM_SET_SERVO_POS]) + struct.pack(">h", int(position * 1000)))


def stop_car(ser):
    send_current_zero(ser)
    send_servo(ser, SERVO_CENTER)
    print("\n[VESC] Motor stopped, steering centred")


def apply_deadzone(v):
    return 0.0 if abs(v) < DEADZONE else v


def steering_to_servo(raw_axis, assist_delta=0.0):
    """
    raw_axis   : right-stick value  [-1, +1]
    assist_delta: additive correction from steering assist [-1, +1] scaled by strength
    Returns servo position clamped to [SERVO_MIN, SERVO_MAX].
    """
    x = apply_deadzone(raw_axis) + assist_delta
    if INVERT_STEERING:
        x = -x
    return max(SERVO_MIN, min(SERVO_MAX, SERVO_CENTER + x * 0.50))


def drive_input_from_stick(y):
    drive = -y
    if INVERT_DRIVE:
        drive = -drive
    return max(-1.0, min(1.0, apply_deadzone(drive)))


def ramp_value(current, target, step):
    if target > current + step:
        return current + step
    if target < current - step:
        return current - step
    return target


def safe_reverse_guard(current_duty, target_duty):
    if current_duty >  0.01 and target_duty < -0.01:
        return 0.0
    if current_duty < -0.01 and target_duty >  0.01:
        return 0.0
    return target_duty


# ── LIDAR SPEED + ASSIST LOGIC ───────────────────────────────

def lidar_speed_zone(front_dist):
    if front_dist is None:
        return MAX_DUTY, "FULL"
    if front_dist >= LIDAR_FULL_SPEED_MM:
        return MAX_DUTY,        "FULL"
    if front_dist >= LIDAR_MODERATE_MM:
        return MODERATE_DUTY_CAP, "MODERATE"
    if front_dist >= LIDAR_SLOW_MM:
        return SLOW_DUTY_CAP,     "SLOW"
    return 0.0, "ESTOP"


def compute_assist_steer(clearances):
    """
    Returns float [-1, +1].
    Negative = nudge left, positive = nudge right.
    Steers toward whichever side has more clearance when the centre sector
    is obstructed.  Returns 0 when the path ahead is fully clear.
    """
    center_c = clearances.get("CENTER", float("inf"))
    if center_c >= LIDAR_FULL_SPEED_MM:
        return 0.0

    left_c = min(
        clearances.get("FAR_LEFT",    float("inf")),
        clearances.get("LEFT",        float("inf")),
        clearances.get("CENTER_LEFT", float("inf")),
    )
    right_c = min(
        clearances.get("FAR_RIGHT",    float("inf")),
        clearances.get("RIGHT",        float("inf")),
        clearances.get("CENTER_RIGHT", float("inf")),
    )

    cap     = float(LIDAR_FULL_SPEED_MM)
    left_c  = min(left_c,  cap)
    right_c = min(right_c, cap)
    denom   = left_c + right_c

    if denom < 1.0:
        return 0.0

    # Negative result → right side is clearer → nudge right (servo direction is inverted)
    return max(-1.0, min(1.0, (left_c - right_c) / denom))


def assist_strength_from_zone(zone_label):
    return {
        "FULL":     ASSIST_STRENGTH_FULL,
        "MODERATE": ASSIST_STRENGTH_MODERATE,
        "SLOW":     ASSIST_STRENGTH_SLOW,
        "ESTOP":    ASSIST_STRENGTH_ESTOP,
    }.get(zone_label, 0.0)


# ── 2D LIDAR MAP DISPLAY ─────────────────────────────────────

class LidarMapDisplay:
    """Live bird's-eye pygame window showing the LiDAR environment."""

    _CX = MAP_W // 2
    _CY = MAP_H // 2

    def __init__(self):
        self._surface = pygame.display.set_mode((MAP_W, MAP_H))
        pygame.display.set_caption("LiDAR 2D Map  —  SteeringAssist")
        self._font_sm = pygame.font.SysFont("monospace", 12)
        self._font_md = pygame.font.SysFont("monospace", 14, bold=True)

    # coordinate helper ──────────────────────────────────────
    @staticmethod
    def _w2s(x_mm, y_mm):
        """World mm → screen px.  +x = right,  +y = forward (up on screen)."""
        sx = LidarMapDisplay._CX + int(x_mm * MAP_SCALE)
        sy = LidarMapDisplay._CY - int(y_mm * MAP_SCALE)
        return sx, sy

    @staticmethod
    def _dist_color(d):
        if d >= LIDAR_FULL_SPEED_MM: return (  0, 210,  60)
        if d >= LIDAR_MODERATE_MM:   return (220, 220,   0)
        if d >= LIDAR_SLOW_MM:       return (255, 130,   0)
        return (255,  50,  50)

    # main render ────────────────────────────────────────────
    def render(self, distances, clearances, assist_steer,
               zone_label, front_dist, assist_on):
        s = self._surface
        s.fill((12, 12, 22))

        self._draw_grid(s)
        self._draw_rings(s)
        self._draw_sector_tints(s, clearances)
        self._draw_scan(s, distances)
        self._draw_car(s)
        if assist_on:
            self._draw_assist_arrow(s, assist_steer)
        self._draw_hud(s, zone_label, front_dist, assist_on, assist_steer, clearances)

        pygame.display.flip()

    # sub-draw helpers ───────────────────────────────────────
    @staticmethod
    def _draw_grid(s):
        step_px = max(1, int(500 * MAP_SCALE))   # 500 mm grid → 50 px
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
            ( 500, ( 70,  30,  30)),
            ( 800, ( 90,  55,  20)),   # ESTOP boundary
            (1500, ( 75,  75,  20)),   # SLOW boundary
            (2500, ( 30,  75,  30)),   # FULL boundary
        ]:
            pygame.draw.circle(s, col, (cx, cy), int(mm * MAP_SCALE), 1)

    @staticmethod
    def _draw_sector_tints(s, clearances):
        """Fill each sector wedge with a translucent danger colour."""
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
        # front direction triangle
        pygame.draw.polygon(s, (255, 255, 100), [
            (cx,      cy - ch // 2 - 10),
            (cx - 5,  cy - ch // 2),
            (cx + 5,  cy - ch // 2),
        ])

    @staticmethod
    def _draw_assist_arrow(s, steer):
        if abs(steer) < 0.04:
            return
        cx, cy   = LidarMapDisplay._CX, LidarMapDisplay._CY
        base_y   = cy + 38
        arrow_px = int(steer * 90)
        ex       = cx + arrow_px
        dx       = 1 if arrow_px > 0 else -1
        pygame.draw.line(s, (80, 200, 255), (cx, base_y), (ex, base_y), 3)
        pygame.draw.polygon(s, (80, 200, 255), [
            (ex,            base_y),
            (ex - dx * 10,  base_y - 5),
            (ex - dx * 10,  base_y + 5),
        ])

    def _draw_hud(self, s, zone_label, front_dist, assist_on,
                  assist_steer, clearances):
        zone_col = {
            "FULL":     (  0, 210,  60),
            "MODERATE": (220, 220,   0),
            "SLOW":     (255, 130,   0),
            "ESTOP":    (255,  50,  50),
        }.get(zone_label, (200, 200, 200))

        fd_str = f"{front_dist} mm" if front_dist else "-- mm"
        assist_str = (f"ON  bias:{assist_steer:+.2f}" if assist_on
                      else "OFF  (Triangle = enable)")

        y = 5
        for text, col in [
            (f"Zone:   {zone_label}", zone_col),
            (f"Front:  {fd_str}",     (200, 200, 200)),
            (f"Assist: {assist_str}", (80, 200, 255) if assist_on else (100, 100, 100)),
        ]:
            s.blit(self._font_md.render(text, True, col), (5, y))
            y += 18

        # sector clearance table — bottom-left corner
        y = MAP_H - 5 - len(SECTORS) * 14
        for name, _, _ in SECTORS:
            d     = clearances.get(name, float("inf"))
            d_str = f"{int(d):5d} mm" if d < float("inf") else "  inf  "
            col   = self._dist_color(d) if d < float("inf") else (70, 70, 70)
            s.blit(self._font_sm.render(f"{name:<15}{d_str}", True, col), (5, y))
            y += 14


# ── MAP RENDER HELPER ────────────────────────────────────────

def maybe_render_map(map_display, lidar, clearances, assist_steer,
                     zone_label, front_dist, assist_on, last_map, map_period):
    now = time.time()
    if map_display and lidar and (now - last_map) >= map_period:
        distances = lidar.get_distances()
        map_display.render(distances, clearances, assist_steer,
                           zone_label, front_dist, assist_on)
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

    # PS4 controller ─────────────────────────────────────────
    print("[INFO] Waiting for PS4 controller...")
    while pygame.joystick.get_count() == 0:
        pygame.event.pump()
        time.sleep(0.5)

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"[PS4] Connected: {joystick.get_name()}")
    print(f"      Axes:{joystick.get_numaxes()}  "
          f"Buttons:{joystick.get_numbuttons()}  "
          f"Hats:{joystick.get_numhats()}")

    print("[INFO] Settling axes...")
    for _ in range(30):
        pygame.event.pump()
        time.sleep(0.02)

    # VESC ───────────────────────────────────────────────────
    print(f"[VESC] Opening {VESC_PORT}...")
    ser = serial.Serial(VESC_PORT, VESC_BAUDRATE, timeout=0.05, write_timeout=0.05)
    send_current_zero(ser)
    send_servo(ser, SERVO_CENTER)

    # Banner ─────────────────────────────────────────────────
    print()
    print("=" * 58)
    print("   STEERING ASSIST + 2D MAP — F1TENTH / ROBORACER")
    print("=" * 58)
    print("  Left stick  up/dn    forward / reverse")
    print("  Right stick lr       steering (blended with assist)")
    print("  Hold X               emergency stop")
    print("  Circle               quit")
    print("  Triangle             toggle steering assist ON/OFF")
    print()
    print(f"  Zones: FULL>={LIDAR_FULL_SPEED_MM}mm  "
          f"MOD>={LIDAR_MODERATE_MM}mm  "
          f"SLOW>={LIDAR_SLOW_MM}mm  "
          f"ESTOP<{LIDAR_ESTOP_MM}mm")
    print(f"  Assist strength: MOD={ASSIST_STRENGTH_MODERATE}  "
          f"SLOW={ASSIST_STRENGTH_SLOW}  "
          f"ESTOP={ASSIST_STRENGTH_ESTOP}")
    print()
    print(f"  LiDAR:  {'active  ' + LIDAR_PORT if lidar else 'NOT connected'}")
    print(f"  Map:    {'active (600x600 window)' if map_display else 'unavailable'}")
    print()
    print("  PUT THE CAR ON A STAND.  VESC TOOL MUST BE CLOSED.")
    print("=" * 58)
    print()

    # State ──────────────────────────────────────────────────
    loop_period  = 1.0 / LOOP_HZ
    print_period = 1.0 / PRINT_HZ
    map_period   = 1.0 / 15          # 15 Hz map refresh
    last_print   = 0.0
    last_map     = 0.0

    estop          = False
    lidar_estop    = False
    assist_on      = ASSIST_ENABLED_DEFAULT
    warmup_counter = 0

    current_duty = drive_input_from_stick(joystick.get_axis(AXIS_DRIVE)) * MAX_DUTY
    assist_steer = 0.0
    clearances   = {name: float("inf") for name, _, _ in SECTORS}
    zone_label   = "FULL"
    front_dist   = None
    speed_cap    = MAX_DUTY

    try:
        while True:
            loop_start = time.time()

            # Event handling ─────────────────────────────────
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_X:
                        estop        = True
                        current_duty = 0.0
                        stop_car(ser)
                    elif event.button == BTN_CIRCLE:
                        raise KeyboardInterrupt
                    elif event.button == BTN_TRIANGLE:
                        assist_on = not assist_on
                        print(f"\n[ASSIST] Steering assist "
                              f"{'ENABLED' if assist_on else 'DISABLED'}")

                if event.type == pygame.JOYBUTTONUP:
                    if event.button == BTN_X:
                        estop = False
                        print("[ESTOP] Released")

                if event.type == pygame.JOYDEVICEREMOVED:
                    stop_car(ser)
                    raise KeyboardInterrupt

            # LiDAR processing ───────────────────────────────
            front_dist   = None
            speed_cap    = MAX_DUTY
            zone_label   = "FULL"
            assist_steer = 0.0

            if lidar and lidar.connected:
                front_dist           = lidar.front_min()
                speed_cap, zone_label = lidar_speed_zone(front_dist)
                clearances           = lidar.sector_clearances()

                if assist_on:
                    raw_assist   = compute_assist_steer(clearances)
                    strength     = assist_strength_from_zone(zone_label)
                    assist_steer = raw_assist * strength

                if zone_label == "ESTOP":
                    if not lidar_estop:
                        lidar_estop  = True
                        current_duty = 0.0
                        stop_car(ser)
                        print(f"\n[LIDAR] EMERGENCY STOP — obstacle at {front_dist} mm")
                elif lidar_estop:
                    lidar_estop = False
                    print(f"[LIDAR] Path clear ({front_dist} mm) — estop released")

            # Manual estop ───────────────────────────────────
            if estop:
                current_duty = 0.0
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                last_map = maybe_render_map(
                    map_display, lidar, clearances, assist_steer,
                    zone_label, front_dist, assist_on, last_map, map_period)
                time.sleep(loop_period)
                continue

            # LiDAR ESTOP — reverse + assisted steering only ─
            if lidar_estop:
                drive_raw = joystick.get_axis(AXIS_DRIVE)
                steer_raw = joystick.get_axis(AXIS_STEERING)
                drive     = drive_input_from_stick(drive_raw)

                servo_pos    = steering_to_servo(steer_raw,
                                                 assist_steer if assist_on else 0.0)
                target_duty  = drive * MAX_DUTY if drive < 0 else 0.0
                current_duty = ramp_value(current_duty, target_duty, DUTY_RAMP_STEP)

                send_servo(ser, servo_pos)
                if abs(current_duty) > 0.002:
                    send_duty(ser, current_duty)
                else:
                    current_duty = 0.0
                    send_current_zero(ser)

                now = time.time()
                if now - last_print >= print_period:
                    fd_str = f"{front_dist}mm" if front_dist else "--"
                    print(
                        f"\r[ESTOP|REVERSE+ASSIST] "
                        f"LeftY:{drive_raw:+.2f} "
                        f"Duty:{current_duty:+.3f} "
                        f"Front:{fd_str} "
                        f"Assist:{assist_steer:+.2f}     ",
                        end=""
                    )
                    last_print = now

                last_map = maybe_render_map(
                    map_display, lidar, clearances, assist_steer,
                    zone_label, front_dist, assist_on, last_map, map_period)

                elapsed = time.time() - loop_start
                if elapsed < loop_period:
                    time.sleep(loop_period - elapsed)
                continue

            # Warmup ─────────────────────────────────────────
            if warmup_counter < WARMUP_LOOPS:
                warmup_counter += 1
                current_duty = (drive_input_from_stick(joystick.get_axis(AXIS_DRIVE))
                                * MAX_DUTY)
                send_current_zero(ser)
                send_servo(ser, SERVO_CENTER)
                time.sleep(loop_period)
                continue

            # Normal drive ───────────────────────────────────
            drive_raw = joystick.get_axis(AXIS_DRIVE)
            steer_raw = joystick.get_axis(AXIS_STEERING)
            drive     = drive_input_from_stick(drive_raw)
            servo_pos = steering_to_servo(steer_raw,
                                          assist_steer if assist_on else 0.0)

            target_duty = drive * MAX_DUTY
            if target_duty > speed_cap:
                target_duty = speed_cap

            target_duty  = safe_reverse_guard(current_duty, target_duty)
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
                fd_str = (f"Front:{front_dist}mm Zone:{zone_label}"
                          if front_dist else "Front:--")
                a_str  = f"Assist:{assist_steer:+.2f}" if assist_on else "Assist:OFF"
                print(
                    f"\r[{mode}] "
                    f"LeftY:{drive_raw:+.2f} "
                    f"RightX:{steer_raw:+.2f} "
                    f"Servo:{servo_pos:.2f} "
                    f"Duty:{current_duty:+.3f} "
                    f"Cap:{speed_cap:.2f} "
                    f"{fd_str} "
                    f"{a_str}     ",
                    end=""
                )
                last_print = now

            last_map = maybe_render_map(
                map_display, lidar, clearances, assist_steer,
                zone_label, front_dist, assist_on, last_map, map_period)

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
