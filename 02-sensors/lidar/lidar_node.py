#!/usr/bin/env python3
"""
lidar_node.py — ROS 2 node for Hokuyo URG-04LX (Roboracer / F1TENTH)

Reads scans over serial (SCIP2.0) and publishes:
  /scan   (sensor_msgs/msg/LaserScan)

Also shows a live 600×600 pygame bird's-eye 2D map (same style as
SteeringAssist.py) — close the window or press Ctrl+C to quit.

Run:
  source /opt/ros/humble/setup.bash
  python3 ~/Desktop/lidar_node.py
"""

import math
import sys
import threading
import time

import pygame
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ============================================================
# HOKUYO URG-04LX PARAMETERS  (match nav code)
# ============================================================

LIDAR_PORT     = "/dev/serial/by-id/usb-Hokuyo_Data_Flex_for_USB_URG-Series_USB_Driver-if00"
LIDAR_BAUDRATE = 19200

STEP_MIN   = 44
STEP_MAX   = 667
STEP_FRONT = 384

STEP_INTERVAL_RAD = 2.0 * math.pi / 1024

ANGLE_MIN = (STEP_MIN - STEP_FRONT) * STEP_INTERVAL_RAD
ANGLE_MAX = (STEP_MAX - STEP_FRONT) * STEP_INTERVAL_RAD

RANGE_MIN_M = 0.020
RANGE_MAX_M = 5.600

FRAME_ID = "laser"

# ============================================================
# 2D MAP DISPLAY SETTINGS  (match SteeringAssist.py)
# ============================================================

MAP_W     = 600
MAP_H     = 600
MAP_SCALE = 0.10   # px per mm  (1 px = 10 mm, ±3 000 mm visible)

ZONE_FULL_MM     = 2500
ZONE_MODERATE_MM = 1500
ZONE_SLOW_MM     =  800

SECTORS = [
    ("FAR_LEFT",     -90, -45),
    ("LEFT",         -45, -15),
    ("CENTER_LEFT",  -15,  -5),
    ("CENTER",        -5,   5),
    ("CENTER_RIGHT",   5,  15),
    ("RIGHT",         15,  45),
    ("FAR_RIGHT",     45,  90),
]

# ============================================================
# PRECOMPUTED ANGLE TABLES
# ============================================================

_N_STEPS = STEP_MAX - STEP_MIN + 1


def _build_angle_tables():
    sins, coss = [], []
    for idx in range(_N_STEPS):
        a = (idx + STEP_MIN - STEP_FRONT) * STEP_INTERVAL_RAD
        sins.append(math.sin(a))
        coss.append(math.cos(a))
    return sins, coss


def _build_sector_masks():
    masks = {name: [] for name, _, _ in SECTORS}
    for idx in range(_N_STEPS):
        deg = (idx + STEP_MIN - STEP_FRONT) * (360.0 / 1024)
        for name, lo, hi in SECTORS:
            if lo <= deg <= hi:
                masks[name].append(idx)
    return masks


_SIN, _COS = _build_angle_tables()
_SECTOR_MASKS = _build_sector_masks()

# ============================================================
# SCIP2.0 DRIVER
# ============================================================

class HokuyoDriver:
    def __init__(self, port: str, baud: int):
        self._port = port
        self._baud = baud
        self._ser: serial.Serial | None = None
        self._lock = threading.Lock()
        self._distances: list[int] = []
        self._running = False
        self._thread: threading.Thread | None = None

    def connect(self):
        self._ser = serial.Serial(self._port, self._baud, timeout=1.0)
        time.sleep(0.2)
        self._ser.write(b"SCIP2.0\n"); time.sleep(0.2)
        self._ser.reset_input_buffer()
        self._ser.write(b"BM\n");      time.sleep(0.2)
        self._ser.reset_input_buffer()

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._ser and self._ser.is_open:
            try:
                self._ser.write(b"QT\n")
            except Exception:
                pass
            self._ser.close()

    def get_distances(self) -> list[int]:
        with self._lock:
            return list(self._distances)

    def sector_clearances(self) -> dict[str, float]:
        d = self.get_distances()
        result = {}
        for name, indices in _SECTOR_MASKS.items():
            vals = [d[i] for i in indices if i < len(d) and d[i] > 20]
            result[name] = min(vals) if vals else float("inf")
        return result

    def front_min(self) -> float | None:
        d = self.get_distances()
        if not d:
            return None
        c  = STEP_FRONT - STEP_MIN
        lo = max(0, c - 50)
        hi = min(len(d) - 1, c + 50)
        vals = [v for v in d[lo:hi + 1] if v > 20]
        return min(vals) if vals else None

    # ── internal ───────────────────────────────────────────────

    def _readline(self) -> bytes:
        return self._ser.readline().rstrip(b"\n")

    def _get_scan(self) -> list[int] | None:
        cmd = f"GD{STEP_MIN:04d}{STEP_MAX:04d}01\n".encode()
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
        return self._decode(raw) if raw else None

    @staticmethod
    def _decode(raw: bytes) -> list[int]:
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

# ============================================================
# 2D MAP DISPLAY  (ported from SteeringAssist.py)
# ============================================================

class LidarMapDisplay:
    _CX = MAP_W // 2
    _CY = MAP_H // 2

    def __init__(self):
        self._surface = pygame.display.set_mode((MAP_W, MAP_H))
        pygame.display.set_caption("LiDAR 2D Map  —  lidar_node")
        self._font_sm = pygame.font.SysFont("monospace", 12)
        self._font_md = pygame.font.SysFont("monospace", 14, bold=True)

    @staticmethod
    def _w2s(x_mm, y_mm):
        sx = LidarMapDisplay._CX + int(x_mm * MAP_SCALE)
        sy = LidarMapDisplay._CY - int(y_mm * MAP_SCALE)
        return sx, sy

    @staticmethod
    def _dist_color(d):
        if d >= ZONE_FULL_MM:     return (  0, 210,  60)
        if d >= ZONE_MODERATE_MM: return (220, 220,   0)
        if d >= ZONE_SLOW_MM:     return (255, 130,   0)
        return (255, 50, 50)

    def render(self, distances, clearances, front_dist):
        s = self._surface
        s.fill((12, 12, 22))

        self._draw_grid(s)
        self._draw_rings(s)
        self._draw_sector_tints(s, clearances)
        self._draw_scan(s, distances)
        self._draw_car(s)
        self._draw_hud(s, front_dist, clearances)

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
            ( 500, ( 70,  30,  30)),
            ( 800, ( 90,  55,  20)),
            (1500, ( 75,  75,  20)),
            (2500, ( 30,  75,  30)),
        ]:
            pygame.draw.circle(s, col, (cx, cy), int(mm * MAP_SCALE), 1)

    @staticmethod
    def _draw_sector_tints(s, clearances):
        cx, cy   = LidarMapDisplay._CX, LidarMapDisplay._CY
        ARC_SEGS = 10
        for name, deg_lo, deg_hi in SECTORS:
            dist = clearances.get(name, float("inf"))
            if dist >= ZONE_FULL_MM:
                continue
            ratio = 1.0 - min(dist, float(ZONE_FULL_MM)) / float(ZONE_FULL_MM)
            glow  = int(ratio * 55)
            if dist < ZONE_SLOW_MM:
                col = (glow * 4, 0, 0)
            elif dist < ZONE_MODERATE_MM:
                col = (glow * 3, glow * 2, 0)
            else:
                col = (0, glow * 2, 0)
            max_r = max(5, int(min(dist, float(ZONE_FULL_MM)) * MAP_SCALE))
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
            (cx,     cy - ch // 2 - 10),
            (cx - 5, cy - ch // 2),
            (cx + 5, cy - ch // 2),
        ])

    def _draw_hud(self, s, front_dist, clearances):
        fd_str = f"{front_dist} mm" if front_dist else "-- mm"
        if front_dist is None:
            zone_label, zone_col = "NO DATA", (100, 100, 100)
        elif front_dist >= ZONE_FULL_MM:
            zone_label, zone_col = "FULL",     (  0, 210,  60)
        elif front_dist >= ZONE_MODERATE_MM:
            zone_label, zone_col = "MODERATE", (220, 220,   0)
        elif front_dist >= ZONE_SLOW_MM:
            zone_label, zone_col = "SLOW",     (255, 130,   0)
        else:
            zone_label, zone_col = "DANGER",   (255,  50,  50)

        y = 5
        for text, col in [
            (f"Zone:   {zone_label}", zone_col),
            (f"Front:  {fd_str}",     (200, 200, 200)),
            ( "Topic:  /scan",        ( 80, 160, 255)),
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

# ============================================================
# ROS 2 NODE
# ============================================================

class LidarNode(Node):
    def __init__(self):
        super().__init__("lidar_node")
        self._pub = self.create_publisher(LaserScan, "/scan", 10)
        self._driver = HokuyoDriver(LIDAR_PORT, LIDAR_BAUDRATE)
        self._driver.connect()
        self._driver.start()
        self.get_logger().info(f"Hokuyo URG-04LX connected on {LIDAR_PORT}")
        self._timer = self.create_timer(0.1, self._publish_scan)
        self._last_log = 0

    def _publish_scan(self):
        distances_mm = self._driver.get_distances()
        if not distances_mm:
            return

        now = self.get_clock().now().to_msg()
        n   = len(distances_mm)
        ranges = [
            d / 1000.0 if RANGE_MIN_M <= d / 1000.0 <= RANGE_MAX_M else 0.0
            for d in distances_mm
        ]

        msg = LaserScan()
        msg.header.stamp    = now
        msg.header.frame_id = FRAME_ID
        msg.angle_min       = ANGLE_MIN
        msg.angle_max       = ANGLE_MAX
        msg.angle_increment = STEP_INTERVAL_RAD
        msg.scan_time       = 0.1
        msg.time_increment  = 0.1 / max(n, 1)
        msg.range_min       = RANGE_MIN_M
        msg.range_max       = RANGE_MAX_M
        msg.ranges          = ranges
        self._pub.publish(msg)

        t = time.time()
        if t - self._last_log > 5.0:
            self._last_log = t
            self.get_logger().info(f"Publishing /scan — {n} points")

    def get_distances(self):
        return self._driver.get_distances()

    def get_clearances(self):
        return self._driver.sector_clearances()

    def get_front_min(self):
        return self._driver.front_min()

    def destroy_node(self):
        self._driver.stop()
        super().destroy_node()

# ============================================================
# ENTRY POINT  — pygame main loop + rclpy.spin_once
# ============================================================

def main():
    rclpy.init(args=sys.argv)

    try:
        node = LidarNode()
    except Exception as e:
        print(f"[ERROR] {e}")
        rclpy.shutdown()
        return

    pygame.init()
    map_display = LidarMapDisplay()

    print("[INFO] lidar_node running")
    print("       Publishing /scan  |  close window or Ctrl+C to quit\n")

    map_period = 1.0 / 15   # 15 Hz render
    last_map   = 0.0

    try:
        while rclpy.ok():
            # Pump pygame events so the window stays responsive
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            # Spin ROS once (non-blocking) to fire the publish timer
            rclpy.spin_once(node, timeout_sec=0)

            # Render map at 15 Hz
            now = time.time()
            if now - last_map >= map_period:
                last_map = now
                distances  = node.get_distances()
                clearances = node.get_clearances()
                front_dist = node.get_front_min()
                map_display.render(distances, clearances, front_dist)

            time.sleep(0.005)   # ~200 Hz loop cap; actual publish rate set by timer

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        pygame.quit()
        print("\n[INFO] Shutdown complete")


if __name__ == "__main__":
    main()
