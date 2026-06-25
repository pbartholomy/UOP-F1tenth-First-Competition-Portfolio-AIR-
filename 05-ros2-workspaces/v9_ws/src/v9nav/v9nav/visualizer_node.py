#!/usr/bin/env python3
"""
visualizer_node.py — real-time LiDAR polar plot + ZED RGB display.

Two windows:
  "LiDAR"      — overhead polar plot, color-coded by distance, steer direction overlay
  "ZED Camera" — live RGB from ZED 2i (shown only if pyzed is available)

Press Q or Esc in either window to quit.
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from ackermann_msgs.msg import AckermannDriveStamped

try:
    import cv2
    _CV2 = True
except ImportError:
    _CV2 = False

# ── Display settings ──────────────────────────────────────────
LIDAR_WIN    = "LiDAR"
LIDAR_SIZE   = 620          # canvas width/height (px)
RANGE_SCALE  = 3500         # mm shown at canvas edge
DISPLAY_FPS  = 20
SERVO_CENTER = 0.50
SERVO_THROW  = 0.35         # matches corridor_node

MODE_NAMES = {0: "AUTONOMOUS", 1: "MANUAL", 2: "MAPPING", 3: "PURE PURSUIT"}


# ── ROS node (subscribes to topics published by corridor_node) ─

class VisualizerNode(Node):
    def __init__(self):
        super().__init__("visualizer_node")
        self._scan  = None
        self._mode  = 0
        self._servo = SERVO_CENTER
        self._duty  = 0.0
        self._erpm  = 0
        self._lock  = threading.Lock()

        self.create_subscription(LaserScan,           "/scan",       self._scan_cb,  10)
        self.create_subscription(Int32,               "/drive_mode", self._mode_cb,  10)
        self.create_subscription(AckermannDriveStamped, "/drive",    self._drive_cb, 10)

    def _scan_cb(self, msg: LaserScan):
        with self._lock:
            self._scan = msg

    def _mode_cb(self, msg: Int32):
        self._mode = msg.data

    def _drive_cb(self, msg: AckermannDriveStamped):
        self._servo = msg.drive.steering_angle
        self._duty  = msg.drive.speed


# ── LiDAR drawing ─────────────────────────────────────────────

def _draw_lidar(scan, mode, servo, duty) -> np.ndarray:
    S  = LIDAR_SIZE
    cx = cy = S // 2
    img = np.zeros((S, S, 3), dtype=np.uint8)

    # Range rings
    for r_mm in [500, 1000, 1500, 2000, 2500, 3000]:
        r_px = int(r_mm / RANGE_SCALE * (S // 2))
        cv2.circle(img, (cx, cy), r_px, (45, 45, 45), 1)
        label_x = cx + r_px + 4
        if label_x < S - 30:
            cv2.putText(img, f"{r_mm//1000}.{(r_mm % 1000)//100}m",
                        (label_x, cy - 3),
                        cv2.FONT_HERSHEY_PLAIN, 0.85, (65, 65, 65), 1)

    # Forward marker line (faint)
    cv2.line(img, (cx, cy), (cx, cy - S // 2), (30, 30, 30), 1)

    # LiDAR scan points
    if scan is not None:
        angle = scan.angle_min
        pts_close = []
        pts_mid   = []
        pts_far   = []
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                r_mm = r * 1000.0
                r_px = int(r_mm / RANGE_SCALE * (S // 2))
                # ROS convention: angle 0 = forward, positive = left
                px = cx + int(r_px * math.sin(angle))
                py = cy - int(r_px * math.cos(angle))
                if 0 <= px < S and 0 <= py < S:
                    pt = (px, py)
                    if r_mm < 600:
                        pts_close.append(pt)
                    elif r_mm < 1400:
                        pts_mid.append(pt)
                    else:
                        pts_far.append(pt)
            angle += scan.angle_increment

        for pt in pts_far:
            cv2.circle(img, pt, 2, (0, 200, 80), -1)    # green — far/safe
        for pt in pts_mid:
            cv2.circle(img, pt, 2, (0, 180, 220), -1)   # cyan — medium
        for pt in pts_close:
            cv2.circle(img, pt, 3, (0, 60, 255), -1)    # red — close/danger

    # Steer direction arrow
    steer_norm  = (servo - SERVO_CENTER) / SERVO_THROW   # -1..+1
    steer_angle = steer_norm * 0.45                       # ~26° max physical angle
    arrow_len   = 90
    ex = cx + int(arrow_len * math.sin(steer_angle))
    ey = cy - int(arrow_len * math.cos(steer_angle))
    cv2.arrowedLine(img, (cx, cy), (ex, ey), (0, 220, 255), 2, tipLength=0.25)

    # Car dot
    cv2.circle(img, (cx, cy), 7, (255, 255, 255), -1)

    # Text overlay
    mode_label = MODE_NAMES.get(mode, str(mode))
    color_mode = (80, 255, 80) if mode == 0 else (255, 180, 0)
    cv2.putText(img, mode_label,        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_mode, 1)
    cv2.putText(img, f"duty  {duty:+.3f}", (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)
    cv2.putText(img, f"servo {servo:.3f}", (8, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1)

    # Legend
    cv2.circle(img, (S - 110, S - 50), 4, (0, 200, 80),  -1); cv2.putText(img, ">1.4m", (S-100, S-46), cv2.FONT_HERSHEY_PLAIN, 0.9, (140,140,140), 1)
    cv2.circle(img, (S - 110, S - 32), 4, (0, 180, 220), -1); cv2.putText(img, "0.6-1.4m", (S-100, S-28), cv2.FONT_HERSHEY_PLAIN, 0.9, (140,140,140), 1)
    cv2.circle(img, (S - 110, S - 14), 4, (0, 60, 255),  -1); cv2.putText(img, "<0.6m", (S-100, S-10), cv2.FONT_HERSHEY_PLAIN, 0.9, (140,140,140), 1)

    return img


# ── Main ──────────────────────────────────────────────────────

def main(args=None):
    if not _CV2:
        print("[visualizer] ERROR: opencv-python not installed")
        return

    rclpy.init(args=args)
    node = VisualizerNode()

    cv2.namedWindow(LIDAR_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(LIDAR_WIN, LIDAR_SIZE, LIDAR_SIZE)

    node.get_logger().info("Visualizer ready (LiDAR) — press Q or Esc to quit")
    node.get_logger().info("ZED display is handled by zed_obstacle_node")

    period = 1.0 / DISPLAY_FPS
    try:
        while rclpy.ok():
            t0 = time.time()
            rclpy.spin_once(node, timeout_sec=0)

            # LiDAR window
            with node._lock:
                scan = node._scan
            lidar_frame = _draw_lidar(scan, node._mode, node._servo, node._duty)
            cv2.imshow(LIDAR_WIN, lidar_frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break

            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[visualizer] Done")


if __name__ == "__main__":
    main()
