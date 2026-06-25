#!/usr/bin/env python3
"""
zed_node.py — ROS 2 node for ZED 2i camera (Roboracer / F1TENTH)

Publishes:
  /zed/left/image_raw   (sensor_msgs/msg/Image, bgr8)
  /zed/depth/image_raw  (sensor_msgs/msg/Image, 32FC1, metres)

Also shows a live cv2 window:
  Left  half — RGB view
  Right half — colourised depth map

Close the window or press Q / Ctrl+C to quit.

Run:
  source /opt/ros/humble/setup.bash
  python3 ~/Desktop/zed_node.py
"""

import sys
import threading
import time

import cv2
import numpy as np
import pyzed.sl as sl
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# ============================================================
# ZED PARAMETERS  (match nav code)
# ============================================================

ZED_RESOLUTION  = sl.RESOLUTION.HD720   # 1280×720
ZED_FPS         = 30
ZED_DEPTH_MODE  = sl.DEPTH_MODE.NEURAL_LIGHT   # PERFORMANCE is deprecated
ZED_COORD_UNITS = sl.UNIT.METER

DISPLAY_W = 640   # each half of the side-by-side window
DISPLAY_H = 360

# ============================================================
# ZED GRAB THREAD
# ============================================================

class ZEDCamera:
    def __init__(self):
        self._cam      = sl.Camera()
        self._lock     = threading.Lock()
        self._bgr      = None
        self._depth    = None
        self._display  = None   # pre-built side-by-side display frame
        self._running  = False
        self._thread   = None
        self.connected = False
        self.model     = ""
        self.serial    = ""
        self._fps_t    = time.time()
        self._fps_cnt  = 0
        self.fps       = 0.0

    def connect(self):
        init = sl.InitParameters()
        init.camera_resolution = ZED_RESOLUTION
        init.camera_fps        = ZED_FPS
        init.depth_mode        = ZED_DEPTH_MODE
        init.coordinate_units  = ZED_COORD_UNITS
        status = self._cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED open failed: {status}")
        info = self._cam.get_camera_information()
        self.model  = str(info.camera_model)
        self.serial = str(info.serial_number)
        self.connected = True

    def start(self):
        self._running = True
        self._thread  = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._cam:
            self._cam.close()
        self.connected = False

    def get_frames(self):
        """Return (bgr, depth) copies for ROS publishing — called at 30 Hz."""
        with self._lock:
            if self._bgr is None:
                return None, None
            return self._bgr.copy(), self._depth.copy()

    def get_display(self):
        """Return the pre-built display frame — no heavy processing in main thread."""
        with self._lock:
            return self._display

    # ── internal ───────────────────────────────────────────────

    def _run(self):
        runtime   = sl.RuntimeParameters()
        image_mat = sl.Mat()
        depth_mat = sl.Mat()

        while self._running:
            if self._cam.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.005)
                continue

            self._cam.retrieve_image(image_mat,   sl.VIEW.LEFT)
            self._cam.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

            bgra  = image_mat.get_data()
            depth = np.asarray(depth_mat.get_data(), dtype=np.float32)
            bgr   = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)

            # Build the display frame here so the main thread does zero processing
            display = _build_display(bgr, depth, self)

            with self._lock:
                self._bgr     = bgr
                self._depth   = depth
                self._display = display

            # Rolling FPS counter
            self._fps_cnt += 1
            now = time.time()
            if now - self._fps_t >= 1.0:
                self.fps      = self._fps_cnt / (now - self._fps_t)
                self._fps_cnt = 0
                self._fps_t   = now

# ============================================================
# ROS 2 NODE
# ============================================================

class ZEDNode(Node):
    def __init__(self, cam: ZEDCamera):
        super().__init__("zed_node")
        self._cam = cam

        self._img_pub   = self.create_publisher(Image, "/zed/left/image_raw",  10)
        self._depth_pub = self.create_publisher(Image, "/zed/depth/image_raw", 10)

        # Publish at ZED capture rate
        self.create_timer(1.0 / ZED_FPS, self._publish)
        self._last_log = 0.0

    def _publish(self):
        bgr, depth = self._cam.get_frames()
        if bgr is None:
            return

        stamp = self.get_clock().now().to_msg()
        self._img_pub.publish(  self._make_image_msg(bgr,   stamp, "bgr8"))
        self._depth_pub.publish(self._make_depth_msg(depth, stamp))

        t = time.time()
        if t - self._last_log > 5.0:
            self._last_log = t
            self.get_logger().info(
                f"Publishing — /zed/left/image_raw + /zed/depth/image_raw  "
                f"({bgr.shape[1]}×{bgr.shape[0]}  {self._cam.fps:.1f} fps)"
            )

    @staticmethod
    def _make_image_msg(bgr: np.ndarray, stamp, encoding: str) -> Image:
        h, w = bgr.shape[:2]
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = "zed_left_camera_frame"
        msg.height    = h
        msg.width     = w
        msg.encoding  = encoding
        msg.is_bigendian = False
        msg.step      = w * 3
        msg.data      = bgr.tobytes()
        return msg

    @staticmethod
    def _make_depth_msg(depth: np.ndarray, stamp) -> Image:
        h, w = depth.shape
        # Replace non-finite values with 0 so downstream nodes don't choke
        d = np.where(np.isfinite(depth), depth, 0.0).astype(np.float32)
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = "zed_left_camera_frame"
        msg.height    = h
        msg.width     = w
        msg.encoding  = "32FC1"
        msg.is_bigendian = False
        msg.step      = w * 4
        msg.data      = d.tobytes()
        return msg

# ============================================================
# DISPLAY HELPERS
# ============================================================

def _colorise_depth(depth: np.ndarray) -> np.ndarray:
    """Convert float32 depth (m) to a colourised uint8 BGR image."""
    valid = np.isfinite(depth) & (depth > 0.1) & (depth < 10.0)
    norm  = np.zeros_like(depth, dtype=np.float32)
    if valid.any():
        d_min, d_max = depth[valid].min(), depth[valid].max()
        if d_max > d_min:
            norm[valid] = (depth[valid] - d_min) / (d_max - d_min)
    grey = (norm * 255).astype(np.uint8)
    return cv2.applyColorMap(grey, cv2.COLORMAP_TURBO)


def _build_display(bgr: np.ndarray, depth: np.ndarray, cam: ZEDCamera) -> np.ndarray:
    left  = cv2.resize(bgr,                          (DISPLAY_W, DISPLAY_H))
    right = cv2.resize(_colorise_depth(depth),       (DISPLAY_W, DISPLAY_H))

    # ── overlays ──────────────────────────────────────────────
    h_c, w_c = depth.shape[0] // 2, depth.shape[1] // 2
    cx_depth  = depth[h_c - 5:h_c + 5, w_c - 5:w_c + 5]
    valid     = cx_depth[np.isfinite(cx_depth) & (cx_depth > 0)]
    cx_m      = float(np.mean(valid)) if valid.size else float("nan")

    def _put(img, text, y, col=(255, 255, 255)):
        cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col,   1, cv2.LINE_AA)

    _put(left,  f"{cam.model}  S/N {cam.serial}", 24)
    _put(left,  f"{cam.fps:.1f} fps  |  {bgr.shape[1]}x{bgr.shape[0]}", 48)
    _put(left,  "/zed/left/image_raw", DISPLAY_H - 10, (100, 220, 100))

    cx_str = f"{cx_m:.2f} m" if np.isfinite(cx_m) else "--"
    _put(right, f"Centre depth: {cx_str}", 24, (80, 200, 255))
    _put(right, "/zed/depth/image_raw  (TURBO)", DISPLAY_H - 10, (100, 220, 100))

    # crosshair on depth view
    cv2.drawMarker(right, (DISPLAY_W // 2, DISPLAY_H // 2),
                   (255, 255, 255), cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)

    # divider
    side = np.hstack([left, right])
    cv2.line(side, (DISPLAY_W, 0), (DISPLAY_W, DISPLAY_H), (60, 60, 60), 2)
    return side

# ============================================================
# ENTRY POINT
# ============================================================

def main():
    rclpy.init(args=sys.argv)

    cam = ZEDCamera()
    try:
        cam.connect()
    except Exception as e:
        print(f"[ERROR] {e}")
        rclpy.shutdown()
        return

    print(f"[ZED] Connected: {cam.model}  S/N {cam.serial}")
    cam.start()

    node = ZEDNode(cam)
    print("[INFO] zed_node running")
    print("       /zed/left/image_raw  — bgr8")
    print("       /zed/depth/image_raw — 32FC1 (metres)")
    print("       Press Q in window or Ctrl+C to quit\n")

    cv2.namedWindow("ZED 2i — sanity check", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("ZED 2i — sanity check", DISPLAY_W * 2, DISPLAY_H)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)

            frame = cam.get_display()
            if frame is not None:
                cv2.imshow("ZED 2i — sanity check", frame)

            # waitKey(33) = ~30 fps display rate, matches the camera
            key = cv2.waitKey(33) & 0xFF
            if key in (ord("q"), ord("Q"), 27):   # Q or Esc
                break

    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()
        print("\n[INFO] Shutdown complete")


if __name__ == "__main__":
    main()
