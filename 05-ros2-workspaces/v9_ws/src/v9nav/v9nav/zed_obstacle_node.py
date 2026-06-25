#!/usr/bin/env python3
"""
zed_obstacle_node.py — v9: ZED 2i depth obstacle avoidance + live display + recording.

Uses pyzed directly (no zed_wrapper needed).

Display window (shown live while running):
  - Per-column depth bars across the scan strip (green=open, red=blocked)
  - Green rectangle + centre line on widest open gap
  - Front / left / right obstacle distances as text overlay
  - Mode label from /drive_mode

Publishes (fused with LiDAR by corridor_node):
  /zed/obstacle_front   Float32  m
  /zed/obstacle_left    Float32  m
  /zed/obstacle_right   Float32  m

Video with overlay saved to ~/Desktop/v9Video/videoN.mp4 on each run.
"""

import os
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32

OUTPUT_DIR  = os.path.expanduser("~/Desktop/v9Video")
SHOW_WINDOW = True   # set False if running headless

MODE_AUTONOMOUS = 0  # matches mode_manager_node / corridor_node convention

try:
    import pyzed.sl as sl
    ZED_OK = True
except Exception:
    sl = None
    ZED_OK = False

try:
    import cv2
    CV2_OK = True
except Exception:
    cv2 = None
    CV2_OK = False

# ── Scan band (fraction of image height, centred) ─────────────
BAND_TOP    = 0.38     # top of the scan strip
BAND_BOT    = 0.62     # bottom of the scan strip
NCOLS       = 40       # number of depth-sample columns
OPEN_M      = 1.5      # columns deeper than this = open
CORNER_M    = 1.5      # front clearance below this → GAP mode label

DEPTH_MIN_M = 0.30
DEPTH_MAX_M = 6.0

# Side/front ROI for obstacle publishers
FRONT_COL_MIN = 0.30
FRONT_COL_MAX = 0.70
SIDE_COL_FRAC = 0.25

PUBLISH_HZ = 30.0


class ZedGapNode(Node):
    def __init__(self):
        super().__init__("zed_obstacle_node")

        if not ZED_OK:
            self.get_logger().error("pyzed not available — ZedGapNode cannot start")
            return
        if not CV2_OK:
            self.get_logger().error("opencv-python not available — ZedGapNode cannot start")
            return

        self.declare_parameter("fps",    30.0)
        self.declare_parameter("width",  672)
        self.declare_parameter("height", 376)
        self._fps = float(self.get_parameter("fps").value)
        self._w   = int(self.get_parameter("width").value)
        self._h   = int(self.get_parameter("height").value)

        # ── ROS publishers ────────────────────────────────────
        self._pub_front = self.create_publisher(Float32, "/zed/obstacle_front", 10)
        self._pub_left  = self.create_publisher(Float32, "/zed/obstacle_left",  10)
        self._pub_right = self.create_publisher(Float32, "/zed/obstacle_right", 10)

        # Camera/recording is deferred until drive_mode first reports AUTONOMOUS
        self._cam_started = False
        self._drive_mode  = 1   # start assumed manual
        self.create_subscription(Int32, "/drive_mode", self._mode_cb, 10)
        self.get_logger().info(
            "ZedGapNode waiting for AUTONOMOUS mode on /drive_mode before opening camera")

        if SHOW_WINDOW and CV2_OK:
            cv2.namedWindow("ZED Camera", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("ZED Camera", 672, 376)

    def _mode_cb(self, msg: Int32):
        self._drive_mode = msg.data
        if not self._cam_started and msg.data == MODE_AUTONOMOUS:
            self._start_camera()

    def _start_camera(self):
        self._cam_started = True

        # ── Video output ──────────────────────────────────────
        os.makedirs(OUTPUT_DIR, exist_ok=True)
        existing = [
            int(f[5:-4]) for f in os.listdir(OUTPUT_DIR)
            if f.startswith("video") and f.endswith(".mp4") and f[5:-4].isdigit()
        ]
        num            = max(existing) + 1 if existing else 1
        self._out_path = os.path.join(OUTPUT_DIR, f"video{num}.mp4")
        fourcc         = cv2.VideoWriter_fourcc(*"mp4v")
        self._writer   = cv2.VideoWriter(self._out_path, fourcc, self._fps,
                                         (self._w, self._h))

        # ── Open ZED ─────────────────────────────────────────
        cam  = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.VGA
        init.camera_fps        = int(self._fps)
        init.depth_mode        = sl.DEPTH_MODE.PERFORMANCE
        init.coordinate_units  = sl.UNIT.METER

        status = cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"ZED open failed: {status}")
            self._writer.release()
            return

        self._cam     = cam
        self._running = True
        self._lock    = threading.Lock()
        self._color   = None
        self._depth   = None

        self._thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._thread.start()

        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.get_logger().info(
            f"ZedGapNode: AUTONOMOUS — recording to {self._out_path}  "
            f"({self._w}x{self._h} @ {self._fps:.0f}fps)"
        )

    # ── Grab thread ───────────────────────────────────────────

    def _grab_loop(self):
        runtime   = sl.RuntimeParameters()
        img_mat   = sl.Mat()
        depth_mat = sl.Mat()

        while self._running:
            if self._cam.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.005)
                continue

            self._cam.retrieve_image(img_mat, sl.VIEW.LEFT)
            self._cam.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

            bgra  = img_mat.get_data()
            depth = np.asarray(depth_mat.get_data(), dtype=np.float32).copy()

            if bgra is None:
                continue

            bgr = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
            with self._lock:
                self._color = bgr
                self._depth = depth

    # ── Process + publish + draw ──────────────────────────────

    def _tick(self):
        with self._lock:
            color = self._color
            depth = self._depth

        if color is None or depth is None:
            return

        dh, dw = depth.shape

        # Sanitize: invalid/NaN pixels → 0.0 (below OPEN_M) so they register as
        # blocked in the column scan, not as "6m clear space" like DEPTH_MAX_M would.
        # _nearest_m ignores them (filters >= DEPTH_MIN_M), so obstacle publishing
        # is unaffected.
        depth_clean = np.where(
            np.isfinite(depth) & (depth >= DEPTH_MIN_M) & (depth <= DEPTH_MAX_M),
            depth, 0.0
        )

        # ── Obstacle distances (for corridor_node fallback) ───
        r0   = int(dh * 0.25)
        r1   = int(dh * 0.75)
        band_full = depth_clean[r0:r1, :]

        fc0     = int(dw * FRONT_COL_MIN)
        fc1     = int(dw * FRONT_COL_MAX)
        sc      = int(dw * SIDE_COL_FRAC)
        front_m = self._nearest_m(band_full, fc0, fc1)
        left_m  = self._nearest_m(band_full, 0,   sc)
        right_m = self._nearest_m(band_full, dw - sc, dw)

        self._pub_front.publish(Float32(data=front_m))
        self._pub_left.publish( Float32(data=left_m))
        self._pub_right.publish(Float32(data=right_m))

        # ── Column scan across the centre band ───────────────
        sy0 = int(dh * BAND_TOP)
        sy1 = int(dh * BAND_BOT)
        col_w = max(1, dw // NCOLS)

        col_depth = []
        for i in range(NCOLS):
            xc0   = i * col_w
            xc1   = min(dw, xc0 + col_w)
            patch = depth_clean[sy0:sy1, xc0:xc1]
            valid = patch[(patch >= DEPTH_MIN_M) & (patch < DEPTH_MAX_M)]
            col_depth.append(float(np.min(valid)) if valid.size else DEPTH_MAX_M)

        open_mask = [d > OPEN_M for d in col_depth]

        # Find widest open run
        best_start, best_len = 0, 0
        cur_start, cur_len   = 0, 0
        for i, is_open in enumerate(open_mask):
            if is_open:
                if cur_len == 0:
                    cur_start = i
                cur_len += 1
                if cur_len > best_len:
                    best_len   = cur_len
                    best_start = cur_start
            else:
                cur_len = 0

        mode = "GAP" if front_m < CORNER_M else "CORRIDOR"

        # ── Draw overlay on colour frame ──────────────────────
        frame = color.copy()
        if frame.shape[1] != self._w or frame.shape[0] != self._h:
            frame     = cv2.resize(frame, (self._w, self._h))
            scale_x   = self._w / dw
            scale_y   = self._h / dh
        else:
            scale_x = scale_y = 1.0

        fw, fh = frame.shape[1], frame.shape[0]
        y0 = int(sy0 * scale_y)
        y1 = int(sy1 * scale_y)
        fw_col = max(1, fw // NCOLS)

        # Per-column depth bars at the bottom of the scan band
        bar_h  = max(1, (y1 - y0) // 5)
        bar_y0 = y1 - bar_h
        bar_y1 = y1
        for i, d in enumerate(col_depth):
            xc0 = i * fw_col
            xc1 = min(fw, xc0 + fw_col) - 1
            if open_mask[i]:
                frac  = min(1.0, (d - OPEN_M) / max(0.01, DEPTH_MAX_M - OPEN_M))
                color_bar = (0, int(100 + 120 * frac), 0)
            else:
                color_bar = (100, 0, 30)
            cv2.rectangle(frame, (xc0, bar_y0), (xc1, bar_y1), color_bar, -1)

        # Scan band outline
        cv2.rectangle(frame, (0, y0), (fw - 1, y1), (160, 160, 160), 1)

        # Green rectangle around widest open gap + centre line
        if best_len > 0:
            open_x0 = best_start * fw_col
            open_x1 = min(fw, (best_start + best_len) * fw_col)
            cx_px   = (open_x0 + open_x1) // 2
            cv2.rectangle(frame, (open_x0, y0), (open_x1, bar_y0), (0, 220, 0), 2)
            cv2.line(frame, (cx_px, y0), (cx_px, bar_y0), (0, 255, 0), 2)

        # Image centre reference line (white, thin)
        cv2.line(frame, (fw // 2, y0), (fw // 2, y1), (220, 220, 220), 1)

        # Text labels
        def _txt(img, text, pt, col=(0, 255, 0)):
            cv2.putText(img, text, pt, cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, text, pt, cv2.FONT_HERSHEY_SIMPLEX,
                        0.52, col,       1, cv2.LINE_AA)

        mode_names = {0: "AUTONOMOUS", 1: "MANUAL", 2: "MAPPING"}
        mode_str   = mode_names.get(self._drive_mode, str(self._drive_mode))
        gap_pct    = int(best_len / NCOLS * 100)
        _txt(frame, f"[{mode_str}] {mode}  gap={gap_pct}%  F:{front_m:.1f}m", (8, y0 - 6))
        _txt(frame, f"L:{left_m:.1f}m  R:{right_m:.1f}m",
             (8, y1 + 16), (80, 210, 255))

        # Live display window
        if SHOW_WINDOW and CV2_OK:
            cv2.imshow("ZED Camera", frame)
            cv2.waitKey(1)

        self._writer.write(frame)

    # ── Helpers ───────────────────────────────────────────────

    @staticmethod
    def _nearest_m(band: np.ndarray, c0: int, c1: int) -> float:
        region = band[:, c0:c1]
        valid  = region[(region >= DEPTH_MIN_M) & (region < DEPTH_MAX_M)]
        return float(np.percentile(valid, 5)) if valid.size else DEPTH_MAX_M

    # ── Cleanup ───────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        if hasattr(self, "_thread"):
            self._thread.join(timeout=3.0)
        if hasattr(self, "_cam"):
            try:
                self._cam.close()
            except Exception:
                pass
        if hasattr(self, "_writer"):
            self._writer.release()
            self.get_logger().info(f"Saved {self._out_path}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
