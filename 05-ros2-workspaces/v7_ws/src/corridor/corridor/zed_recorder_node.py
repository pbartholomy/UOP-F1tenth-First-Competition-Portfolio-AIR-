#!/usr/bin/env python3
"""
zed_recorder_node.py — records ZED 2i left color stream to MP4 using pyzed.

Opens the camera directly (no zed_wrapper needed).  Saves auto-numbered
video files to ~/Desktop/v7Video/.  A new file is created each run.
"""

import os
import threading
import time
import rclpy
from rclpy.node import Node

OUTPUT_DIR = os.path.expanduser("~/Desktop/v7Video")

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


class ZedRecorderNode(Node):
    def __init__(self):
        super().__init__("zed_recorder_node")

        if not ZED_OK:
            self.get_logger().error("pyzed not available — ZedRecorderNode cannot start")
            return
        if not CV2_OK:
            self.get_logger().error("opencv-python not available — ZedRecorderNode cannot start")
            return

        self.declare_parameter("fps",    30.0)
        self.declare_parameter("width",  672)
        self.declare_parameter("height", 376)

        fps = float(self.get_parameter("fps").value)
        w   = int(self.get_parameter("width").value)
        h   = int(self.get_parameter("height").value)

        os.makedirs(OUTPUT_DIR, exist_ok=True)
        existing = [
            int(f[5:-4]) for f in os.listdir(OUTPUT_DIR)
            if f.startswith("video") and f.endswith(".mp4") and f[5:-4].isdigit()
        ]
        num      = max(existing) + 1 if existing else 1
        out_path = os.path.join(OUTPUT_DIR, f"video{num}.mp4")

        fourcc       = cv2.VideoWriter_fourcc(*"mp4v")
        self._writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h))
        self._out_path = out_path
        self._w = w
        self._h = h

        cam  = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.VGA
        init.camera_fps        = int(fps)
        init.depth_mode        = sl.DEPTH_MODE.NONE
        init.coordinate_units  = sl.UNIT.METER

        status = cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"ZED open failed: {status} — recorder disabled")
            self._writer.release()
            return

        self._cam     = cam
        self._running = True

        self._thread = threading.Thread(target=self._grab_loop, args=(fps,), daemon=True)
        self._thread.start()

        self.get_logger().info(f"ZedRecorderNode: recording to {out_path}  ({w}x{h} @ {fps:.0f}fps)")

    def _grab_loop(self, fps: float):
        import numpy as np
        runtime   = sl.RuntimeParameters()
        img_mat   = sl.Mat()
        period    = 1.0 / fps

        while self._running:
            t0 = time.time()
            if self._cam.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.005)
                continue

            self._cam.retrieve_image(img_mat, sl.VIEW.LEFT)
            arr = img_mat.get_data()          # BGRA uint8

            if arr is None:
                continue

            import numpy as np
            frame = cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
            if frame.shape[1] != self._w or frame.shape[0] != self._h:
                frame = cv2.resize(frame, (self._w, self._h))

            self._writer.write(frame)

            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

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
            self.get_logger().info(f"ZedRecorderNode: saved {self._out_path}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedRecorderNode()
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
