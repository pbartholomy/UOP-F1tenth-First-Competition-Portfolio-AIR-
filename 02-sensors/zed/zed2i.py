"""
ZED 2i camera interface for the Roboracer/F1TENTH autonomy system.

Hardware:
  Model   : ZED 2i  (Stereolabs, VID 2b03 PID f880)
  Serial  : 36476709
  USB     : Bus 2, port 1.3 (USB 3.0, Jetson controller 3610000.usb)
  Devices : /dev/video0 (left), /dev/video1 (right)
"""

import pyzed.sl as sl
import numpy as np
import cv2


class ZED2i:
    # USB 3.0 path on the Jetson carrier board — port 1.3
    USB_PATH = "platform-3610000.usb-usb-0:1.3:1.0"
    SERIAL_NUMBER = 36476709

    def __init__(
        self,
        resolution: sl.RESOLUTION = sl.RESOLUTION.HD720,
        fps: int = 30,
        depth_mode: sl.DEPTH_MODE = sl.DEPTH_MODE.NEURAL,
        units: sl.UNIT = sl.UNIT.METER,
    ):
        self._cam = sl.Camera()
        self._runtime = sl.RuntimeParameters()

        init = sl.InitParameters()
        init.camera_resolution = resolution
        init.camera_fps = fps
        init.depth_mode = depth_mode
        init.coordinate_units = units
        self._init_params = init

        self._image = sl.Mat()
        self._depth = sl.Mat()
        self._point_cloud = sl.Mat()
        self._opened = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> bool:
        status = self._cam.open(self._init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            print(f"[ZED2i] Failed to open: {status}")
            return False
        self._opened = True
        info = self.camera_info
        print(
            f"[ZED2i] Opened — {info.camera_model} "
            f"S/N {info.serial_number} "
            f"FW {info.camera_configuration.firmware_version}"
        )
        return True

    def close(self):
        if self._opened:
            self._cam.close()
            self._opened = False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    # ------------------------------------------------------------------
    # Grab
    # ------------------------------------------------------------------

    def grab(self) -> bool:
        return self._cam.grab(self._runtime) == sl.ERROR_CODE.SUCCESS

    # ------------------------------------------------------------------
    # Data retrieval
    # ------------------------------------------------------------------

    def get_left_image(self) -> np.ndarray:
        """Returns the left rectified image as an BGRA uint8 array (H, W, 4)."""
        self._cam.retrieve_image(self._image, sl.VIEW.LEFT)
        return self._image.get_data()

    def get_depth_map(self) -> np.ndarray:
        """Returns the depth map in the configured units (H, W) float32. NaN = invalid."""
        self._cam.retrieve_measure(self._depth, sl.MEASURE.DEPTH)
        return self._depth.get_data()

    def get_point_cloud(self) -> np.ndarray:
        """Returns the XYZRGBA point cloud (H, W, 4) float32."""
        self._cam.retrieve_measure(self._point_cloud, sl.MEASURE.XYZRGBA)
        return self._point_cloud.get_data()

    def get_depth_at(self, x: int, y: int) -> float:
        """Returns depth in configured units at pixel (x, y). NaN if invalid."""
        err, value = self._depth.get_value(x, y)
        return float(value) if err == sl.ERROR_CODE.SUCCESS else float("nan")

    # ------------------------------------------------------------------
    # Info
    # ------------------------------------------------------------------

    @property
    def camera_info(self) -> sl.CameraInformation:
        return self._cam.get_camera_information()

    @property
    def resolution(self):
        r = self.camera_info.camera_configuration.resolution
        return r.width, r.height


# ------------------------------------------------------------------
# Live viewer
# ------------------------------------------------------------------

def _view():
    with ZED2i() as cam:
        w, h = cam.resolution
        print(f"[ZED2i] Resolution: {w}x{h}")
        print("[ZED2i] Live view started — press Q to quit.")
        cv2.namedWindow("ZED 2i — Left Camera", cv2.WINDOW_NORMAL)

        while True:
            if not cam.grab():
                continue

            frame = cam.get_left_image()          # BGRA (H, W, 4)
            bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            cx, cy = w // 2, h // 2
            d = cam.get_depth_at(cx, cy)
            label = f"Center depth: {d:.3f} m" if not np.isnan(d) else "Center depth: --"
            cv2.putText(bgr, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.putText(bgr, "Press Q to quit", (20, h - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)

            cv2.imshow("ZED 2i — Left Camera", bgr)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        cv2.destroyAllWindows()
    print("[ZED2i] Viewer closed.")


if __name__ == "__main__":
    _view()
