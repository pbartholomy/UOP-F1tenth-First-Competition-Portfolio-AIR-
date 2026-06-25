"""
ZED 2i sanity check — left camera, right camera, and depth map displayed live.
Press Q to quit.

Hardware:
  Model  : ZED 2i  S/N 36476709
  USB    : Bus 2, port 1.3 (USB 3.0, Jetson 3610000.usb)
  Devices: /dev/video0 (left), /dev/video1 (right)
"""

import pyzed.sl as sl
import numpy as np
import cv2

PANEL_W = 640
PANEL_H = 360
WINDOW = "ZED 2i Sanity Check  |  Left  |  Right  |  Depth  |  Q to quit"


def colorize_depth(depth_map: np.ndarray) -> np.ndarray:
    """Convert float32 depth map to a false-color BGR image."""
    finite = depth_map[np.isfinite(depth_map)]
    if finite.size == 0:
        return np.zeros((depth_map.shape[0], depth_map.shape[1], 3), dtype=np.uint8)
    d_min, d_max = finite.min(), finite.max()
    if d_max == d_min:
        norm = np.zeros_like(depth_map, dtype=np.uint8)
    else:
        norm = np.clip(1.0 - (depth_map - d_min) / (d_max - d_min), 0.0, 1.0)
        norm = (norm * 255).astype(np.uint8)
    norm[~np.isfinite(depth_map)] = 0
    return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)


def main():
    cam = sl.Camera()

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.METER

    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ZED2i] Failed to open camera: {status}")
        return

    info = cam.get_camera_information()
    print(f"[ZED2i] {info.camera_model}  S/N {info.serial_number}  FW {info.camera_configuration.firmware_version}")
    print(f"[ZED2i] Sanity check live — press Q in the window to quit.\n")

    runtime = sl.RuntimeParameters()
    left_mat   = sl.Mat()
    right_mat  = sl.Mat()
    depth_mat  = sl.Mat()

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, PANEL_W * 3, PANEL_H + 40)

    frame_count = 0

    while True:
        if cam.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        cam.retrieve_image(left_mat,  sl.VIEW.LEFT)
        cam.retrieve_image(right_mat, sl.VIEW.RIGHT)
        cam.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

        left_bgr  = cv2.cvtColor(left_mat.get_data(),  cv2.COLOR_BGRA2BGR)
        right_bgr = cv2.cvtColor(right_mat.get_data(), cv2.COLOR_BGRA2BGR)
        depth_vis = colorize_depth(depth_mat.get_data())

        left_panel  = cv2.resize(left_bgr,  (PANEL_W, PANEL_H))
        right_panel = cv2.resize(right_bgr, (PANEL_W, PANEL_H))
        depth_panel = cv2.resize(depth_vis, (PANEL_W, PANEL_H))

        # center depth reading
        h_orig, w_orig = left_bgr.shape[:2]
        err, center_d = depth_mat.get_value(w_orig // 2, h_orig // 2)
        depth_label = f"{center_d:.3f} m" if (err == sl.ERROR_CODE.SUCCESS and np.isfinite(center_d)) else "-- m"

        # labels on each panel
        for panel, label in [
            (left_panel,  "LEFT"),
            (right_panel, "RIGHT"),
            (depth_panel, f"DEPTH  center={depth_label}"),
        ]:
            cv2.putText(panel, label, (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)

        row = np.hstack([left_panel, right_panel, depth_panel])

        # bottom status bar
        frame_count += 1
        bar = np.zeros((40, row.shape[1], 3), dtype=np.uint8)
        status_text = (f"Frame {frame_count:05d}   |   "
                       f"S/N {info.serial_number}   |   "
                       f"HD720 @ 30 FPS   |   "
                       f"NEURAL depth   |   Press Q to quit")
        cv2.putText(bar, status_text, (10, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)

        display = np.vstack([row, bar])
        cv2.imshow(WINDOW, display)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cam.close()
    cv2.destroyAllWindows()
    print(f"\n[ZED2i] Closed after {frame_count} frames. Sanity check done.")


if __name__ == "__main__":
    main()
