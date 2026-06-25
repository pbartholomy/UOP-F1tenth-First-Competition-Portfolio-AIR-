"""
ZED 2i orange object detection — left camera, right camera, depth map, and
orange detection mask displayed in a 2x2 grid.

Detected orange objects are circled on the left camera feed with their
center depth labeled. Press Q to quit.

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
WINDOW  = "ZED 2i Orange Detection  |  Q to quit"

# HSV range for orange — two bands needed because orange wraps near hue=0
ORANGE_LOWER_1 = np.array([0,   150, 80],  dtype=np.uint8)
ORANGE_UPPER_1 = np.array([15,  255, 255], dtype=np.uint8)
ORANGE_LOWER_2 = np.array([160, 150, 80],  dtype=np.uint8)
ORANGE_UPPER_2 = np.array([180, 255, 255], dtype=np.uint8)

MIN_CONTOUR_AREA = 500   # px² — ignore tiny noise blobs
MIN_RADIUS       = 10    # px  — ignore circles too small to matter


def colorize_depth(depth_map: np.ndarray) -> np.ndarray:
    finite = depth_map[np.isfinite(depth_map)]
    if finite.size == 0:
        return np.zeros((*depth_map.shape[:2], 3), dtype=np.uint8)
    d_min, d_max = finite.min(), finite.max()
    if d_max == d_min:
        norm = np.zeros_like(depth_map, dtype=np.uint8)
    else:
        norm = np.clip(1.0 - (depth_map - d_min) / (d_max - d_min), 0.0, 1.0)
        norm = (norm * 255).astype(np.uint8)
    norm[~np.isfinite(depth_map)] = 0
    return cv2.applyColorMap(norm, cv2.COLORMAP_TURBO)


def detect_orange(bgr: np.ndarray):
    """
    Returns (mask, detections) where detections is a list of
    (center_x, center_y, radius) in the original image coordinates.
    """
    hsv  = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, ORANGE_LOWER_1, ORANGE_UPPER_1) | \
           cv2.inRange(hsv, ORANGE_LOWER_2, ORANGE_UPPER_2)

    # clean up the mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detections = []
    for cnt in contours:
        if cv2.contourArea(cnt) < MIN_CONTOUR_AREA:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        if radius >= MIN_RADIUS:
            detections.append((int(cx), int(cy), int(radius)))

    return mask, detections


def main():
    cam = sl.Camera()

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps        = 30
    init.depth_mode        = sl.DEPTH_MODE.NEURAL
    init.coordinate_units  = sl.UNIT.METER

    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ZED2i] Failed to open camera: {status}")
        return

    info = cam.get_camera_information()
    print(f"[ZED2i] {info.camera_model}  S/N {info.serial_number}  FW {info.camera_configuration.firmware_version}")
    print("[ZED2i] Orange detection live — press Q in the window to quit.\n")

    runtime   = sl.RuntimeParameters()
    left_mat  = sl.Mat()
    right_mat = sl.Mat()
    depth_mat = sl.Mat()

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, PANEL_W * 2, PANEL_H * 2 + 40)

    frame_count  = 0
    detect_count = 0

    while True:
        if cam.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue

        cam.retrieve_image(left_mat,  sl.VIEW.LEFT)
        cam.retrieve_image(right_mat, sl.VIEW.RIGHT)
        cam.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)

        left_bgr  = cv2.cvtColor(left_mat.get_data(),  cv2.COLOR_BGRA2BGR)
        right_bgr = cv2.cvtColor(right_mat.get_data(), cv2.COLOR_BGRA2BGR)
        depth_raw = depth_mat.get_data()
        depth_vis = colorize_depth(depth_raw)

        h_orig, w_orig = left_bgr.shape[:2]

        # --- orange detection on full-res left frame ---
        orange_mask, detections = detect_orange(left_bgr)
        detect_count = len(detections)

        left_annotated = left_bgr.copy()
        for (cx, cy, radius) in detections:
            # depth at detection center
            err, d = depth_mat.get_value(cx, cy)
            d_label = f"{d:.2f}m" if (err == sl.ERROR_CODE.SUCCESS and np.isfinite(d)) else "--"

            cv2.circle(left_annotated, (cx, cy), radius, (0, 140, 255), 2)
            cv2.circle(left_annotated, (cx, cy), 4,      (0, 140, 255), -1)
            cv2.putText(left_annotated, d_label,
                        (cx - 30, cy - radius - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 140, 255), 2, cv2.LINE_AA)

        # --- build mask panel (3-channel so it stacks with BGR panels) ---
        mask_bgr   = cv2.cvtColor(orange_mask, cv2.COLOR_GRAY2BGR)
        mask_color = mask_bgr.copy()
        mask_color[orange_mask > 0] = (0, 140, 255)   # orange tint on detections

        # --- resize all panels to PANEL_W x PANEL_H ---
        left_panel  = cv2.resize(left_annotated, (PANEL_W, PANEL_H))
        right_panel = cv2.resize(right_bgr,      (PANEL_W, PANEL_H))
        depth_panel = cv2.resize(depth_vis,      (PANEL_W, PANEL_H))
        mask_panel  = cv2.resize(mask_color,     (PANEL_W, PANEL_H))

        # center depth
        err, center_d = depth_mat.get_value(w_orig // 2, h_orig // 2)
        depth_label = f"{center_d:.3f} m" if (err == sl.ERROR_CODE.SUCCESS and np.isfinite(center_d)) else "-- m"

        # panel labels
        for panel, label in [
            (left_panel,  f"LEFT  [{detect_count} orange object(s)]"),
            (right_panel, "RIGHT"),
            (depth_panel, f"DEPTH  center={depth_label}"),
            (mask_panel,  "ORANGE MASK"),
        ]:
            cv2.putText(panel, label, (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

        # 2x2 grid
        top_row    = np.hstack([left_panel,  right_panel])
        bottom_row = np.hstack([depth_panel, mask_panel])
        grid       = np.vstack([top_row, bottom_row])

        # status bar
        frame_count += 1
        bar = np.zeros((40, grid.shape[1], 3), dtype=np.uint8)
        status_text = (f"Frame {frame_count:05d}   |   "
                       f"S/N {info.serial_number}   |   "
                       f"HD720 @ 30 FPS   |   "
                       f"Orange objects: {detect_count}   |   Press Q to quit")
        cv2.putText(bar, status_text, (10, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)

        cv2.imshow(WINDOW, np.vstack([grid, bar]))

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cam.close()
    cv2.destroyAllWindows()
    print(f"\n[ZED2i] Closed after {frame_count} frames.")


if __name__ == "__main__":
    main()
