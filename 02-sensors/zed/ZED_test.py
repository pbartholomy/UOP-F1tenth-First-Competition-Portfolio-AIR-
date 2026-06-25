#!/usr/bin/env python3
"""
ZED_test.py — Quick connection test for the ZED stereo camera.

Shows a live window with:
  - Left eye feed
  - Right eye feed
  - FPS counter
  - Resolution readout

Press Q or Esc to quit.
"""

import cv2
import time

ZED_DEVICE = 0          # change to 1 or 2 if video0 is something else
ZED_CAPTURE_WIDTH  = 2560
ZED_CAPTURE_HEIGHT = 720

print(f"[ZED] Opening /dev/video{ZED_DEVICE} ...")
cap = cv2.VideoCapture(ZED_DEVICE)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,  ZED_CAPTURE_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ZED_CAPTURE_HEIGHT)

if not cap.isOpened():
    print("[ZED] ERROR: Could not open camera.")
    print("      - Check the USB cable is plugged in")
    print("      - Try changing ZED_DEVICE to 1 or 2")
    raise SystemExit(1)

actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[ZED] Connected — frame size: {actual_w}x{actual_h}")
print("[ZED] Press Q or Esc to quit")

fps_counter = 0
fps_display = 0.0
fps_timer   = time.time()

while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        print("[ZED] WARNING: failed to grab frame")
        continue

    # Split stereo frame into left and right eyes
    mid   = frame.shape[1] // 2
    left  = frame[:, :mid]
    right = frame[:, mid:]

    # FPS calculation
    fps_counter += 1
    now = time.time()
    if now - fps_timer >= 1.0:
        fps_display = fps_counter / (now - fps_timer)
        fps_counter = 0
        fps_timer   = now

    # Annotate left eye
    cv2.putText(left, "LEFT EYE", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(left, f"FPS: {fps_display:.1f}", (10, 58),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(left, f"Res: {actual_w}x{actual_h}", (10, 88),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)

    # Annotate right eye
    cv2.putText(right, "RIGHT EYE", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2, cv2.LINE_AA)

    # Stack side by side at half scale so it fits on screen
    combined = cv2.hconcat([left, right])
    display  = cv2.resize(combined, (0, 0), fx=0.5, fy=0.5)

    cv2.imshow("ZED Camera Test — Q to quit", display)

    key = cv2.waitKey(1) & 0xFF
    if key in (ord('q'), ord('Q'), 27):   # Q or Esc
        break

cap.release()
cv2.destroyAllWindows()
print("[ZED] Closed.")
