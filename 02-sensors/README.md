# Stage 2 — Sensors

Standalone drivers and first perception experiments for the two sensors that
drive every navigation decision: the **Hokuyo URG-04LX LiDAR** and the
**ZED 2i stereo camera**. Each was brought up in isolation and visualized before
being fused into navigation.

## LiDAR (`lidar/`)

| File | Purpose |
|------|---------|
| `LiDARCode.py` | Raw Hokuyo URG-04LX reader — connect, parse scans, print ranges. |
| `lidar_node.py` | LiDAR wrapped as a reusable module with sector clearances. |
| `follow_the_gap.py` | First reactive algorithm: steer toward the largest open gap in the scan. |

> Note on geometry: the URG-04LX scans right-to-left, so "far-left" array indices
> correspond to the car's physical **right** — a quirk handled throughout the code.

## ZED 2i (`zed/`)

| File | Purpose |
|------|---------|
| `zed2i.py` | Open the ZED, grab left image + depth, basic display. |
| `zed2i_sanity_check.py` | Verify depth values and camera health. |
| `zed2i_orange_detect.py` | Color-segment the orange track walls the LiDAR sees well. |
| `detection.py` | Minimal detection helper. |
| `ZED_test.py` | Scratch test harness for camera experiments. |
| `zed_node.py` | ZED wrapped as a module publishing front/left/right obstacle distances. |

**Why the camera matters:** the LiDAR is blind to the black rubber walls used on
parts of the track. The ZED — being a visible-light depth sensor — can see them,
which became the basis for the fusion strategy in later stages.
