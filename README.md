# Autonomous 1/10-Scale Race Car — Roboracer / F1TENTH

A from-scratch autonomous racing platform built on a modified RC chassis, running
ROS 2 on an NVIDIA Jetson. This repository documents the **full engineering
journey** — from first powering the motor controller, through reactive obstacle
avoidance, to SLAM-based mapping with pure-pursuit path following and an ML
steering model used at competition.

> _Built and raced by Patrick Bartholomy, Michail Alam, and Uzair Naseer.  Raced in 38th RoboRacer competition in Detroit Michigan.

---

## The platform

| Subsystem        | Hardware |
|------------------|----------|
| Chassis          | Traxxas Ford Fiesta ST Rally VXL (74276-4), 1/10 scale |
| Compute          | NVIDIA Jetson Orin Nano |
| Motor controller | VESC 6 MkVI (UART control) |
| LiDAR            | Hokuyo URG-04LX (2D, ~4 m range) |
| Camera           | Stereolabs ZED 2i (stereo depth, VGA) |
| Middleware       | ROS 2 Humble + colcon |

The car races on tracks bounded by walls — some brightly colored (visible to
LiDAR) and some **black rubber** (invisible to LiDAR, the central engineering
challenge of this project).

---

## How to read this repo

The project is laid out as a **progression**, each stage building on the last:

| Stage | Folder | What it covers |
|-------|--------|----------------|
| 1 | [`01-hardware-bringup/`](01-hardware-bringup/) | Talking to the VESC: motor duty, servo steering, PS4 controller teleop |
| 2 | [`02-sensors/`](02-sensors/) | Standalone LiDAR and ZED 2i drivers + first perception experiments |
| 3 | [`03-reactive-nav/`](03-reactive-nav/) | Reactive autonomy: follow-the-gap, corridor centering, ZED-assisted steering |
| 4 | [`04-slam-ml/`](04-slam-ml/) | SLAM mapping, path recording, and an ML steering network |
| 5 | [`05-ros2-workspaces/`](05-ros2-workspaces/) | The packaged ROS 2 workspaces (v4 → v9), each a competition iteration |
| — | [`docs/`](docs/) | System report, VESC & controller documentation, design notes |

The **standalone scripts** (stages 1–4) were the prototyping ground — quick,
single-file experiments to validate one idea at a time. The **ROS 2 workspaces**
(stage 5) are where the proven ideas were packaged into a real multi-node system.

---

## Key engineering themes

- **Sensor fusion with complementary failure modes.** LiDAR is reliable on light
  walls but blind on black rubber; the ZED stereo camera fills that gap. The
  final design hands control to the ZED *only* when the LiDAR confirms it is
  blind, then fails safe to straight when the camera data is untrustworthy.
- **Reactive vs. map-based navigation.** Early versions reacted purely to live
  sensor data (gap-following + corridor centering). Later versions (v9) add
  `slam_toolbox` mapping and pure-pursuit following of a recorded racing line.
- **Tuning under real constraints.** Speed, steering authority, and corner
  behavior were tuned iteratively on the physical car — the version history here
  reflects that real loop of test → adjust → retest.

---

## Running the latest build

The competition build lives in [`05-ros2-workspaces/v8_ws`](05-ros2-workspaces/v8_ws)
(reactive) and [`v9_ws`](05-ros2-workspaces/v9_ws) (SLAM + pure pursuit). After a
`colcon build` and sourcing the workspace:

```bash
# Reactive corridor navigation
ros2 launch corridor corridor.launch.py

# SLAM mapping + pure-pursuit replay
ros2 launch v9nav v9nav.launch.py
```

See the per-stage READMEs for details on each script and node.

---

## What's not in this repo

To keep it lightweight, recorded media and large binaries are intentionally
excluded (see `.gitignore`): onboard video, ZED `.svo` recordings, ROS bags,
build/install artifacts, and vendor PDFs. Demo videos are hosted separately —
_add your video/Drive links here._
