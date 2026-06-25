# Stage 5 — ROS 2 Workspaces (v4 → v9)

The proven prototypes from earlier stages, packaged into real **ROS 2 Humble**
workspaces. Each `vN_ws` is a competition iteration — only the `src/` package
sources are versioned here (colcon `build/`, `install/`, and `log/` are excluded;
rebuild with `colcon build`).

| Workspace | Package(s) | Focus |
|-----------|-----------|-------|
| `v4_ws` | `corridor` | First packaged corridor-centering node. |
| `v5_ws` | `corridor` | Refined reactive corridor navigation. |
| `v6_ws` | `corridor` | Further tuning + obstacle handling. |
| `v7_ws` | `corridor`, `f1tenth_lifelong_racing` | Reactive nav alongside a lifelong-racing package. |
| `v8_ws` | `corridor` | **Mature reactive build** — gap-follow + corridor centering + gated ZED fusion for black-wall zones. The primary competition build. |
| `v9_ws` | `v9nav` | **SLAM build** — `slam_toolbox` mapping, path recording, and pure-pursuit replay. |

## Building any workspace

```bash
cd 05-ros2-workspaces/v8_ws
colcon build
source install/setup.bash
ros2 launch corridor corridor.launch.py
```

```bash
cd 05-ros2-workspaces/v9_ws
colcon build
source install/setup.bash
# Map the track manually, then drive it autonomously:
ros2 launch v9nav v9nav.launch.py
```

## Architecture (v8 `corridor`)

Multiple nodes cooperate over ROS 2 topics:

- **`corridor_node`** — reads the LiDAR, computes steering (corridor centering /
  follow-the-gap), fuses ZED distances when the LiDAR is blind, and commands the VESC.
- **`zed_obstacle_node`** — publishes front/left/right depth from the ZED 2i.
- **`car_node`** — applies manual drive commands in teleop mode.
- **`joy_node`** — reads the PS4 controller.
- **`mode_manager_node`** — toggles MANUAL / AUTONOMOUS (and MAPPING in v9).
- **`visualizer_node`** — live debug display.

The v9 `v9nav` package adds `mapping_node` (records the driven racing line) and
`pure_pursuit_node` (follows it), with `slam_toolbox` providing the map → base_link
pose.

## Sensor-fusion strategy (the headline feature)

LiDAR drives the car at all times. The ZED camera is engaged **only** when the
LiDAR confirms it is blind (the black rubber walls), then:
1. fills in the front distance the laser can't see,
2. chooses turn direction from the left/right depth difference,
3. gently centers the car between walls it can see,
4. and the car slows down for more reaction time.

When the camera data is itself untrustworthy (black rubber defeats stereo too),
the car **fails safe to driving straight** rather than steering on bad data.
