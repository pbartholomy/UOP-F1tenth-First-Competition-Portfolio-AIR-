# Stage 4 — SLAM & Machine Learning

Moving beyond purely reactive driving: building a **map** of the track for
path-based racing, and training an **ML steering model** from recorded driving.

| File | Purpose |
|------|---------|
| `zed_lidar_track_nav.py` | Track navigation fusing ZED + LiDAR. |
| `zed_lidar_track_nav_v2.py` | Iteration with improved fusion / tuning. |
| `zed_lidar_slam_nav_v1.py` … `_v4.py` | Progressive SLAM-based navigation: build a map, localize, and follow it. v4 is the most complete (SLAM + ML). |
| `GitHubData.py` | Data collection / logging utility for training and analysis. |
| `testing_1.py` | Experiment scratchpad. |
| `models/steering_net.pt` | Trained PyTorch steering network (predicts steering from sensor input). |
| `models/slam_map_save.npz` | A saved SLAM map artifact. |

**The progression `v1 → v4`** tracks the move from raw SLAM scan-matching toward a
hybrid system: SLAM for localization, a recorded path for the racing line, and a
learned steering policy for smoother control. The production version of this idea
is packaged in [`../05-ros2-workspaces/v9_ws`](../05-ros2-workspaces/v9_ws) using
`slam_toolbox` + pure pursuit.

> The `.pt` model is small enough to version here; larger training datasets and
> recordings are kept out of git (see the top-level `.gitignore`).
