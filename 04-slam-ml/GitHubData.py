"""
GitHubData.py
=============
F1TENTH / RoboRacer competition algorithms sourced from GitHub and tuned for:

  Car    : Traxxas Ford Fiesta ST Rally VXL (74276-4)  1/10 scale
  ESC    : VESC 6 MkVI
  Motor  : Velineon 3500 Kv sensorless brushless
  LiDAR  : Hokuyo URG series  (steps 44-667, front=384, ~219° coverage)
  Camera : ZED 2i stereo
  Compute: Jetson

Sources
-------
- GapFollower / DisparityExtender
    https://github.com/f1tenth/ESweek2021_educationclassA3
- PurePursuitPlanner
    https://github.com/f1tenth/ESweek2021_educationclassA3
- Wall-follow PID concept
    https://github.com/CL2-UWaterloo/f1tenth_ws
- F1TENTH system drivers (VESC + LiDAR)
    https://github.com/f1tenth/f1tenth_system
- Full competition software stack (gap + pure pursuit + safety)
    https://github.com/derekhanbaliq/f1tenth-software-stack
- Algorithm benchmarks
    https://github.com/BDEvan5/f1tenth_benchmarks

All numeric constants below are adjusted from the default F1TENTH values to
match this car's dimensions and sensor geometry.
"""

import math
import numpy as np

# ── CAR CONSTANTS (Traxxas Fiesta ST Rally VXL 74276-4) ──────────────────────
WHEELBASE_M   = 0.272   # 272 mm wheelbase
CAR_WIDTH_M   = 0.240   # 240 mm total width
CAR_HALF_W_M  = CAR_WIDTH_M / 2.0

# ── HOKUYO URG GEOMETRY ───────────────────────────────────────────────────────
# Steps 44-667, front at step 384, 360/1024 deg per step
LIDAR_STEP_MIN   = 44
LIDAR_STEP_MAX   = 667
LIDAR_STEP_FRONT = 384
LIDAR_TOTAL_STEPS = LIDAR_STEP_MAX - LIDAR_STEP_MIN + 1   # 624 steps
RAD_PER_STEP     = (2.0 * math.pi) / 1024.0
LIDAR_FOV_DEG    = LIDAR_TOTAL_STEPS * math.degrees(RAD_PER_STEP)  # ~219°


# ══════════════════════════════════════════════════════════════════════════════
# 1.  FOLLOW-THE-GAP  (reactive, no map needed)
#     Source: github.com/f1tenth/ESweek2021_educationclassA3
#     Tuned for 219° Hokuyo scan and 240 mm car width
# ══════════════════════════════════════════════════════════════════════════════

class GapFollower:
    """
    Standard Follow-the-Gap algorithm.
    Finds the largest obstacle-free arc in the LiDAR scan and steers toward it.

    Workflow
    --------
    1. preprocess_lidar  – smooth + clip raw scan
    2. find closest point, zero-out a safety bubble around it
    3. find_max_gap       – largest contiguous non-zero arc
    4. find_best_point    – furthest averaged point inside that gap
    5. get_angle          – convert point index → steering angle
    """

    # --- Tunable parameters (adapted for this car) ---
    # Bubble radius in scan-index units.  At ~0.35°/step and typical obstacle
    # distances of 0.5–1.5m, 100 steps covers ~35° which is enough to hide the
    # full car width.
    BUBBLE_RADIUS        = 100          # steps  (default F1TENTH uses 160 for wider scan)
    PREPROCESS_CONV_SIZE = 3            # smoothing window (samples)
    BEST_POINT_CONV_SIZE = 60           # sliding-window for best-point selection
    MAX_LIDAR_DIST_MM    = 3000         # clip at 3 m (tube-wall track is small)

    # Speed (in VESC duty-cycle units, 0.0–1.0)
    STRAIGHTS_DUTY       = 0.060        # AUTO_DRIVE_DUTY from tuned params
    CORNERS_DUTY         = 0.050
    STRAIGHT_ANGLE_RAD   = math.radians(8)   # below this → "straight"

    def __init__(self):
        self._rad_per_step = RAD_PER_STEP

    # ------------------------------------------------------------------
    def preprocess_lidar(self, distances: list) -> np.ndarray:
        """
        distances: raw list from HokuyoLidar.get_distances() (624 values, mm).
        Returns smoothed, clipped numpy array.
        """
        arr = np.array(distances, dtype=float)
        # Smooth
        arr = np.convolve(arr, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        arr = np.clip(arr, 0, self.MAX_LIDAR_DIST_MM)
        return arr

    def _bubble_around(self, arr: np.ndarray, center: int) -> np.ndarray:
        lo = max(0, center - self.BUBBLE_RADIUS)
        hi = min(len(arr) - 1, center + self.BUBBLE_RADIUS)
        arr[lo:hi] = 0
        return arr

    def find_max_gap(self, arr: np.ndarray):
        """Return (start_idx, end_idx) of the largest contiguous non-zero arc."""
        masked = np.ma.masked_where(arr == 0, arr)
        slices = np.ma.notmasked_contiguous(masked)
        if not slices:
            return 0, len(arr) - 1
        best = max(slices, key=lambda s: s.stop - s.start)
        return best.start, best.stop

    def find_best_point(self, start_i: int, end_i: int, arr: np.ndarray) -> int:
        """Furthest averaged point inside the gap – smoothed to avoid clipping corners."""
        window = np.ones(self.BEST_POINT_CONV_SIZE)
        averaged = np.convolve(arr[start_i:end_i], window, 'same') / self.BEST_POINT_CONV_SIZE
        return averaged.argmax() + start_i

    def get_angle(self, best_idx: int, n_steps: int) -> float:
        """Convert scan index to steering angle (radians, signed)."""
        lidar_angle = (best_idx - n_steps / 2.0) * self._rad_per_step
        # Halve: car does not need to point directly at best gap
        return lidar_angle / 2.0

    def process_lidar(self, distances: list):
        """
        Main entry point.
        Returns (duty_cycle, steering_angle_rad).
        steering_angle_rad: positive = left, negative = right  (matches nav_node convention)
        """
        arr = self.preprocess_lidar(distances)
        closest_idx = int(arr.argmin())
        arr = self._bubble_around(arr, closest_idx)
        gap_start, gap_end = self.find_max_gap(arr)
        best_idx = self.find_best_point(gap_start, gap_end, arr)
        steer_rad = self.get_angle(best_idx, len(arr))

        if abs(steer_rad) > self.STRAIGHT_ANGLE_RAD:
            duty = self.CORNERS_DUTY
        else:
            duty = self.STRAIGHTS_DUTY

        return duty, steer_rad


# ══════════════════════════════════════════════════════════════════════════════
# 2.  DISPARITY EXTENDER  (safer gap follower, better near corners)
#     Source: github.com/f1tenth/ESweek2021_educationclassA3
#     Tuned: CAR_WIDTH → 0.240 m, SAFETY_PERCENTAGE raised for tube walls
# ══════════════════════════════════════════════════════════════════════════════

class DisparityExtender:
    """
    Improvement over basic GapFollower: 'extends' close edges at depth
    disparities so the car's full width is accounted for.
    Better performance in narrow corridors and near tube-wall corners.
    """

    CAR_WIDTH_M       = 0.240          # Traxxas Fiesta ST 1/10
    DIFFERENCE_THRESH_MM = 300         # mm — depth jump triggers extension
    DUTY              = 0.058          # ~mid-range duty
    # Extra safety margin on top of half-car-width (%)
    # Raised from default 300% to 400% for soft tube walls that compress
    SAFETY_PCT        = 400.0

    def __init__(self):
        self._rad_per_step = RAD_PER_STEP

    def preprocess_lidar(self, distances: list) -> np.ndarray:
        """Strip rear 1/8 on each side, return numpy array (mm)."""
        arr = np.array(distances, dtype=float)
        eighth = len(arr) // 8
        return arr[eighth:-eighth]

    def _get_differences(self, arr: np.ndarray) -> list:
        diffs = [0.0]
        for i in range(1, len(arr)):
            diffs.append(abs(float(arr[i]) - float(arr[i - 1])))
        return diffs

    def _get_disparities(self, diffs: list) -> list:
        return [i for i, d in enumerate(diffs) if d > self.DIFFERENCE_THRESH_MM]

    def _points_to_cover(self, dist_mm: float) -> int:
        """How many scan steps span half-car-width + safety at dist_mm."""
        width_m = (self.CAR_WIDTH_M / 2.0) * (1.0 + self.SAFETY_PCT / 100.0)
        if dist_mm < 1.0:
            return 0
        angle = 2.0 * math.asin(min(1.0, (width_m) / (dist_mm / 1000.0 * 2.0)))
        return int(math.ceil(angle / self._rad_per_step))

    def _cover_points(self, n: int, start_idx: int, cover_right: bool,
                      arr: np.ndarray) -> np.ndarray:
        new_dist = arr[start_idx]
        direction = range(1, n + 1) if cover_right else range(-1, -(n + 1), -1)
        for offset in direction:
            idx = start_idx + offset
            if 0 <= idx < len(arr) and arr[idx] > new_dist:
                arr[idx] = new_dist
        return arr

    def _extend_disparities(self, disparities: list, arr: np.ndarray) -> np.ndarray:
        for idx in disparities:
            fi = idx - 1
            if fi < 0 or fi + 1 >= len(arr):
                continue
            close_idx = fi + int(arr[fi] > arr[fi + 1])
            far_idx   = fi + int(arr[fi] <= arr[fi + 1])
            n = self._points_to_cover(float(arr[close_idx]))
            cover_right = close_idx < far_idx
            arr = self._cover_points(n, close_idx, cover_right, arr)
        return arr

    def _steering_angle(self, best_idx: int, n: int) -> float:
        angle = (best_idx - n / 2.0) * self._rad_per_step
        return float(np.clip(angle, -math.pi / 2, math.pi / 2))

    def process_lidar(self, distances: list):
        """
        Returns (duty_cycle, steering_angle_rad).
        steering_angle_rad: positive = left.
        """
        arr = self.preprocess_lidar(distances)
        diffs = self._get_differences(arr)
        disparities = self._get_disparities(diffs)
        arr = self._extend_disparities(disparities, arr)
        best_idx = int(arr.argmax())
        steer_rad = self._steering_angle(best_idx, len(arr))
        return self.DUTY, steer_rad


# ══════════════════════════════════════════════════════════════════════════════
# 3.  PURE PURSUIT  (waypoint follower — needs a pre-recorded lap)
#     Source: github.com/f1tenth/ESweek2021_educationclassA3
#     Tuned: wheelbase = 0.272 m (Traxxas Fiesta ST 74276-4)
# ══════════════════════════════════════════════════════════════════════════════

class PurePursuit:
    """
    Geometric path follower.
    Requires a CSV waypoint file with columns  [x_m, y_m, speed_duty].
    Generate waypoints by recording /odom (or SLAM pose) during a manual lap.

    Usage
    -----
        pp = PurePursuit('waypoints.csv')
        duty, steer_rad = pp.plan(pose_x, pose_y, pose_theta)
    """

    WHEELBASE_M     = WHEELBASE_M       # 0.272 m
    LOOKAHEAD_M     = 0.60              # look 60 cm ahead (reduce for tight track)
    MAX_REACQUIRE_M = 1.50              # re-acquire waypoint within 1.5 m
    SPEED_GAIN      = 1.0               # scale factor on CSV speed column
    DEFAULT_DUTY    = 0.055             # fallback if no waypoint found

    def __init__(self, waypoint_csv: str):
        """
        waypoint_csv: path to file with rows  x_m, y_m, duty_cycle
        """
        self.waypoints = np.loadtxt(waypoint_csv, delimiter=',')
        # columns: 0=x, 1=y, 2=duty
        self._wpts_xy = self.waypoints[:, :2]

    # --- geometry helpers ---------------------------------------------------

    @staticmethod
    def _nearest_point(pos: np.ndarray, traj: np.ndarray):
        """Return (projection, dist, t, segment_idx) of nearest point on trajectory."""
        diffs = traj[1:] - traj[:-1]
        l2s   = np.sum(diffs ** 2, axis=1)
        dots  = np.sum((pos - traj[:-1]) * diffs, axis=1)
        t     = np.clip(dots / np.maximum(l2s, 1e-9), 0.0, 1.0)
        proj  = traj[:-1] + (t[:, None] * diffs)
        dists = np.linalg.norm(pos - proj, axis=1)
        i     = int(np.argmin(dists))
        return proj[i], dists[i], t[i], i

    @staticmethod
    def _circle_intersect(pos: np.ndarray, r: float, traj: np.ndarray,
                          start_t: float = 0.0):
        """First point on trajectory at distance r from pos."""
        si = int(start_t)
        for i in range(si, len(traj) - 1):
            s, e = traj[i], traj[(i + 1) % len(traj)] + 1e-9
            V = e - s
            a = float(np.dot(V, V))
            b = 2.0 * float(np.dot(V, s - pos))
            c = float(np.dot(s, s) + np.dot(pos, pos)
                      - 2.0 * np.dot(s, pos) - r * r)
            disc = b * b - 4 * a * c
            if disc < 0:
                continue
            sq = math.sqrt(disc)
            for t_val in [(-b - sq) / (2 * a), (-b + sq) / (2 * a)]:
                if 0.0 <= t_val <= 1.0 and (i > si or t_val >= start_t % 1.0):
                    return s + t_val * V, i
        return None, None

    def _get_waypoint(self, pos: np.ndarray, theta: float):
        """Return lookahead waypoint (x, y, duty) or None."""
        _, nearest_dist, t, seg_i = self._nearest_point(pos, self._wpts_xy)
        if nearest_dist < self.LOOKAHEAD_M:
            pt, i2 = self._circle_intersect(pos, self.LOOKAHEAD_M,
                                             self._wpts_xy, seg_i + t)
            if pt is None:
                return None
            duty = float(self.waypoints[i2, 2])
            return np.array([pt[0], pt[1], duty])
        elif nearest_dist < self.MAX_REACQUIRE_M:
            return np.array([self._wpts_xy[seg_i, 0],
                             self._wpts_xy[seg_i, 1],
                             float(self.waypoints[seg_i, 2])])
        return None

    def plan(self, pose_x: float, pose_y: float, pose_theta: float):
        """
        Returns (duty_cycle, steering_angle_rad).
        steering_angle_rad: positive = left (matches nav_node convention).
        """
        pos = np.array([pose_x, pose_y])
        wp  = self._get_waypoint(pos, pose_theta)
        if wp is None:
            return self.DEFAULT_DUTY, 0.0

        # Lateral offset of waypoint in vehicle frame
        wp_y = (math.sin(-pose_theta) * (wp[0] - pos[0])
                + math.cos(-pose_theta) * (wp[1] - pos[1]))

        if abs(wp_y) < 1e-6:
            steer = 0.0
        else:
            radius = (self.LOOKAHEAD_M ** 2) / (2.0 * wp_y)
            steer  = math.atan(self.WHEELBASE_M / radius)

        duty = float(wp[2]) * self.SPEED_GAIN
        return duty, steer


# ══════════════════════════════════════════════════════════════════════════════
# 4.  WALL FOLLOWER  (PID — keeps constant distance from one wall)
#     Concept: github.com/CL2-UWaterloo/f1tenth_ws  wall_follow
#     Tuned for tube-wall track with ~25 cm high tubes
# ══════════════════════════════════════════════════════════════════════════════

class WallFollower:
    """
    Maintains a target distance from the right wall using two LiDAR beams
    and a PD controller.  Mirrors the classic F1TENTH wall-follow lab.

    The two beams are taken at 0° (perpendicular right) and +45° (forward-right)
    so the controller can predict upcoming wall curvature.
    """

    TARGET_DIST_M   = 0.50      # desired right-wall clearance (m) — half track width ~0.5m
    KP              = 0.8       # proportional gain
    KD              = 0.04      # derivative gain
    LOOKAHEAD_M     = 0.30      # project distance ahead for error smoothing
    DRIVE_DUTY      = 0.058

    # Step indices for the two beams (right = negative angles in our LiDAR frame)
    # 0° right  → offset 0 steps from right-90° = LIDAR_STEP_FRONT - 256 steps
    # 45° fwd-r → offset +128 steps from that
    _STEP_RIGHT_90  = LIDAR_STEP_FRONT - int(90 * 1024 / 360)   # pure right
    _STEP_RIGHT_45  = LIDAR_STEP_FRONT - int(45 * 1024 / 360)   # forward-right

    def __init__(self):
        self._prev_error = 0.0

    def _dist_from_steps(self, distances: list, step: int) -> float:
        idx = max(0, min(len(distances) - 1, step - LIDAR_STEP_MIN))
        return distances[idx] / 1000.0   # mm → m

    def _estimated_wall_dist(self, a_m: float, b_m: float,
                             theta_rad: float = math.radians(45)) -> float:
        """
        Use two beam distances a (45°) and b (90°) to estimate perpendicular
        wall distance Dt ahead by LOOKAHEAD_M.
        """
        alpha = math.atan2(
            a_m * math.cos(theta_rad) - b_m,
            a_m * math.sin(theta_rad)
        )
        # current perpendicular dist
        Dt = b_m * math.cos(alpha)
        # predicted dist one lookahead step ahead
        Dt1 = Dt + self.LOOKAHEAD_M * math.sin(alpha)
        return Dt1

    def process_lidar(self, distances: list):
        """
        Returns (duty_cycle, steering_angle_rad).
        Positive steer = left (away from right wall).
        """
        a = self._dist_from_steps(distances, self._STEP_RIGHT_45)
        b = self._dist_from_steps(distances, self._STEP_RIGHT_90)

        # Clamp to valid range
        a = max(0.05, min(a, 4.0))
        b = max(0.05, min(b, 4.0))

        wall_dist = self._estimated_wall_dist(a, b)
        error     = self.TARGET_DIST_M - wall_dist
        d_error   = error - self._prev_error
        self._prev_error = error

        steer = self.KP * error + self.KD * d_error
        steer = float(np.clip(steer, -1.0, 1.0))

        return self.DRIVE_DUTY, steer


# ══════════════════════════════════════════════════════════════════════════════
# 5.  SAFETY / AEB  (Automatic Emergency Braking)
#     Concept: github.com/derekhanbaliq/f1tenth-software-stack  safety_node
#     Adapted to use mm LiDAR distances and VESC duty-cycle stopping
# ══════════════════════════════════════════════════════════════════════════════

class SafetyAEB:
    """
    Time-To-Collision (TTC) based emergency braking.
    Computes the projected time to impact for every forward LiDAR beam and
    triggers a stop if any TTC drops below the threshold.

    Integrate with nav_node: call check() each loop; if it returns True,
    zero the duty cycle immediately.
    """

    TTC_THRESHOLD_S  = 0.25     # stop if any beam hits in < 250 ms
    FORWARD_STEPS    = 130      # ±65 steps from front = ±23° forward cone

    def __init__(self):
        self._speed_m_s = 0.0   # must be updated each loop from VESC telemetry

    def update_speed(self, speed_m_s: float):
        self._speed_m_s = speed_m_s

    def check(self, distances: list) -> bool:
        """
        Returns True if emergency stop should be triggered.
        distances: list from HokuyoLidar.get_distances() (mm)
        """
        if self._speed_m_s <= 0.01:
            return False
        lo = max(0, LIDAR_TOTAL_STEPS // 2 - self.FORWARD_STEPS)
        hi = min(LIDAR_TOTAL_STEPS - 1, LIDAR_TOTAL_STEPS // 2 + self.FORWARD_STEPS)
        for idx in range(lo, hi):
            d_m = distances[idx] / 1000.0
            if d_m < 0.05:
                continue
            # angle relative to dead-ahead
            step = idx + LIDAR_STEP_MIN
            angle = (step - LIDAR_STEP_FRONT) * RAD_PER_STEP
            # rate of closure along the beam direction
            range_rate = self._speed_m_s * math.cos(angle)
            if range_rate > 0 and (d_m / range_rate) < self.TTC_THRESHOLD_S:
                return True
        return False


# ══════════════════════════════════════════════════════════════════════════════
# 6.  ALGORITHM SELECTOR — drop-in replacement for select_reactive_steer()
#     Switch between the above algorithms using the Triangle button on PS4.
# ══════════════════════════════════════════════════════════════════════════════

class AlgorithmSelector:
    """
    Wraps all algorithms so you can hot-swap during a run.
    Returns (duty, steer_rad) regardless of which algorithm is active.

    Modes (cycle with Triangle):
      0 - GapFollower         (reactive, no map)
      1 - DisparityExtender   (safer near walls)
      2 - WallFollower        (right-wall hugging)
      3 - PurePursuit         (requires waypoints.csv)
    """

    MODE_GAP        = 0
    MODE_DISPARITY  = 1
    MODE_WALL       = 2
    MODE_PURE       = 3
    MODE_NAMES      = ["GAP_FOLLOW", "DISPARITY_EXT", "WALL_FOLLOW", "PURE_PURSUIT"]

    def __init__(self, waypoint_csv: str = None):
        self.gap       = GapFollower()
        self.disparity = DisparityExtender()
        self.wall      = WallFollower()
        self.pure      = PurePursuit(waypoint_csv) if waypoint_csv else None
        self.mode      = self.MODE_GAP

    def next_mode(self):
        n = len(self.MODE_NAMES) if self.pure else len(self.MODE_NAMES) - 1
        self.mode = (self.mode + 1) % n
        return self.MODE_NAMES[self.mode]

    def process(self, distances: list,
                pose_x: float = 0.0, pose_y: float = 0.0,
                pose_theta: float = 0.0):
        """
        Returns (duty_cycle, steer_rad).
        steer_rad sign convention: positive = LEFT  (matches nav_node).
        """
        if self.mode == self.MODE_GAP:
            return self.gap.process_lidar(distances)
        if self.mode == self.MODE_DISPARITY:
            return self.disparity.process_lidar(distances)
        if self.mode == self.MODE_WALL:
            return self.wall.process_lidar(distances)
        if self.mode == self.MODE_PURE and self.pure:
            return self.pure.plan(pose_x, pose_y, pose_theta)
        return self.gap.process_lidar(distances)


# ══════════════════════════════════════════════════════════════════════════════
# QUICK PARAMETER REFERENCE (all algorithms, this car)
# ══════════════════════════════════════════════════════════════════════════════
"""
Algorithm          Key param               Value    Rationale
---------------------------------------------------------------------------
GapFollower        BUBBLE_RADIUS           100      ~35° bubble at 1m
                   MAX_LIDAR_DIST_MM       3000     tube track fits in 3m
                   STRAIGHTS_DUTY          0.060    safe top speed (rosrun11)
                   CORNERS_DUTY            0.050    matches CAP tuning
                   STRAIGHT_ANGLE_RAD      0.14     ~8°

DisparityExtender  CAR_WIDTH_M             0.240    measured Fiesta 1/10
                   DIFFERENCE_THRESH_MM    300      tube walls create big jumps
                   SAFETY_PCT              400%     extra margin, soft tubes
                   DUTY                    0.058

WallFollower       TARGET_DIST_M           0.50     ~half of 1m corridor
                   KP / KD                 0.8/0.04 light PD, car responds fast

PurePursuit        WHEELBASE_M             0.272    Traxxas Fiesta ST 74276-4
                   LOOKAHEAD_M             0.60     shorter = tighter turns
                   SPEED_GAIN              1.0      scale CSV column duty values

SafetyAEB          TTC_THRESHOLD_S         0.25     emergency stop < 250 ms
                   FORWARD_STEPS           130      ±23° forward cone

LiDAR geometry     STEP_MIN/MAX            44/667   619 active steps
                   STEP_FRONT              384      forward = step 384
                   RAD_PER_STEP            0.00614  360°/1024 per step
                   TOTAL_FOV               ~219°    actual coverage

VESC               AUTO_DRIVE_DUTY         0.060    from rosrun11 baseline
                   MIN_EFFECTIVE_DUTY      0.051
                   REVERSE_DUTY            0.080
"""
