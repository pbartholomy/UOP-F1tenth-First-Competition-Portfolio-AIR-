# Stage 3 — Reactive Navigation

Autonomy with **no map** — the car reacts purely to live sensor data each tick.
This stage is where the core driving behavior was developed: follow-the-gap for
corners, corridor centering for straights, and the first ZED-assisted steering in
the LiDAR's blind spots. These single-file prototypes fed directly into the ROS 2
`corridor` package (Stage 5).

| File | Purpose |
|------|---------|
| `CAR.py` | Early integrated reactive driver — LiDAR in, VESC out. |
| `Reactive.py` | Core reactive navigation: gap-following + speed control. |
| `Reactive_ftg.py` | Follow-the-gap focused variant. |
| `Reactive_option2_gap.py` | Alternative gap-selection strategy. |
| `Reactive_ZED_gap.py` | Reactive nav fusing ZED depth into the gap decision. |
| `orange_track_nav.py` | Navigation tuned for the orange-walled track sections. |
| `SteeringAssist.py` | Steering-assist layer blending reactive correction with control input. |
| `AssistedAutonomyV1.py` | First "assisted autonomy" combining manual + autonomous control. |
| `corridor_node.py` | The corridor-centering node — the design that became the v8 `corridor` package. |
| `car_joy_node.py` | Joystick/teleop handling alongside autonomous control. |

**Core ideas that survived to the final build:**
- **Corridor centering** — keep equidistant from both walls using LiDAR sector clearances.
- **Follow-the-gap** — when a wall appears ahead, steer toward the widest opening.
- **Blind-zone handling** — when LiDAR loses both walls (black rubber), hold straight
  and let the ZED gently correct, rather than steering on noise.
