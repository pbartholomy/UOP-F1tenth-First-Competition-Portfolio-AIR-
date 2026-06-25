# Stage 1 — Hardware Bring-up

Getting the Jetson to actually drive the car: commanding the **VESC 6 MkVI** over
UART for motor and steering, and reading a **PS4 controller** for manual teleop.
These were the first scripts written — proving the control path end to end before
any autonomy.

| File | Purpose |
|------|---------|
| `vesc_ping_linux.py` | Minimal VESC connectivity check — confirm UART comms and firmware response. |
| `servo_float_linux.py` | Send raw servo position commands to find steering center/limits. |
| `servo_only_linux_hold.py` | Hold a fixed servo angle — mechanical steering calibration. |
| `duty_test_linux.py` | Drive the motor at a set duty cycle — validate throttle direction and scaling. |
| `ps4_vesc_controller_duty_linux.py` | Full PS4 → VESC teleop: stick axes mapped to duty + steering. |
| `acegamer_controller.py` | Controller-reading variant for a different gamepad. |
| `roboracer.py` | Early combined control script tying controller input to VESC output. |

**Key lessons captured here:** VESC duty-cycle vs. current control, the motor
direction inversion, and the servo trim offset needed to drive straight — all of
which carried forward into the autonomous nodes.

See also [`../docs/VESC_6_MkVI_Documentation.md`](../docs/VESC_6_MkVI_Documentation.md)
and [`../docs/PS4_VESC_Controller_Documentation.md`](../docs/PS4_VESC_Controller_Documentation.md).
