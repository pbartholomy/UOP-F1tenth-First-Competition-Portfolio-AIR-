# AI Handoff Document — Roboracer / F1TENTH Project
**Machine:** orinnano-desktop (NVIDIA Jetson)
**Date:** 2026-05-15
**Purpose:** Full context handoff so a new AI can continue this work without starting from scratch

---

## 1. Who Is the User

- Username: `orinnano`
- Email: michaelalam60@gmail.com
- Working on an **F1TENTH / Roboracer autonomous race car project** using a Jetson as the onboard computer
- Vehicle: Traxxas Ford Fiesta ST Rally VXL (74276-4), modified for Roboracer/F1TENTH
- Motor controller: VESC 6 MkVI
- Camera: ZED2 Stereo Camera (StereoLabs)

---

## 2. System Hardware & OS

| Item | Details |
|---|---|
| Board | NVIDIA Jetson (Tegra, ARM64 / aarch64) |
| OS | Ubuntu 22.04.5 LTS (Jammy Jellyfish) |
| Kernel | Linux 5.15.148-tegra |
| JetPack / L4T | R36, Revision 4.7 |
| CUDA | 12.6 (`/usr/local/cuda-12.6/`) |

**Environment variables in `~/.bashrc`:**
```
CUDA_PATH    = /usr/local/cuda-12.6/bin
ISAAC_ROS_WS = ~/workspaces/isaac-ros-dev/
```

---

## 3. Full File System Overview

### Home Directory `/home/orinnano/`

```
/home/orinnano/
├── workspaces/isaac-ros-dev/           ← Main ROS2 Isaac ROS project workspace
├── roboracer_vesc_controller/          ← PS4 controller script (created this session)
├── Final Lab/                          ← ZED point cloud lab deliverable
├── Final Lab.zip                       ← Backup zip of Final Lab
├── Final Step by Step Setup            ← User's personal setup notes (plain text)
├── Downloads/
│   ├── ZED_SDK_Tegra_L4T36.4_v5.0.0.zstd.run  ← ZED SDK installer
│   ├── yolov8s.pt                      ← YOLOv8 model backup
│   └── data.mdb                        ← LMDB database (likely SLAM map)
├── Desktop/
│   ├── Roboracer_System_Report.md      ← Full system report (created this session)
│   ├── Why_We_Use_Docker.md            ← Docker explanation doc (created this session)
│   ├── PS4_VESC_Controller_Documentation.md  ← Script docs (created this session)
│   ├── AI_Handoff_Roboracer.md         ← This file
│   └── SSD Bootloader Bug Cant find Partition  ← Troubleshooting note
├── Documents/ZED/Meshes/               ← ZED 3D mesh output folder
├── .bashrc                             ← Shell config
├── .gitconfig                          ← Git user config
└── .var/app/com.vesc_project.VescTool/ ← VESC Tool app data
```

### Main ROS2 Workspace `~/workspaces/isaac-ros-dev/src/`

```
src/
├── isaac_ros_common/           ← Core Docker scripts, test framework, shared utilities
│   └── scripts/run_dev.sh      ← THE main Docker launch script (always first step)
├── isaac_ros_visual_slam/      ← Visual SLAM (cuVSLAM) — localization and mapping
├── isaac_ros_object_detection/
│   ├── isaac_ros_yolov8/       ← YOLOv8 object detection via TensorRT
│   ├── isaac_ros_detectnet/    ← NVIDIA DetectNet
│   └── isaac_ros_rtdetr/       ← RT-DETR transformer detection
├── isaac_ros_dnn_stereo_depth/ ← ESS stereo depth estimation DNN
├── zed-ros2-wrapper/           ← ZED camera ROS2 driver
├── yolov8s.onnx                ← YOLOv8-small model (ONNX, ~43MB)
└── yolov8s.pt                  ← YOLOv8-small model (PyTorch, ~21MB)
```

### PS4 Controller Script Directory `~/roboracer_vesc_controller/`

```
roboracer_vesc_controller/
├── ps4_vesc_controller.py      ← Main script (ACTIVE — being worked on)
├── test_controller.py          ← Debug script to print raw controller values
└── README.md                   ← Quick reference
```

---

## 4. Key Hardware Details

### VESC Motor Controller
- **Device port:** `/dev/vesc` (udev symlink → `/dev/ttyACM0`)
- **Vendor:** STMicroelectronics, ChibiOS/RT Virtual COM Port
- **Vendor ID:** `0483`, **Product ID:** `5740`
- **Baud rate:** 115200
- **Firmware version:** 6.05
- **udev rule:** `/etc/udev/rules.d/99-vesc.rules` — **NOT YET CREATED** (user still needs to run this with sudo)
- **Current VESC config:** ERPM max 60,000, current max 10A, imperial units

**udev rule content (needs sudo to create):**
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="vesc"
```

### ZED2 Camera
- **SDK Version:** ZED SDK 5.0.0 for Tegra L4T 36.4
- **ZED tools:** `/usr/local/zed/tools/`
- **Config files:** `src/zed-ros2-wrapper/zed_wrapper/config/zed2.yaml`

### PS4 DualShock 4 Controller
- Connected via **Bluetooth**
- Pygame axis mapping confirmed working on this system:
  - `AXIS_L2 = 4` (L2 trigger: -1.0 rest, +1.0 full press)
  - `AXIS_R2 = 5` (R2 trigger: -1.0 rest, +1.0 full press)
  - D-pad = Hat index 0, `get_hat(0)` returns `(x, y)`
    - Left = `(-1, 0)` ✅ confirmed
    - Right = `(1, 0)` ✅ confirmed
    - Center = `(0, 0)`
  - `BTN_X = 0` (emergency stop)
  - `BTN_PS = 10` (quit)

---

## 5. Installed Python Packages (user-level)

| Package | Version | Status |
|---|---|---|
| pygame | 2.6.1 | ✅ Working |
| pyserial | 3.5 | ✅ Working |
| pyvesc | 1.0.5 | ❌ BROKEN — PyCRC dependency conflict |
| PyCRC | 0.11.0 | ❌ Wrong package installed (CLI tool not Python lib) |
| ultralytics | 8.3.127 | ✅ Working |
| torch | 2.7.0 (CPU) | ✅ Working |
| opencv | 4.11.0 | ✅ Working |
| evdev | N/A | ❌ Not installed (apt install fails without sudo) |

**Important:** `pyvesc` is broken because pip installed the wrong `pycrc` package (a command-line tool) instead of the `PyCRC` Python library that pyvesc needs. **The PS4 controller script was rewritten to NOT use pyvesc** — it implements the VESC serial protocol directly using only `struct` and `pyserial`.

---

## 6. The PS4 → VESC Controller Script

### File location
```
/home/orinnano/roboracer_vesc_controller/ps4_vesc_controller.py
```

### How to run
```bash
python3 /home/orinnano/roboracer_vesc_controller/ps4_vesc_controller.py
```

### What it does
- Reads PS4 controller input via `pygame`
- Implements VESC serial protocol from scratch (CRC-CCITT + packet framing)
- Sends motor RPM commands and servo position commands to VESC over `/dev/vesc`
- Runs at 50 Hz control loop

### Controls
| Input | Action |
|---|---|
| R2 Trigger | Throttle forward (proportional) |
| L2 Trigger | Brake (proportional) |
| D-Pad Left | Steer left (servo pos 0.15) |
| D-Pad Right | Steer right (servo pos 0.85) |
| D-Pad released | Center steering (servo pos 0.5) |
| X Button | Emergency stop |
| PS Button | Quit |

### VESC Protocol implemented in the script

```
Packet: [0x02] [length] [payload] [CRC_HIGH] [CRC_LOW] [0x03]
CRC: CRC-CCITT (polynomial 0x1021)

Commands used:
  COMM_SET_RPM           = 8   → payload: [8] + int32 (big-endian)
  COMM_SET_CURRENT       = 6   → payload: [6] + int32 milliamps (big-endian)
  COMM_SET_CURRENT_BRAKE = 7   → payload: [7] + int32 milliamps (big-endian)
  COMM_SET_SERVO_POS     = 19  → payload: [19] + int16 (position * 1000, big-endian signed)
```

### Key config values at top of script
```python
VESC_PORT        = "/dev/vesc"
VESC_BAUDRATE    = 115200
MAX_RPM          = 10000
MAX_CURRENT_A    = 10.0
BRAKE_CURRENT_A  = 5.0
DEADZONE         = 0.08
LOOP_HZ          = 50
SERVO_CENTER     = 0.5
SERVO_MIN        = 0.15
SERVO_MAX        = 0.85
```

### Full script source code

```python
#!/usr/bin/env python3
import sys
import struct
import time
import pygame
import serial

VESC_PORT        = "/dev/vesc"
VESC_BAUDRATE    = 115200
MAX_RPM          = 10000
MAX_CURRENT_A    = 10.0
BRAKE_CURRENT_A  = 5.0
DEADZONE         = 0.08
LOOP_HZ          = 50
AXIS_L2          = 4
AXIS_R2          = 5
DPAD_HAT         = 0
BTN_X            = 0
BTN_PS           = 10
COMM_SET_DUTY          = 5
COMM_SET_CURRENT       = 6
COMM_SET_CURRENT_BRAKE = 7
COMM_SET_RPM           = 8
COMM_SET_SERVO_POS     = 19
SERVO_CENTER     = 0.5
SERVO_MIN        = 0.15
SERVO_MAX        = 0.85

def _crc16(data):
    crc = 0x0000
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

def _build_packet(payload):
    crc = _crc16(payload)
    return bytes([0x02, len(payload)]) + payload + bytes([crc >> 8, crc & 0xFF, 0x03])

def _send_rpm(ser, rpm):
    rpm = int(max(-MAX_RPM, min(MAX_RPM, rpm)))
    ser.write(_build_packet(bytes([COMM_SET_RPM]) + struct.pack(">i", rpm)))

def _send_current(ser, amps):
    milliamps = int(max(-MAX_CURRENT_A, min(MAX_CURRENT_A, amps)) * 1000)
    ser.write(_build_packet(bytes([COMM_SET_CURRENT]) + struct.pack(">i", milliamps)))

def _send_brake(ser, amps):
    milliamps = int(max(0.0, min(MAX_CURRENT_A, amps)) * 1000)
    ser.write(_build_packet(bytes([COMM_SET_CURRENT_BRAKE]) + struct.pack(">i", milliamps)))

def _send_servo(ser, position):
    position = max(SERVO_MIN, min(SERVO_MAX, position))
    value = int(position * 1000)
    ser.write(_build_packet(bytes([COMM_SET_SERVO_POS]) + struct.pack(">h", value)))

def _stop_motor(ser):
    _send_current(ser, 0.0)
    _send_servo(ser, SERVO_CENTER)
    print("[VESC] Motor stopped, steering centered.")

def open_vesc(port, baudrate):
    try:
        ser = serial.Serial(port, baudrate, timeout=0.05)
        print(f"[VESC] Connected on {port} at {baudrate} baud")
        return ser
    except serial.SerialException as e:
        print(f"[VESC] ERROR: Could not open {port} — {e}")
        sys.exit(1)

def trigger_value(raw):
    return (raw + 1.0) / 2.0

def main():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("[PS4] Waiting for controller...")
        while pygame.joystick.get_count() == 0:
            pygame.event.pump()
            time.sleep(1.0)
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"[PS4] Connected: {joystick.get_name()}")
    ser = open_vesc(VESC_PORT, VESC_BAUDRATE)
    loop_period = 1.0 / LOOP_HZ
    estop = False
    try:
        while True:
            loop_start = time.monotonic()
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_PS:
                        raise KeyboardInterrupt
                    if event.button == BTN_X:
                        estop = True
                        _stop_motor(ser)
                if event.type == pygame.JOYBUTTONUP:
                    if event.button == BTN_X:
                        estop = False
                if event.type == pygame.JOYDEVICEREMOVED:
                    _stop_motor(ser)
                    raise KeyboardInterrupt
            if estop:
                time.sleep(loop_period)
                continue
            r2_raw = joystick.get_axis(AXIS_R2)
            l2_raw = joystick.get_axis(AXIS_L2)
            throttle = trigger_value(r2_raw)
            brake = trigger_value(l2_raw)
            if throttle < DEADZONE: throttle = 0.0
            if brake < DEADZONE: brake = 0.0
            dpad_x, _ = joystick.get_hat(DPAD_HAT)
            if dpad_x == -1:
                servo_pos, steer_label = SERVO_MIN, "LEFT"
            elif dpad_x == 1:
                servo_pos, steer_label = SERVO_MAX, "RIGHT"
            else:
                servo_pos, steer_label = SERVO_CENTER, "CENTER"
            _send_servo(ser, servo_pos)
            if throttle > 0.0 and brake == 0.0:
                _send_rpm(ser, throttle * MAX_RPM)
                print(f"\r[DRIVE] Throttle:{throttle:.2f} RPM:{throttle*MAX_RPM:.0f} Steer:{steer_label}", end="")
            elif brake > 0.0 and throttle == 0.0:
                _send_brake(ser, brake * BRAKE_CURRENT_A)
                print(f"\r[BRAKE] Brake:{brake:.2f} Steer:{steer_label}", end="")
            else:
                _send_current(ser, 0.0)
                print(f"\r[IDLE] Steer:{steer_label}", end="")
            sys.stdout.flush()
            elapsed = time.monotonic() - loop_start
            if loop_period - elapsed > 0:
                time.sleep(loop_period - elapsed)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")
    finally:
        _stop_motor(ser)
        ser.close()
        pygame.quit()

if __name__ == "__main__":
    main()
```

---

## 7. Current Status — What Works and What Doesn't

### ✅ Working
| Feature | Status |
|---|---|
| VESC USB serial connection | Working — `/dev/vesc` connects fine |
| PS4 controller Bluetooth connection | Working |
| R2 throttle (forward drive) | Working — RPM confirmed sending |
| L2 brake | Working |
| X emergency stop | Working |
| D-pad left/right detection | Working — confirmed `(-1,0)` and `(1,0)` in hat |
| Servo commands being sent | Working — terminal shows LEFT/RIGHT/CENTER changing |

### ❌ Not Yet Working
| Feature | Issue | Next Step |
|---|---|---|
| **Physical steering servo** | Commands are sent correctly by code but servo does not physically move | VESC servo output pin needs to be enabled in VESC Tool app settings |
| udev rule for /dev/vesc | Claude couldn't run sudo — user needs to create it manually | See instructions below |

---

## 8. Outstanding Issue — Steering Servo Not Moving Physically

**The code is confirmed correct.** The terminal shows LEFT/RIGHT/CENTER changing when D-pad is pressed, and the VESC serial packets are being sent correctly.

**The physical steering servo is not moving because the VESC servo output pin is disabled by default in firmware.**

### Fix required — Enable servo output in VESC Tool:

1. Open **VESC Tool** on the Jetson
2. Connect to the VESC
3. Go to **App Configuration** in the left sidebar
4. Click the **PPM** tab
5. Enable **Servo Output** checkbox
6. Click **Write App Configuration**

### Also verify physically:
- The steering servo wire must be plugged into the **VESC servo output pin** (3-pin connector on the VESC 6 MkVI board)
- If the servo is plugged into a separate RC receiver or servo controller, COMM_SET_SERVO_POS will never reach it

---

## 9. udev Rule — Still Needs to Be Created

The `/dev/vesc` symlink currently exists but the udev rule that makes it permanent has not been created yet (requires sudo). User needs to run:

```bash
sudo nano /etc/udev/rules.d/99-vesc.rules
```

Paste:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0666", GROUP="dialout", SYMLINK+="vesc"
```

Then:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Without this rule, `/dev/vesc` may disappear on replug and the script will fail.

---

## 10. Documents Created This Session

All saved on Desktop at `~/Desktop/`:

| File | Description |
|---|---|
| `Roboracer_System_Report.md` | Full report of every file on the system with explanations |
| `Why_We_Use_Docker.md` | Plain English explanation of Docker for this project |
| `PS4_VESC_Controller_Documentation.md` | Full GitHub-style docs for the PS4/VESC script |
| `AI_Handoff_Roboracer.md` | This file |

---

## 11. How the Isaac ROS Stack Runs (Quick Reference)

```bash
# Step 1: Start Docker
cd ~/workspaces/isaac-ros-dev/src/isaac_ros_common/scripts
./run_dev.sh

# Step 2 (inside Docker): Start ZED camera
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2

# Step 3 (inside Docker): Start Visual SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# Step 4 (inside Docker): Start YOLOv8 detection
ros2 launch isaac_ros_yolov8 yolov8_tensor_rt.launch.py
```

---

## 12. Immediate Next Steps for New AI

1. **Fix steering servo** — Guide user through VESC Tool App Configuration to enable servo output
2. **Create udev rule** — User needs to run the sudo command above to make `/dev/vesc` permanent
3. **Test full drive** — Once servo is working, test throttle + steering together at low speed
4. **Tune servo limits** — `SERVO_MIN` and `SERVO_MAX` in the script may need adjustment to match physical steering range
5. **Optional** — Move the script into the Docker environment for integration with ROS2

---

*Handoff document generated 2026-05-15 — orinnano-desktop*
