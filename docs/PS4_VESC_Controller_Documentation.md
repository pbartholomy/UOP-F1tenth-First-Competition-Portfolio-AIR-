# PS4 Controller → VESC Motor Controller
## Roboracer / F1TENTH Project — Complete Documentation

> Control your F1TENTH race car with a PS4 DualShock 4 controller over Bluetooth.  
> The script reads controller input and sends motor commands to the VESC in real time.

---

## Table of Contents

1. [What This Script Does](#1-what-this-script-does)
2. [How It Works — Simple Overview](#2-how-it-works--simple-overview)
3. [Hardware You Need](#3-hardware-you-need)
4. [Software Requirements](#4-software-requirements)
5. [Setup — Step by Step](#5-setup--step-by-step)
6. [Running the Script](#6-running-the-script)
7. [Controller Layout & Controls](#7-controller-layout--controls)
8. [Understanding the Code](#8-understanding-the-code)
9. [Configuration Settings](#9-configuration-settings)
10. [Troubleshooting](#10-troubleshooting)
11. [File Structure](#11-file-structure)

---

## 1. What This Script Does

This Python script acts as the **bridge between your PS4 controller and the VESC motor controller** on the Roboracer car.

- You press **R2** → the car drives forward
- You press **L2** → the car brakes
- You press **X** → emergency stop
- The car's speed is proportional to how hard you press the trigger (soft press = slow, full press = fast)

It runs directly on the NVIDIA Jetson (the computer on the car) and communicates with the VESC over USB serial.

---

## 2. How It Works — Simple Overview

```
[PS4 Controller]
      |
      | Bluetooth (wireless)
      |
[NVIDIA Jetson]
      |
      | Runs ps4_vesc_controller.py
      | Reads controller input via pygame
      | Calculates motor RPM from trigger position
      |
      | USB Serial (/dev/vesc)
      |
[VESC Motor Controller]
      |
      | PWM signal
      |
[Brushless Motor → Wheels]
```

**Step by step what happens every 1/50th of a second (50 times per second):**

1. The script reads the current position of every button and trigger on the PS4 controller
2. It converts the R2 trigger (0% to 100% pressed) into a target RPM (0 to MAX_RPM)
3. It sends that RPM command to the VESC over the USB cable as a serial message
4. The VESC receives the command and spins the motor at the requested speed
5. Repeat

---

## 3. Hardware You Need

| Hardware | Purpose |
|---|---|
| NVIDIA Jetson (any model) | The onboard computer that runs this script |
| PS4 DualShock 4 Controller | The wireless controller you hold |
| VESC Motor Controller | Controls the brushless drive motor |
| USB Cable (VESC to Jetson) | Serial communication between Jetson and VESC |
| Bluetooth adapter | Built into the Jetson — pairs the PS4 controller wirelessly |

---

## 4. Software Requirements

### Python Version
Python 3.10 (already installed on the Jetson)

### Required Python Packages

Install them with one command:

```bash
pip3 install pygame pyvesc pyserial
```

| Package | What It Does |
|---|---|
| `pygame` | Reads input from the PS4 controller (buttons, triggers, sticks) |
| `pyvesc` | Formats and encodes commands in the VESC communication protocol |
| `pyserial` | Sends those commands over the USB serial connection to the VESC |

### Required System Setup

The VESC must appear as `/dev/vesc` — this is set up using a **udev rule**.  
See the udev rule setup in the main System Report if this is not already done.

To verify the VESC is visible:
```bash
ls /dev/vesc
```

---

## 5. Setup — Step by Step

### Step 1 — Install the Python packages

```bash
pip3 install pygame pyvesc pyserial
```

### Step 2 — Pair the PS4 Controller via Bluetooth

1. On the Jetson, open **Bluetooth Settings** (top right menu bar)
2. On the PS4 controller, hold **Share button + PS button** at the same time until the light bar flashes rapidly — this means it is in pairing mode
3. In Bluetooth Settings, click the PS4 controller when it appears in the list
4. Click **Pair** and wait for it to connect — the light bar will turn solid blue when connected

Verify the controller is recognized as a joystick:
```bash
ls /dev/input/js*
# Should show: /dev/input/js0
```

### Step 3 — Verify the VESC is connected

```bash
ls /dev/vesc
# Should show: /dev/vesc -> ttyACM0
```

If you get "No such file or directory" the VESC is not plugged in or the udev rule is not set up.

### Step 4 — (Optional) Check your permissions

```bash
groups
```

You should see `dialout` in the list. If not, run:

```bash
sudo usermod -a -G dialout $USER
```

Then log out and back in.

---

## 6. Running the Script

Navigate to the script folder and run it:

```bash
cd ~/roboracer_vesc_controller
python3 ps4_vesc_controller.py
```

### What You Will See

```
==================================================
  Roboracer PS4 -> VESC Controller
==================================================
[PS4] Controller connected: Wireless Controller
      Axes: 6  Buttons: 13
[VESC] Connected on /dev/vesc at 115200 baud

  Controls:
    R2          = Throttle (forward)
    L2          = Brake / Reverse
    Left Stick  = Steering (logged, for future use)
    X Button    = Emergency stop
    PS Button   = Quit

[IDLE]  Steer: 0.00
```

The bottom line updates in real time as you move the controller.

### Stopping the Script

- Press the **PS button** on the controller to quit cleanly
- Or press `Ctrl+C` in the terminal
- Both methods safely stop the motor before exiting

---

## 7. Controller Layout & Controls

```
        L1          R1
        L2          R2          ← Throttle (forward)
    ___________________________
   /  [Share]    [Options]      \
  | [d-pad]   (left)  (right)   |
  |            stick   stick    |
  |                             |
  |  [PS]                       |
  |         [X] ← E-Stop        |
   \___________________________/

Left Stick X axis = Steering input (logged for future Ackermann use)
L2 Trigger        = Brake
R2 Trigger        = Throttle
```

| Input | Action | Notes |
|---|---|---|
| **R2 Trigger** | Drive forward | Proportional — soft press = slow, full press = max RPM |
| **L2 Trigger** | Brake | Applies brake current to slow the motor |
| **Both R2 + L2** | Brake wins | Safety behavior — braking always takes priority |
| **Left Stick X** | Steering | Currently logged only — ready for servo/Ackermann integration |
| **X Button (hold)** | Emergency stop | Immediately cuts motor power while held |
| **X Button (release)** | Resume | Releases the emergency stop |
| **PS Button** | Quit | Safely stops the motor and exits the script |

---

## 8. Understanding the Code

The script is organized into four sections. Here is a plain-English explanation of each part:

### Section 1 — Configuration (top of file)

```python
VESC_PORT     = "/dev/vesc"
MAX_RPM       = 10000
MAX_CURRENT_A = 10.0
LOOP_HZ       = 50
```

All the numbers you might want to change are at the very top so you never have to dig through the code. See Section 9 for what each one does.

---

### Section 2 — VESC Communication Functions

These are helper functions that handle talking to the VESC:

**`open_vesc(port, baudrate)`**
Opens the USB serial connection to the VESC. If the VESC is not plugged in, the script prints a clear error and exits instead of crashing with a confusing error message.

**`send_rpm(ser, rpm)`**
Tells the VESC to spin the motor at a specific RPM. Automatically clamps the value so it never exceeds `MAX_RPM` even if you pass in a bigger number.

**`send_current(ser, amps)`**
Tells the VESC to apply a specific amount of electrical current to the motor. Used for coasting (zero current = motor spins freely).

**`send_brake(ser, amps)`**
Applies braking current — this actively resists the motor spinning, slowing the car down.

**`stop_motor(ser)`**
Sets current to zero. Called automatically on exit or emergency stop to make sure the motor never keeps running after the script ends.

---

### Section 3 — Controller Reading Functions

**`get_axis(joystick, axis_id)`**
Reads a joystick axis and applies the **deadzone** — a small zone around the center position that is ignored. Without this, the controller would send tiny wobbling commands even when you are not touching it (controllers are never perfectly centered).

**`trigger_value(raw)`**
PS4 triggers are unusual — they report `-1.0` when fully released and `+1.0` when fully pressed. This function converts that into a more intuitive `0.0` (not pressed) to `1.0` (fully pressed) range.

---

### Section 4 — Main Control Loop

This is the heart of the script. It runs **50 times per second** and does the following each time:

```
1. Check for any button presses (PS button to quit, X button for e-stop)
2. Read the current trigger positions
3. Convert trigger position to motor command
4. Send the command to the VESC
5. Wait until it is time for the next loop cycle
```

**Priority logic (important for safety):**

```
If R2 pressed AND L2 not pressed  →  Drive forward at throttle RPM
If L2 pressed AND R2 not pressed  →  Apply brake current
If both pressed                   →  Brake wins (safety)
If neither pressed                →  Coast (zero current)
If X button held                  →  Emergency stop, ignore everything else
```

---

## 9. Configuration Settings

All settings are at the top of `ps4_vesc_controller.py`. Edit these to tune the car's behavior.

| Setting | Default | What It Controls | When to Change |
|---|---|---|---|
| `VESC_PORT` | `/dev/vesc` | Which serial port the VESC is on | If udev rule is not set up, change to `/dev/ttyACM0` |
| `VESC_BAUDRATE` | `115200` | Serial communication speed | Only change if your VESC firmware uses a different baud rate |
| `MAX_RPM` | `10000` | Maximum motor speed | **Start low (3000) and increase gradually during testing** |
| `MAX_CURRENT_A` | `10.0` | Maximum drive current (amps) | Match this to your VESC Tool configuration |
| `BRAKE_CURRENT_A` | `5.0` | How hard the brakes apply | Increase for stronger braking |
| `DEADZONE` | `0.08` | Minimum trigger input that does anything | Increase if the car creeps without input |
| `LOOP_HZ` | `50` | How many times per second the loop runs | 50 Hz is standard — no need to change |

### Recommended Starting Settings for First Test

```python
MAX_RPM       = 3000    # Start slow — increase only after verifying behavior
MAX_CURRENT_A = 5.0     # Half the VESC limit for initial testing
BRAKE_CURRENT_A = 3.0   # Gentle braking for testing
```

---

## 10. Troubleshooting

### "No controller detected. Please connect your PS4 controller via Bluetooth."

The script is waiting for the controller to connect. The PS4 controller is not paired or Bluetooth is off.

**Fix:**
1. Check Bluetooth is on in system settings
2. Put controller in pairing mode (hold Share + PS)
3. Pair it in Bluetooth settings
4. The script will detect it automatically once connected — no need to restart

---

### "ERROR: Could not open /dev/vesc"

The VESC is not visible to the system.

**Fix — check these in order:**
```bash
# Is the VESC plugged in?
ls /dev/vesc

# If not found, is it on a different port?
ls /dev/ttyACM*

# If ttyACM0 exists but not /dev/vesc, the udev rule is missing
# Temporary fix:
sudo chmod 666 /dev/ttyACM0
# Then edit VESC_PORT = "/dev/ttyACM0" in the script
```

---

### "Permission denied: /dev/vesc"

Your user does not have permission to access the serial port.

**Fix:**
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in, then try again
```

---

### Car moves without touching the controller (creeping)

The controller deadzone is too small.

**Fix:** Increase `DEADZONE` in the config section:
```python
DEADZONE = 0.15   # Try a larger value
```

---

### Script exits immediately after starting

Usually a missing Python package.

**Fix:**
```bash
pip3 install pygame pyvesc pyserial
```

---

### Controller connects but car does not move

1. Verify the VESC is powered on (LED on VESC should be lit)
2. Check that `/dev/vesc` exists and is accessible
3. Try pressing R2 fully — check the terminal output shows throttle > 0
4. Check `MAX_RPM` is not set to 0

---

## 11. File Structure

```
~/roboracer_vesc_controller/
├── ps4_vesc_controller.py    ← Main script — run this
└── README.md                 ← Quick reference guide
```

---

## Quick Start Cheat Sheet

```bash
# 1. Install packages (first time only)
pip3 install pygame pyvesc pyserial

# 2. Pair PS4 controller: hold Share + PS until light bar flashes

# 3. Plug in VESC via USB

# 4. Run the script
cd ~/roboracer_vesc_controller
python3 ps4_vesc_controller.py

# 5. Drive!
#    R2 = throttle   L2 = brake   X = e-stop   PS = quit
```

---

*Documentation by Claude Code — Roboracer / F1TENTH Project — orinnano-desktop — 2026-05-15*
