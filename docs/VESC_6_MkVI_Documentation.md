# VESC 6 MkVI — Motor Controller Documentation
## Roboracer / F1TENTH Project — orinnano-desktop
### Source: Official TRAMPA VESC 6 MkVI Manual (rev 1.0)

---

## Table of Contents

1. [What is the VESC?](#1-what-is-the-vesc)
2. [What Does It Do on the Race Car?](#2-what-does-it-do-on-the-race-car)
3. [Technical Specifications](#3-technical-specifications)
4. [Full Feature List](#4-full-feature-list)
5. [All Connectors & Pin Mappings](#5-all-connectors--pin-mappings)
6. [LED Status Indicators](#6-led-status-indicators)
7. [Motor Modes](#7-motor-modes)
8. [Motor Settings Explained](#8-motor-settings-explained)
9. [Power Switch Wiring Options](#9-power-switch-wiring-options)
10. [Ground Loop Warning](#10-ground-loop-warning)
11. [How It Connects to the Jetson](#11-how-it-connects-to-the-jetson)
12. [How the Script Talks to the VESC](#12-how-the-script-talks-to-the-vesc)
13. [Commands Used in This Project](#13-commands-used-in-this-project)
14. [Wiring Guide for This Car](#14-wiring-guide-for-this-car)
15. [Troubleshooting](#15-troubleshooting)

---

## 1. What is the VESC?

**VESC** stands for **Vedder Electronic Speed Controller** — designed by Benjamin Vedder. It is an open-source, professional-grade motor controller for brushless DC (BLDC) and DC motors.

The **VESC 6 MkVI** is manufactured by **TRAMPA Boards Ltd** (Nottingham, UK). It is the standard motor controller used on F1TENTH race cars worldwide.

Think of it as the **brain between the battery and the motor**. The Jetson tells the VESC what speed to run, and the VESC handles all the complex electrical work of spinning the motor safely and precisely.

---

## 2. What Does It Do on the Race Car?

The VESC has two main jobs on the Roboracer:

### Job 1 — Drive Motor Control
Controls the rear brushless motor that drives the car.
- Receives RPM or current commands from the Jetson over USB
- Converts commands into 3-phase electrical signals (A, B, C)
- Handles acceleration, deceleration, and regenerative braking

### Job 2 — Steering Servo Control
Outputs a PPM/PWM signal to the steering servo.
- Receives a servo position (0.0 to 1.0) from the Jetson
- Outputs a PWM signal on the SERVO pin
- The servo physically moves the front wheels left or right

```
[Jetson]
   |
   | USB (/dev/vesc)
   |
[VESC 6 MkVI]
   |                  |
   | Phase A, B, C    | PWM (SERVO pin)
   |                  |
[Brushless Motor]  [Steering Servo]
   |
[Rear Wheels → Car moves]
```

---

## 3. Technical Specifications

*From the official TRAMPA VESC 6 MkVI Technical Data Sheet*

| Specification | Value |
|---|---|
| **Input Voltage** | 6V – 60V (safe for 3S–12S LiPo) |
| **Max Input Voltage (absolute)** | 60V — spikes must not exceed this! |
| **Recommended Max Operating** | 52V input |
| **Battery Configuration** | 3S to 12S Li-ion / LiPo (11.1V – 44.4V typical) |
| **Continuous Current** | 80–100A (depends on temperature & airflow) |
| **Burst Current** | 120A |
| **5V Output** | 1A — for external electronics (receiver, servo power) |
| **3.3V Output** | 0.5A — for external electronics |
| **Motor Types** | DC, BLDC, PMSM (FOC sinusoidal) |
| **Supported Sensors** | ABI, HALL, AS5047, TS5700N8501 + more |
| **Microcontroller** | STMicroelectronics STM32 |
| **Firmware OS** | ChibiOS/RT |
| **Firmware Version (this car)** | 6.05 |
| **USB** | Micro USB (configuration & serial commands) |
| **Certifications** | FCC, RoHS2, CE, UKCA |
| **Manufacturer** | TRAMPA Boards Ltd, Nottingham, UK |

---

## 4. Full Feature List

*Directly from the TRAMPA manual:*

- Current and voltage measurement on all phases
- **Regenerative braking** — motor acts as generator to slow the car and charge battery
- **Advanced FOC motor control** — smooth, quiet, efficient motor operation
- Sensored or sensorless operation + hybrid mode
- Sensorless high torque motor startup (HFI, sHFI, VSS)
- Configurable RPM, current, voltage, and power limits
- **Input sources:** PPM (PWM), Analog, NRF, CAN
- **Communication ports:** USB, CAN, 2x UART, I²C, SPI
- Throttle curve and ramping for all input sources
- Seamless 4-quadrant operation (forward/reverse/brake/regen)
- Motor revolution, amp hour, watt hour counting
- Real time data analysis via communication ports
- Built-in Power Switch (Hibernation mode)
- **Custom script loading** — add features via VESC scripting
- Spin to start, auto power off
- Adjustable protection against:
  - Low input voltage
  - High input voltage
  - High motor current
  - High input current
  - High regenerative braking current
  - High RPM (separate limits per direction)
  - Over temperature (MOSFET and motor)

---

## 5. All Connectors & Pin Mappings

*From the official connector diagram in the TRAMPA manual*

---

### SENSOR Port — Motor Position Sensors
For **ABI**, **HALL**, or **AS5047P** motor position sensors.
Allows precise and powerful rotation tracking from standstill.
> Check and adjust sensor voltage in VESC Tool (3.3V or 5V setting)!

---

### HALL / Sensor Connector

| Pin | Label | Purpose |
|---|---|---|
| 1 | VMP | Voltage / power for sensor |
| 2 | EMP | Enable / sensor power |
| 3 | GND | Ground |
| 4 | MOSI / MOI | SPI data out (AS5047) |
| 5 | SCK | SPI clock |
| 6 | CLK | Clock signal |
| 7 | GND | Ground |
| 8 | IO | Data I/O |
| 9 | RST | Reset |

> Hall sensors (A, B, C) connect here for sensored BLDC operation.

---

### NRF Connector — Wireless UART
For connecting **NRF transceivers** or other UART wireless devices (e.g. VESC remote).

| Pin | Label | Purpose |
|---|---|---|
| 1 | TX | UART transmit |
| 2 | VCC | Power |
| 3 | GND | Ground |
| 4 | IO | Data I/O |
| 5 | RST | Reset |

---

### COMM Connector — I2C / UART / ADC
**I2C, UART, and ADC interface** for microcontrollers such as Arduino, Raspberry Pi, or the NVIDIA Jetson. Also supports analogue throttle input.

| Pin | Label | Purpose |
|---|---|---|
| 1 | Power Switch | Built-in power switch connection |
| 2 | MISO | SPI data in |
| 3 | SCL | I2C clock |
| 4 | MOSI | SPI data out |
| 5 | SDA | I2C data |
| 6 | NSS | SPI chip select |
| 7 | SCK | SPI clock |
| 8 | ADC2 | Analogue input 2 (e.g. throttle pot) |
| 9 | TX | UART transmit |
| 10 | ADC1 | Analogue input 1 |
| 11 | RX | UART receive |
| 12 | GND | Ground |
| 13 | VCC | Voltage output |
| 14 | 5V | 5V output |

---

### PPM / SERVO Connector — Steering Servo Output
**This is the connector used for steering in this project.**
Connects a PPM receiver (RC radio) OR outputs servo signal to the steering servo.

```
┌─────────┬─────┬─────┐
│  SERVO  │ 5V  │ GND │
└─────────┴─────┴─────┘
```

| Pin | Label | Purpose |
|---|---|---|
| 1 | SERVO | PPM signal input OR servo PWM output |
| 2 | 5V | 5V power (from VESC 1A output) |
| 3 | GND | Ground |

> **For this project:** The steering servo signal wire connects to the SERVO pin.  
> **WARNING from manual:** NEVER connect one receiver to two or more VESCs using Y-PPM wiring. Permanent damage may result!

---

### CAN Bus Connector — Multi-VESC Communication
For connecting multiple VESC controllers together (e.g. dual motor setups).

| Pin | Label | Purpose |
|---|---|---|
| 1 | GND | Ground |
| 2 | CL | CAN Low signal |
| 3 | CH | CAN High signal |
| 4 | 5V | 5V output |

> **WARNING from manual:** Only connect CAN L to CAN L and CAN H to CAN H.  
> **DO NOT interconnect 5V and GND between VESCs in an array — permanent damage may result!**

---

### STS — Spin To Start
Bridge these two pins to enable **wake-up by spinning the motor** (auto power on when wheel spins).  
Add a jumper on the STS port + wire bridge on each controller to activate.

---

### USB — Configuration & Serial Commands
**Micro USB port** — connects to the Jetson for:
- VESC Tool configuration
- Firmware updates
- Real-time data analysis
- Serial commands from the Python script (`/dev/vesc`)

---

### SWD — Serial Wire Debug
**Serial Wire Debug** port for direct access to the STM32 chip.
Used for advanced diagnostics and real-time data — not needed for normal operation.

---

### Motor Terminals — A, B, C
Three-phase motor output connections.

| Terminal | Wire Color (typical) | Phase |
|---|---|---|
| A | Yellow | Phase A |
| B | Blue | Phase B |
| C | Red | Phase C |

> Motor wires can be plugged in randomly in most cases.  
> If the motor spins backwards — swap any two of the three phase wires.  
> DC motors: use only A and C — leave B unplugged.

---

### Main Power Terminals

| Terminal | Purpose |
|---|---|
| B+ (large pad) | Battery positive — up to 60V |
| B- (large pad) | Battery negative / ground |

> Always use an Anti-Spark connector (XT90S recommended) and a fuse in the battery line.

---

## 6. LED Status Indicators

*Directly from the TRAMPA manual:*

| LED | Meaning |
|---|---|
| **Blue** | Device is powered up |
| **Green (dim)** | SW Running — firmware installed and running |
| **Green (bright)** | Device is actively driving the motor |
| **Red** | Fault code — something is wrong! Read the fault code in VESC Tool |

---

## 7. Motor Modes

| Mode | Description |
|---|---|
| **BLDC** | Block Commutation (Trapezoidal). More noise, less efficient, simpler. Less likely to have problems. |
| **FOC** | Field Oriented Control — Sinusoidal Commutation (Sine Wave). Silent, efficient, more complex. Recommended for smooth operation. |
| **DC** | For brushed DC motors (use terminals A and C only) |

---

## 8. Motor Settings Explained

*From the TRAMPA manual Motor Settings Panel:*

| Setting | What It Does |
|---|---|
| **Battery Cutoff Start** | VESC reduces power when voltage drops below this value (e.g. 3.4V/cell for LiPo) — protects battery |
| **Battery Cutoff End** | VESC stops motor completely below this value (e.g. 3.1V/cell) — prevents battery damage |
| **Motor Current Max** | Maximum amps the motor can draw. Set according to motor specs. Too high = overheating |
| **Motor Current Max Brake** | Maximum current generated during regenerative braking. Check battery can handle it |
| **Absolute Maximum Current** | Peak amp flow allowed in the system |
| **Battery Current Max** | Maximum continuous current drain from battery |
| **Battery Current Max Regen** | Maximum current fed back into battery during braking — check battery data sheet |
| **MOSFET Temp Cutoff Start** | VESC reduces power at this temp (default: 80°C) |
| **MOSFET Temp Cutoff End** | VESC stops completely at this temp (default: 100°C) |
| **Motor Temp Cutoff Start** | Reduces power at this motor temp (default: 80°C) — requires temp sensor |
| **Motor Temp Cutoff End** | Stops motor at this motor temp (default: 100°C) — requires temp sensor |

> **Warning from manual:** Wrong settings may overstress your motor and/or battery. Start with safe values and check if anything gets hot during operation.

---

## 9. Power Switch Wiring Options

*From the TRAMPA switch wiring diagram:*

| Option | Method | Features |
|---|---|---|
| **Option 1** | LED (5V) illuminated power switch | Manual on/off, roll-to-start, LED shows state, auto power off |
| **Option 2** | Non-illuminated power switch | Manual on/off, roll-to-start, auto power off |
| **Option 3** | Spin To Start (STS jumper + wire bridge) | No manual switch needed — powers on when wheel spins, auto power off |
| **Option 4** | External soft-start power switch | No roll-to-start, no auto power off, no integrated switch used |

> For F1TENTH: **Option 4** (external power switch) is most common — the car has its own power switch on the chassis.

---

## 10. Ground Loop Warning

*Critical safety information from the TRAMPA manual:*

**Ground loops will damage your devices.**

- When the Jetson (or any computer) is connected to the VESC via USB **and** also shares a ground through another connection — a ground loop is created
- Ground loops can permanently damage the VESC, the Jetson, or both
- **If using USB from a mains-powered computer:** use a **USB isolator**
- **The Jetson running on battery through the VESC's 5V:** this is ground-loop free ✓
- **Only share battery ground between VESCs** — never share 5V or signal grounds

---

## 11. How It Connects to the Jetson

```
VESC 6 MkVI  ──Micro USB──▶  Jetson USB port
                                   │
                              /dev/ttyACM0
                                   │
                         udev rule (99-vesc.rules)
                                   │
                               /dev/vesc   ← script uses this
```

Verify connection:
```bash
ls /dev/vesc
udevadm info --name=/dev/vesc
```

---

## 12. How the Script Talks to the VESC

The Python script communicates using the **VESC serial protocol** over USB.

### Packet Structure
```
┌────────┬────────┬─────────────────┬──────────────┬─────────┐
│  0x02  │ Length │     Payload     │  CRC 2 bytes │  0x03  │
│ (start)│ 1 byte │ command + data  │  (CRC-CCITT) │  (stop) │
└────────┴────────┴─────────────────┴──────────────┴─────────┘
```

- **0x02** — start of packet
- **Length** — number of bytes in payload
- **Payload** — command ID + data bytes
- **CRC** — 2-byte checksum so VESC can verify packet integrity
- **0x03** — end of packet

---

## 13. Commands Used in This Project

| Command | ID | Data Format | What It Does |
|---|---|---|---|
| `COMM_SET_RPM` | 8 | 4-byte signed int (RPM) | Spins motor at target RPM |
| `COMM_SET_CURRENT` | 6 | 4-byte signed int (milliamps) | Applies current — 0A = coast |
| `COMM_SET_CURRENT_BRAKE` | 7 | 4-byte signed int (milliamps) | Applies braking current |
| `COMM_SET_SERVO_POS` | 19 | 2-byte unsigned int (pos × 1000) | Moves steering servo |

### Servo Position Reference

| Script Value | Value Sent | Steering |
|---|---|---|
| `SERVO_MIN = 0.15` | 150 | Full left |
| `SERVO_CENTER = 0.5` | 500 | Straight ahead |
| `SERVO_MAX = 0.85` | 850 | Full right |

---

## 14. Wiring Guide for This Car

### Steering Servo → PPM/SERVO Connector

```
Servo Wire          VESC PPM Connector
──────────          ──────────────────
Black / Brown  ──▶  GND  (pin 3)
Red            ──▶  5V   (pin 2)
White / Yellow ──▶  SERVO (pin 1)  ← PWM signal
```

### Motor → Phase Terminals

```
Motor yellow wire ──▶  Terminal A
Motor blue wire   ──▶  Terminal B
Motor red wire    ──▶  Terminal C
```
> If motor spins backwards — swap any two phase wires.

### Battery → Power Terminals

```
Battery (+) ──▶  B+ (large terminal)
Battery (-) ──▶  B- (large terminal)
```
> Always use XT90S anti-spark connector + inline fuse.

### Safety Requirements (from manual)
1. **Safety power cut-off** (switch or BMS)
2. **Fuse** rated for your electrical system
3. **Compatible input device** (legal, interference-free)
4. **Safe settings** configured in VESC Tool
5. **BMS** if using motor as generator / regenerative braking

---

## 15. Troubleshooting

### `/dev/vesc` not found
```bash
ls /dev/ttyACM*          # Check if VESC detected at all
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Permission denied
```bash
sudo usermod -a -G dialout $USER   # Log out and back in after
```

### Motor spins wrong direction
Swap any two of the three motor phase wires (A↔B, B↔C, or A↔C).

### Steering servo not moving
- Confirm servo signal wire is on SERVO pin (not 5V or GND)
- In VESC Tool → App Settings → verify PPM/servo output is enabled
- Check terminal shows `Servo:` value changing when D-pad pressed

### Red LED (fault)
Connect via VESC Tool → Real Time Data → read the fault code.  
Common faults: over-current, over-temperature, low voltage cutoff.

### VESC gets hot
- Reduce `MAX_CURRENT_A` in the script
- Ensure airflow around the VESC
- Check motor is not mechanically jammed

---

## Quick Reference

```bash
# Verify VESC connected
ls /dev/vesc

# Launch VESC Tool
flatpak run com.vesc_project.VescTool

# Run PS4 controller script
python3 ~/roboracer_vesc_controller/ps4_vesc_controller.py

# Get full VESC device info
udevadm info --name=/dev/vesc
```

---

## 16. Full Pinout Diagram

```
╔══════════════════════════════════════════════════════════════════════╗
║                        VESC 6 MkVI — Full Pinout                    ║
║                     Designed by Benjamin Vedder                      ║
║                       Manufactured by TRAMPA                        ║
╚══════════════════════════════════════════════════════════════════════╝

━━━━━━━━━━━━━━━━━ MOTOR PHASE TERMINALS (top) ━━━━━━━━━━━━━━━━━━━━━━━

         ┌──────┐   ┌──────┐   ┌──────┐
         │  C   │   │  B   │   │  A   │
         │(Red) │   │(Blue)│   │(Yel) │
         └──────┘   └──────┘   └──────┘
         Phase C    Phase B    Phase A
         ← Swap any two wires if motor spins the wrong direction →

━━━━━━━━━━━━━━━━━━━━━━━━━ BOARD TOP VIEW ━━━━━━━━━━━━━━━━━━━━━━━━━━━━

┌────────────────────────────────────────────────────────────────────┐
│  ┌── SENSE ──┐                              ┌─ NRF ─┐             │
│  │ 1  VMP    │                              │ 1  TX  │             │
│  │ 2  EMP    │   ┌──────────────────┐       │ 2  VCC │             │
│  │ 3  GND    │   │                  │       │ 3  GND │             │
│  │ 4  MOSI   │   │   VESC  6 MkVI  │       │ 4  IO  │             │
│  │ 5  SCK    │   │                  │       │ 5  RST │             │
│  │ 6  CLK    │   │  www.vesc-       │       └────────┘             │
│  │ 7  GND    │   │  project.com     │                              │
│  │ 8  IO     │   │                  │       ┌─── SWD ───┐          │
│  │ 9  RST    │   │  TRAMPA          │       │ 1  GND    │          │
│  └───────────┘   └──────────────────┘       │ 2  IO     │          │
│                                             │ 3  RST    │          │
│  Supports:                                  └───────────┘          │
│  • HALL sensors (A, B, C)                                          │
│  • ABI encoder                              ┌───── COMM ──────┐    │
│  • AS5047P magnetic encoder                 │ 1  Power Switch │    │
│  • TS5700N8501                              │ 2  MISO        │    │
│  Check voltage in VESC Tool (3.3V or 5V)!  │ 3  SCL         │    │
│                                             │ 4  MOSI        │    │
│                                             │ 5  SDA         │    │
│                                             │ 6  NSS         │    │
│                                             │ 7  SCK         │    │
│                                             │ 8  ADC2        │    │
│                                             │ 9  TX          │    │
│                                             │ 10 ADC1        │    │
│                                             │ 11 RX          │    │
│                                             │ 12 GND         │    │
│                                             │ 13 VCC         │    │
│                                             │ 14 5V          │    │
│                                             └────────────────┘    │
│                                             I2C / UART / ADC       │
│                                             (Jetson, Arduino, RPi) │
└────────────────────────────────────────────────────────────────────┘

━━━━━━━━━━━━━━━━━ BOTTOM CONNECTORS ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  ┌─── PPM / SERVO ───┐   ┌─── CAN ───┐   ┌─ STS ─┐   ┌── USB ──┐
  │ 1  SERVO (PWM) ●  │   │ 1  GND    │   │bridge │   │ Micro   │
  │ 2  5V  (power) ●  │   │ 2  CL     │   │for STS│   │  USB    │
  │ 3  GND         ●  │   │ 3  CH     │   └───────┘   └─────────┘
  └───────────────────┘   │ 4  5V     │   Spin-To-Start
                          └───────────┘
  Servo wiring:           CAN Bus
  Red    wire → pin 2 (5V)    (multi-VESC arrays)
  Black  wire → pin 3 (GND)
  Signal wire → pin 1 (SERVO) ← THIS ONE STEERS THE CAR

━━━━━━━━━━━━━━━━ MAIN POWER TERMINALS (bottom pads) ━━━━━━━━━━━━━━━━

         ┌─────────────┐         ┌─────────────┐
         │     B+      │         │     B-      │
         │  Battery +  │         │  Battery -  │
         │  (up to 60V)│         │   (Ground)  │
         └─────────────┘         └─────────────┘
                    ↑                   ↑
              XT90S Anti-Spark connector recommended
              Always use an inline FUSE

━━━━━━━━━━━━━━━━━━━━━━━ LED INDICATORS ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  Blue              = Powered up
  Green (dim)       = Firmware running
  Green (bright)    = Actively driving motor
  Red               = FAULT — read fault code in VESC Tool

━━━━━━━━━━━━━━━━━━━━━━━ QUICK WIRING SUMMARY ━━━━━━━━━━━━━━━━━━━━━━━

  For this Roboracer project:

  ┌──────────────────┬──────────────┬─────────────────────────────┐
  │ What             │ Connector    │ Pin                         │
  ├──────────────────┼──────────────┼─────────────────────────────┤
  │ Jetson (serial)  │ Micro USB    │ Full port → /dev/vesc       │
  │ Steering servo   │ PPM/SERVO    │ Signal → Pin 1 (SERVO)      │
  │ Servo power      │ PPM/SERVO    │ Red    → Pin 2 (5V)         │
  │ Servo ground     │ PPM/SERVO    │ Black  → Pin 3 (GND)        │
  │ Motor phase A    │ Terminal A   │ Yellow motor wire           │
  │ Motor phase B    │ Terminal B   │ Blue motor wire             │
  │ Motor phase C    │ Terminal C   │ Red motor wire              │
  │ Hall sensors     │ SENSE port   │ GND, 5V, Hall A/B/C        │
  │ Battery positive │ B+ pad       │ Through XT90S + fuse        │
  │ Battery negative │ B- pad       │ Direct                      │
  └──────────────────┴──────────────┴─────────────────────────────┘

  WARNINGS FROM MANUAL:
  • NEVER exceed 60V on B+ (spikes included)
  • NEVER connect 5V and GND between multiple VESCs on CAN bus
  • NEVER use Y-PPM wiring across multiple VESCs
  • ALWAYS use a fuse and anti-spark connector on the battery
  • Run Jetson from battery — not mains — to avoid ground loops
```

---

*Documentation by Claude Code — based on official TRAMPA VESC 6 MkVI Manual rev 1.0*  
*Roboracer / F1TENTH Project — orinnano-desktop — 2026-05-15*
