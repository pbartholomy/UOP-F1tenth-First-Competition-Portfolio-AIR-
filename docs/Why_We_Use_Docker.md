# Why We Use Docker
## Roboracer / F1TENTH Project — orinnano-desktop

---

## What is Docker?

Think of Docker as a **lunchbox that contains everything a program needs to run** — the food, the utensils, the napkin — all packed together. You can hand that lunchbox to any computer and it just works, because everything it needs is already inside.

Technically it is a **container** — a sealed, isolated environment with its own OS packages, libraries, and tools, completely separate from the rest of your machine. Nothing inside the container affects your real operating system, and nothing on your real OS interferes with the container.

---

## Why Docker for THIS Project Specifically?

### 1. The Dependency Hell Problem

Your Roboracer stack needs very specific versions of many things — and they all have to work together perfectly:

- ROS2 Humble (exact version)
- CUDA 12.6
- TensorRT (exact version matched to CUDA)
- ZED SDK 5.0
- cuVSLAM GPU libraries
- Python packages tied to specific library versions

If you install all of these directly on the Jetson and one thing updates, conflicts, or gets installed in the wrong order — **everything breaks**. Tracking down which library broke which other library can take days.

Docker solves this by keeping every dependency **frozen and isolated**. The versions are locked. They never clash with each other or with anything else on your system.

---

### 2. NVIDIA Built the Container For You

NVIDIA ships a pre-built Isaac ROS Docker image that already has:
- CUDA wired up correctly
- TensorRT installed and linked
- ROS2 Humble configured
- All GPU acceleration libraries ready

Without Docker, setting all of that up manually on a Jetson would take days and requires deep knowledge of how each library links to the others. With Docker you just run one command:

```bash
./run_dev.sh
```

And the entire environment is ready instantly.

---

### 3. The Jetson is Unusual Hardware

The NVIDIA Jetson is not a standard PC. It runs:
- **ARM64 (aarch64)** processor architecture — not the x86 chips in most laptops/desktops
- **Jetson-specific GPU drivers** built into the L4T kernel
- **Shared CPU/GPU memory** architecture unique to Jetson

Software compiled for a regular PC often will not run on the Jetson without changes. The Isaac ROS Docker image is built specifically for Jetson hardware, handling all of those quirks automatically so you never have to think about them.

---

### 4. Reproducibility — Everyone Gets the Same Environment

If another team member needs to work on the project, or if you get a new Jetson, setup is simple:

1. Clone the workspace
2. Run `run_dev.sh`
3. Done — identical environment, every time

No "it works on my machine but not yours" problems. No spending a day installing things. The container is the environment, and it is the same for everyone.

---

### 5. Safe to Experiment

If you install something inside Docker and it breaks everything inside the container, you just exit and start a fresh one. **Your real Jetson OS is completely untouched.**

This is exactly why the SSD bootloader bug documented on this Desktop was so painful — that was a problem on the real OS. Problems inside Docker just disappear when you restart the container.

---

### 6. Keeps the Real OS Clean

Everything ROS2, ZED, and Isaac-related lives inside the container. Your actual Ubuntu installation stays minimal and clean. If you ever need to wipe and redo the project environment, you just pull a fresh Docker image — no need to reinstall the whole OS.

---

## Simple Comparison Table

| Situation | Without Docker | With Docker |
|---|---|---|
| Setting up ROS2 + CUDA + ZED | Hours to days of manual work | `./run_dev.sh` — done in minutes |
| A package update breaks something | Entire Jetson setup may be broken | Exit container, start fresh one |
| New team member joins | Repeats the whole setup process | Pulls the same image, instantly ready |
| You experiment and break things | Risk breaking the real OS | Container resets cleanly |
| CUDA / TensorRT version conflicts | Very hard to fix manually | Locked inside the container, no conflicts |
| Jetson ARM64 compatibility issues | Must manually compile libraries | Pre-built for Jetson in the image |

---

## How It Works in This Project

```
Your Real Jetson OS (Ubuntu 22.04)
│
└── Docker Container (Isaac ROS Image)
    ├── ROS2 Humble
    ├── CUDA 12.6 + TensorRT
    ├── ZED SDK 5.0
    ├── cuVSLAM libraries
    ├── All Python dependencies
    └── Your workspace (mounted from ~/workspaces/isaac-ros-dev/)
              ↑
    Your code files live on the real OS and are
    shared into the container — so edits you make
    outside Docker are instantly visible inside it.
```

Your actual project files (`~/workspaces/isaac-ros-dev/`) live on the real Jetson and are **mounted** (shared) into the container when you run `run_dev.sh`. This means:
- You can edit files with any editor on the real OS
- The container sees those changes immediately
- If the container is deleted, your code is safe on the real OS

---

## The One Command That Does It All

```bash
cd ~/workspaces/isaac-ros-dev/src/isaac_ros_common/scripts
./run_dev.sh
```

This single script:
1. Pulls the correct Isaac ROS Docker image for your Jetson
2. Grants the container access to the Jetson GPU
3. Mounts your workspace folder into the container
4. Connects the ZED camera USB to the container
5. Opens a terminal inside the ready-to-use environment

---

## Bottom Line

Docker is used on this project because NVIDIA's Isaac ROS stack has dozens of tightly-coupled GPU dependencies that are nearly impossible to manage manually on Jetson hardware. The container packages all of it into one command, keeps your real OS safe and clean, and guarantees that the environment is identical every single time you run it.

---

*Document created by Claude Code — orinnano-desktop, 2026-05-15*
