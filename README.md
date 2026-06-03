<div align="center">

# Jetson Orin Nano — Autonomous Drone & Vision System

**v0.1.0** — *AI-powered autonomous drone platform with Pixhawk 6C, YOLOv8 computer vision, and real-time MAVLink telemetry*

[![Python](https://img.shields.io/badge/Python-3.8+-3776AB?style=flat&logo=python&logoColor=white)](https://python.org)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Ultralytics-00FFFF?style=flat&logo=ultralytics&logoColor=white)](https://ultralytics.com)
[![Flask](https://img.shields.io/badge/Flask-3.0-000000?style=flat&logo=flask&logoColor=white)](https://flask.palletsprojects.com)
[![PyMAVLink](https://img.shields.io/badge/PyMAVLink-2.4+-FF6F00?style=flat&logo=python&logoColor=white)](https://github.com/ArduPilot/pymavlink)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Jetson](https://img.shields.io/badge/Platform-Jetson%20Orin-76B900?style=flat&logo=nvidia&logoColor=white)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/)

**Created by [Kumar Satyam](mailto:kumarsatyam3135@gmail.com)**

A terminal-driven autonomous drone system running on NVIDIA Jetson Orin with Pixhawk 6C flight controller, real-time YOLOv8 object tracking, and live MAVLink telemetry streaming.

</div>

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [System Design](#system-design)
  - [High-Level Design (HLD)](#high-level-design-hld)
  - [Low-Level Design (LLD)](#low-level-design-lld)
- [Project Structure](#project-structure)
- [Hardware Requirements](#hardware-requirements)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
  - [Camera Vision Streams](#camera-vision-streams)
  - [Drone Telemetry & Control](#drone-telemetry--control)
  - [Telemetry Dashboard](#telemetry-dashboard)
- [Commands Reference](#commands-reference)
- [Vision Module](#vision-module)
- [Drone Module](#drone-module)
- [Configuration](#configuration)
- [Models](#models)
- [Heuristics & Lessons Learned](#heuristics--lessons-learned)
- [Troubleshooting](#troubleshooting)
- [Development](#development)
- [Testing](#testing)
- [License](#license)

---

## Overview

This project turns an **NVIDIA Jetson Orin** into the brains of an autonomous drone. It connects to a **Pixhawk 6C** flight controller over serial MAVLink, reads live telemetry (attitude, GPS, heading), and runs **YOLOv8** object detection on camera feeds — all streamed to a browser via MJPEG/WebSocket.

The system runs entirely from the terminal with `make` commands. No GUI, no IDE, no cloud dependency. Every script has a single entry point, a single port, and a single purpose.

**What you can do with it:**

- Stream CSI or USB camera feeds to any browser on the network
- Track people in real time with YOLOv8 + ByteTrack (instant lock or 60-second timer)
- Read live Pixhawk telemetry (attitude, GPS, battery, heading)
- Run an autonomous corridor-scout mission with waypoint navigation
- View a simulated drone dashboard with real-time motor telemetry

---

## Features

- **Dual Camera Support** — CSI camera (GStreamer) and USB camera (UVC) with automatic fallback
- **YOLOv8 Object Tracking** — ByteTrack-based person tracking with configurable lock behavior
- **Two Lock Strategies** — Instant permanent lock (`usb_stream.py`) or 60-second observation timer (`usb_tracker.py`)
- **Live MAVLink Telemetry** — ATTITUDE, VFR_HUD, and GPS_RAW_INT messages streamed to terminal
- **Autonomous Corridor Mission** — Zigzag lawnmower pattern with OSD-overlaid video feed
- **Simulated Dashboard** — Web-based drone telemetry UI with real-time WebSocket updates (no hardware needed)
- **Makefile-Driven** — Every operation is a single `make <command>` away
- **Production-Grade Repo** — `.gitignore` blocks models/binaries/logs, `pyproject.toml` for pip install, no secrets committed

---

## System Design

### High-Level Design (HLD)

```
┌─────────────────────────────────────────────────────────────────┐
│                        JETSON ORIN (L4T)                        │
│                                                                  │
│  ┌──────────────┐    ┌─────────────┐    ┌───────────────────┐   │
│  │  CSI Camera   │    │  USB Camera  │    │  Pixhawk 6C       │   │
│  │  (IMX219)     │    │  (Arducam)   │    │  (FC / Autopilot) │   │
│  │  /dev/video0  │    │  /dev/video1 │    │  /dev/ttyACM0     │   │
│  └──────┬───────┘    └──────┬──────┘    └──────────┬─────────┘   │
│         │                   │                       │             │
│         ▼                   ▼                       ▼             │
│  ┌──────────────┐    ┌─────────────┐    ┌───────────────────┐   │
│  │ GStreamer     │    │ OpenCV      │    │ pymavlink          │   │
│  │ nvarguscamerasrc│   │ VideoCapture│    │ MAVLink protocol   │   │
│  └──────┬───────┘    └──────┬──────┘    └──────────┬─────────┘   │
│         │                   │                       │             │
│         ▼                   ▼                       ▼             │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │                    Flask HTTP Server                          │ │
│  │  ┌─────────────┐  ┌──────────────┐  ┌────────────────────┐  │ │
│  │  │ MJPEG Stream │  │ YOLOv8+      │  │ SocketIO WebSocket │  │ │
│  │  │ (vision/)    │  │ ByteTrack    │  │ (dashboard/)       │  │ │
│  │  │ :5000        │  │ (vision/)    │  │ :5000              │  │ │
│  │  └─────────────┘  └──────────────┘  └────────────────────┘  │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                              │                                     │
│                              ▼                                     │
│                    ┌──────────────────┐                            │
│                    │  Browser Client   │                            │
│                    │  http://<ip>:port │                            │
│                    └──────────────────┘                            │
└─────────────────────────────────────────────────────────────────┘
```

### Low-Level Design (LLD)

```
┌─────────────────────────────────────────────────────────────────────┐
│                         csi_stream.py                                │
│                                                                     │
│  ┌────────────┐    ┌────────────┐    ┌────────────────────────┐    │
│  │ Capture     │───▶│ JPEG Encode│───▶│ Frame Buffer (Lock)    │    │
│  │ Thread      │    │ (cv2)      │    │ (threading.Lock)       │    │
│  └────────────┘    └────────────┘    └───────────┬────────────┘    │
│                                                   │                  │
│  ┌────────────────────────────────────────────────┴──────────┐     │
│  │ Flask Thread: generate() → yield multipart/x-mixed-replace│     │
│  └───────────────────────────────────────────────────────────┘     │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                      usb_stream.py / usb_tracker.py                  │
│                                                                     │
│  ┌────────────┐    ┌────────────┐    ┌────────────────────────┐    │
│  │ Camera Read │───▶│ YOLOv8     │───▶│ Tracking Logic         │    │
│  │ (cv2)       │    │ model.track│    │ (remembered_human_ids) │    │
│  └────────────┘    └────────────┘    └───────────┬────────────┘    │
│                                                   │                  │
│                                                   ▼                  │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │ Draw bounding boxes + labels → JPEG encode → yield MJPEG     │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│                       corridor_scout.py                              │
│                                                                     │
│  ┌──────────────────────┐    ┌──────────────────────────────────┐  │
│  │ mavlink_loop() thread │───▶│ GPS position @ 10 Hz            │  │
│  │ (pymavlink, /dev/tty*)│    │ current_lat/lon/alt             │  │
│  └──────────────────────┘    └────────────┬─────────────────────┘  │
│                                           │                         │
│  ┌──────────────────────┐    ┌────────────┴─────────────────────┐  │
│  │ Video Server Thread   │    │ Main Loop:                      │  │
│  │ HTTP MJPEG @ :8000   │    │ • Read camera → OSD → serve     │  │
│  │ (socketserver)        │    │ • Check alt > 3m → start mission│  │
│  └──────────────────────┘    │ • fly_to() waypoint every 0.5s  │  │
│                               │ • Advance on 3m proximity       │  │
│                               └────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Project Structure

```
jetson-orin-nano/
│
├── README.md                 Project documentation & heuristics
├── pyproject.toml            pip-installable project metadata
├── Makefile                  Single-entry commands (make stream-csi, etc.)
├── requirements.txt          Python dependency manifest
├── .gitignore                Blocks models/, logs/, binaries, IDE files
│
├── vision/                   Camera & YOLO streaming tools
│   ├── csi_stream.py         CSI camera → MJPEG stream (no AI, GStreamer)
│   ├── usb_stream.py         USB camera + YOLO → instant permanent lock
│   ├── usb_tracker.py        USB camera + YOLO → 60-second observation lock
│   ├── camera_test.py        Serial camera raw-data connectivity test
│   └── samples/              Sample captured frames
│
├── drone/                    Drone autonomy system
│   ├── telemetry.py          Live MAVLink telemetry readout
│   ├── corridor_scout.py     Autonomous zigzag corridor mission + video OSD
│   └── dashboard/            Simulated real-time telemetry dashboard
│       ├── app.py            Flask-SocketIO backend
│       ├── launch.sh         Kiosk boot script
│       └── templates/        Cyberpunk-style dashboard.html
│
├── models/                   YOLO weights (gitignored — run download_models.sh)
├── scripts/                  Utility scripts
│   └── download_models.sh    Fetches YOLO .pt files from Ultralytics
├── config/                   Reference configuration
│   └── mavros.yaml           MAVROS config for Pixhawk 6C
├── logs/                     Flight telemetry logs (gitignored)
│   ├── mav.tlog / .raw       MAVLink telemetry log
│   └── nidarhex/             NidarHex flight data
└── data/                     Performance monitoring
    └── performance/          jetson_stats GPU/CPU/temp/power CSVs
```

---

## Hardware Requirements

| Component | Specification |
|-----------|--------------|
| **SBC** | NVIDIA Jetson Orin (any variant) — JetPack 6 / L4T |
| **Flight Controller** | Pixhawk 6C — USB serial at `/dev/ttyACM0`, 115200 baud |
| **CSI Camera** | IMX219 or compatible — GStreamer `nvarguscamerasrc` |
| **USB Camera** | Arducam or any UVC-compatible camera — `/dev/video0` |
| **Battery (drone)** | 4S LiPo 14.8–16.5V |
| **Battery (Jetson)** | Barrel jack or USB-C PD power supply |

> **Note**: The Pixhawk and serial camera both use `/dev/ttyACM0`. Only one can be connected at a time.

---

## Installation

```bash
# Clone the repository
git clone https://github.com/krsatyam36/jetson-orin-nano.git
cd jetson-orin-nano

# Install system dependencies (Jetson L4T)
sudo apt update
sudo apt install -y python3-pip espeak alsa-utils     # espeak for voice alerts

# Install Python dependencies
pip install -r requirements.txt

# Or install the package in editable mode
pip install -e .
```

---

## Quick Start

```bash
# Download YOLO models (≈30 MB each, required for tracking)
make download-models

# Option A — Stream CSI camera to browser (no AI needed)
make stream-csi
# → Open http://<jetson-ip>:5000

# Option B — Read live Pixhawk telemetry
make telemetry
# → Terminal shows ATTITUDE, VFR_HUD, GPS_RAW_INT

# Option C — Simulated drone dashboard (no hardware needed)
make dashboard
# → Open http://<jetson-ip>:5000

# Option D — USB camera + YOLO instant person lock
make stream-usb
# → Open http://<jetson-ip>:5000
```

---

## Usage

### Camera Vision Streams

```bash
# CSI camera — pure video, no AI (uses GStreamer nvarguscamerasrc)
make stream-csi              # port :5000

# USB camera — YOLO instant lock (person locked on first sight)
make stream-usb              # port :5000

# USB camera — YOLO with 60-second observation timer
make stream-track            # port :5000

# Serial camera — raw data test
make camera-test             # terminal only
```

### Drone Telemetry & Control

```bash
# Live Pixhawk readout (verify link before any mission)
make telemetry               # terminal only

# Autonomous corridor scout mission (requires Pixhawk + GPS + GUIDED mode)
make scout                   # video on :8000
```

### Telemetry Dashboard

```bash
# Simulated dashboard — no Pixhawk needed
make dashboard               # port :5000, auto-opens browser
```

---

## Commands Reference

| Command | Action | Port | Hardware Needed |
|---------|--------|------|-----------------|
| `make install` | Install Python deps | — | — |
| `make download-models` | Download YOLO .pt files | — | Internet |
| `make stream-csi` | CSI camera → MJPEG stream | `:5000` | CSI camera |
| `make stream-usb` | USB camera + YOLO instant lock | `:5000` | USB camera |
| `make stream-track` | USB camera + YOLO 60s timer | `:5000` | USB camera |
| `make camera-test` | Serial camera raw dump | — | Camera on `/dev/ttyACM0` |
| `make telemetry` | Live Pixhawk MAVLink readout | — | Pixhawk 6C |
| `make scout` | Autonomous corridor mission | `:8000` | Pixhawk + GPS + USB cam |
| `make dashboard` | Simulated telemetry dashboard | `:5000` | None |
| `make clean` | Remove `__pycache__` | — | — |

---

## Vision Module

### csi_stream.py

Captures video from a **Jetson CSI camera** (IMX219) using a GStreamer pipeline in a background thread and serves it as an MJPEG stream via Flask. No AI inference — pure camera feed.

| Property | Detail |
|----------|--------|
| **Input** | `nvarguscamerasrc sensor-id=0` — CSI port |
| **Pipeline** | `nvarguscamerasrc → nvvidconv → videoconvert → appsink` |
| **Capture Resolution** | 1280×720 |
| **Stream Resolution** | 640×360 (configurable) |
| **Architecture** | 1 capture thread + 1 Flask thread, shared frame buffer with `threading.Lock` |
| **Run** | `make stream-csi` |

### usb_stream.py

Reads a **USB camera**, runs **YOLOv8 + ByteTrack** on every frame, and applies an **instant permanent lock** — the first time a person (track ID) appears, they are immediately added to a permanent set and marked red. Subsequent appearances show a green box.

| Property | Detail |
|----------|--------|
| **Model** | `yolov8n.pt` (download with `make download-models`) |
| **Tracker** | `model.track(persist=True)` — ByteTrack cross-frame IDs |
| **Confidence** | `>0.40` for "person" class |
| **Lock Strategy** | **Instant** — lock on first sight, permanent for session |
| **Colors** | Red = "NEW TARGET LOCKED", Green = "ALREADY LOCKED" |
| **Run** | `make stream-usb` |

### usb_tracker.py

Same as `usb_stream.py` but with a **60-second observation timer** before locking. This is the more sophisticated tracking variant.

| Property | Detail |
|----------|--------|
| **Lock Strategy** | **60-second timer** — person must be visible continuously for 60s |
| **Grace Period** | If person leaves frame for >5s during countdown → timer resets |
| **Three States** | Blue "Human" (counting) → Red "TARGET LOCKED" (locked) → Red "TARGET LOCKED" (revisit) |
| **Data Structures** | `first_seen_times{}` (per-ID timer), `remembered_human_ids` (locked set), `last_seen_times{}` (grace tracker) |
| **Run** | `make stream-track` |

### camera_test.py

Opens `/dev/ttyACM0` at 115200 baud and dumps raw serial data to the terminal. Used to verify a serial camera module is transmitting.

> **Caution**: If a Pixhawk is connected to `/dev/ttyACM0`, this will read MAVLink binary data — not camera data.

---

## Drone Module

### telemetry.py

Connects to **Pixhawk 6C** over MAVLink and streams three message types to the terminal:

| Message | Data |
|---------|------|
| `ATTITUDE` | Roll, Pitch, Yaw (radians) |
| `VFR_HUD` | Altitude (m), Heading (deg), Throttle (%) |
| `GPS_RAW_INT` | Satellite count, Fix type |

| Property | Detail |
|----------|--------|
| **Port** | `/dev/ttyACM0`, 115200 baud |
| **Heartbeat Timeout** | 15 seconds |
| **Poll Rate** | 10 Hz (100ms sleep) |
| **Run First** | Always run this before any autonomous mission to verify the MAVLink link |
| **Run** | `make telemetry` |

### corridor_scout.py

Full **autonomous corridor reconnaissance mission**. Generates a zigzag (lawnmower) path, navigates waypoints via MAVLink `SET_POSITION_TARGET_GLOBAL_INT`, and serves a live OSD-overlaid camera feed on `:8000`.

| Property | Detail |
|----------|--------|
| **Prerequisites** | Pixhawk connected, GPS 3D fix, GUIDED mode, armed, airborne >3m |
| **Flight Altitude** | 50 ft ≈ 15.24 m (`FLIGHT_ALTITUDE`) |
| **Corridor** | 100m length × 40m width, 10m lane spacing (configurable) |
| **Navigation** | `fly_to()` every 0.5s — waypoint reached within 3m radius |
| **Video** | HTTP MJPEG server on `:8000` with OSD overlay |
| **Camera** | Auto-detects USB camera (`/dev/video0`–`/dev/video3`) |
| **Audio** | `espeak` voice announcements |
| **Run** | `make scout` |

**Mission Sequence:**

```
1. mavlink_loop() thread → auto-detect Pixhawk on /dev/ttyACM*
2. Video server → HTTP MJPEG on :8000
3. Wait for GPS lock (current_lat != 0)
4. Generate zigzag waypoints relative to takeoff point
5. Initialize USB camera
6. Main loop:
   a. Read camera frame → draw OSD → serve via MJPEG
   b. Check altitude > 3m → mission_started = True
   c. fly_to() current waypoint (re-sent every 0.5s)
   d. Within 3m of target → advance to next waypoint
   e. All waypoints complete → hover + "Scan Complete"
```

---

## Configuration

### config/mavros.yaml

MAVROS configuration reference for connecting ROS 2 to Pixhawk 6C. Contains serial port, baud rate, frame IDs, and telemetry streaming rates.

> **Note**: ROS 2 + MAVROS integration is not yet wired into the Python scripts. This file serves as setup documentation for future ROS 2 expansion.

---

## Models

All models go in `models/`. They are **gitignored** — you must download them after cloning.

| File | Size | Used By |
|------|------|---------|
| `yolov8n.pt` | ≈6 MB | `usb_stream.py`, `usb_tracker.py`, `corridor_scout.py` |
| `yolov8s.pt` | ≈22 MB | Optional — higher accuracy, slower inference |
| `yolov8m.pt` | ≈52 MB | Optional — medium variant |
| `yolo11n.pt` | ≈6 MB | Optional — newer YOLO11 architecture |

```bash
# Download all models
bash scripts/download_models.sh

# Export to ONNX or TensorRT engine (optional performance optimization)
yolo export model=models/yolov8n.pt format=onnx
yolo export model=models/yolov8n.pt format=engine device=0
```

---

## Heuristics & Lessons Learned

### General

1. **Port conflicts** — `OSError: [Errno 98] Address already in use` means something is already on that port. Kill it: `fuser -k 5000/tcp`
2. **Camera is exclusive** — Only one process can open a camera at a time. Kill competing processes first.
3. **Serial is exclusive** — `/dev/ttyACM0` can only be used by one process. Don't run `telemetry.py` and `corridor_scout.py` simultaneously.
4. **Test the link first** — Always run `make telemetry` before any autonomous mission.
5. **Models must exist** — YOLO scripts crash with `ModelNotFoundError` if `models/yolov8n.pt` is missing. Run `make download-models`.
6. **Offline Jetson** — If the Jetson has no internet, download `yolov8n.pt` on another machine and SCP it into `models/`.

### Pixhawk / MAVLink

7. **Port enumeration** — Pixhawk may appear as `/dev/ttyACM0` or `/dev/ttyACM1`. Check with `ls /dev/ttyACM*`.
8. **Baud rate** — Pixhawk 6C typically uses 115200 (telemetry) or 57600. `telemetry.py` uses 115200; `corridor_scout.py` auto-detects at 57600.
9. **Heartbeat failure** — If `wait_heartbeat()` times out: check the USB cable, reboot the Pixhawk, verify the port.
10. **GUIDED mode required** — `corridor_scout.py` needs the FC in GUIDED mode. Set via RC transmitter or QGroundControl.
11. **Arm manually** — The script does not arm the drone. Arm via RC or GCS before takeoff.
12. **Takeoff detection** — The mission starts automatically when altitude >3m. You must manually take off to at least 3m.

### Camera / Vision

13. **CSI is Jetson-only** — `nvarguscamerasrc` only works on Jetson L4T, not regular Linux or Ubuntu Desktop.
14. **Sensor ID** — If the CSI camera doesn't open, try `sensor_id=1` in `gstreamer_pipeline()`.
15. **USB camera index** — Default is `cv2.VideoCapture(0)`. Try indices 1, 2, or 3 for multiple cameras.
16. **Confidence threshold** — Person detection threshold is 0.40. Adjust by editing the `conf > 0.40` line.
17. **Class name case** — Some YOLO models use "Person" (capital P). Verify `model.names[class_id]` matches.
18. **Stream latency** — MJPEG over WiFi has 1–3s lag. Use Ethernet for low-latency video.

### Dashboard

19. **Eventlet is required** — `app.py` uses `eventlet.monkey_patch()`. Don't run with the default Werkzeug dev server.
20. **Headless mode** — `webbrowser.open()` auto-launches a browser. Comment this line if running headless (no display).

### Repository

21. **Models are gitignored** — `models/*.pt` is in `.gitignore`. Never commit large binary files.
22. **Logs are gitignored** — `logs/` can contain GBs of MAVLink data. Keep it out of git.
23. **Check before push** — `git diff --cached` to verify no secrets, tokens, or IPs are staged.
24. **Commit style** — Use conventional commits: `feat:`, `fix:`, `docs:`, `chore:`.

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `ImportError: No module named 'cv2'` | OpenCV not installed | `pip install opencv-python` |
| `ModelNotFoundError: yolov8n.pt` | Models not downloaded | `bash scripts/download_models.sh` |
| `[Errno 98] Address already in use` | Port occupied | `fuser -k 5000/tcp` or `fuser -k 8000/tcp` |
| `wait_heartbeat() timeout` | Pixhawk not connected | Check USB, `ls /dev/ttyACM*`, verify baud rate |
| Camera opens but no frames | Wrong sensor or camera index | Try `sensor_id=1` (CSI) or `VideoCapture(1..3)` (USB) |
| `RuntimeError: eventlet required` | Wrong WSGI server | Ensure `eventlet.monkey_patch()` runs before Flask import |
| `espeak: command not found` | espeak not installed | `sudo apt install espeak alsa-utils` |
| CSV logs empty | jetson_stats not running | Install `jetson_stats` and launch `jtop` once |
| `[Errno 13] Permission denied: /dev/ttyACM0` | User not in dialout group | `sudo usermod -a -G dialout $USER && logout` |

---

## Development

### Adding a New Vision Script

1. Place your `.py` file in `vision/`
2. Add a `Makefile` target in the `## ─── Vision / Camera ────────────────────────────────────────` section
3. Update this README
4. Document it in the [Vision Module](#vision-module) section of this README
5. Add an entry to the [Commands Reference](#commands-reference) table
6. List required models in the [Models](#models) section

### Adding a New Drone Script

1. Place your `.py` file in `drone/`
2. Add a `Makefile` target
3. Document it in the [Drone Module](#drone-module) section
4. If it uses a serial/config file, add it to `config/`

### Code Style

- No inline comments (header docstring only)
- `snake_case` for variables and functions
- `CAPITAL_SNAKE_CASE` for configuration constants
- Type hints preferred but not required
- All scripts should be runnable with `python <path>` from the repo root

---

## Testing

No automated test suite is currently included. To manually verify:

```bash
# Verify camera (choose one):
python vision/csi_stream.py        # CSI camera
python vision/usb_stream.py        # USB camera + YOLO
python vision/camera_test.py       # Serial camera

# Verify Pixhawk link:
python drone/telemetry.py

# Verify dashboard:
python drone/dashboard/app.py
```

---

## License

MIT — see [LICENSE](LICENSE) for details.
