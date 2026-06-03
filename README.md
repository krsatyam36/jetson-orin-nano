<div align="center">

# Jetson Orin Nano — Autonomous Drone & Vision System

**v0.2.0** — *AI-powered autonomous drone platform with Pixhawk 6C, YOLOv8 computer vision, and real-time MAVLink telemetry*

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
- Track people in real time with YOLOv8 + ByteTrack (PyTorch or TensorRT backend)
- Use two lock strategies — instant permanent lock or 60-second observation timer
- Read live Pixhawk telemetry (attitude, GPS, battery, heading) via a shared DroneController
- Run an autonomous corridor-scout mission with automated takeoff, waypoint navigation, and RTL
- Follow a person autonomously with vision-based steering
- Access a health endpoint (`/health`) exposing full drone state as JSON
- View a simulated drone dashboard with real-time motor telemetry

---

## Features

- **Dual Camera Support** — CSI camera (GStreamer) and USB camera (UVC) with automatic fallback
- **Shared Detection Engine** — `vision/detector.py` wraps YOLO with auto backend select (PyTorch `.pt`, TensorRT `.engine`, ONNX `.onnx`)
- **Two Lock Strategies** — Instant permanent lock (`usb_stream.py`) or 60-second observation timer (`usb_tracker.py`)
- **Live MAVLink Telemetry** — ATTITUDE, VFR_HUD, and GPS_RAW_INT messages streamed to terminal
- **Drone Flight Controller** — `drone/controller.py` handles arm, takeoff, land, RTL, geofence, watchdog, pre-arm checklist
- **Autonomous Corridor Mission** — Zigzag lawnmower pattern with automated takeoff, OSD video, and RTL on completion
- **Person-Follow Mode** — Vision-based target tracking with closed-loop steering via DroneController
- **Health Endpoint** — `/health` JSON endpoint exposing GPS, battery, attitude, pre-arm status
- **Simulated Dashboard** — Web-based drone telemetry UI with real-time WebSocket updates (no hardware needed)
- **MAVLink Log Parser** — Parse `.tlog` files and export to CSV
- **Makefile-Driven** — Every operation is a single `make <command>` away
- **Production-Grade Repo** — `.gitignore`, `pyproject.toml`, `pre-commit` hooks, CI pipeline, Dockerfile, pytest suite

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
│   ├── detector.py           Shared detection engine (PyTorch / TensorRT / ONNX backend)
│   ├── csi_stream.py         CSI camera → MJPEG stream (no AI, GStreamer)
│   ├── usb_stream.py         USB camera + YOLO → instant permanent lock
│   ├── usb_tracker.py        USB camera + YOLO → 60-second observation lock
│   ├── camera_test.py        Serial camera raw-data connectivity test
│   └── samples/              Sample captured frames
│
├── drone/                    Drone autonomy system
│   ├── controller.py         Flight controller abstraction (arm, takeoff, land, RTL, geofence, watchdog, pre-arm)
│   ├── telemetry.py          Live MAVLink telemetry readout (wraps controller.py)
│   ├── corridor_scout.py     Autonomous zigzag corridor mission + automated takeoff + RTL
│   ├── follow.py             Person-follow mode (vision-based target tracking + steering)
│   ├── health.py             HTTP health endpoint (/health → full drone state as JSON)
│   └── dashboard/            Simulated real-time telemetry dashboard
│       ├── app.py            Flask-SocketIO backend
│       ├── launch.sh         Kiosk boot script
│       └── templates/        Cyberpunk-style dashboard.html
│
├── models/                   YOLO weights (gitignored — run download_models.sh)
├── scripts/                  Utility scripts
│   ├── download_models.sh    Fetches YOLO .pt files from Ultralytics
│   └── tlog_parser.py        Parse MAVLink .tlog files and export to CSV
├── config/                   Configuration files
│   ├── mavros.yaml           MAVROS config for Pixhawk 6C
│   └── corridor.yaml         Corridor mission parameters
├── tests/                    Pytest suite
│   ├── test_detector.py
│   ├── test_controller.py
│   └── test_health.py
├── logs/                     Flight telemetry logs (gitignored)
├── data/                     Performance monitoring
├── .github/workflows/ci.yml  GitHub Actions CI pipeline
├── .pre-commit-config.yaml   Pre-commit hooks (black, flake8, mypy, secrets scan)
├── Dockerfile                Container image for health endpoint
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

# Option E — Full autonomous corridor mission (Pixhawk required)
make scout
# → Video on :8000, health on :9090

# Option F — Person follow mode
make follow
# → Video on :5001
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
make telemetry               # terminal only, health on :9090

# Full autonomous corridor scout mission (arm + GUIDED + >3m alt required)
make scout                   # video on :8000, health on :9090

# Person-follow mode — tracks and steers toward a person
make follow                  # video on :5001
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
| `make install-dev` | Install + pre-commit hooks | — | — |
| `make download-models` | Download YOLO .pt files | — | Internet |
| `make stream-csi` | CSI camera → MJPEG stream | `:5000` | CSI camera |
| `make stream-usb` | USB camera + YOLO instant lock | `:5000` | USB camera |
| `make stream-track` | USB camera + YOLO 60s timer | `:5000` | USB camera |
| `make camera-test` | Serial camera raw dump | — | Camera on `/dev/ttyACM0` |
| `make telemetry` | Live Pixhawk MAVLink readout | `:9090` health | Pixhawk 6C |
| `make scout` | Autonomous corridor mission | `:8000` video, `:9090` health | Pixhawk + GPS + USB cam |
| `make scout-custom` | Scout with custom params | `:8000` + `:9090` | Pixhawk + GPS + USB cam |
| `make follow` | Person-follow mode | `:5001` video | Pixhawk + USB cam |
| `make dashboard` | Simulated telemetry dashboard | `:5000` | None |
| `make health` | Standalone health endpoint | `:9090` | Pixhawk (optional) |
| `make tlog-parse FILE=logs/mav.tlog` | Parse MAVLink log | — | — |
| `make tlog-csv FILE=logs/mav.tlog OUT=out.csv` | Export MAVLink log to CSV | — | — |
| `make test` | Run pytest suite | — | — |
| `make lint` | Run flake8 | — | — |
| `make format` | Auto-format with black | — | — |
| `make pre-commit` | Run pre-commit hooks | — | — |
| `make docker-build` | Build Docker image | — | — |
| `make docker-run` | Run health endpoint in Docker | `:9090` | — |
| `make clean` | Remove `__pycache__` | — | — |

---

## Vision Module

### `detector.py` — Shared Detection Engine

The shared inference wrapper used by all vision scripts. Automatically selects the backend based on model file extension:

| Extension | Backend | Speed |
|-----------|---------|-------|
| `.pt` | PyTorch (via Ultralytics) | Baseline |
| `.engine` | TensorRT (via Ultralytics + TensorRT) | Fastest on Jetson |
| `.onnx` | ONNX Runtime (via Ultralytics) | Faster than PT |

```python
from vision.detector import Detector, draw_detections

det = Detector("models/yolov8n.pt", conf_threshold=0.4)

# Single-frame detection
results = det.detect(frame)

# Cross-frame tracking (ByteTrack)
results = det.track(frame, persist=True)

# Draw bounding boxes
draw_detections(frame, results)
```

Backend selection is automatic — just point to the right file extension.

### `csi_stream.py`

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

### `controller.py` — Flight Controller Abstraction

Wraps pymavlink with a clean state-machine API. Used by `telemetry.py`, `corridor_scout.py`, and `follow.py`.

| Feature | Detail |
|---------|--------|
| **Connection** | Auto-detects Pixhawk on `/dev/ttyACM*` or connects to specific port |
| **Pre-arm Checklist** | GPS 3D fix, ≥8 satellites, battery ≥14V, heartbeat OK, EKF healthy, mode GUIDED |
| **Arm / Disarm** | `ctrl.arm()` / `ctrl.disarm()` |
| **Takeoff** | `ctrl.takeoff(alt_m)` — blocks until altitude reached (30s timeout) |
| **Land** | `ctrl.land()` — blocks until on ground (60s timeout) |
| **RTL** | `ctrl.rtl()` — return to launch |
| **Fly To** | `ctrl.fly_to(lat, lon, alt)` — `SET_POSITION_TARGET_GLOBAL_INT` |
| **Set Mode** | `ctrl.set_mode("GUIDED")` |
| **Geofence** | Background thread enforces max altitude + max radius — triggers RTL on breach |
| **Watchdog** | Background thread — if heartbeat lost for N seconds, triggers configurable fail action |
| **State** | `ctrl.state()` returns thread-safe `DroneState` snapshot (GPS, attitude, battery, EKF, mode, armed) |

```python
from drone.controller import DroneController
ctrl = DroneController()
ctrl.connect()
ctrl.pre_arm_check()   # returns [] if all good
ctrl.arm()
ctrl.takeoff(15.24)
ctrl.fly_to(lat, lon, 15.24)
ctrl.rtl()
```

### `telemetry.py`

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

Autonomous corridor mission using `DroneController` and `Detector`. Runs pre-arm check, arms, takes off, navigates zigzag path, and RTL on completion.

| Property | Detail |
|----------|--------|
| **Prerequisites** | Pixhawk, GPS 3D fix, GUIDED mode, armed |
| **New in v0.2** | Auto takeoff, pre-arm, geofence, watchdog, health endpoint, CLI args |
| **Flight Altitude** | 50 ft / 15.24 m (`--alt N`) |
| **Corridor** | 100m × 40m, 10m lanes (`--length`, `--width`, `--lane`) |
| **Navigation** | `fly_to()` every 0.5s, 3m acceptance radius |
| **Video** | HTTP MJPEG on `:8000` with YOLO overlay + OSD |
| **Health** | `/health` on `:9090` |
| **Audio** | `espeak` voice |
| **Run** | `make scout` / `make scout-custom` |

**Mission Sequence:** Detector init → Connect + geofence + watchdog + health → GPS → Waypoints → Pre-arm → Arm → Takeoff → Navigate → RTL

### `follow.py` — Person-Follow Mode

Tracks a person using the vision Detector and steers the drone via DroneController.

| Property | Detail |
|----------|--------|
| **State Machine** | `SEEKING` → `FOLLOWING` → `LOST` → `SEEKING` |
| **Selection** | Auto-selects highest-confidence person in frame |
| **Steering** | Proportional control from target offset to frame center |
| **Timeout** | `FOLLOWING` → `LOST` after 5s without target |
| **Re-acquisition** | Any person re-entering view resumes following |
| **Video** | MJPEG on `:5001` with follow-mode OSD |
| **Run** | `make follow` |
| **Hardware** | Pixhawk + USB camera |

### `health.py` — Health Endpoint

Full drone state as JSON over HTTP.

| Property | Detail |
|----------|--------|
| **GET /health/ping** | Liveness probe → `pong` |
| **GET /health** | GPS, attitude, speed, battery, pre-arm, EKF, mode |
| **Port** | `:9090` — standalone or embedded |
| **Architecture** | Flask Blueprint — mountable on any Flask app |
| **Run** | `make health` |

---

## Configuration

### config/mavros.yaml

MAVROS configuration reference for connecting ROS 2 to Pixhawk 6C. Contains serial port, baud rate, frame IDs, and telemetry streaming rates.

> ROS 2 + MAVROS integration is not yet wired into the Python scripts. This file serves as setup documentation.

### config/corridor.yaml

Corridor scout mission parameters — flight altitude, corridor dimensions, detection confidence, geofence limits, watchdog settings. All values can be overridden via CLI flags (`--alt`, `--length`, etc.).

---

## Models

All models go in `models/`. They are **gitignored** — you must download them after cloning.

| File | Size | Used By |
|------|------|---------|
| `yolov8n.pt` | ≈6 MB | `detector.py` → `usb_stream.py`, `usb_tracker.py`, `corridor_scout.py`, `follow.py` |
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
10. **GUIDED mode** — All scripts set GUIDED automatically via `DroneController.set_mode()`.
11. **Auto-arm** — `corridor_scout.py` runs a pre-arm checklist then arms and takes off automatically.
12. **Geofence** — A background thread enforces max altitude and radius. Breach triggers RTL.
13. **Watchdog** — A background thread detects heartbeat loss and triggers a configurable fail action (RTL/LAND).

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

21. **Models are gitignored** — `models/*.pt`, `*.engine`, `*.onnx` are in `.gitignore`. Never commit large binary files.
22. **Logs are gitignored** — `logs/` can contain GBs of MAVLink data. Keep it out of git.
23. **Check before push** — `git diff --cached` to verify no secrets, tokens, or IPs are staged.
24. **Commit style** — Use conventional commits: `feat:`, `fix:`, `docs:`, `chore:`.
25. **Docker** — `make docker-build && make docker-run` for a containerized environment.
26. **CI** — `.github/workflows/ci.yml` runs lint + tests on every push.
27. **Pre-commit** — `make precommit` runs ruff, trailing-whitespace, end-of-file-fixer checks.

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

### Linting & Pre-commit

```bash
make precommit          # Run all pre-commit hooks (ruff, trailing-whitespace, etc.)
make lint               # Run ruff check
```

### Testing

```bash
make test               # Run all tests
make test-detector      # Run vision detector tests only
make test-controller    # Run drone controller tests only
```

Tests use pytest with mocked MAVLink / camera. Add new test files in `tests/` following the `test_*.py` pattern.

### Docker

```bash
make docker-build       # Build Docker image (uses Dockerfile)
make docker-run         # Run container with --privileged for USB/serial access
make docker-shell       # Open interactive shell inside container
```

### TLog Analysis

```bash
make parse-tlog         # Parse logs/*.tlog to CSV
make parse-tlog FILE=path/to/log.tlog
```

---

## Testing

Python `pytest` suite in `tests/` with 15+ test cases covering the core modules.

```bash
make test               # Run all tests
make test-detector      # tests/test_detector.py
make test-controller    # tests/test_controller.py
make test-health        # tests/test_health.py
```

### Test Coverage

| Module | File | Tests |
|--------|------|-------|
| Vision | `tests/test_detector.py` | Model load, image inference, frame overlay, confidence filter, empty frame |
| Drone | `tests/test_controller.py` | Connect, arm, disarm, takeoff, land, RTL, fly_to, set_mode, pre-arm, geofence, watchdog, state |
| Health | `tests/test_health.py` | Ping endpoint, full health state |

### Manual Verification

```bash
# Camera:
python vision/csi_stream.py          # CSI camera
python vision/usb_stream.py          # USB + YOLO
python vision/usb_tracker.py         # USB + YOLO + tracking

# Pixhawk link:
python drone/telemetry.py

# Autonomous mission:
python drone/corridor_scout.py --no-vision --alt 10 --length 50 --width 20
```

---

## Project Tree

```
.
├── config/
│   ├── corridor.yaml          # Corridor scout mission config
│   └── mavros.yaml            # MAVROS ROS 2 config
├── data/performance/          # Jetson stats CSV logs
├── drone/
│   ├── controller.py          # Flight controller abstraction
│   ├── corridor_scout.py      # Autonomous corridor mission
│   ├── dashboard/
│   │   ├── app.py             # Flask dashboard
│   │   ├── launch.sh          # Dashboard launcher
│   │   └── templates/
│   │       └── dashboard.html # Dashboard template
│   ├── follow.py              # Person-follow mode
│   ├── health.py              # Health endpoint
│   └── telemetry.py           # MAVLink telemetry dump
├── models/                    # YOLO models (gitignored)
├── scripts/
│   ├── download_models.sh     # Model downloader
│   └── tlog_parser.py         # MAVLink log parser
├── tests/
│   ├── conftest.py            # Pytest fixtures / mocks
│   ├── test_controller.py     # Controller unit tests
│   ├── test_detector.py       # Detector unit tests
│   └── test_health.py         # Health endpoint tests
├── vision/
│   ├── camera_test.py         # Serial camera test
│   ├── csi_stream.py          # CSI camera stream
│   ├── detector.py            # Shared YOLO inference engine
│   ├── samples/               # Sample images
│   ├── usb_stream.py          # USB + YOLO stream
│   └── usb_tracker.py         # USB + YOLO + tracking
├── .github/workflows/
│   └── ci.yml                 # CI pipeline
├── .pre-commit-config.yaml    # Pre-commit hooks
├── Dockerfile                 # Container build
├── Makefile                   # Command reference
├── pyproject.toml             # Python project config
├── README.md                  # This file
├── requirements.txt           # Python dependencies
└── LICENSE                    # MIT license
```

---

## License

MIT — see [LICENSE](LICENSE) for details.
