# Jetson Orin Nano — Autonomous Drone & Vision System

AI-powered autonomous drone platform running on **NVIDIA Jetson Orin** with
**Pixhawk 6C** flight controller, **YOLOv8** computer vision, and real-time
MAVLink telemetry.

## Hardware Requirements

| Component | Details |
|-----------|---------|
| SBC | NVIDIA Jetson Orin (JetPack 6 / L4T) |
| Flight Controller | Pixhawk 6C — connected via USB (`/dev/ttyACM0`, 115200 baud) |
| CSI Camera | IMX219 — GStreamer pipeline via `nvarguscamerasrc` |
| USB Camera | Arducam (or any UVC camera) — `/dev/video0` |
| Serial Camera | Optional — connected to `/dev/ttyACM0` (shared with Pixhawk, use one at a time) |
| Battery | 4S LiPo (14.8–16.5V) for drone; barrel jack / PD for Jetson |

## Quick Start

```bash
# 1. Clone and enter
git clone https://github.com/krsatyam36/jetson-orin-nano.git
cd jetson-orin-nano

# 2. Install Python dependencies
pip install -r requirements.txt

# 3. Download YOLO models (≈30 MB each)
bash scripts/download_models.sh
# Or: make download-models

# 4. Run something — pick one:
make stream-csi        # CSI camera → browser stream (no AI)
make stream-usb        # USB camera + YOLO instant-lock
make telemetry         # Live Pixhawk readout
make dashboard         # Simulated drone telemetry UI
```

> **Ports used**: `:5000` (Flask streams) · `:8000` (corridor_scout video)
> Open `http://<jetson-ip>:<port>` in any browser.

---

## Project Structure

```
jetson-orin-nano/
├── README.md                 ← You are here — exhaustive docs & heuristics
├── pyproject.toml            ← pip-installable project metadata
├── Makefile                  ← Common commands (make stream-csi, make telemetry, etc.)
├── requirements.txt          ← Python dependencies (pinned)
├── .gitignore                ← Ignores models/, logs/, binaries, IDE files
│
├── vision/                   ─── Camera & YOLO streaming tools ───
│   ├── csi_stream.py         CSI camera → MJPEG stream (no AI, GStreamer pipeline)
│   ├── usb_stream.py         USB camera + YOLO → instant permanent lock on sight
│   ├── usb_tracker.py        USB camera + YOLO → 60-second observation before locking
│   ├── camera_test.py        Serial camera raw-data dump (test if module is alive)
│   └── samples/              Sample captured frames (live.jpg)
│
├── drone/                    ─── Drone autonomy system ───
│   ├── telemetry.py          Live MAVLink telemetry readout (attitude, GPS, HUD)
│   ├── corridor_scout.py     Autonomous zigzag corridor mission with video OSD
│   └── dashboard/            Simulated telemetry web dashboard
│       ├── app.py            Flask-SocketIO backend (mock motor data)
│       ├── launch.sh         Boot script: starts app + opens Chromium kiosk
│       └── templates/        dashboard.html (cyberpunk-style UI)
│
├── models/                   ─── YOLO weights (gitignored — run download_models.sh) ───
│   ├── .gitkeep              Ensures the folder is tracked
│   ├── yolov8n.pt            Used by: usb_stream.py, usb_tracker.py, corridor_scout.py
│   ├── yolov8s.pt            Higher-accuracy variant (optional)
│   ├── yolov8m.pt            Medium variant (optional)
│   └── yolo11n.pt            YOLO11 architecture (optional)
│
├── scripts/
│   ├── download_models.sh    Downloads YOLO .pt files from Ultralytics
│   └── references/           Arducam Pivariety driver scripts (RPi — reference only)
│
├── config/
│   └── mavros.yaml           MAVROS configuration for Pixhawk
│
├── logs/                     ─── Flight telemetry logs (gitignored) ───
│   ├── mav.tlog / .raw       MAVLink telemetry log + raw binary
│   ├── mav.parm              Full parameter dump from Pixhawk
│   └── nidarhex/             NidarHex flight logs (flight.tlog, defaults.parm)
│
└── data/
    └── performance/          jetson_stats CSV logs (GPU/CPU temp, power, RAM)
```

---

## Module: Vision (`vision/`)

### `csi_stream.py`

| Field | Value |
|-------|-------|
| **Purpose** | Stream CSI camera to browser — no AI, pure video |
| **Camera** | Jetson CSI port (IMX219) via GStreamer `nvarguscamerasrc` |
| **Port** | `:5000` |
| **Pipeline** | `nvarguscamerasrc → nvvidconv → videoconvert → appsink` |
| **Resolution** | Captures 1280×720, streams 640×360 (configurable) |
| **Run** | `make stream-csi` or `python vision/csi_stream.py` |
| **See it** | `http://<jetson-ip>:5000` |
| **Heuristics** | • Uses a background thread for capture + thread-safe frame buffer<br>• If camera doesn't open, check `sensor_id` (default 0) — some jetsons have sensor at ID 1<br>• Requires `nvarguscamerasrc` — only works on Jetson L4T, not regular Linux<br>• No AI inference — good for baseline camera sanity check |

### `usb_stream.py`

| Field | Value |
|-------|-------|
| **Purpose** | USB camera + YOLOv8 → lock-on-sight, permanent IDs, MJPEG stream |
| **Camera** | `/dev/video0` (Arducam or any UVC) |
| **Model** | `yolov8n.pt` (must be in `models/`) |
| **Port** | `:5000` |
| **Lock Logic** | First time a person (track ID) appears → **immediately** added to permanent set. Red box "NEW TARGET LOCKED". Subsequent appearances → green box "ALREADY LOCKED". |
| **Run** | `make stream-usb` or `python vision/usb_stream.py` |
| **Heuristics** | • Uses ByteTrack (`model.track(..., persist=True)`) — IDs persist across frames<br>• Confidence threshold: 0.40 for "person" class<br>• No timer — once locked, stays locked for the session<br>• Prints "🔒 NEW TARGET LOCKED: Assigned Permanent ID #N" to terminal |

### `usb_tracker.py`

| Field | Value |
|-------|-------|
| **Purpose** | USB camera + YOLOv8 → **60-second observation** before locking, MJPEG stream |
| **Camera** | `/dev/video0` (Arducam) |
| **Model** | `yolov8n.pt` |
| **Port** | `:5000` |
| **Lock Logic** | New person appears → blue box "Human" + 60s countdown begins. After 60 continuous seconds visible → red box "TARGET LOCKED #N" + prints lock timestamp. If person leaves for >5s during countdown → timer resets. |
| **Three states** | 1. **Blue** "Human" — in observation<br>2. **Red** "TARGET LOCKED" — passed 60s threshold (added to `remembered_human_ids`)<br>3. **Red** "TARGET LOCKED" — previously locked ID (from `remembered_human_ids`) |
| **Run** | `make stream-track` or `python vision/usb_tracker.py` |
| **Heuristics** | • `first_seen_times{}` tracks when each ID first appeared<br>• `remembered_human_ids` set = permanently locked targets<br>• `last_seen_times{}` tracks last appearance for 5s grace period<br>• Buffer cleanup runs every frame: removes unlocked IDs that left >5s ago<br>• This is the more sophisticated version of `usb_stream.py` — choose based on your use case |

### `camera_test.py`

| Field | Value |
|-------|-------|
| **Purpose** | Open serial port and dump raw data — verify camera module is alive |
| **Port** | `/dev/ttyACM0`, 115200 baud |
| **Run** | `make camera-test` or `python vision/camera_test.py` |
| **Heuristics** | • If Pixhawk is connected on `/dev/ttyACM0`, this will read Pixhawk data, not camera data<br>• Only one device can use `/dev/ttyACM0` at a time<br>• Press Ctrl+C to exit |

### File Relationships (Vision)

```
usb_stream.py ── simpler ──┐
                           ├── both use yolov8n.pt model.track()
usb_tracker.py ── 60s timer┘    
                           
csi_stream.py ── standalone (no AI, different camera hardware)

camera_test.py ── standalone (serial, different hardware)
```

---

## Module: Drone (`drone/`)

### `telemetry.py`

| Field | Value |
|-------|-------|
| **Purpose** | Connect to Pixhawk 6C and stream live flight data to terminal |
| **Connection** | `/dev/ttyACM0`, 115200 baud (pymavlink) |
| **Heartbeat** | Waits up to 15s for autopilot heartbeat before starting |
| **Messages** | `ATTITUDE` (roll/pitch/yaw), `VFR_HUD` (alt/heading/throttle), `GPS_RAW_INT` (satellites/fix) |
| **Run** | `make telemetry` or `python drone/telemetry.py` |
| **Heuristics** | • Pixhawk must be powered and connected via USB<br>• If timeout on heartbeat: check USB cable, check `ls /dev/ttyACM*`, try different baud (57600)<br>• Press Ctrl+C to stop gracefully<br>• Prints every 100ms (0.1s sleep between polls)<br>• Run this FIRST before any mission to verify the link is working |

### `corridor_scout.py`

| Field | Value |
|-------|-------|
| **Purpose** | Full autonomous corridor reconnaissance mission |
| **Prerequisites** | Pixhawk connected, GPS 3D fix, GUIDED mode, drone armed, takeoff to >3m |
| **Flight Pattern** | Zigzag (lawnmower) corridor — configurable length/width/lane spacing |
| **Altitude** | 50 ft ≈ 15.24m (hardcoded `FLIGHT_ALTITUDE`) |
| **Video Port** | `:8000` — HTTP MJPEG stream with OSD overlay |
| **Camera** | Auto-detects USB camera (tries `/dev/video0`–`/dev/video3`) |
| **Audio** | Uses `espeak` for voice ("Scout Unit Online", "Scan Complete") |
| **Run** | `make scout` or `python drone/corridor_scout.py` |
| **Heuristics** | • **Auto-detects Pixhawk** — scans `/dev/ttyACM*` at 57600 baud<br>• Requests `MAV_DATA_STREAM_POSITION` at 10 Hz<br>• Mission starts **automatically** when altitude exceeds 3m (post-takeoff detection)<br>• Waypoints generated as corridor grid — flies north-south lines, shifts east each pass<br>• `fly_to()` sends `SET_POSITION_TARGET_GLOBAL_INT` — drone must be in **GUIDED** mode<br>• Waypoint considered reached when within **3m** of target<br>• Sends waypoint commands every 0.5s (not waiting for arrival ACK — primitive control loop)<br>• Prints fake "UPLINK SENT / MULE ACK" every 3s for demo effect<br>• Video OSD shows: scan status, GPS coordinates, uplink indicator<br>• `espeak` voice requires `espeak` and `aplay` installed (`sudo apt install espeak alsa-utils`)<br>• This is a **prototype** — no collision avoidance, no RTL, no failsafe |

### Flight Sequence (corridor_scout.py)

```
1. mavlink_loop() thread starts → auto-detects Pixhawk on /dev/ttyACM*
2. Video server starts on :8000
3. Waits for GPS lock (current_lat != 0)
4. Generates zigzag waypoints relative to takeoff point
5. Initializes USB camera
6. MAIN LOOP:
   a. Reads camera frame → draws OSD → serves via MJPEG
   b. Checks altitude → if >3m, mission_started = True
   c. Sends fly_to() for current waypoint every 0.5s
   d. If within 3m → advance to next waypoint
   e. All waypoints done → hover + "Scan Complete" voice
```

---

## Dashboard (`drone/dashboard/`)

| Field | Value |
|-------|-------|
| **Purpose** | Real-time simulated telemetry dashboard — no Pixhawk needed |
| **Port** | `:5000` |
| **Tech** | Flask + SocketIO + Eventlet |
| **Data** | Mock 4-motor telemetry: RPM, thrust (g), current (A), voltage, direction |
| **UI** | Cyberpunk-style dark theme, SVG drone visualization, color-coded motor overload warning |
| **Run** | `make dashboard` or `python drone/dashboard/app.py` |
| **Heuristics** | • Pure simulation — no hardware required<br>• Updates every 0.5s via WebSocket<br>• Motor current >17A triggers red warning + blink animation<br>• Opens browser automatically (`webbrowser.open`)<br>• Uses Eventlet async mode — do NOT use Werkzeug dev server<br>• `launch.sh` starts app + opens Chromium in kiosk for fullscreen display |

---

## Configuration (`config/`)

### `mavros.yaml`

MAVROS configuration for connecting ROS 2 to Pixhawk 6C. Contains:
- Serial port and baud rate
- Frame IDs
- Streaming rates for telemetry

> This file is included as reference. Actual ROS 2 integration with MAVROS
> requires a running ROS 2 environment — not yet integrated with the Python
> scripts in this repo.

---

## Models

| File | Size | Used By |
|------|------|---------|
| `yolov8n.pt` | ≈6 MB | `vision/usb_stream.py`, `vision/usb_tracker.py`, `drone/corridor_scout.py` |
| `yolov8s.pt` | ≈22 MB | (optional upgrade — more accurate, slower) |
| `yolov8m.pt` | ≈52 MB | (optional) |
| `yolo11n.pt` | ≈6 MB | (optional — YOLO11 architecture) |

**To download**: `bash scripts/download_models.sh`
**To convert to ONNX/TensorRT**: Use Ultralytics `yolo export` or TensorRT directly.
**Repo policy**: Models are gitignored. You must download them manually after clone.

---

## Heuristics & Lessons Learned

### General

1. **Port conflicts**: If you get `OSError: [Errno 98] Address already in use`, another process is on that port. Kill it with `fuser -k 5000/tcp` or `lsof -ti:5000 | xargs kill`.
2. **Camera conflicts**: Only ONE process can open a camera at a time. Kill any other Python scripts using the camera first.
3. **Serial conflicts**: Only ONE process can use `/dev/ttyACM0`. `scout.py` and `telemetry.py` both use Pixhawk — don't run both simultaneously.
4. **Always test the link first**: Run `telemetry.py` before any autonomous mission to confirm Pixhawk communication.
5. **Models directory**: If YOLO scripts crash with `Model not found`, run `bash scripts/download_models.sh`.
6. **No internet on Jetson?** Download models on another machine and SCP them into `models/`.

### Pixhawk / MAVLink

7. **Port enumeration**: Pixhawk may appear as `/dev/ttyACM0` or `/dev/ttyACM1`. Run `ls /dev/ttyACM*` to check.
8. **Baud rate**: Most Pixhawk 6C use 115200 for telemetry, 57600 for some configurations. `telemetry.py` uses 115200; `corridor_scout.py` auto-detects with 57600.
9. **Heartbeat timeout**: If `wait_heartbeat()` fails: check cable, reboot Pixhawk, double-check port.
10. **GUIDED mode**: `corridor_scout.py` requires the flight controller to be in GUIDED mode. Set it via your RC transmitter or QGroundControl before running.
11. **Arm the drone**: The script does NOT arm the drone — you must arm via RC or GCS.
12. **Takeoff**: `corridor_scout.py` detects takeoff when altitude >3m. You must manually take off to at least 3m before the mission starts.

### Camera / Vision

13. **CSI camera**: Only works on Jetson L4T (not on regular Linux). Requires `nvarguscamerasrc` GStreamer plugin.
14. **CSI sensor ID**: If your camera doesn't open, try changing `sensor_id=0` to `sensor_id=1` in the `gstreamer_pipeline()` call.
15. **USB camera index**: Default is `cv2.VideoCapture(0)` — if you have multiple cameras, try indices 1, 2, 3.
16. **YOLO confidence threshold**: Set at 0.40 for "person" class. Adjust via the `conf > 0.40` check in each script.
17. **No bounding boxes?** Check that `model.names[class_id]` matches "person" (lowercase). Some model variants use "Person" with capital P.
18. **Stream lag**: MJPEG over WiFi can have 1–3s latency. For low latency, use a wired connection or reduce resolution.

### Dashboard

19. **Eventlet vs Werkzeug**: `app.py` uses Eventlet async mode. If you see `RuntimeError: You need to use the eventlet server`, make sure you're not running with the default Werkzeug dev server.
20. **Auto-opening browser**: `webbrowser.open()` runs automatically — useful on desktop, skip it if running headless.

### Repo / Git

21. **Never commit models to git**: Files in `models/` are gitignored. Use `download_models.sh` or Git LFS.
22. **Never commit logs**: `logs/` is gitignored. Telemetry logs can be several GB.
23. **Check for secrets before push**: Run `git diff --cached` before committing. No tokens, no IPs, no passwords.
24. **Commit message style**: Use conventional commits: `feat:`, `fix:`, `docs:`, `chore:`.

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `ImportError: No module named 'cv2'` | OpenCV not installed | `pip install opencv-python` |
| `ModelNotFoundError: yolov8n.pt` | Models not downloaded | `bash scripts/download_models.sh` |
| `[Errno 98] Address already in use` | Port 5000 taken | `fuser -k 5000/tcp` |
| `wait_heartbeat() timeout` | Pixhawk not connected / wrong port | Check USB, run `ls /dev/ttyACM*`, verify baud |
| Camera opens but no frames | Wrong camera index / sensor ID | Try `sensor_id=1` (CSI) or `/dev/video1`–3 (USB) |
| `RuntimeError: eventlet` | Wrong WSGI server | Ensure `eventlet.monkey_patch()` is called before Flask |
| CSV logs empty | No DuCo/monitoring agent running | Install `jetson_stats` and run `jtop` once to enable logging |
| `espeak: command not found` | espeak not installed | `sudo apt install espeak alsa-utils` |

---

## File Reference (Quick Lookup)

```bash
# ── I want to...                                           ── Run this ──

# Stream CSI camera to browser (quick sanity check)
make stream-csi                                    # port :5000

# Stream USB camera + YOLO instant-lock
make stream-usb                                    # port :5000

# Stream USB camera + YOLO with 60-second timer
make stream-track                                  # port :5000

# Test if serial camera is alive
make camera-test

# Read live Pixhawk telemetry
make telemetry

# Run autonomous corridor scout mission
make scout                                         # port :8000

# Open simulated telemetry dashboard
make dashboard                                     # port :5000

# Download models for fresh clone
make download-models

# Clean Python cache
make clean
```

---

## License

MIT
