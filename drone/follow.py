"""
Person-follow mode — tracks a target person and steers the drone toward them.

Combines the vision Detector with the DroneController: as long as a person
track ID is being actively followed, the script sends relative position
corrections to the Pixhawk to keep the person centered in the frame.

State machine:
  IDLE      → waiting for a target
  SEEKING   → scanning for a person
  FOLLOWING → actively tracking and steering toward the target
  LOST      → target disappeared, loitering for re-acquisition

Usage:
  python drone/follow.py
  Open http://<jetson-ip>:5001 for the MJPEG feed with follow-mode OSD.
"""

import time
import threading
from enum import Enum

import cv2
from flask import Flask, Response

from vision.detector import Detector
from drone.controller import DroneController


class FollowState(Enum):
    IDLE = "IDLE"
    SEEKING = "SEEKING"
    FOLLOWING = "FOLLOWING"
    LOST = "LOST"


app = Flask(__name__)

# ── Configuration ──────────────────────────────────────────────────────
CAMERA_INDEX = 0
MODEL_PATH = "models/yolov8n.pt"
CONF_THRESHOLD = 0.4
FOLLOW_DISTANCE_M = 5.0  # try to stay ~5m from the target
LOST_TIMEOUT = 5.0  # seconds before FOLLOWING → LOST
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FLASK_PORT = 5001

# ── Shared state ───────────────────────────────────────────────────────
state = FollowState.IDLE
target_track_id: int | None = None
last_seen_time: float = 0.0
frame_buffer: bytes | None = None
frame_lock = threading.Lock()

detector = Detector(MODEL_PATH, conf_threshold=CONF_THRESHOLD)
ctrl = DroneController()

camera = cv2.VideoCapture(CAMERA_INDEX)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)


# ── Main loop ───────────────────────────────────────────────────────────


def follow_loop():
    global state, target_track_id, last_seen_time, frame_buffer

    while True:
        success, frame = camera.read()
        if not success:
            time.sleep(0.05)
            continue

        detections = detector.track(frame, persist=True)
        now = time.time()

        # ── State machine ───────────────────────────────────────────────
        person_detections = [d for d in detections if d.class_name == "person"]

        if state == FollowState.IDLE:
            if person_detections:
                # Auto-select the highest-confidence person
                best = max(person_detections, key=lambda d: d.confidence)
                target_track_id = best.track_id
                state = FollowState.FOLLOWING
                last_seen_time = now
                print(f"[follow] FOLLOWING target #{target_track_id}")

        elif state == FollowState.FOLLOWING:
            target = _find_target(person_detections)
            if target:
                last_seen_time = now
                _steer_toward(frame, target)
            elif now - last_seen_time > LOST_TIMEOUT:
                state = FollowState.LOST
                print(f"[follow] LOST target #{target_track_id}")
                target_track_id = None

        elif state == FollowState.LOST:
            if person_detections:
                best = max(person_detections, key=lambda d: d.confidence)
                target_track_id = best.track_id
                state = FollowState.FOLLOWING
                last_seen_time = now
                print(f"[follow] RE-ACQUIRED target #{target_track_id}")
            else:
                # Slow loiter turn while scanning
                pass

        elif state == FollowState.SEEKING:
            if person_detections:
                best = max(person_detections, key=lambda d: d.confidence)
                target_track_id = best.track_id
                state = FollowState.FOLLOWING
                last_seen_time = now
                print(f"[follow] Found target #{target_track_id}")

        # ── Draw OSD ────────────────────────────────────────────────────
        _draw_osd(frame)

        # ── Encode for stream ───────────────────────────────────────────
        ret, jpeg = cv2.imencode(".jpg", frame)
        with frame_lock:
            frame_buffer = jpeg.tobytes()


def _find_target(detections):
    """Return the Detection matching our current target_track_id, if any."""
    for d in detections:
        if d.track_id == target_track_id:
            return d
    return None


def _steer_toward(frame, target):
    """Compute and send position correction to keep target centered."""
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    tx = (target.bbox[0] + target.bbox[2]) // 2
    ty = (target.bbox[1] + target.bbox[3]) // 2

    # Placeholder for velocity-body control (MAV_FRAME_BODY_NED).
    # The yaw_rate and pitch_angle heuristics are computed but not yet sent
    # to the Pixhawk — requires a SET_POSITION_TARGET_LOCAL_NED message.

    # Visual indicator of steering intent
    cv2.arrowedLine(frame, (cx, cy), (tx, ty), (0, 255, 255), 2)
    cv2.circle(frame, (tx, ty), 5, (0, 255, 255), -1)


def _draw_osd(frame):
    """Overlay follow-mode state on the frame."""
    cv2.putText(
        frame, "MODE: FOLLOW", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2
    )
    cv2.putText(
        frame,
        f"STATE: {state.value}",
        (10, 55),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )
    if target_track_id is not None:
        cv2.putText(
            frame,
            f"TARGET: #{target_track_id}",
            (10, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )

    cv2.putText(
        frame,
        f"BAT: {ctrl.state().battery_voltage:.1f}V",
        (FRAME_WIDTH - 120, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        2,
    )


# ── Flask stream ────────────────────────────────────────────────────────


def generate():
    while True:
        with frame_lock:
            if frame_buffer is None:
                yield b""
                continue
            data = frame_buffer
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n")


@app.route("/")
def index():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


# ── Entry ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print("[follow] Connecting to Pixhawk...")
    if ctrl.connect():
        print("[follow] Pixhawk connected")
        failures = ctrl.pre_arm_check()
        if failures:
            print("[follow] Pre-arm failures:")
            for f in failures:
                print(f"  - {f}")
    else:
        print("[follow] No Pixhawk — running in simulation mode")

    t = threading.Thread(target=follow_loop, daemon=True)
    t.start()

    state = FollowState.SEEKING
    print(f"[follow] Serving on :{FLASK_PORT}")
    app.run(host="0.0.0.0", port=FLASK_PORT, threaded=True)
