"""
USB Camera YOLOv8 Person Tracking with 60-Second Lock Timer + MJPEG Stream

Captures video from a USB camera (e.g. Arducam), runs YOLOv8 object
detection + ByteTrack tracking via the shared Detector, and serves the
annotated feed as an MJPEG stream via Flask at http://<jetson-ip>:5000.

Person-locking logic (3 scenarios):
  1. If a tracked person ID is in `remembered_human_ids` (already locked
     from a previous session or after the 60s threshold), box is RED with
     "TARGET LOCKED" label.
  2. If a new person appears, a stopwatch starts. After 60 continuous
     seconds of being visible, the ID is permanently locked (RED).
  3. During the 60-second countdown, box is BLUE with "Human" label.
  4. If an unlocked person leaves frame for >5 seconds, their timer resets.

Usage:
  python vision/usb_tracker.py
  Then open http://<jetson-ip>:5000 in a browser.
"""

import time
from datetime import datetime
import cv2
from flask import Flask, Response
from vision.detector import Detector

app = Flask(__name__)

detector = Detector("models/yolov8n.pt", conf_threshold=0.4)

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

first_seen_times = {}
remembered_human_ids = set()
last_seen_times = {}


def generate_frames():
    global first_seen_times, remembered_human_ids, last_seen_times

    while True:
        success, frame = camera.read()
        if not success:
            break

        current_time = time.time()
        detections = detector.track(frame, persist=True)
        active_ids_in_frame = set()

        for d in detections:
            if d.class_name != "person":
                continue

            tid = d.track_id
            x1, y1, x2, y2 = d.bbox
            active_ids_in_frame.add(tid)
            last_seen_times[tid] = current_time

            if tid in remembered_human_ids:
                color = (0, 0, 255)
                label = f"TARGET LOCKED #{tid}"
                thickness = 3
            else:
                if tid not in first_seen_times:
                    first_seen_times[tid] = current_time

                elapsed = current_time - first_seen_times[tid]

                if elapsed >= 60.0:
                    remembered_human_ids.add(tid)
                    lock_ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                    print(f"\n[!] TARGET LOCKED | ID #{tid} secured at: {lock_ts}\n")
                    color = (0, 0, 255)
                    label = f"TARGET LOCKED #{tid}"
                    thickness = 3
                else:
                    color = (255, 100, 0)
                    label = "Human"
                    thickness = 1

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        for test_id in list(first_seen_times.keys()):
            if test_id not in active_ids_in_frame and test_id not in remembered_human_ids:
                elapsed_lost = current_time - last_seen_times.get(test_id, current_time)
                if elapsed_lost > 5.0:
                    del first_seen_times[test_id]
                    last_seen_times.pop(test_id, None)

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
