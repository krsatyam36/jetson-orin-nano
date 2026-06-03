"""
USB Camera YOLOv8 Instant Person Lock + MJPEG Stream

Captures video from a USB camera (e.g. Arducam), runs YOLOv8 object
detection + ByteTrack tracking via the shared Detector, and serves the
annotated feed as an MJPEG stream via Flask at http://<jetson-ip>:5000.

Person-locking logic:
  - The first time a person (track ID) is seen, they are immediately
    added to `remembered_human_ids` and boxed RED with "NEW TARGET LOCKED".
  - On subsequent appearances, same ID is boxed GREEN with "ALREADY LOCKED".
  - No grace period or timer — lock-on-sight, permanent for the session.

Usage:
  python vision/usb_stream.py
  Then open http://<jetson-ip>:5000 in a browser.
"""

import cv2
from flask import Flask, Response
from vision.detector import Detector

app = Flask(__name__)

detector = Detector("models/yolov8n.pt", conf_threshold=0.4)

camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

remembered_human_ids = set()


def generate_frames():
    global remembered_human_ids

    while True:
        success, frame = camera.read()
        if not success:
            break

        detections = detector.track(frame, persist=True)

        for d in detections:
            if d.class_name != "person":
                continue

            tid = d.track_id
            x1, y1, x2, y2 = d.bbox

            if tid in remembered_human_ids:
                color = (0, 255, 0)
                label = f"ALREADY LOCKED #{tid}"
                thickness = 2
            else:
                remembered_human_ids.add(tid)
                print(f"NEW TARGET LOCKED: Assigned Permanent ID #{tid}")
                color = (0, 0, 255)
                label = f"NEW TARGET LOCKED #{tid}"
                thickness = 3

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


@app.route('/')
def index():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
