"""
CSI Camera MJPEG Streaming Server (Jetson)

Captures video from a Jetson CSI camera (e.g. IMX219) using a GStreamer
pipeline (nvarguscamerasrc) in a background thread, and serves the frames
as an MJPEG stream over HTTP via Flask at http://<jetson-ip>:5000.

How it works:
- A separate thread continuously reads frames from the CSI camera.
- Each frame is JPEG-encoded and stored in a shared buffer with a lock.
- The Flask endpoint '/' returns a multipart/x-mixed-replace response that
  streams the JPEG frames to any browser or video client.

Usage:
  python csi_stream.py
  Then open http://<jetson-ip>:5000 in a browser.
"""

import cv2
import threading
from flask import Flask, Response

app = Flask(__name__)
frame_buffer = None
lock = threading.Lock()


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    display_width=640,
    display_height=360,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def capture_thread():
    global frame_buffer
    # Open Camera with GStreamer
    cap = cv2.VideoCapture(
        gstreamer_pipeline(sensor_id=0, flip_method=0), cv2.CAP_GSTREAMER
    )

    if not cap.isOpened():
        print("ERROR: Could not open CSI Camera!")
        return

    print("CAMERA ACTIVE. Streaming...")
    while True:
        ret, frame = cap.read()
        if ret:
            with lock:
                _, encoded = cv2.imencode(".jpg", frame)
                frame_buffer = encoded.tobytes()


def generate():
    while True:
        with lock:
            if frame_buffer is None:
                continue
            data = frame_buffer
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n")


@app.route("/")
def index():
    return Response(generate(), mimetype="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    t = threading.Thread(target=capture_thread)
    t.start()
    app.run(host="0.0.0.0", port=5000, debug=False)
