"""
USB Camera YOLOv8 Instant Person Lock + MJPEG Stream

Captures video from a USB camera (e.g. Arducam), runs YOLOv8 object
detection + ByteTrack tracking, and serves the annotated feed as an
MJPEG stream via Flask at http://<jetson-ip>:5000.

Person-locking logic:
  - The first time a new person (track ID) is seen, they are immediately
    added to `remembered_human_ids` and boxed RED with "NEW TARGET LOCKED".
  - On subsequent appearances, same ID is boxed GREEN with "ALREADY LOCKED".
  - No grace period or timer — lock-on-sight, permanent for the session.

Usage:
  python usb_stream.py
  Then open http://<jetson-ip>:5000 in a browser.
"""

import cv2
from flask import Flask, Response
from ultralytics import YOLO

app = Flask(__name__)

# Load the lightweight YOLOv8 model
model = YOLO('yolov8n.pt')

# Open the Arducam USB port
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# This list will act as our permanent database memory for human IDs
remembered_human_ids = set()

def generate_frames():
    global remembered_human_ids
    
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # Run ByteTrack tracking. persist=True tracks IDs across frames
            results = model.track(frame, persist=True, verbose=False)
            
            # Check if any objects with tracking IDs are currently on screen
            if results[0].boxes.id is not None:
                boxes = results[0].boxes.xyxy.int().cpu().tolist()
                class_ids = results[0].boxes.cls.int().cpu().tolist()
                confidences = results[0].boxes.conf.cpu().tolist()
                track_ids = results[0].boxes.id.int().cpu().tolist()
                
                for box, class_id, conf, track_id in zip(boxes, class_ids, confidences, track_ids):
                    # Filter specifically for humans ("person") with good confidence
                    if model.names[class_id] == "person" and conf > 0.40:
                        x1, y1, x2, y2 = box
                        
                        # SCENARIO A: We have already seen this person before
                        if track_id in remembered_human_ids:
                            color = (0, 255, 0)  # Green color
                            label_text = f"ALREADY LOCKED #{track_id}"
                            thickness = 2
                        
                        # SCENARIO B: This is a brand new person entering the camera view
                        else:
                            remembered_human_ids.add(track_id)  # Save to memory forever
                            print(f"🔒 NEW TARGET LOCKED: Assigned Permanent ID #{track_id}")
                            
                            color = (0, 0, 255)  # Red color
                            label_text = f"NEW TARGET LOCKED #{track_id}"
                            thickness = 3
                        
                        # Draw bounding box around the human
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                        # Draw label text banner
                        cv2.putText(frame, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Compress processed frame to JPEG for browser streaming
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
