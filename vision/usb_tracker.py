"""
USB Camera YOLOv8 Person Tracking with 60-Second Lock Timer + MJPEG Stream

Captures video from a USB camera (e.g. Arducam), runs YOLOv8 object
detection + ByteTrack tracking on each frame, and serves the annotated
feed as an MJPEG stream via Flask at http://<jetson-ip>:5000.

Person-locking logic (3 scenarios):
  1. If a tracked person ID is in `remembered_human_ids` (already locked
     from a previous session or after the 60s threshold), box is RED with
     "TARGET LOCKED" label.
  2. If a new person appears, a stopwatch starts. After 60 continuous
     seconds of being visible, the ID is permanently locked (RED).
  3. During the 60-second countdown, box is BLUE with "Human" label.
  4. If an unlocked person leaves frame for >5 seconds, their timer resets.

Usage:
  python usb_tracker.py
  Then open http://<jetson-ip>:5000 in a browser.
"""

import cv2
import time
from datetime import datetime
from flask import Flask, Response
from ultralytics import YOLO

app = Flask(__name__)

# Load the lightweight YOLOv8 model
model = YOLO('yolov8n.pt')

# Open the Arducam USB port
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Memory Databases
first_seen_times = {}     # Tracks when an ID first appeared {track_id: timestamp}
remembered_human_ids = set()  # Permanent database for fully locked targets
last_seen_times = {}      # Grace period tracker to prevent accidental resets

def generate_frames():
    global first_seen_times, remembered_human_ids, last_seen_times
    
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            current_time = time.time()
            
            # Run ByteTrack tracking
            results = model.track(frame, persist=True, verbose=False)
            
            # Keep track of IDs present in the current frame
            active_ids_in_frame = set()
            
            if results[0].boxes.id is not None:
                boxes = results[0].boxes.xyxy.int().cpu().tolist()
                class_ids = results[0].boxes.cls.int().cpu().tolist()
                confidences = results[0].boxes.conf.cpu().tolist()
                track_ids = results[0].boxes.id.int().cpu().tolist()
                
                for box, class_id, conf, track_id in zip(boxes, class_ids, confidences, track_ids):
                    if model.names[class_id] == "person" and conf > 0.40:
                        x1, y1, x2, y2 = box
                        active_ids_in_frame.add(track_id)
                        last_seen_times[track_id] = current_time  # Update last seen timestamp
                        
                        # SCENARIO 1: Human is already a fully locked target or has been seen before
                        if track_id in remembered_human_ids:
                            color = (0, 0, 255)  # Red
                            label_text = f"TARGET LOCKED #{track_id}"
                            thickness = 3
                        
                        else:
                            # Start the stopwatch if this is a brand new person
                            if track_id not in first_seen_times:
                                first_seen_times[track_id] = current_time
                            
                            elapsed_time = current_time - first_seen_times[track_id]
                            
                            # SCENARIO 2: Human has hit the 1-minute (60 seconds) threshold right now!
                            if elapsed_time >= 60.0:
                                remembered_human_ids.add(track_id)
                                
                                # Print the lock confirmation and exact time to the Jetson terminal window
                                lock_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                                print(f"\n[!] 🔒 TARGET LOCKED | ID #{track_id} secured at: {lock_timestamp}\n")
                                
                                color = (0, 0, 255)  # Red
                                label_text = f"TARGET LOCKED #{track_id}"
                                thickness = 3
                            
                            # SCENARIO 3: Human is detected but hasn't reached 1 minute yet
                            else:
                                color = (255, 100, 0)  # Neutral Blue/Cyan
                                label_text = "Human"   # Just says "Human" per your requirement
                                thickness = 1
                        
                        # Draw visual bounding overlays
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                        cv2.putText(frame, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Cleanup Buffer: If an unlocked person leaves the frame for more than 5 seconds, reset their timer
            for test_id in list(first_seen_times.keys()):
                if test_id not in active_ids_in_frame and test_id not in remembered_human_ids:
                    time_since_lost = current_time - last_seen_times.get(test_id, current_time)
                    if time_since_lost > 5.0:  
                        del first_seen_times[test_id]
                        if test_id in last_seen_times:
                            del last_seen_times[test_id]

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
