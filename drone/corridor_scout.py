"""
Autonomous Corridor Scout Mission — revised.

Uses DroneController for MAVLink and Detector for vision. Adds:
  - Pre-arm checklist                              - Automated takeoff
  - Geofence + watchdog (via controller)            - Health endpoint
  - RTL on mission complete                        - Configurable corridor

Usage:
  python drone/corridor_scout.py --length 150 --width 50 --alt 20
  Then open http://<jetson-ip>:8000 for the video feed.

The drone must be armed and in GUIDED mode. The mission starts automatically
after takeoff (altitude > 3m).
"""

import time
import math
import os
import threading
import argparse

import cv2
import socketserver
from http import server
from threading import Condition

from vision.detector import Detector
from drone.controller import DroneController
from drone.health import init_health, health_bp
from flask import Flask

# ── Configuration ───────────────────────────────────────────────────────
FLIGHT_ALTITUDE = 15.24       # 50 ft
CORRIDOR_LENGTH = 100.0       # metres
CORRIDOR_WIDTH = 40.0
LANE_SPACING = 10.0
DETECTOR_CONF = 0.4
CAMERA_INDEX = 0


# ── Global state ────────────────────────────────────────────────────────
class StreamingOutput:
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, frame):
        with self.condition:
            self.frame = frame
            self.condition.notify_all()


output = StreamingOutput()
waypoints: list[tuple[float, float]] = []
wp_index = 0
mission_started = False
detector: Detector | None = None
ctrl = DroneController()


# ── Path planning ───────────────────────────────────────────────────────

def offset_lat_lon(lat: float, lon: float, north_m: float, east_m: float) -> tuple[float, float]:
    new_lat = lat + (north_m / 111319.9)
    new_lon = lon + (east_m / (111319.9 * math.cos(math.radians(lat))))
    return new_lat, new_lon


def generate_zigzag_path(start_lat: float, start_lon: float) -> list[tuple[float, float]]:
    print(f"[planner] Generating corridor: {CORRIDOR_LENGTH}m × {CORRIDOR_WIDTH}m")
    points = []
    num_lanes = int(CORRIDOR_WIDTH / LANE_SPACING) + 1

    for i in range(num_lanes):
        offset_east = i * LANE_SPACING
        if i % 2 == 0:
            p1 = offset_lat_lon(start_lat, start_lon, 0, offset_east)
            p2 = offset_lat_lon(start_lat, start_lon, CORRIDOR_LENGTH, offset_east)
        else:
            p1 = offset_lat_lon(start_lat, start_lon, CORRIDOR_LENGTH, offset_east)
            p2 = offset_lat_lon(start_lat, start_lon, 0, offset_east)
        points.append(p1)
        points.append(p2)

    print(f"[planner] {len(points)} waypoints generated")
    return points


# ── Video server ────────────────────────────────────────────────────────

class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(b"<html><body style='background:black; color:cyan;'>")
            self.wfile.write(b"<h1>SCOUT - CORRIDOR SCAN</h1>")
            self.wfile.write(b"<img src='stream.mjpg' style='width:100%;'/></body></html>")

        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    ret, jpeg = cv2.imencode('.jpg', frame)
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.end_headers()
                    self.wfile.write(jpeg.tobytes())
                    self.wfile.write(b'\r\n')
            except Exception:
                pass


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def run_video_server(port: int = 8000):
    server = StreamingServer(('', port), StreamingHandler)
    print(f"[video] Streaming on :{port}")
    server.serve_forever()


# ── Mission ─────────────────────────────────────────────────────────────

def run_mission():
    global waypoints, wp_index, mission_started

    # 1. Get GPS
    print("[mission] Waiting for GPS lock...")
    while ctrl.state().lat == 0:
        time.sleep(1)
    lat, lon = ctrl.state().lat, ctrl.state().lon
    print(f"[mission] GPS locked at {lat:.6f}, {lon:.6f}")

    # 2. Generate path
    waypoints = generate_zigzag_path(lat, lon)

    # 3. Pre-arm check
    failures = ctrl.pre_arm_check()
    if failures:
        print("[mission] Pre-arm FAILED:")
        for f in failures:
            print(f"  ✗ {f}")
        print("[mission] Fix issues and re-run")
        return

    print("[mission] Pre-arm passed")

    # 4. Arm
    if not ctrl.arm():
        print("[mission] Arm failed")
        return
    print("[mission] Armed")

    # 5. Takeoff
    if not ctrl.takeoff(FLIGHT_ALTITUDE):
        print("[mission] Takeoff failed")
        return

    # 6. Main navigation loop
    wp_index = 0
    mission_started = True
    print(f"[mission] Mission started — {len(waypoints)} waypoints")

    try:
        while True:
            s = ctrl.state()
            if wp_index < len(waypoints):
                target = waypoints[wp_index]
                dist = ctrl.distance_to(target[0], target[1])
                ctrl.fly_to(target[0], target[1], FLIGHT_ALTITUDE)
                print(f"[nav] WP {wp_index + 1}/{len(waypoints)} — {dist:.1f}m away")

                if dist < 3.0:
                    print(f"[nav] Reached WP {wp_index + 1}")
                    wp_index += 1
                    if wp_index >= len(waypoints):
                        print("[nav] All waypoints complete — returning to launch")
                        os.system('espeak "Scan complete. Returning to launch." 2>/dev/null &')
                        ctrl.rtl()
                        break
            else:
                break

            # Camera + OSD
            cap = cv2.VideoCapture(CAMERA_INDEX)
            ret, frame = cap.read()
            if ret:
                detections = detector.track(frame) if detector else []
                for d in detections:
                    if d.class_name == "person":
                        x1, y1, x2, y2 = d.bbox
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                        cv2.putText(frame, f"{d.class_name} #{d.track_id}", (x1, y1 - 8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                cv2.putText(frame, f"WP: {wp_index}/{len(waypoints)}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"GPS: {s.lat:.5f}, {s.lon:.5f}", (10, 55),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"ALT: {s.alt_rel:.1f}m", (10, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(frame, f"BAT: {s.battery_voltage:.1f}V", (10, 105),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if wp_index < len(waypoints):
                    cv2.putText(frame, "MISSION: ACTIVE", (10, 130),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                else:
                    cv2.putText(frame, "MISSION: COMPLETE — RTL", (10, 130),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                output.write(frame)
            cap.release()

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\n[mission] Interrupted — RTL")
        ctrl.rtl()
    finally:
        mission_started = False
        ctrl.close()


# ── Entry point ─────────────────────────────────────────────────────────

def main():
    global detector, FLIGHT_ALTITUDE, CORRIDOR_LENGTH, CORRIDOR_WIDTH, LANE_SPACING, DETECTOR_CONF

    parser = argparse.ArgumentParser(description="Corridor scout mission")
    parser.add_argument("--alt", type=float, default=FLIGHT_ALTITUDE, help="Flight altitude (m)")
    parser.add_argument("--length", type=float, default=CORRIDOR_LENGTH, help="Corridor length (m)")
    parser.add_argument("--width", type=float, default=CORRIDOR_WIDTH, help="Corridor width (m)")
    parser.add_argument("--lane", type=float, default=LANE_SPACING, help="Lane spacing (m)")
    parser.add_argument("--conf", type=float, default=DETECTOR_CONF, help="Detection confidence")
    parser.add_argument("--no-vision", action="store_true", help="Skip YOLO detection")
    parser.add_argument("--port", type=int, default=8000, help="Video stream port")
    args = parser.parse_args()

    FLIGHT_ALTITUDE = args.alt
    CORRIDOR_LENGTH = args.length
    CORRIDOR_WIDTH = args.width
    LANE_SPACING = args.lane
    DETECTOR_CONF = args.conf

    # Init detector
    if not args.no_vision:
        try:
            detector = Detector("models/yolov8n.pt", conf_threshold=DETECTOR_CONF)
        except Exception as e:
            print(f"[main] Detector init failed: {e}")
            detector = None

    # Connect controller
    print("[main] Connecting to Pixhawk...")
    if not ctrl.connect():
        print("[main] No Pixhawk — running in simulation mode")

    # Set geofence
    ctrl.set_geofence(max_alt=FLIGHT_ALTITUDE + 20, max_radius=CORRIDOR_WIDTH * 3)

    # Health endpoint on a daemon thread via Flask
    health_app = Flask(__name__)
    health_app.register_blueprint(health_bp)
    init_health(ctrl)
    threading.Thread(target=health_app.run, kwargs={"host": "0.0.0.0", "port": 9090, "debug": False}, daemon=True).start()

    # Video server thread
    threading.Thread(target=run_video_server, args=(args.port,), daemon=True).start()

    # Run mission (blocking)
    run_mission()


if __name__ == "__main__":
    main()
