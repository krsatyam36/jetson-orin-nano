"""
Health endpoint — exposes drone telemetry as JSON over HTTP.

Designed as a Flask Blueprint so it can be mounted onto any existing Flask
app (vision streams, dashboard, etc.) or run standalone.

Endpoints:
  GET /health       → Full drone state + pre-arm summary
  GET /health/ping  → Lightweight liveness probe (200 OK)

Usage (standalone):
  python drone/health.py --port 9090

Usage (mount on existing app):
  from flask import Flask
  from drone.health import health_bp
  app = Flask(__name__)
  app.register_blueprint(health_bp)
"""

from __future__ import annotations

import argparse
import time
import threading
from typing import Optional

from flask import Blueprint, jsonify

health_bp = Blueprint("health", __name__)

# These are set by the caller so the blueprint doesn't import controller directly.
_drone_controller: Optional[object] = None
_start_time: float = time.time()


def init_health(controller=None):
    """Link a DroneController instance so /health can read live state."""
    global _drone_controller
    _drone_controller = controller


@health_bp.route("/health/ping")
def ping():
    return "pong", 200


@health_bp.route("/health")
def health():
    ctrl = _drone_controller
    uptime = time.time() - _start_time

    if ctrl is None:
        return jsonify({
            "status": "degraded",
            "uptime_seconds": round(uptime),
            "drone": None,
            "message": "No DroneController linked",
        })

    s = ctrl.state()

    # Pre-arm status
    pre_arm = {"status": "unknown", "failures": []}
    if not s.armed:
        failures = ctrl.pre_arm_check(require_gps_3d=False)
        pre_arm = {
            "status": "fail" if failures else "pass",
            "failures": failures,
        }

    return jsonify({
        "status": "ok" if s.heartbeat_ok else "degraded",
        "uptime_seconds": round(uptime),
        "drone": {
            "armed": s.armed,
            "mode": s.mode,
            "heartbeat": s.heartbeat_ok,
            "ekf_ok": s.ekf_ok,
            "position": {
                "lat": s.lat,
                "lon": s.lon,
                "alt_rel_m": round(s.alt_rel, 1),
                "alt_abs_m": round(s.alt_abs, 1),
                "heading_deg": round(s.heading, 1),
            },
            "attitude": {
                "roll_rad": round(s.roll, 3),
                "pitch_rad": round(s.pitch, 3),
                "yaw_rad": round(s.yaw, 3),
            },
            "speed": {
                "ground_mps": round(s.ground_speed, 1),
                "air_mps": round(s.air_speed, 1),
            },
            "battery": {
                "voltage": round(s.battery_voltage, 2),
                "current_a": round(s.battery_current, 2),
                "remaining_pct": round(s.battery_remaining, 1),
            },
            "gps": {
                "satellites": s.satellites_visible,
                "fix_type": s.fix_type,
            },
        },
        "pre_arm": pre_arm,
    })


def run_health_server(controller=None, port: int = 9090):
    """Run health endpoint as a standalone Flask server (blocking)."""
    from flask import Flask

    app = Flask(__name__)
    app.register_blueprint(health_bp)
    init_health(controller)

    print(f"[health] Serving on :{port}")
    app.run(host="0.0.0.0", port=port, debug=False)


def run_health_thread(controller=None, port: int = 9090) -> threading.Thread:
    """Run health endpoint in a daemon thread."""
    t = threading.Thread(target=run_health_server, args=(controller, port), daemon=True)
    t.start()
    return t


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone health endpoint")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()
    run_health_server(port=args.port)
