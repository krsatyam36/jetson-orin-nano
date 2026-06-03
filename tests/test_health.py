"""
Tests for drone/health.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import json
from drone.health import health_bp, init_health
from flask import Flask


def test_ping():
    app = Flask(__name__)
    app.register_blueprint(health_bp)
    with app.test_client() as client:
        resp = client.get("/health/ping")
        assert resp.status_code == 200
        assert resp.data.decode() == "pong"


def test_health_no_controller():
    app = Flask(__name__)
    app.register_blueprint(health_bp)
    init_health(None)
    with app.test_client() as client:
        resp = client.get("/health")
        assert resp.status_code == 200
        data = json.loads(resp.data)
        assert data["status"] == "degraded"
        assert data["drone"] is None
