"""
Tests for drone/controller.py
"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))

from drone.controller import DroneState, GeofenceConfig, WatchdogConfig, DroneController


def test_dronestate_defaults():
    s = DroneState()
    assert s.lat == 0.0
    assert s.lon == 0.0
    assert s.alt_rel == 0.0
    assert s.armed is False
    assert s.heartbeat_ok is False


def test_geofence_config_defaults():
    g = GeofenceConfig()
    assert g.max_alt_m == 100.0
    assert g.max_radius_m == 500.0
    assert g.home_lat == 0.0


def test_watchdog_config_defaults():
    w = WatchdogConfig()
    assert w.timeout_seconds == 5.0
    assert w.fail_action == "RTL"


def test_haversine():
    c = DroneController()
    # Distance from equator to 1 degree north ≈ 111.32 km
    dist = c._haversine(0, 0, 1, 0)
    assert abs(dist - 111319.5) < 10  # within 10 metres


def test_offset_position():
    c = DroneController()
    lat, lon = c.offset_position(111319.9, 0)
    assert abs(lat - 1.0) < 0.001
    assert abs(lon - 0.0) < 0.001


def test_drone_controller_init():
    c = DroneController()
    assert c._baud == 57600
    assert c._running is False
