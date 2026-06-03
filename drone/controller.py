"""
Drone flight controller abstraction — arm, takeoff, land, RTL, geofence, watchdog, pre-arm.

Wraps pymavlink with a clean state-machine API so mission scripts never need to
touch MAVLink protocol details directly.

Usage:
    from drone.controller import DroneController

    ctrl = DroneController()
    ctrl.connect()
    if ctrl.pre_arm_check():
        ctrl.arm()
        ctrl.takeoff(15.24)
        # ... fly_to() ...
        ctrl.rtl()

Geofence and watchdog are automatically enforced in a background thread.
"""

import time
import math
import glob
import threading
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class DroneState:
    """Read-only snapshot of the drone's current state."""
    lat: float = 0.0
    lon: float = 0.0
    alt_rel: float = 0.0
    alt_abs: float = 0.0
    heading: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    ground_speed: float = 0.0
    air_speed: float = 0.0
    throttle: float = 0.0
    battery_voltage: float = 0.0
    battery_current: float = 0.0
    battery_remaining: float = 0.0
    satellites_visible: int = 0
    fix_type: int = 0  # 0=no GPS, 2=2D, 3=3D
    armed: bool = False
    mode: str = "UNKNOWN"
    heartbeat_ok: bool = False
    ekf_ok: bool = True


@dataclass
class GeofenceConfig:
    """Configurable geofence limits. Set any to 0 to disable that axis."""
    max_alt_m: float = 100.0
    max_radius_m: float = 500.0
    home_lat: float = 0.0
    home_lon: float = 0.0


@dataclass
class WatchdogConfig:
    """If heartbeat is lost for timeout_seconds, trigger the fail action."""
    timeout_seconds: float = 5.0
    fail_action: str = "RTL"  # "RTL", "LAND", "HOVER", "DISARM"


class DroneController:
    """High-level drone control with automatic geofence and watchdog."""

    STATE_NAMES = {
        "DISARMED": "the drone is not armed and will not respond to commands",
        "ARMED": "the drone is armed and ready for flight",
        "GUIDED": "the drone accepts position targets from the onboard computer",
        "AUTO": "the drone follows a pre-loaded mission",
        "RTL": "the drone is returning to launch",
        "LAND": "the drone is landing",
        "LOITER": "the drone is holding position",
        "STABILIZE": "the drone is in manual stabilize mode",
        "ALT_HOLD": "the drone holds altitude but accepts roll/pitch",
    }

    def __init__(
        self,
        geofence: Optional[GeofenceConfig] = None,
        watchdog: Optional[WatchdogConfig] = None,
        connection_baud: int = 57600,
    ):
        self._link = None
        self._baud = connection_baud
        self._state = DroneState()
        self._state_lock = threading.Lock()
        self._geofence = geofence or GeofenceConfig()
        self._watchdog = watchdog or WatchdogConfig()
        self._running = False
        self._watchdog_event = threading.Event()
        self._watchdog_event.set()  # start healthy
        self._last_heartbeat = 0.0
        self._home_set = False

    # ── Connection ──────────────────────────────────────────────────────

    def connect(self, port: Optional[str] = None, timeout: int = 10) -> bool:
        """Auto-detect Pixhawk on /dev/ttyACM* or connect to a specific port.

        Returns True if a heartbeat was received within *timeout* seconds.
        """
        import pymavlink.mavutil as mavutil

        if port is None:
            ports = sorted(glob.glob("/dev/ttyACM*"))
            if not ports:
                print("[controller] No /dev/ttyACM* devices found")
                return False
            port = self._probe_ports(ports, timeout)
            if port is None:
                return False
        else:
            self._link = mavutil.mavlink_connection(port, baud=self._baud)
            msg = self._link.wait_heartbeat(timeout=timeout)
            if msg is None:
                print(f"[controller] No heartbeat on {port}")
                self._link = None
                return False

        print(f"[controller] Connected to Pixhawk on {port}")
        self._running = True
        self._last_heartbeat = time.time()

        # Request streams
        self._link.mav.request_data_stream_send(
            self._link.target_system,
            self._link.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10, 1,
        )
        time.sleep(0.5)

        # Record home position
        self._state.heartbeat_ok = True
        self._set_home_from_gps()

        # Start background threads
        threading.Thread(target=self._update_loop, daemon=True).start()
        threading.Thread(target=self._watchdog_loop, daemon=True).start()
        threading.Thread(target=self._geofence_loop, daemon=True).start()

        return True

    def _probe_ports(self, ports: list[str], timeout: int) -> Optional[str]:
        import pymavlink.mavutil as mavutil

        for port in ports:
            try:
                link = mavutil.mavlink_connection(port, baud=self._baud)
                msg = link.wait_heartbeat(timeout=min(timeout, 3))
                if msg:
                    self._link = link
                    return port
                link.close()
            except Exception:
                continue
        return None

    # ── State updater ──────────────────────────────────────────────────

    def _update_loop(self):
        import pymavlink.mavutil as mavutil

        while self._running:
            if self._link is None:
                time.sleep(1)
                continue

            msg = self._link.recv_match(blocking=False)
            if msg is None:
                time.sleep(0.01)
                continue

            msg_type = msg.get_type()
            with self._state_lock:
                if msg_type == "HEARTBEAT":
                    self._state.heartbeat_ok = True
                    self._last_heartbeat = time.time()
                    self._watchdog_event.set()
                    self._state.armed = (msg.base_mode & 0b10000000) != 0
                    self._state.mode = mavutil.mode_mapping_acm.get(msg.custom_mode, str(msg.custom_mode))

                elif msg_type == "GLOBAL_POSITION_INT":
                    self._state.lat = msg.lat / 1e7
                    self._state.lon = msg.lon / 1e7
                    self._state.alt_rel = msg.relative_alt / 1000.0
                    self._state.alt_abs = msg.alt / 1000.0
                    self._state.heading = msg.hdg / 100.0

                elif msg_type == "ATTITUDE":
                    self._state.roll = msg.roll
                    self._state.pitch = msg.pitch
                    self._state.yaw = msg.yaw

                elif msg_type == "VFR_HUD":
                    self._state.air_speed = msg.airspeed
                    self._state.ground_speed = msg.groundspeed
                    self._state.heading = msg.heading
                    self._state.throttle = msg.throttle
                    self._state.alt_rel = msg.alt

                elif msg_type == "GPS_RAW_INT":
                    self._state.satellites_visible = msg.satellites_visible
                    self._state.fix_type = msg.fix_type

                elif msg_type == "SYS_STATUS":
                    self._state.battery_voltage = msg.voltage_battery / 1000.0
                    self._state.battery_current = msg.current_battery / 100.0
                    self._state.battery_remaining = msg.battery_remaining

                elif msg_type == "EKF_STATUS_REPORT":
                    self._state.ekf_ok = (msg.flags & 0b00000001) != 0

    # ── Watchdog ────────────────────────────────────────────────────────

    def _watchdog_loop(self):
        while self._running:
            if self._link is not None and self._state.armed:
                elapsed = time.time() - self._last_heartbeat
                if elapsed > self._watchdog.timeout_seconds:
                    print(f"[watchdog] Heartbeat lost for {elapsed:.1f}s — executing {self._watchdog.fail_action}")
                    if self._watchdog.fail_action == "RTL":
                        self.rtl()
                    elif self._watchdog.fail_action == "LAND":
                        self.land()
                    elif self._watchdog.fail_action == "DISARM":
                        self.disarm()
                    self._watchdog_event.clear()
                    self._watchdog_event.wait(timeout=30)
            time.sleep(0.5)

    # ── Geofence ───────────────────────────────────────────────────────

    def _geofence_loop(self):
        while self._running:
            if self._geofence.max_alt_m > 0 and self._state.alt_rel > self._geofence.max_alt_m:
                print(f"[geofence] Altitude {self._state.alt_rel:.1f}m exceeds limit {self._geofence.max_alt_m}m — RTL")
                self.rtl()

            if self._geofence.max_radius_m > 0 and self._geofence.home_lat != 0:
                dist = self._haversine(
                    self._geofence.home_lat, self._geofence.home_lon,
                    self._state.lat, self._state.lon,
                )
                if dist > self._geofence.max_radius_m:
                    print(f"[geofence] Distance {dist:.0f}m exceeds limit {self._geofence.max_radius_m}m — RTL")
                    self.rtl()

            time.sleep(1.0)

    def set_geofence(self, max_alt: float = 0, max_radius: float = 0):
        self._geofence.max_alt_m = max_alt
        self._geofence.max_radius_m = max_radius
        print(f"[geofence] Set limits: alt={max_alt}m, radius={max_radius}m")

    # ── Pre-arm checklist ───────────────────────────────────────────────

    def pre_arm_check(self, require_gps_3d: bool = True, min_voltage: float = 14.0) -> list[str]:
        """Run checks and return a list of failures (empty = all good)."""
        failures = []
        time.sleep(1.0)  # let state settle

        with self._state_lock:
            if self._state.armed:
                failures.append("Drone is already armed — disarm first")

            if require_gps_3d and self._state.fix_type < 3:
                failures.append(f"GPS fix type {self._state.fix_type} — need 3D fix (3)")

            if self._state.satellites_visible < 8:
                failures.append(f"Only {self._state.satellites_visible} satellites — need ≥8")

            if self._state.battery_voltage > 0 and self._state.battery_voltage < min_voltage:
                failures.append(f"Battery {self._state.battery_voltage:.1f}V below minimum {min_voltage}V")

            if not self._state.heartbeat_ok:
                failures.append("No MAVLink heartbeat — check connection")

            if not self._state.ekf_ok:
                failures.append("EKF not healthy — check sensor calibration")

            if self._state.mode != "GUIDED":
                failures.append(f"Mode is {self._state.mode} — switch to GUIDED")

        return failures

    # ── Commands ────────────────────────────────────────────────────────

    def arm(self) -> bool:
        """Arm the drone. Returns True if successful."""
        if self._link is None:
            return False

        self._link.mav.command_long_send(
            self._link.target_system,
            self._link.target_component,
            400,  # MAV_CMD_COMPONENT_ARM_DISARM
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        time.sleep(1.0)
        with self._state_lock:
            return self._state.armed

    def disarm(self) -> bool:
        if self._link is None:
            return False
        self._link.mav.command_long_send(
            self._link.target_system,
            self._link.target_component,
            400, 0, 0, 0, 0, 0, 0, 0, 0,
        )
        time.sleep(1.0)
        with self._state_lock:
            return not self._state.armed

    def takeoff(self, altitude_m: float = 15.24) -> bool:
        """Command autonomous takeoff to *altitude_m*. Returns True if altitude reached."""
        if self._link is None or not self._state.armed:
            print("[controller] Cannot takeoff — not armed")
            return False

        print(f"[controller] Takeoff to {altitude_m}m")
        self._link.mav.command_long_send(
            self._link.target_system,
            self._link.target_component,
            22,  # MAV_CMD_NAV_TAKEOFF
            0, 0, 0, 0, 0, 0, 0, altitude_m,
        )

        # Wait until altitude is reached (with timeout)
        timeout = time.time() + 30
        while time.time() < timeout:
            time.sleep(0.5)
            with self._state_lock:
                if self._state.alt_rel >= altitude_m * 0.9:
                    print(f"[controller] Takeoff complete at {self._state.alt_rel:.1f}m")
                    return True
        print(f"[controller] Takeoff timeout — at {self._state.alt_rel:.1f}m")
        return False

    def land(self) -> bool:
        """Command landing. Returns True when on ground."""
        if self._link is None:
            return False

        print("[controller] Landing")
        self._link.mav.command_long_send(
            self._link.target_system,
            self._link.target_component,
            21,  # MAV_CMD_NAV_LAND
            0, 0, 0, 0, 0, 0, 0, 0,
        )

        timeout = time.time() + 60
        while time.time() < timeout:
            time.sleep(1.0)
            with self._state_lock:
                if self._state.alt_rel < 0.3:
                    print("[controller] Landed")
                    return True
        return False

    def rtl(self) -> bool:
        """Return to launch."""
        if self._link is None:
            return False

        print("[controller] Returning to launch")
        self._link.mav.command_long_send(
            self._link.target_system,
            self._link.target_component,
            20,  # MAV_CMD_NAV_RETURN_TO_LAUNCH
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        return True

    def fly_to(self, lat: float, lon: float, alt: Optional[float] = None):
        """Send SET_POSITION_TARGET_GLOBAL_INT waypoint."""
        if self._link is None:
            return

        alt = alt if alt is not None else self._state.alt_rel
        self._link.mav.set_position_target_global_int_send(
            0, 0, 0,
            3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
            0b0000111111111000,
            int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

    def set_mode(self, mode: str) -> bool:
        """Set flight mode (e.g. "GUIDED", "RTL", "LAND", "STABILIZE")."""
        if self._link is None:
            return False

        mode_id = self._link.mode_mapping().get(mode)
        if mode_id is None:
            print(f"[controller] Unknown mode: {mode}")
            return False

        self._link.mav.command_long_send(
            self._link.target_system,
            self._link.target_component,
            11, 0, mode_id, 0, 0, 0, 0, 0, 0,
        )
        time.sleep(0.5)
        return True

    # ── Position helpers ────────────────────────────────────────────────

    def distance_to(self, lat: float, lon: float) -> float:
        """Haversine distance from current position to (lat, lon) in metres."""
        return self._haversine(self._state.lat, self._state.lon, lat, lon)

    def offset_position(self, north_m: float, east_m: float):
        """Return (lat, lon) offset by metres north/east from current position."""
        lat = self._state.lat + (north_m / 111319.9)
        lon = self._state.lon + (east_m / (111319.9 * math.cos(math.radians(self._state.lat))))
        return lat, lon

    def state(self) -> DroneState:
        """Thread-safe snapshot of current drone state."""
        with self._state_lock:
            import copy
            return copy.deepcopy(self._state)

    # ── Internals ───────────────────────────────────────────────────────

    def _set_home_from_gps(self):
        timeout = time.time() + 15
        while time.time() < timeout:
            with self._state_lock:
                if self._state.lat != 0 and self._state.lon != 0:
                    self._geofence.home_lat = self._state.lat
                    self._geofence.home_lon = self._state.lon
                    print(f"[controller] Home set to {self._state.lat:.6f}, {self._state.lon:.6f}")
                    self._home_set = True
                    return True
            time.sleep(1)
        print("[controller] Could not set home — no GPS fix")
        return False

    @staticmethod
    def _haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return 6371000 * c

    def close(self):
        self._running = False
        if self._link:
            self._link.close()
