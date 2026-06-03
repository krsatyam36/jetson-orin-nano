"""
Pixhawk 6C Live MAVLink Telemetry Reader

Connects to a Pixhawk 6C flight controller over serial (/dev/ttyACM0,
115200 baud) via pymavlink, waits for a heartbeat, then streams live
flight data to the terminal:

  - ATTITUDE: Roll, Pitch, Yaw (radians)
  - VFR_HUD: Altitude (m), Heading (deg), Throttle (%)
  - GPS_RAW_INT: Satellite count, Fix type

Usage:
  python telemetry.py
  Press Ctrl+C to stop.
"""

import time
from pymavlink import mavutil


def read_telemetry():
    port = "/dev/ttyACM0"
    baud = 115200

    print(f"Opening connection to Pixhawk 6C on {port}...")

    try:
        master = mavutil.mavlink_connection(port, baud=baud)

        print("Waiting for autopilot heartbeat signal...")
        master.wait_heartbeat(timeout=15)
        print("Connected! Heartbeat detected from Pixhawk.")

        print("\nStreaming Live Flight Data (Press Ctrl+C to stop):\n" + "-" * 50)

        while True:
            msg = master.recv_match(
                type=["ATTITUDE", "VFR_HUD", "GPS_RAW_INT"], blocking=True, timeout=1.0
            )

            if msg is not None:
                msg_type = msg.get_type()

                if msg_type == "ATTITUDE":
                    print(
                        f"[ATTITUDE] Roll: {msg.roll:.3f} | Pitch: {msg.pitch:.3f} | Yaw: {msg.yaw:.3f}"
                    )

                elif msg_type == "VFR_HUD":
                    print(
                        f"[HUD] Alt: {msg.alt:.1f}m | Heading: {msg.heading}° | Throttle: {msg.throttle}%"
                    )

                elif msg_type == "GPS_RAW_INT":
                    print(
                        f"[GPS] Satellites: {msg.satellites_visible} | Fix Type: {msg.fix_type}"
                    )

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping telemetry collection.")
    except Exception as e:
        print(f"\nConnection Error: {e}")


if __name__ == "__main__":
    read_telemetry()
