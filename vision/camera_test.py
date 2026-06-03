"""
Serial Camera Raw Data Dump (Test Script)

Opens a serial connection to /dev/ttyACM0 at 115200 baud and continuously
prints any incoming raw data line-by-line. Used to verify that a serial
camera module is connected and transmitting.

Usage:
  python camera_test.py
  Press Ctrl+C to stop.
"""

import serial
import time

# Try to open the serial port
# Note: It might be ttyACM0 or ttyACM1. Check ls /dev/ttyACM* if this fails.
try:
    ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
    print("Success: Serial port opened on /dev/ttyACM0")
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit()

print("Reading data from camera (Press Ctrl+C to stop)...")

try:
    while True:
        if ser.in_waiting > 0:
            # Read a line of data
            data = ser.readline()
            # Print it so we know it's alive
            print(f"Raw Data: {data}")
        else:
            time.sleep(0.1)
except KeyboardInterrupt:
    print("\nExiting...")
    ser.close()
