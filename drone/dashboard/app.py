"""
Scout Drone Telemetry Dashboard (Simulated)

A Flask-SocketIO web dashboard that simulates real-time drone telemetry
data for a quadcopter (Scout Drone, ~2.8 kg TOW) and displays it in a
browser at http://<jetson-ip>:5000.

Simulated data for each of 4 motors:
  - RPM = Voltage × KV_RATING (935) × 0.85 efficiency
  - Current draw (12-17.5 A)
  - Thrust estimate based on 10" prop formula
  - Voltage (simulated 4S LiPo: 15.5-16.5 V)
  - Direction (CW / CCW)

Data is emitted via WebSocket every 0.5s. Opens browser automatically
on start.

Usage:
  python app.py
  Then open http://<jetson-ip>:5000 in a browser.
"""

import eventlet
eventlet.monkey_patch()

from flask import Flask, render_template
from flask_socketio import SocketIO
import random
import threading
import time
import webbrowser # Trigger the browser automatically

app = Flask(__name__)
socketio = SocketIO(app, async_mode='eventlet')

# Scout Drone Specs
KV_RATING = 935
POLES = 14

def simulate_drone_data():
    """Generates mock telemetry for the Scout Drone (2.8kg TOW)"""
    while True:
        # Simulate a 4S battery (14.8V - 16.5V)
        voltage = round(random.uniform(15.5, 16.5), 2)
        motor_data = []
        
        for i in range(1, 5):
            # Realistic current for Emax 2213 under high load
            current = round(random.uniform(12.0, 17.5), 2)
            # RPM = Voltage * KV * Efficiency factor
            rpm = int(voltage * KV_RATING * 0.85) 
            # Thrust calculation based on RPM for 10-inch props
            thrust_grams = int((rpm / 1000)**2 * 11) 
            
            motor_data.append({
                "id": i,
                "rpm": rpm,
                "voltage": voltage,
                "current": current,
                "thrust": thrust_grams,
                "direction": "CW" if i in [3, 4] else "CCW"
            })
        
        socketio.emit('telemetry_update', motor_data)
        time.sleep(0.5)

@app.route('/')
def index():
    """Renders the dashboard template"""
    return render_template('template.html')

if __name__ == '__main__':
    # 1. Start simulation thread
    threading.Thread(target=simulate_drone_data, daemon=True).start()
    
    # 2. Automatically open the browser on the Jetson Orin
    # This happens only when you run the script
    webbrowser.open("http://127.0.0.1:5000")
    
    # 3. Run using socketio to avoid Eventlet RuntimeErrors
    socketio.run(app, host='0.0.0.0', port=5000, debug=True, log_output=True)
