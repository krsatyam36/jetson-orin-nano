#!/bin/bash
# Navigate to project folder
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"

# Start the Python backend in the background
python3 app.py &

# Wait 5 seconds for the server to initialize
sleep 5

# Open Chromium in Kiosk mode pointing to localhost
chromium-browser --kiosk http://127.0.0.1:5000
