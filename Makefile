SHELL := /bin/bash

.PHONY: install install-dev download-models \
        stream-csi stream-usb stream-track camera-test \
        telemetry scout dashboard \
        clean

## ─── Setup ─────────────────────────────────────────────────────────────────

install:           ## Install Python dependencies
	pip install -r requirements.txt

install-dev:       ## Install in editable mode (for development)
	pip install -e .

download-models:   ## Download YOLO model weights
	chmod +x scripts/download_models.sh
	./scripts/download_models.sh

## ─── Vision / Camera ────────────────────────────────────────────────────────

stream-csi:        ## CSI camera → MJPEG stream (port 5000)
	python vision/csi_stream.py

stream-usb:        ## USB camera + YOLO instant-lock (port 5000)
	python vision/usb_stream.py

stream-track:      ## USB camera + YOLO 60s timer lock (port 5000)
	python vision/usb_tracker.py

camera-test:       ## Serial camera connectivity test
	python vision/camera_test.py

## ─── Drone / Pixhawk ────────────────────────────────────────────────────────

telemetry:         ## Live MAVLink telemetry readout
	python drone/telemetry.py

scout:             ## Autonomous corridor scout mission
	python drone/corridor_scout.py

dashboard:         ## Simulated telemetry dashboard (port 5000)
	python drone/dashboard/app.py

## ─── Housekeeping ───────────────────────────────────────────────────────────

clean:             ## Remove Python cache files
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name '*.pyc' -delete 2>/dev/null || true
