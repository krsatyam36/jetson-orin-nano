SHELL := /bin/bash

.PHONY: install install-dev download-models \
        stream-csi stream-usb stream-track camera-test \
        telemetry scout follow dashboard health \
        tlog-parse test lint pre-commit docker clean

## ─── Setup ─────────────────────────────────────────────────────────────────

install:           ## Install Python dependencies
	pip install -r requirements.txt

install-dev:       ## Install in editable mode (for development)
	pip install -e .
	pre-commit install

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

telemetry:         ## Live MAVLink telemetry readout (port 9090 health)
	python drone/telemetry.py

scout:             ## Autonomous corridor scout mission (ports 8000 video + 9090 health)
	python drone/corridor_scout.py

scout-custom:      ## Scout with custom parameters
	python drone/corridor_scout.py --alt 20 --length 150 --width 50 --lane 12

follow:            ## Person-follow mode (port 5001 video)
	python drone/follow.py

dashboard:         ## Simulated telemetry dashboard (port 5000)
	python drone/dashboard/app.py

health:            ## Standalone health endpoint (port 9090)
	python drone/health.py --port 9090

## ─── Utilities ──────────────────────────────────────────────────────────────

tlog-parse:        ## Parse MAVLink .tlog file (usage: make tlog-parse FILE=logs/mav.tlog)
	python scripts/tlog_parser.py $(FILE)

tlog-csv:          ## Export .tlog to CSV (usage: make tlog-csv FILE=logs/mav.tlog OUT=flight.csv)
	python scripts/tlog_parser.py $(FILE) --csv $(OUT)

## ─── Quality ────────────────────────────────────────────────────────────────

test:              ## Run pytest suite
	python -m pytest tests/ -v --cov=. --cov-report=term-missing

lint:              ## Run flake8 linter
	flake8 . --max-line-length=120 --ignore=E203,W503 --exclude=.git,__pycache__,tests

format-check:      ## Check formatting with black (dry run)
	black --check --diff .

format:            ## Auto-format with black
	black .

pre-commit:        ## Run all pre-commit hooks
	pre-commit run --all-files

## ─── Docker ─────────────────────────────────────────────────────────────────

docker-build:      ## Build Docker image
	docker build -t jetson-drone-project .

docker-run:        ## Run health endpoint in Docker
	docker run -d --name scout-health --restart unless-stopped -p 9090:9090 jetson-drone-project

## ─── Housekeeping ───────────────────────────────────────────────────────────

clean:             ## Remove Python cache files
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name '*.pyc' -delete 2>/dev/null || true
