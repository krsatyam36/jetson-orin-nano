#!/usr/bin/env bash
#
# download_models.sh
#
# Downloads YOLO model weights from Ultralytics into the models/ directory.
# These files are gitignored, so you must run this once after cloning.
#
# Usage:
#   make download-models
#   # or directly:
#   bash scripts/download_models.sh
#
# Required models (what each script uses):
#   yolov8n.pt  → vision/usb_stream.py, vision/usb_tracker.py, drone/corridor_scout.py
#   yolov8s.pt  → (optional, higher accuracy)
#   yolov8m.pt  → (optional, medium精度)
#   yolo11n.pt  → (optional, newer architecture)
#

set -euo pipefail

MODELS_DIR="$(cd "$(dirname "$0")/../models" && pwd)"
mkdir -p "$MODELS_DIR"

echo "[*] Downloading YOLO models to $MODELS_DIR"
echo ""

download() {
    local name="$1"
    local url="$2"
    local dest="$MODELS_DIR/$name"

    if [[ -f "$dest" ]]; then
        echo "   [SKIP] $name already exists"
        return
    fi

    echo "   [FETCH] $name ..."
    wget -q --show-progress "$url" -O "$dest" && echo "   [OK] $name" || echo "   [FAIL] $name"
}

download "yolov8n.pt" "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8n.pt"
download "yolov8s.pt" "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8s.pt"
download "yolov8m.pt" "https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8m.pt"
download "yolo11n.pt" "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt"

echo ""
echo "[*] Done. Models available:"
ls -1h "$MODELS_DIR"/*.pt 2>/dev/null || echo "   (no .pt files)"
