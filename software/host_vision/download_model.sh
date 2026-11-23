#!/usr/bin/env bash
# Download the YOLO-world weights on the computer (not in the ROS container).
# Usage: software/host_vision/download_model.sh [target.pt]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TARGET="${1:-${YOLO_MODEL_PATH:-$SCRIPT_DIR/models/yolov8s-world.pt}}"
URL="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8s-world.pt"

mkdir -p "$(dirname "$TARGET")"
if [ -s "$TARGET" ]; then
    echo "YOLO weights already present: $TARGET"
else
    echo "Downloading YOLO-world weights to $TARGET"
    curl -fsSL -o "$TARGET" "$URL"
    test -s "$TARGET"
fi
echo "$TARGET"
