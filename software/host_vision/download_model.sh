#!/usr/bin/env bash
# Download open-vocabulary detector weights on the computer (not in Docker).
# Usage: download_model.sh [target.pt] [url]
# With no arguments it downloads the YOLO-World small checkpoint used by the
# yolo_world detector. The service start script passes both explicitly for
# yolo_world and yoloe_26; Grounding DINO is fetched via Hugging Face instead.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TARGET="${1:-${YOLO_MODEL_PATH:-$SCRIPT_DIR/models/yolov8s-world.pt}}"
URL="${2:-https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8s-world.pt}"

mkdir -p "$(dirname "$TARGET")"
if [ -s "$TARGET" ]; then
    echo "Detector weights already present: $TARGET"
else
    echo "Downloading detector weights to $TARGET"
    curl -fsSL -o "$TARGET" "$URL"
    test -s "$TARGET"
fi
echo "$TARGET"
