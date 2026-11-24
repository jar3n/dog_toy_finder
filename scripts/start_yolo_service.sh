#!/usr/bin/env bash
# Start the host-side open-vocabulary detector service used by open_vocab.
#
# One detector runs at a time, chosen with DETECTOR_MODEL (or pass --detector):
#   yolo_world     yolov8s-world.pt       (Ultralytics YOLO-World)
#   yoloe_26       yoloe-26s-seg.pt       (Ultralytics YOLOE-26, text prompts)
#   grounding_dino IDEA-Research/grounding-dino-tiny (Hugging Face transformers)
#
# The ROS container only sends JPEGs over HTTP; torch, ultralytics,
# transformers, and the weights live here on the computer. The first run creates
# a local venv, installs the dependencies, and downloads the selected weights.
# Extra arguments are forwarded to yolo_service.py, e.g.:
#   DETECTOR_MODEL=yoloe_26 ./scripts/start_yolo_service.sh --device cpu
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HOST_VISION="$REPO_ROOT/software/host_vision"
VENV="$HOST_VISION/.venv"
MODELS="$HOST_VISION/models"
DETECTOR="${DETECTOR_MODEL:-yolo_world}"

mkdir -p "$MODELS"
# YOLOE downloads its CLIP text encoder into the current directory on first
# set_classes(); keep it beside the weights. Grounding DINO uses HF_HOME.
cd "$MODELS"
export HF_HOME="${HF_HOME:-$MODELS/hf}"
mkdir -p "$HF_HOME"

case "$DETECTOR" in
    yolo_world)
        MODEL="${YOLO_MODEL_PATH:-$MODELS/yolov8s-world.pt}"
        URL="https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8s-world.pt"
        LOCAL_WEIGHTS=1
        ;;
    yoloe_26)
        MODEL="${YOLOE_MODEL_PATH:-$MODELS/yoloe-26s-seg.pt}"
        URL="https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26s-seg.pt"
        LOCAL_WEIGHTS=1
        ;;
    grounding_dino)
        MODEL="${GROUNDING_DINO_MODEL:-IDEA-Research/grounding-dino-tiny}"
        LOCAL_WEIGHTS=0
        ;;
    *)
        echo "Unknown DETECTOR_MODEL '$DETECTOR' (choose yolo_world, yoloe_26, grounding_dino)" >&2
        exit 2
        ;;
esac
export DETECTOR_MODEL="$DETECTOR"

if [ ! -x "$VENV/bin/python" ]; then
    echo "Creating host vision venv at $VENV"
    python3 -m venv "$VENV"
fi

if ! "$VENV/bin/python" -c \
    'import torch, torchvision, ultralytics, transformers, PIL, yaml' >/dev/null 2>&1; then
    echo "Installing host vision dependencies (CPU torch + ultralytics + transformers)"
    # torchvision from PyPI and torch from the CPU index are not ABI-compatible;
    # install the matching pair from the same index or ultralytics' NMS fails.
    "$VENV/bin/pip" install --no-cache-dir \
        'torch==2.15.0+cpu' 'torchvision==0.29.2+cpu' \
        --index-url https://download.pytorch.org/whl/cpu
    "$VENV/bin/pip" install --no-cache-dir -r "$HOST_VISION/requirements.txt"
fi

if [ "$LOCAL_WEIGHTS" -eq 1 ]; then
    if [ ! -s "$MODEL" ]; then
        YOLO_MODEL_PATH="$MODEL" "$HOST_VISION/download_model.sh" "$MODEL" "$URL"
    fi
    export YOLO_MODEL_PATH="$MODEL"
    export YOLOE_MODEL_PATH="$MODEL"
else
    # Hugging Face resolves the model id to files under HF_HOME; download them
    # once here so the first detection request does not wait on the network.
    GROUNDING_DINO_MODEL="$MODEL" "$VENV/bin/python" - <<'PY'
import os
from huggingface_hub import snapshot_download
snapshot_download(os.environ["GROUNDING_DINO_MODEL"])
PY
    export GROUNDING_DINO_MODEL="$MODEL"
fi

exec "$VENV/bin/python" "$HOST_VISION/yolo_service.py" "$@"
