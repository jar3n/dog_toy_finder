#!/usr/bin/env bash
# Start the host-side open-vocabulary detector service in the background.
#
# No DETECTOR_MODEL variable is needed: the service starts unconfigured, and the
# ROS client (vision_node or vision_eval) selects the backend with POST
# /configure. All detector weights are prepared here on first run so any backend
# can be selected later:
#   yolo_world     yolov8s-world.pt       (Ultralytics YOLO-World)
#   yoloe_26       yoloe-26s-seg.pt       (Ultralytics YOLOE-26, text prompts)
#   grounding_dino IDEA-Research/grounding-dino-tiny (Hugging Face transformers)
#
# Extra arguments are forwarded to yolo_service.py, e.g.
#   ./scripts/start_yolo_service.sh --device cpu
# Stop it with ./scripts/stop_yolo_service.sh (or stop both host services with
# ./scripts/stop_host_vision_services.sh).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HOST_VISION="$REPO_ROOT/software/host_vision"
VENV="$HOST_VISION/.venv"
MODELS="$HOST_VISION/models"
PID_FILE="/tmp/dtf_yolo_service.pid"
LOG_FILE="/tmp/dtf_yolo_service.log"
PORT=5006

previous=""
for argument in "$@"; do
    if [ "$previous" = "--port" ]; then
        PORT="$argument"
    fi
    previous="$argument"
done

mkdir -p "$MODELS"
# YOLOE downloads its CLIP text encoder into the current directory on first
# set_classes(); keep it beside the weights. Grounding DINO uses HF_HOME.
cd "$MODELS"
export HF_HOME="${HF_HOME:-$MODELS/hf}"
mkdir -p "$HF_HOME"

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

echo "Preparing detector weights (one-time download if missing)"
YOLO_MODEL_PATH="$MODELS/yolov8s-world.pt" \
    "$HOST_VISION/download_model.sh" "$MODELS/yolov8s-world.pt" \
    "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8s-world.pt" >/dev/null
YOLOE_MODEL_PATH="$MODELS/yoloe-26s-seg.pt" \
    "$HOST_VISION/download_model.sh" "$MODELS/yoloe-26s-seg.pt" \
    "https://github.com/ultralytics/assets/releases/download/v8.4.0/yoloe-26s-seg.pt" >/dev/null
GROUNDING_DINO_MODEL="IDEA-Research/grounding-dino-tiny" \
    "$VENV/bin/python" - <<'PY' >/dev/null
import os
from huggingface_hub import snapshot_download
snapshot_download(os.environ["GROUNDING_DINO_MODEL"])
PY

# The service resolves default weights from these per-backend variables, so no
# model needs to be chosen when the process starts.
export YOLO_MODEL_PATH="$MODELS/yolov8s-world.pt"
export YOLOE_MODEL_PATH="$MODELS/yoloe-26s-seg.pt"
export GROUNDING_DINO_MODEL="IDEA-Research/grounding-dino-tiny"

if [ -s "$PID_FILE" ]; then
    old_pid="$(cat "$PID_FILE")"
    if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null \
            && tr '\0' ' ' < "/proc/$old_pid/cmdline" 2>/dev/null \
                | grep -q 'yolo_service.py'; then
        echo "Detector service is already running (pid $old_pid)"
        echo "Use ./scripts/stop_yolo_service.sh to stop it first."
        exit 1
    fi
    rm -f "$PID_FILE"
fi

nohup "$VENV/bin/python" "$HOST_VISION/yolo_service.py" "$@" \
    > "$LOG_FILE" 2>&1 &
pid=$!
echo "$pid" > "$PID_FILE"

ready=0
for _ in $(seq 1 50); do
    if curl -fsS --max-time 1 "http://127.0.0.1:$PORT/health" >/dev/null 2>&1; then
        ready=1
        break
    fi
    if ! kill -0 "$pid" 2>/dev/null; then
        break
    fi
    sleep 0.2
done
if [ "$ready" -ne 1 ]; then
    echo "Detector service failed to start; last log lines:" >&2
    tail -n 20 "$LOG_FILE" >&2 || true
    rm -f "$PID_FILE"
    exit 1
fi

echo "Detector service started (pid $pid): http://127.0.0.1:$PORT"
echo "No detector selected; clients send POST /configure."
echo "Log: $LOG_FILE"
