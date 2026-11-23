#!/usr/bin/env bash
# Start the host-side YOLO-world detector service used by the open_vocab pipeline.
#
# The ROS 2 container only sends JPEGs over HTTP; torch, ultralytics, and the
# weights live here on the computer. The first run creates a local venv and
# downloads the weights, then execs the service in the foreground. Extra
# arguments are forwarded to software/host_vision/yolo_service.py, e.g.:
#   ./scripts/start_yolo_service.sh --port 5007 --device cpu
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HOST_VISION="$REPO_ROOT/software/host_vision"
VENV="$HOST_VISION/.venv"
MODEL="${YOLO_MODEL_PATH:-$HOST_VISION/models/yolov8s-world.pt}"

if [ ! -x "$VENV/bin/python" ]; then
    echo "Creating host vision venv at $VENV"
    python3 -m venv "$VENV"
fi

if ! "$VENV/bin/python" -c 'import ultralytics, PIL, yaml' >/dev/null 2>&1; then
    echo "Installing host vision dependencies (CPU torch + ultralytics)"
    "$VENV/bin/pip" install --no-cache-dir torch \
        --index-url https://download.pytorch.org/whl/cpu
    "$VENV/bin/pip" install --no-cache-dir -r "$HOST_VISION/requirements.txt"
fi

if [ ! -s "$MODEL" ]; then
    YOLO_MODEL_PATH="$MODEL" "$HOST_VISION/download_model.sh" "$MODEL"
fi

export YOLO_MODEL_PATH="$MODEL"
exec "$VENV/bin/python" "$HOST_VISION/yolo_service.py" "$@"
