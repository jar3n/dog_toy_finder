#!/usr/bin/env bash
# Start the host-side dense-stereo depth service in the background.
#
# No DEPTH_MODEL variable is needed: the service starts unconfigured, and the
# ROS client (vision_node or vision_eval) selects the backend with POST
# /configure. All three source repositories and checkpoints are prepared here on
# first run so any backend can be selected later:
#   lite_any_stereo          LAS2_S.pth
#   fast_foundation_stereo   model_best_bp2_serialize.pth + cfg.yaml
#   rt_igevpp                sceneflow.pth
#
# fast_foundation_stereo uses @torch.compile; TorchInductor needs Python.h from
# the host's python3-devel/python3-dev package for its first inference compile.
#
# The ROS container only sends JPEGs and sample coordinates over HTTP. Torch,
# the upstream GitHub repositories, and the weights live here on the computer.
# Extra arguments are forwarded to stereo_service.py, e.g.
#   ./scripts/start_stereo_service.sh --device cpu
# Stop it with ./scripts/stop_stereo_service.sh (or stop both host services with
# ./scripts/stop_host_vision_services.sh).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HOST_VISION="$REPO_ROOT/software/host_vision"
VENV="$HOST_VISION/.venv"
MODELS="$HOST_VISION/models"
REPOS="$MODELS/repos"
PID_FILE="/tmp/dtf_stereo_service.pid"
LOG_FILE="/tmp/dtf_stereo_service.log"
PORT=5007

previous=""
for argument in "$@"; do
    if [ "$previous" = "--port" ]; then
        PORT="$argument"
    fi
    previous="$argument"
done

mkdir -p "$MODELS" "$REPOS"
export HF_HOME="${HF_HOME:-$MODELS/hf}"
mkdir -p "$HF_HOME"

if [ ! -x "$VENV/bin/python" ]; then
    echo "Creating host vision venv at $VENV"
    python3 -m venv "$VENV"
fi

if ! "$VENV/bin/python" -c 'import torch, torchvision' >/dev/null 2>&1; then
    echo "Installing CPU torch/torchvision (replace with your CUDA build if you have a GPU)"
    # Keep torchvision on the same index as torch; the PyPI build breaks NMS.
    "$VENV/bin/pip" install --no-cache-dir \
        'torch==2.15.0+cpu' 'torchvision==0.29.2+cpu' \
        --index-url https://download.pytorch.org/whl/cpu
fi
if ! "$VENV/bin/python" -c 'import huggingface_hub, omegaconf, einops, cv2' >/dev/null 2>&1; then
    echo "Installing host dense-stereo dependencies"
    "$VENV/bin/pip" install --no-cache-dir -r "$HOST_VISION/requirements.txt"
fi

echo "Preparing dense-stereo sources and checkpoints (one-time download if missing)"

clone_once() {
    local url="$1" dir="$2"
    if [ ! -d "$dir/.git" ]; then
        echo "Cloning $url into $dir"
        git clone --depth 1 "$url" "$dir"
    fi
}

clone_once https://github.com/TomTomTommi/LiteAnyStereo.git "$REPOS/LiteAnyStereo"
clone_once https://github.com/NVlabs/Fast-FoundationStereo.git "$REPOS/Fast-FoundationStereo"
clone_once https://github.com/gangweiX/IGEV-plusplus.git "$REPOS/IGEV-plusplus"

fetch_hf() {
    "$VENV/bin/python" - "$@" <<'PY'
import os
import shutil
import sys

from huggingface_hub import hf_hub_download

repo, filename, target = sys.argv[1:4]
os.makedirs(os.path.dirname(target), exist_ok=True)
shutil.copyfile(hf_hub_download(repo_id=repo, filename=filename), target)
for index in range(4, len(sys.argv), 2):
    extra_file, extra_target = sys.argv[index:index + 2]
    os.makedirs(os.path.dirname(extra_target), exist_ok=True)
    shutil.copyfile(hf_hub_download(repo_id=repo, filename=extra_file),
                    extra_target)
PY
}

if [ ! -s "$MODELS/LAS2_S.pth" ]; then
    echo "Downloading lite_any_stereo weights"
    fetch_hf tomtomtommi/LiteAnyStereoV2 LAS2_S.pth "$MODELS/LAS2_S.pth"
fi
if [ ! -s "$MODELS/fast_foundation_stereo/model_best_bp2_serialize.pth" ] \
        || [ ! -s "$MODELS/fast_foundation_stereo/cfg.yaml" ]; then
    echo "Downloading fast_foundation_stereo weights and config"
    fetch_hf Miayan/stereo-matching-weights \
        foundationstereo/23-36-37/model_best_bp2_serialize.pth \
        "$MODELS/fast_foundation_stereo/model_best_bp2_serialize.pth" \
        foundationstereo/23-36-37/cfg.yaml \
        "$MODELS/fast_foundation_stereo/cfg.yaml"
fi
if [ ! -s "$MODELS/rt_igevpp/sceneflow.pth" ]; then
    echo "Downloading rt_igevpp weights"
    fetch_hf wdy413/igevplusplus sceneflow.pth "$MODELS/rt_igevpp/sceneflow.pth"
fi

# The service resolves default weights and source paths from these variables,
# so no model needs to be chosen when the process starts.
export LITE_ANY_STEREO_REPO="$REPOS/LiteAnyStereo"
export FAST_FOUNDATION_STEREO_REPO="$REPOS/Fast-FoundationStereo"
export IGEV_PLUSPLUS_REPO="$REPOS/IGEV-plusplus"
export LITE_ANY_STEREO_MODEL_PATH="$MODELS/LAS2_S.pth"
export FAST_FOUNDATION_STEREO_MODEL_PATH="$MODELS/fast_foundation_stereo/model_best_bp2_serialize.pth"
export RT_IGEVPP_MODEL_PATH="$MODELS/rt_igevpp/sceneflow.pth"

if [ -s "$PID_FILE" ]; then
    old_pid="$(cat "$PID_FILE")"
    if [ -n "$old_pid" ] && kill -0 "$old_pid" 2>/dev/null \
            && tr '\0' ' ' < "/proc/$old_pid/cmdline" 2>/dev/null \
                | grep -q 'stereo_service.py'; then
        echo "Stereo service is already running (pid $old_pid)"
        echo "Use ./scripts/stop_stereo_service.sh to stop it first."
        exit 1
    fi
    rm -f "$PID_FILE"
fi

nohup "$VENV/bin/python" "$HOST_VISION/stereo_service.py" "$@" \
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
    echo "Stereo service failed to start; last log lines:" >&2
    tail -n 20 "$LOG_FILE" >&2 || true
    rm -f "$PID_FILE"
    exit 1
fi

echo "Stereo depth service started (pid $pid): http://127.0.0.1:$PORT"
echo "No depth model selected; clients send POST /configure."
echo "Log: $LOG_FILE"
