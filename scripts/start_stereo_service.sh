#!/usr/bin/env bash
# Start the host-side dense-stereo depth service used by the stereo-net pipeline.
#
# One depth model runs at a time, chosen with DEPTH_MODEL (or pass --model):
#   lite_any_stereo          Lite Any Stereo V2, small (LAS2_S.pth)
#   fast_foundation_stereo   NVIDIA Fast-FoundationStereo 23-36-37
#   rt_igevpp                IGEV++ real-time network (core_rt, 6 iterations)
#
# The ROS container only sends JPEGs and sample coordinates over HTTP. Torch,
# the upstream GitHub repositories, and the weights live here on the computer.
# The first run creates a local venv, installs dependencies, clones the selected
# repository over HTTPS, and downloads its weights from Hugging Face. Extra
# arguments are forwarded to stereo_service.py, e.g.:
#   DEPTH_MODEL=rt_igevpp ./scripts/start_stereo_service.sh --device cpu
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HOST_VISION="$REPO_ROOT/software/host_vision"
VENV="$HOST_VISION/.venv"
MODELS="$HOST_VISION/models"
REPOS="$MODELS/repos"
DEPTH_MODEL="${DEPTH_MODEL:-lite_any_stereo}"

mkdir -p "$MODELS" "$REPOS"
export HF_HOME="${HF_HOME:-$MODELS/hf}"
mkdir -p "$HF_HOME"

case "$DEPTH_MODEL" in
    lite_any_stereo)
        REPO_URL="https://github.com/TomTomTommi/LiteAnyStereo.git"
        REPO_DIR="${LITE_ANY_STEREO_REPO:-$REPOS/LiteAnyStereo}"
        HF_REPO="tomtomtommi/LiteAnyStereoV2"
        HF_FILE="LAS2_S.pth"
        TARGET="${LITE_ANY_STEREO_MODEL_PATH:-$MODELS/LAS2_S.pth}"
        ;;
    fast_foundation_stereo)
        REPO_URL="https://github.com/NVlabs/Fast-FoundationStereo.git"
        REPO_DIR="${FAST_FOUNDATION_STEREO_REPO:-$REPOS/Fast-FoundationStereo}"
        HF_REPO="Miayan/stereo-matching-weights"
        HF_FILE="foundationstereo/23-36-37/model_best_bp2_serialize.pth"
        TARGET="${FAST_FOUNDATION_STEREO_MODEL_PATH:-$MODELS/fast_foundation_stereo/model_best_bp2_serialize.pth}"
        EXTRA_FILE="foundationstereo/23-36-37/cfg.yaml"
        EXTRA_TARGET="$MODELS/fast_foundation_stereo/cfg.yaml"
        ;;
    rt_igevpp)
        REPO_URL="https://github.com/gangweiX/IGEV-plusplus.git"
        REPO_DIR="${IGEV_PLUSPLUS_REPO:-$REPOS/IGEV-plusplus}"
        HF_REPO="wdy413/igevplusplus"
        HF_FILE="sceneflow.pth"
        TARGET="${RT_IGEVPP_MODEL_PATH:-$MODELS/rt_igevpp/sceneflow.pth}"
        ;;
    *)
        echo "Unknown DEPTH_MODEL '$DEPTH_MODEL' (choose lite_any_stereo, fast_foundation_stereo, rt_igevpp)" >&2
        exit 2
        ;;
esac
export DEPTH_MODEL
export LITE_ANY_STEREO_REPO="$REPO_DIR"
export FAST_FOUNDATION_STEREO_REPO="$REPO_DIR"
export IGEV_PLUSPLUS_REPO="$REPO_DIR"

if [ ! -d "$REPO_DIR/.git" ]; then
    echo "Cloning $DEPTH_MODEL source into $REPO_DIR"
    git clone --depth 1 "$REPO_URL" "$REPO_DIR"
fi

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

if [ ! -s "$TARGET" ]; then
    echo "Downloading $DEPTH_MODEL weights from $HF_REPO/$HF_FILE"
    "$VENV/bin/python" - "$HF_REPO" "$HF_FILE" "$TARGET" "${EXTRA_FILE:-}" "${EXTRA_TARGET:-}" <<'PY'
import os
import shutil
import sys

from huggingface_hub import hf_hub_download

repo, filename, target, extra_file, extra_target = sys.argv[1:6]
os.makedirs(os.path.dirname(target), exist_ok=True)
shutil.copyfile(hf_hub_download(repo_id=repo, filename=filename), target)
if extra_file and extra_target:
    os.makedirs(os.path.dirname(extra_target), exist_ok=True)
    shutil.copyfile(hf_hub_download(repo_id=repo, filename=extra_file),
                    extra_target)
PY
fi

case "$DEPTH_MODEL" in
    lite_any_stereo) export LITE_ANY_STEREO_MODEL_PATH="$TARGET" ;;
    fast_foundation_stereo) export FAST_FOUNDATION_STEREO_MODEL_PATH="$TARGET" ;;
    rt_igevpp) export RT_IGEVPP_MODEL_PATH="$TARGET" ;;
esac

exec "$VENV/bin/python" "$HOST_VISION/stereo_service.py" "$@"
