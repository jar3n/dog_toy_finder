#!/usr/bin/env bash
# Stop both background host vision services with one command.
# See scripts/stop_yolo_service.sh and scripts/stop_stereo_service.sh.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
"$REPO_ROOT/scripts/stop_yolo_service.sh"
"$REPO_ROOT/scripts/stop_stereo_service.sh"
