#!/usr/bin/env bash
# Start both host vision services in the background with one command.
# Neither service needs a model variable: vision_node / vision_eval select the
# detector and depth backends with POST /configure when they run.
# For service-specific flags (e.g. --device cpu), call the individual scripts.
# Stop both with ./scripts/stop_host_vision_services.sh.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
"$REPO_ROOT/scripts/start_yolo_service.sh"
"$REPO_ROOT/scripts/start_stereo_service.sh"
