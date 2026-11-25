#!/usr/bin/env bash
# Capture one left/right checkerboard pair at a time, waiting for Enter.
set -Eeuo pipefail

usage() {
  cat <<EOF
Usage: $0 [pair_count] [output_dir]

Prompts before each pair, captures both cameras, and shows a temporary
side-by-side preview for confirmation.

Defaults: 15 pairs, /tmp/dtf_calib
Camera URLs:
  DTF_LEFT_CAPTURE_URL   (default http://esp32cam-left.local/capture)
  DTF_RIGHT_CAPTURE_URL  (default http://esp32cam-right.local/capture)
Set DTF_CALIB_PREVIEW=0 to disable the preview.

At the capture prompt, press Enter to capture or q to quit.
At the preview prompt, press Enter to accept, r to recapture, or q to quit.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

COUNT="${1:-15}"
OUTDIR="${2:-/tmp/dtf_calib}"
LEFT_URL="${DTF_LEFT_CAPTURE_URL:-http://esp32cam-left.local/capture}"
RIGHT_URL="${DTF_RIGHT_CAPTURE_URL:-http://esp32cam-right.local/capture}"

if [[ ! "$COUNT" =~ ^[1-9][0-9]*$ ]]; then
  printf 'error: pair_count must be a positive integer\n' >&2
  exit 2
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREVIEW_PYTHON="$REPO_ROOT/software/host_vision/.venv/bin/python"
[[ -x "$PREVIEW_PYTHON" ]] || PREVIEW_PYTHON="python3"
PREVIEW_TMP="$(mktemp -d)"
VIEWER_PID=""

POSE_LABELS=(
  "Distance 0.6 m, centered, yaw 0°, pitch 0°"
  "Distance 0.8 m, centered, yaw 0°, pitch 0°"
  "Distance 1.0 m, centered, yaw 0°, pitch 0°"
  "Distance 1.5 m, centered, yaw 0°, pitch 0°"
  "Angle 1.0 m, centered, yaw +20°, pitch 0°"
  "Angle 1.0 m, centered, yaw -20°, pitch 0°"
  "Angle 1.0 m, centered, yaw 0°, pitch +15°"
  "Angle 1.0 m, centered, yaw 0°, pitch -15°"
  "Angle 1.0 m, centered, yaw +20°, pitch -15°"
  "Angle 1.0 m, centered, yaw -20°, pitch +15°"
  "Angle 1.0 m, centered, yaw +20°, pitch +15°"
  "Angle 1.0 m, centered, yaw -20°, pitch -15°"
  "Offset 1.0 m, 0.20 m left, yaw 0°, pitch 0°"
  "Offset 1.0 m, 0.20 m right, yaw 0°, pitch 0°"
  "Offset 1.0 m, 0.20 m up, yaw 0°, pitch 0°"
)

capture_one() {
  local url="$1"
  local dest="$2"
  local tmp="${dest}.tmp"

  if curl -fsS --connect-timeout 5 --max-time 15 "$url" -o "$tmp" && [[ -s "$tmp" ]]; then
    mv -f "$tmp" "$dest"
    return 0
  fi
  rm -f "$tmp"
  return 1
}

make_preview() {
  local left="$1"
  local right="$2"
  local dest="$3"

  "$PREVIEW_PYTHON" - "$left" "$right" "$dest" <<'PY'
import sys

import cv2
import numpy as np

left = cv2.imread(sys.argv[1])
right = cv2.imread(sys.argv[2])
if left is None or right is None:
    sys.exit("could not read one or both captures")


def fit(image):
    height, width = image.shape[:2]
    scale = min(1.0, 640.0 / width, 480.0 / height)
    if scale < 1.0:
        image = cv2.resize(image, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA)
    return image


left = fit(left)
right = fit(right)
height = max(left.shape[0], right.shape[0])
separator = 8
header = 32
canvas = np.full(
    (height + header, left.shape[1] + separator + right.shape[1], 3),
    35, dtype=np.uint8)
canvas[header:header + left.shape[0], :left.shape[1]] = left
right_x = left.shape[1] + separator
canvas[header:header + right.shape[0], right_x:right_x + right.shape[1]] = right
cv2.putText(canvas, "LEFT", (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (255, 255, 255), 2, cv2.LINE_AA)
cv2.putText(canvas, "RIGHT", (right_x + 10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (255, 255, 255), 2, cv2.LINE_AA)
if not cv2.imwrite(sys.argv[3], canvas):
    sys.exit("could not write preview image")
PY
}

preview_available() {
  [[ "${DTF_CALIB_PREVIEW:-1}" != "0" &&
     ( -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ) ]] &&
    "$PREVIEW_PYTHON" -c 'import cv2' >/dev/null 2>&1
}

start_viewer() {
  local image="$1"
  if command -v feh >/dev/null 2>&1; then
    (
      trap - EXIT
      exec feh --hide-pointer --title "DTF calibration pair" "$image"
    ) >/dev/null 2>&1 &
  elif command -v display >/dev/null 2>&1; then
    (
      trap - EXIT
      exec display "$image"
    ) >/dev/null 2>&1 &
  elif command -v xdg-open >/dev/null 2>&1; then
    (
      trap - EXIT
      exec xdg-open "$image"
    ) >/dev/null 2>&1 &
  else
    return 1
  fi
  VIEWER_PID=$!
}

stop_viewer() {
  if [[ -n "$VIEWER_PID" ]]; then
    kill "$VIEWER_PID" 2>/dev/null || true
    wait "$VIEWER_PID" 2>/dev/null || true
    VIEWER_PID=""
  fi
}

cleanup() {
  stop_viewer
  rm -rf "$PREVIEW_TMP"
}
trap cleanup EXIT

mkdir -p "$OUTDIR/left" "$OUTDIR/right"

pair=1
while (( pair <= COUNT )); do
  stem="$(printf '%02d' "$pair")"
  if (( pair <= ${#POSE_LABELS[@]} )); then
    label="${POSE_LABELS[pair - 1]}"
  else
    label="Additional pair"
  fi

  printf '\nPair %s/%s: %s\n' "$stem" "$COUNT" "$label"
  printf 'Set up this pose, then press Enter to capture (q to quit): '
  if ! IFS= read -r reply; then
    printf '\nNo input; stopping before pair %s.\n' "$stem" >&2
    exit 130
  fi
  case "$reply" in
    q|Q)
      printf '\nStopped before pair %s.\n' "$stem"
      exit 0
      ;;
    "")
      ;;
    *)
      printf 'Please press Enter to capture or q to quit.\n' >&2
      continue
      ;;
  esac

  if ! capture_one "$LEFT_URL" "$OUTDIR/left/$stem.jpg" ||
     ! capture_one "$RIGHT_URL" "$OUTDIR/right/$stem.jpg"; then
    printf 'capture failed; retrying pair %s.\n' "$stem" >&2
    continue
  fi
  printf 'saved %s/left/%s.jpg and %s/right/%s.jpg\n' \
    "$OUTDIR" "$stem" "$OUTDIR" "$stem"

  if preview_available; then
    preview_file="$PREVIEW_TMP/pair_${stem}.jpg"
    if make_preview "$OUTDIR/left/$stem.jpg" "$OUTDIR/right/$stem.jpg" \
        "$preview_file" && start_viewer "$preview_file"; then
      printf '\nPreview opened. Check both eyes: full board, correct square size, no blur or glare.\n'
      printf 'Press Enter to accept, r to recapture, q to quit: '
      if ! IFS= read -r confirm; then
        printf '\nNo input; stopping after pair %s.\n' "$stem" >&2
        stop_viewer
        exit 130
      fi
      stop_viewer
      case "$confirm" in
        q|Q)
          printf '\nStopped after pair %s.\n' "$stem"
          exit 0
          ;;
        r|R)
          printf '\nRecapturing pair %s.\n' "$stem"
          continue
          ;;
        "")
          ;;
        *)
          printf 'Treating response as accept.\n' >&2
          ;;
      esac
    else
      printf 'Preview unavailable; continuing without it.\n' >&2
    fi
  fi

  pair=$((pair + 1))
done

printf '\nCaptured %s pairs in %s.\n' "$COUNT" "$OUTDIR"
