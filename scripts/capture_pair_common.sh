#!/usr/bin/env bash
# Shared helpers for the calibration pair capture scripts.
#
# Source this file; do not execute it directly.

DTF_CAPTURE_POSE_SETS=(
  Distance Distance Distance Distance
  Angle Angle Angle Angle Angle Angle Angle Angle
  Offset Offset Offset
)
DTF_CAPTURE_POSE_DISTANCES=(
  0.6 0.8 1.0 1.5
  1.0 1.0 1.0 1.0 1.0 1.0 1.0 1.0
  1.0 1.0 1.0
)
DTF_CAPTURE_POSE_OFFSETS=(
  centered centered centered centered
  centered centered centered centered centered centered centered centered
  "0.20 m left" "0.20 m right" "0.20 m up"
)
DTF_CAPTURE_POSE_YAWS=(
  0 0 0 0
  +20 -20 0 0 +20 -20 +20 -20
  0 0 0
)
DTF_CAPTURE_POSE_PITCHES=(
  0 0 0 0
  0 0 +15 -15 -15 +15 +15 -15
  0 0 0
)

: "${DTF_VIEWER_PID:=}"
: "${DTF_PREVIEW_TMP:=}"

dtf_pose_values() {
  local pair="$1"
  if (( pair >= 1 && pair <= ${#DTF_CAPTURE_POSE_SETS[@]} )); then
    local i=$((pair - 1))
    printf '%s\t%s\t%s\t%s\t%s\n' \
      "${DTF_CAPTURE_POSE_SETS[i]}" \
      "${DTF_CAPTURE_POSE_DISTANCES[i]}" \
      "${DTF_CAPTURE_POSE_OFFSETS[i]}" \
      "${DTF_CAPTURE_POSE_YAWS[i]}" \
      "${DTF_CAPTURE_POSE_PITCHES[i]}"
  else
    printf 'Additional\t1.0\tcentered\t0\t0\n'
  fi
}

dtf_format_pose() {
  local set="$1"
  local distance="$2"
  local offset="$3"
  local yaw="$4"
  local pitch="$5"
  printf '%s %s m, %s, yaw %s°, pitch %s°' \
    "$set" "$distance" "$offset" "$yaw" "$pitch"
}

dtf_pose_label() {
  local set distance offset yaw pitch
  IFS=$'\t' read -r set distance offset yaw pitch \
    < <(dtf_pose_values "$1")
  dtf_format_pose "$set" "$distance" "$offset" "$yaw" "$pitch"
}

dtf_capture_one() {
  local url="$1"
  local dest="$2"
  local tmp="${dest}.tmp"

  if curl -fsS --connect-timeout 5 --max-time 15 "$url" -o "$tmp" &&
     [[ -s "$tmp" ]]; then
    mv -f "$tmp" "$dest"
    return 0
  fi
  rm -f "$tmp"
  return 1
}

dtf_make_preview() {
  local preview_python="$1"
  local left="$2"
  local right="$3"
  local dest="$4"

  "$preview_python" - "$left" "$right" "$dest" <<'PY'
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

dtf_preview_available() {
  local preview_python="$1"
  [[ "${DTF_CALIB_PREVIEW:-1}" != "0" &&
     ( -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ) ]] &&
    "$preview_python" -c 'import cv2' >/dev/null 2>&1
}

dtf_start_viewer() {
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
  DTF_VIEWER_PID=$!
}

dtf_stop_viewer() {
  if [[ -n "$DTF_VIEWER_PID" ]]; then
    kill "$DTF_VIEWER_PID" 2>/dev/null || true
    wait "$DTF_VIEWER_PID" 2>/dev/null || true
    DTF_VIEWER_PID=""
  fi
}

dtf_cleanup_preview() {
  dtf_stop_viewer
  if [[ -n "$DTF_PREVIEW_TMP" && -d "$DTF_PREVIEW_TMP" ]]; then
    rm -rf -- "$DTF_PREVIEW_TMP"
  fi
}
