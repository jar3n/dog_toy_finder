#!/usr/bin/env bash
# Capture one numbered left/right calibration pair with preview confirmation.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/capture_pair_common.sh"

usage() {
  cat <<EOF
Usage: $0 [pair_number] [output_dir] [--distance METERS] [--yaw DEG] [--pitch DEG]

Prompts once, captures both cameras, and shows a temporary side-by-side
preview for confirmation. The pair number matches the pose table in
docs/setup/camera_calibration.md and overwrites that numbered JPEG pair.
The overrides change only the displayed pose; the pair number sets the
output filenames.

Defaults: pair 1, /tmp/dtf_calib
Camera URLs:
  DTF_LEFT_CAPTURE_URL   (default http://esp32cam-left.local/capture)
  DTF_RIGHT_CAPTURE_URL  (default http://esp32cam-right.local/capture)
Set DTF_CALIB_PREVIEW=0 to disable the preview.

Examples:
  $0 4 --distance 1.2
  $0 9 --yaw 15 --pitch -10

At the capture prompt, press Enter to capture or q to quit.
At the preview prompt, press Enter to accept, r to recapture, or q to quit.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

PAIR_RAW=""
OUTDIR=""
DISTANCE_OVERRIDE=""
YAW_OVERRIDE=""
PITCH_OVERRIDE=""
LEFT_URL="${DTF_LEFT_CAPTURE_URL:-http://esp32cam-left.local/capture}"
RIGHT_URL="${DTF_RIGHT_CAPTURE_URL:-http://esp32cam-right.local/capture}"

while (( $# > 0 )); do
  case "$1" in
    --distance|--dist)
      if (( $# < 2 )); then
        printf 'error: %s needs a distance in metres\n' "$1" >&2
        exit 2
      fi
      DISTANCE_OVERRIDE="$2"
      shift 2
      ;;
    --yaw)
      if (( $# < 2 )); then
        printf 'error: --yaw needs an angle in degrees\n' >&2
        exit 2
      fi
      YAW_OVERRIDE="$2"
      shift 2
      ;;
    --pitch)
      if (( $# < 2 )); then
        printf 'error: --pitch needs an angle in degrees\n' >&2
        exit 2
      fi
      PITCH_OVERRIDE="$2"
      shift 2
      ;;
    -*)
      printf 'error: unknown option %s\n' "$1" >&2
      exit 2
      ;;
    *)
      if [[ -z "$PAIR_RAW" ]]; then
        PAIR_RAW="$1"
      elif [[ -z "$OUTDIR" ]]; then
        OUTDIR="$1"
      else
        printf 'error: unexpected argument %s\n' "$1" >&2
        exit 2
      fi
      shift
      ;;
  esac
done

PAIR_RAW="${PAIR_RAW:-1}"
OUTDIR="${OUTDIR:-/tmp/dtf_calib}"

if [[ ! "$PAIR_RAW" =~ ^[0-9]+$ ]]; then
  printf 'error: pair_number must be a positive integer\n' >&2
  exit 2
fi
PAIR=$((10#$PAIR_RAW))
if (( PAIR < 1 || PAIR > 99 )); then
  printf 'error: pair_number must be between 1 and 99\n' >&2
  exit 2
fi

distance_re='^[0-9]+([.][0-9]+)?$'
angle_re='^[+-]?[0-9]+([.][0-9]+)?$'
if [[ -n "$DISTANCE_OVERRIDE" && ! "$DISTANCE_OVERRIDE" =~ $distance_re ]]; then
  printf 'error: --distance must be a positive number in metres\n' >&2
  exit 2
fi
if [[ -n "$YAW_OVERRIDE" && ! "$YAW_OVERRIDE" =~ $angle_re ]]; then
  printf 'error: --yaw must be an angle in degrees\n' >&2
  exit 2
fi
if [[ -n "$PITCH_OVERRIDE" && ! "$PITCH_OVERRIDE" =~ $angle_re ]]; then
  printf 'error: --pitch must be an angle in degrees\n' >&2
  exit 2
fi

PREVIEW_PYTHON="$REPO_ROOT/software/host_vision/.venv/bin/python"
[[ -x "$PREVIEW_PYTHON" ]] || PREVIEW_PYTHON="python3"
DTF_PREVIEW_TMP="$(mktemp -d)"
DTF_VIEWER_PID=""
trap dtf_cleanup_preview EXIT

stem="$(printf '%02d' "$PAIR")"
IFS=$'\t' read -r set distance offset yaw pitch \
  < <(dtf_pose_values "$PAIR")
[[ -n "$DISTANCE_OVERRIDE" ]] && distance="$DISTANCE_OVERRIDE"
[[ -n "$YAW_OVERRIDE" ]] && yaw="$YAW_OVERRIDE"
[[ -n "$PITCH_OVERRIDE" ]] && pitch="$PITCH_OVERRIDE"
label="$(dtf_format_pose "$set" "$distance" "$offset" "$yaw" "$pitch")"
mkdir -p "$OUTDIR/left" "$OUTDIR/right"

while true; do
  printf '\nPair %s: %s\n' "$stem" "$label"
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

  if ! dtf_capture_one "$LEFT_URL" "$OUTDIR/left/$stem.jpg" ||
     ! dtf_capture_one "$RIGHT_URL" "$OUTDIR/right/$stem.jpg"; then
    printf 'capture failed; retrying pair %s.\n' "$stem" >&2
    continue
  fi
  printf 'saved %s/left/%s.jpg and %s/right/%s.jpg\n' \
    "$OUTDIR" "$stem" "$OUTDIR" "$stem"

  if dtf_preview_available "$PREVIEW_PYTHON"; then
    preview_file="$DTF_PREVIEW_TMP/pair_${stem}.jpg"
    if dtf_make_preview "$PREVIEW_PYTHON" \
        "$OUTDIR/left/$stem.jpg" "$OUTDIR/right/$stem.jpg" "$preview_file" &&
       dtf_start_viewer "$preview_file"; then
      printf '\nPreview opened. Check both eyes: full board, correct square size, no blur or glare.\n'
      printf 'Press Enter to accept, r to recapture, q to quit: '
      if ! IFS= read -r confirm; then
        printf '\nNo input; stopping after pair %s.\n' "$stem" >&2
        dtf_stop_viewer
        exit 130
      fi
      dtf_stop_viewer
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
          break
          ;;
        *)
          printf 'Treating response as accept.\n' >&2
          break
          ;;
      esac
    else
      printf 'Preview unavailable; continuing without it.\n' >&2
    fi
  fi

  break
done

printf '\nCaptured pair %s in %s.\n' "$stem" "$OUTDIR"
