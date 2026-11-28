#!/usr/bin/env bash
# Capture one left/right checkerboard pair at a time, waiting for Enter.
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
source "$SCRIPT_DIR/capture_pair_common.sh"

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

PREVIEW_PYTHON="$REPO_ROOT/software/host_vision/.venv/bin/python"
[[ -x "$PREVIEW_PYTHON" ]] || PREVIEW_PYTHON="python3"
DTF_PREVIEW_TMP="$(mktemp -d)"
DTF_VIEWER_PID=""
trap dtf_cleanup_preview EXIT

mkdir -p "$OUTDIR/left" "$OUTDIR/right"

pair=1
while (( pair <= COUNT )); do
  stem="$(printf '%02d' "$pair")"
  label="$(dtf_pose_label "$pair")"

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
