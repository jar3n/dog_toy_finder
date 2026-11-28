#!/usr/bin/env bash
# Stop the background host-side dense-stereo service. No model variables needed.
# This only kills the process recorded in the PID file written by
# scripts/start_stereo_service.sh; it does not touch any other process.
set -euo pipefail

PID_FILE="${DTF_STEREO_PID_FILE:-/tmp/dtf_stereo_service.pid}"
NAME="stereo depth"

if [ ! -s "$PID_FILE" ]; then
    echo "$NAME service is not running (no PID file at $PID_FILE)"
    exit 0
fi

pid="$(cat "$PID_FILE")"
if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
    echo "$NAME service is not running (stale PID file at $PID_FILE)"
    rm -f "$PID_FILE"
    exit 0
fi
if ! tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null \
        | grep -q 'stereo_service.py'; then
    echo "PID $pid is not a stereo depth service; refusing to stop it" >&2
    exit 1
fi

kill "$pid"
for _ in $(seq 1 50); do
    if ! kill -0 "$pid" 2>/dev/null; then
        break
    fi
    sleep 0.1
done
if kill -0 "$pid" 2>/dev/null; then
    echo "$NAME service (pid $pid) did not stop; sending SIGKILL" >&2
    kill -9 "$pid"
fi
rm -f "$PID_FILE"
echo "$NAME service stopped (pid $pid)"
