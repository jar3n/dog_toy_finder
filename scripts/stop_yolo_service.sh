#!/usr/bin/env bash
# Stop the background host-side detector service. No model variables needed.
# This only kills the process recorded in the PID file written by
# scripts/start_yolo_service.sh; it does not touch any other process.
set -euo pipefail

PID_FILE="${DTF_YOLO_PID_FILE:-/tmp/dtf_yolo_service.pid}"
NAME="detector"

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
        | grep -q 'yolo_service.py'; then
    echo "PID $pid is not a detector service; refusing to stop it" >&2
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
