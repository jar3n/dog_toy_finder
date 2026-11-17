#!/usr/bin/env bash
# Verify that the Pico contributes exactly one node and one endpoint of each
# type to the ROS 2 graph. Run after flashing/rebooting the Pico and before any
# live motion test.

set -Eeuo pipefail

NODE_NAME="${DTF_PICO_NODE_NAME:-/dtf_pico}"
EXPECTED_COUNT=1
# Endpoint discovery can lag a few seconds behind a fresh Pico boot.
MAX_ATTEMPTS="${DTF_PICO_GRAPH_ATTEMPTS:-15}"
if [[ ! "$MAX_ATTEMPTS" =~ ^[1-9][0-9]*$ ]]; then
  printf 'error: DTF_PICO_GRAPH_ATTEMPTS must be a positive integer\n' >&2
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  cat >&2 <<'EOF'
error: ros2 was not found in PATH.

Run this script inside the ROS 2 container:
  docker exec -it rosa-ros2 bash -lc \
    'source /opt/ros/humble/setup.bash; /workspace/firmware/scripts/check_pico_graph.sh'
EOF
  exit 127
fi

# Count complete endpoint records. Each record ends at the GID line, which
# avoids counting headings such as "Publisher count: 2" as endpoints.
count_endpoint() {
  local output="$1"
  local wanted_name="$2"
  local wanted_type="$3"

  printf '%s\n' "$output" | awk \
    -v wanted_name="$wanted_name" \
    -v wanted_type="$wanted_type" '
    /^Node name:/      { name = $3; next }
    /^Node namespace:/ { namespace = $3; next }
    /^Endpoint type:/  { endpoint = $3; next }
    /^GID:/ {
      if (name == wanted_name && (namespace == "/" || namespace == "") &&
          endpoint == wanted_type) {
        count++
      }
      name = ""
      namespace = ""
      endpoint = ""
      next
    }
    END { print count + 0 }
  '
}

failures=0
check_count() {
  local label="$1"
  local actual="$2"

  if [[ "$actual" -eq "$EXPECTED_COUNT" ]]; then
    printf 'ok:   %s = %s\n' "$label" "$actual"
  else
    printf 'FAIL: %s = %s (expected %s)\n' "$label" "$actual" "$EXPECTED_COUNT"
    failures=$((failures + 1))
  fi
}

for ((attempt = 1; attempt <= MAX_ATTEMPTS; attempt++)); do
  node_list="$(ros2 node list --no-daemon 2>/dev/null || true)"
  node_count="$(printf '%s\n' "$node_list" | grep -cx -- "$NODE_NAME" || true)"

  cmd_vel_info="$(ros2 topic info /cmd_vel --no-daemon -v 2>/dev/null || true)"
  imu_info="$(ros2 topic info /imu/data --no-daemon -v 2>/dev/null || true)"

  cmd_vel_subs="$(count_endpoint "$cmd_vel_info" "${NODE_NAME#/}" SUBSCRIPTION)"
  imu_pubs="$(count_endpoint "$imu_info" "${NODE_NAME#/}" PUBLISHER)"

  if [[ "$node_count" -eq "$EXPECTED_COUNT" &&
        "$cmd_vel_subs" -eq "$EXPECTED_COUNT" &&
        "$imu_pubs" -eq "$EXPECTED_COUNT" ]]; then
    check_count "nodes named $NODE_NAME" "$node_count"
    check_count "/cmd_vel subscriptions from ${NODE_NAME#/}" "$cmd_vel_subs"
    check_count "/imu/data publishers from ${NODE_NAME#/}" "$imu_pubs"
    printf '\nPico graph check passed: exactly one %s session is advertised.\n' "$NODE_NAME"
    exit 0
  fi

  if [[ "$attempt" -lt "$MAX_ATTEMPTS" ]]; then
    sleep 1
  fi
done

# Report the final sample so a persistent duplicate is easy to diagnose.
failures=0
check_count "nodes named $NODE_NAME" "$node_count"
check_count "/cmd_vel subscriptions from ${NODE_NAME#/}" "$cmd_vel_subs"
check_count "/imu/data publishers from ${NODE_NAME#/}" "$imu_pubs"

if [[ "$failures" -ne 0 ]]; then
  cat >&2 <<'EOF'

Stale micro-ROS sessions are still in the graph. Clear them and retry:

  (cd docker && docker compose restart micro_ros_agent)
  # wait for the Pico to reconnect, then rerun this script

Only flash firmware with a stable ROS_CLIENT_KEY (0x44544650) before rebooting
again; older firmware will create a new client key on every boot.
EOF
  exit 1
fi
