#!/usr/bin/env bash
# Launch the whole Dog Toy Finder stack in separate tmux panes.
#
# Usage:
#   ./scripts/launch_all.sh [config-file] [--dry-run]
#
# On the host this starts the micro-ROS agent container and one ros2 container
# per long-running component. Inside the container it runs everything in tmux
# panes sharing the same ROS 2 context. With --dry-run it prints the commands
# instead of starting tmux (useful to check the configuration).
#
# All software parameters come from a config file, by default
# scripts/full_system_eval.conf (override with the first argument or by setting
# FULL_SYSTEM_CONFIG). Values already set in the environment win over the
# config file, so a one-off override works without editing it:
#   WEBMAP_PORT=9000 ./scripts/launch_all.sh

set -euo pipefail

SESSION="dog_toy_finder"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

DRY_RUN=0
CONFIG=""
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        *) CONFIG="$arg" ;;
    esac
done
CONFIG="${FULL_SYSTEM_CONFIG:-$CONFIG}"
CONFIG="${CONFIG:-$SCRIPT_DIR/full_system_eval.conf}"

if [ ! -f "$CONFIG" ]; then
    echo "launch_all.sh: config file not found: $CONFIG" >&2
    exit 1
fi
# shellcheck disable=SC1090
. "$CONFIG"

# One ros2-level command per component, built from the config values above.
BRINGUP_CMD="ros2 launch dog_toy_finder_bringup robot.launch.py"
BRINGUP_CMD="$BRINGUP_CMD use_sim_time:=$USE_SIM_TIME"
BRINGUP_CMD="$BRINGUP_CMD wheel_radius:=$WHEEL_RADIUS"
BRINGUP_CMD="$BRINGUP_CMD wheel_separation:=$WHEEL_SEPARATION"
BRINGUP_CMD="$BRINGUP_CMD publish_tf:=$PUBLISH_TF"
BRINGUP_CMD="$BRINGUP_CMD health_monitor:=$HEALTH_MONITOR"
BRINGUP_CMD="$BRINGUP_CMD imu_x:=$IMU_X imu_y:=$IMU_Y imu_z:=$IMU_Z"
BRINGUP_CMD="$BRINGUP_CMD imu_roll:=$IMU_ROLL imu_yaw:=$IMU_YAW"
BRINGUP_CMD="$BRINGUP_CMD health_check_period:=$HEALTH_CHECK_PERIOD"
BRINGUP_CMD="$BRINGUP_CMD health_startup_grace:=$HEALTH_STARTUP_GRACE"
BRINGUP_CMD="$BRINGUP_CMD health_imu_timeout:=$HEALTH_IMU_TIMEOUT"
BRINGUP_CMD="$BRINGUP_CMD health_joint_states_timeout:=$HEALTH_JOINT_STATES_TIMEOUT"
BRINGUP_CMD="$BRINGUP_CMD health_ir_timeout:=$HEALTH_IR_TIMEOUT"
BRINGUP_CMD="$BRINGUP_CMD health_left_camera_timeout:=$HEALTH_LEFT_CAMERA_TIMEOUT"
BRINGUP_CMD="$BRINGUP_CMD health_right_camera_timeout:=$HEALTH_RIGHT_CAMERA_TIMEOUT"
BRINGUP_CMD="$BRINGUP_CMD health_odometry_timeout:=$HEALTH_ODOMETRY_TIMEOUT"

STEREO_CMD="ros2 launch dog_toy_finder_vision stereo.launch.py"
STEREO_CMD="$STEREO_CMD left_url:=$LEFT_URL right_url:=$RIGHT_URL"
STEREO_CMD="$STEREO_CMD rate:=$CAMERA_RATE width:=$IMAGE_WIDTH height:=$IMAGE_HEIGHT"
STEREO_CMD="$STEREO_CMD fx:=$CAMERA_FX fy:=$CAMERA_FY cx:=$CAMERA_CX cy:=$CAMERA_CY"
STEREO_CMD="$STEREO_CMD baseline:=$STEREO_BASELINE"

VISION_CMD="ros2 launch dog_toy_finder_vision vision.launch.py"
VISION_CMD="$VISION_CMD model:=$VISION_MODEL models:=$VISION_MODELS"
VISION_CMD="$VISION_CMD ollama_url:=$VISION_OLLAMA_URL"

MAP_CMD="ros2 launch dog_toy_finder_mapping mapping.launch.py"
MAP_CMD="$MAP_CMD resolution:=$MAP_RESOLUTION width:=$MAP_WIDTH height:=$MAP_HEIGHT"
MAP_CMD="$MAP_CMD origin_x:=$MAP_ORIGIN_X origin_y:=$MAP_ORIGIN_Y"
MAP_CMD="$MAP_CMD obstacle_range:=$OBSTACLE_RANGE publish_period:=$MAP_PUBLISH_PERIOD"
MAP_CMD="$MAP_CMD ir_topic:=$IR_TOPIC use_vision:=$MAP_USE_VISION"
MAP_CMD="$MAP_CMD vision_obstacle_range:=$VISION_OBSTACLE_RANGE"
MAP_CMD="$MAP_CMD vision_timeout:=$VISION_TIMEOUT"
MAP_CMD="$MAP_CMD toy_forward_distance:=$TOY_FORWARD_DISTANCE"
MAP_CMD="$MAP_CMD toy_side_offset:=$TOY_SIDE_OFFSET"
MAP_CMD="$MAP_CMD toy_max_distance:=$TOY_MAX_DISTANCE"
MAP_CMD="$MAP_CMD toy_dedupe_radius:=$TOY_DEDUPE_RADIUS"
MAP_CMD="$MAP_CMD toy_occupancy_radius:=$TOY_OCCUPANCY_RADIUS"

WEB_CMD="ros2 run dog_toy_finder_mapping map_webserver --ros-args -p port:=$WEBMAP_PORT"

ROSA_CMD="OLLAMA_MODEL=$ROSA_MODEL OLLAMA_URL=$ROSA_OLLAMA_URL"
ROSA_CMD="$ROSA_CMD ROSA_REQUIRE_APPROVAL=$ROSA_REQUIRE_APPROVAL python3 -m rosa_agent"

if [ "$DRY_RUN" -eq 1 ]; then
    echo "launch_all.sh --dry-run (config: $CONFIG)"
    echo "bringup: $BRINGUP_CMD"
    echo "stereo:  $STEREO_CMD"
    echo "vision:  $VISION_CMD"
    echo "mapping: $MAP_CMD"
    echo "webmap:  $WEB_CMD"
    echo "rosa:    cd <repo>/software/rosa_agent && $ROSA_CMD"
    exit 0
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true

if [ -f /opt/ros/humble/setup.bash ]; then
    # ---------- running inside the ROS 2 container ----------
    REPO=/workspace
    WS="$REPO/software/ros2_ws"
    source /opt/ros/humble/setup.bash
    source "$WS/install/setup.bash"

    tmux new-session -d -s "$SESSION" -n "bringup" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; $BRINGUP_CMD"
    tmux new-window -t "$SESSION" -n "stereo" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; $STEREO_CMD"
    tmux new-window -t "$SESSION" -n "vision" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; $VISION_CMD"
    tmux new-window -t "$SESSION" -n "mapping" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; $MAP_CMD"
    tmux new-window -t "$SESSION" -n "webmap" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; $WEB_CMD"
    tmux new-window -t "$SESSION" -n "rosa" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; cd $REPO/software/rosa_agent && $ROSA_CMD"
    tmux select-window -t "$SESSION:bringup"
    tmux attach -t "$SESSION"
else
    # ---------- running on the host ----------
    REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
    COMPOSE="$REPO_ROOT/docker/docker-compose.yml"

    # micro-ROS agent (host network)
    tmux new-session -d -s "$SESSION" -n "agent" \
        "cd $REPO_ROOT && docker compose -f $COMPOSE up micro_ros_agent"

    # ROS 2 container windows
    run_in_ros2() {
        local name="$1"; shift
        tmux new-window -t "$SESSION" -n "$name" \
            "cd $REPO_ROOT && docker compose -f $COMPOSE run --rm ros2 bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/software/ros2_ws/install/setup.bash; $*'"
    }

    run_in_ros2 "bringup" "$BRINGUP_CMD"
    run_in_ros2 "stereo"  "$STEREO_CMD"
    run_in_ros2 "vision"  "$VISION_CMD"
    run_in_ros2 "mapping" "$MAP_CMD"
    run_in_ros2 "webmap"  "$WEB_CMD"
    run_in_ros2 "rosa"    "cd /workspace/software/rosa_agent && $ROSA_CMD"

    tmux select-window -t "$SESSION:bringup"
    echo "Attaching to tmux session '$SESSION'. Detach with Ctrl-b d."
    tmux attach -t "$SESSION"
fi
