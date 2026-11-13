#!/usr/bin/env bash
# Launch the whole Dog Toy Finder stack in separate tmux panes.
#
# Usage:
#   ./scripts/launch_all.sh [config-file] [--manual] [--dry-run]
#
# On the host this starts the micro-ROS agent container and one ros2 container
# per long-running component. Inside the container it runs everything in tmux
# panes sharing the same ROS 2 context. With --dry-run it prints the commands
# instead of starting tmux (useful to check the configuration).
#
# --manual adds an extra tmux window with the keyboard manual drive node and
# selects it, so the same command both starts the stack and lets you drive:
#   ./scripts/launch_all.sh --manual
# It can also be enabled permanently with MANUAL_DRIVE=true in the config.
#
# All software parameters come from a config file, by default
# scripts/full_system_eval.conf (override with the first argument or by setting
# FULL_SYSTEM_CONFIG). Values already set in the environment win over the
# config file, so a one-off override works without editing it:
#   WEBMAP_PORT=9000 ./scripts/launch_all.sh

set -euo pipefail

SESSION="dog_toy_finder"
HAS_DOCKER=false
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

DRY_RUN=0
MANUAL_REQUESTED=0
KONSOLE_REQUESTED=""
CONFIG=""
for arg in "$@"; do
    case "$arg" in
        --dry-run) DRY_RUN=1 ;;
        --manual) MANUAL_REQUESTED=1 ;;
        --konsole) KONSOLE_REQUESTED="true" ;;
        --tmux) KONSOLE_REQUESTED="false" ;;
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

# --manual wins over the config so a one-off drive does not need an edit.
if [ "$MANUAL_REQUESTED" -eq 1 ]; then
    MANUAL_DRIVE=true
fi

# Resolve konsole vs tmux.
if [ -n "$KONSOLE_REQUESTED" ]; then
    USE_KONSOLE="$KONSOLE_REQUESTED"
elif [ -n "${KONSOLE:-}" ]; then
    USE_KONSOLE="$KONSOLE"
else
    USE_KONSOLE="auto"
fi
case "$(printf '%s' "$USE_KONSOLE" | tr '[:upper:]' '[:lower:]')" in
    auto)
        if command -v konsole >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
            USE_KONSOLE=true
        else
            USE_KONSOLE=false
        fi
        ;;
esac

# Accept the usual spellings for a boolean config value.
is_true() {
    case "$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')" in
        true|1|yes|on) return 0 ;;
        *) return 1 ;;
    esac
}

# Clean up all child processes (tmux session or konsole) on Ctrl-C / termination.
cleanup() {
    if is_true "${USE_KONSOLE:-false}"; then
        pkill -f "launch_konsole.py" 2>/dev/null || true
    else
        tmux kill-session -t "$SESSION" 2>/dev/null || true
    fi
    # Stop any docker containers started by this launch (compose services
    # and the per-component `docker compose run` containers).
    if [ "$HAS_DOCKER" = true ]; then
        if [ -n "${REPO_ROOT:-}" ] && [ -f "$REPO_ROOT/docker/docker-compose.yml" ]; then
            (cd "$REPO_ROOT" && docker compose -f docker/docker-compose.yml down 2>/dev/null) || true
        fi
        docker ps -q --filter "name=docker-ros2-run" | xargs -r docker rm -f >/dev/null 2>&1 || true
        docker ps -q --filter "name=micro-ros-agent" | xargs -r docker rm -f >/dev/null 2>&1 || true
    fi
}
trap cleanup INT TERM

# One ros2-level command per component, built from the config values above.
BRINGUP_CMD="ros2 launch dog_toy_finder_bringup robot.launch.py"
BRINGUP_CMD="$BRINGUP_CMD use_sim_time:=$USE_SIM_TIME"
BRINGUP_CMD="$BRINGUP_CMD wheel_radius:=$WHEEL_RADIUS"
BRINGUP_CMD="$BRINGUP_CMD wheel_separation:=$WHEEL_SEPARATION"
BRINGUP_CMD="$BRINGUP_CMD publish_tf:=$PUBLISH_TF"
BRINGUP_CMD="$BRINGUP_CMD health_monitor:=$HEALTH_MONITOR"
BRINGUP_CMD="$BRINGUP_CMD cmd_vel_mux:=$CMD_VEL_MUX"
BRINGUP_CMD="$BRINGUP_CMD cmd_vel_manual_timeout:=$CMD_VEL_MANUAL_TIMEOUT"
BRINGUP_CMD="$BRINGUP_CMD cmd_vel_auto_timeout:=$CMD_VEL_AUTO_TIMEOUT"
BRINGUP_CMD="$BRINGUP_CMD cmd_vel_release_timeout:=$CMD_VEL_RELEASE_TIMEOUT"
BRINGUP_CMD="$BRINGUP_CMD cmd_vel_publish_rate:=$CMD_VEL_PUBLISH_RATE"
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
VISION_CMD="$VISION_CMD model:=$VISION_MODEL"
if [ -n "${VISION_MODELS:-}" ]; then
    VISION_CMD="$VISION_CMD models:=$VISION_MODELS"
fi
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
MAP_CMD="$MAP_CMD toy_merge_radius:=$TOY_MERGE_RADIUS"
MAP_CMD="$MAP_CMD toy_occupancy_radius:=$TOY_OCCUPANCY_RADIUS"

WEB_CMD="ros2 run dog_toy_finder_mapping map_webserver --ros-args -p port:=$WEBMAP_PORT"

MANUAL_CMD="ros2 run dog_toy_finder_motion manual_drive --ros-args"
MANUAL_CMD="$MANUAL_CMD -p linear_speed:=${MANUAL_LINEAR_SPEED:-0.12}"
MANUAL_CMD="$MANUAL_CMD -p angular_speed:=${MANUAL_ANGULAR_SPEED:-0.5}"
MANUAL_CMD="$MANUAL_CMD -p key_timeout:=${MANUAL_KEY_TIMEOUT:-0.6}"
MANUAL_CMD="$MANUAL_CMD -p publish_rate:=${MANUAL_PUBLISH_RATE:-20.0}"

ROSA_CMD="OLLAMA_MODEL=$ROSA_MODEL OLLAMA_URL=$ROSA_OLLAMA_URL"
ROSA_CMD="$ROSA_CMD ROSA_REQUIRE_APPROVAL=$ROSA_REQUIRE_APPROVAL python3 -m rosa_agent"

if [ "$DRY_RUN" -eq 1 ]; then
    echo "launch_all.sh --dry-run (config: $CONFIG)"
    echo "launcher: $([ "$USE_KONSOLE" = true ] && echo konsole || echo tmux)"
    echo "bringup: $BRINGUP_CMD"
    echo "stereo:  $STEREO_CMD"
    echo "vision:  $VISION_CMD"
    echo "mapping: $MAP_CMD"
    echo "webmap:  $WEB_CMD"
    echo "manual:  $MANUAL_CMD"
    echo "rosa:    cd <repo>/software/rosa_agent && $ROSA_CMD"
    if is_true "${MANUAL_DRIVE:-false}"; then
        echo "manual drive: enabled; windows: manual (drive), shell (cooked TTY for tmux cmds), bringup"
    else
        echo "manual drive: disabled (add --manual to start it and select its window)"
    fi
    exit 0
fi

# ---------- Konsole (split panes) path ----------
if is_true "$USE_KONSOLE"; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        REPO=/workspace
        REPO_ROOT="$REPO"
        WS="$REPO/software/ros2_ws"
        ROS_SETUP="source /opt/ros/humble/setup.bash; source $WS/install/setup.bash"
        CMD_FILE="$(mktemp /tmp/dog_toy_finder_cmds.XXXXXX)"
        trap 'rm -f "$CMD_FILE"' EXIT
        {
            printf 'bringup\t%s; %s\n' "$ROS_SETUP" "$BRINGUP_CMD"
            printf 'stereo\t%s; %s\n' "$ROS_SETUP" "$STEREO_CMD"
            printf 'vision\t%s; %s\n' "$ROS_SETUP" "$VISION_CMD"
            printf 'mapping\t%s; %s\n' "$ROS_SETUP" "$MAP_CMD"
            printf 'webmap\t%s; %s\n' "$ROS_SETUP" "$WEB_CMD"
            printf 'rosa\t%s; cd %s/software/rosa_agent && %s\n' "$ROS_SETUP" "$REPO" "$ROSA_CMD"
            if is_true "${MANUAL_DRIVE:-false}"; then
                printf 'manual\t%s; %s\n' "$ROS_SETUP" "$MANUAL_CMD"
                printf 'shell\t%s; exec bash\n' "$ROS_SETUP"
            fi
        } > "$CMD_FILE"
        REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
        python3 "$REPO_ROOT/scripts/launch_konsole.py" "$CMD_FILE" "$REPO"
    else
        REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
        COMPOSE="$REPO_ROOT/docker/docker-compose.yml"
        HAS_DOCKER=true
        CMD_FILE="$(mktemp /tmp/dog_toy_finder_cmds.XXXXXX)"
        trap 'rm -f "$CMD_FILE"' EXIT
        {
            printf 'agent\tcd %s && docker compose -f %s up micro_ros_agent\n' "$REPO_ROOT" "$COMPOSE"
            printf 'bringup\tcd %s && docker compose -f %s run --rm ros2 bash -lc '"'"'source /opt/ros/humble/setup.bash; source /workspace/software/ros2_ws/install/setup.bash; %s'"'"'\n' "$REPO_ROOT" "$COMPOSE" "$BRINGUP_CMD"
            printf 'stereo\tcd %s && docker compose -f %s run --rm ros2 bash -lc '"'"'source /opt/ros/humble/setup.bash; source /workspace/software/ros2_ws/install/setup.bash; %s'"'"'\n' "$REPO_ROOT" "$COMPOSE" "$STEREO_CMD"
            printf 'vision\tcd %s && docker compose -f %s run --rm ros2 bash -lc '"'"'source /opt/ros/humble/setup.bash; source /workspace/software/ros2_ws/install/setup.bash; %s'"'"'\n' "$REPO_ROOT" "$COMPOSE" "$VISION_CMD"
            printf 'mapping\tcd %s && docker compose -f %s run --rm ros2 bash -lc '"'"'source /opt/ros/humble/setup.bash; source /workspace/software/ros2_ws/install/setup.bash; %s'"'"'\n' "$REPO_ROOT" "$COMPOSE" "$MAP_CMD"
            printf 'webmap\tcd %s && docker compose -f %s run --rm ros2 bash -lc '"'"'source /opt/ros/humble/setup.bash; source /workspace/software/ros2_ws/install/setup.bash; %s'"'"'\n' "$REPO_ROOT" "$COMPOSE" "$WEB_CMD"
            printf 'rosa\tcd %s && docker compose -f %s run --rm ros2 bash -lc '"'"'source /opt/ros/humble/setup.bash; source /workspace/software/ros2_ws/install/setup.bash; cd /workspace/software/rosa_agent && %s'"'"'\n' "$REPO_ROOT" "$COMPOSE" "$ROSA_CMD"
            if is_true "${MANUAL_DRIVE:-false}"; then
                printf 'manual\tcd %s && docker compose -f %s run --rm ros2 bash -lc '"'"'source /opt/ros/humble/setup.bash; source /workspace/software/ros2_ws/install/setup.bash; %s'"'"'\n' "$REPO_ROOT" "$COMPOSE" "$MANUAL_CMD"
                printf 'shell\tcd %s && docker compose -f %s run --rm ros2 bash -lc '"'"'source /opt/ros/humble/setup.bash; source /workspace/software/ros2_ws/install/setup.bash; exec bash'"'"'\n' "$REPO_ROOT" "$COMPOSE"
            fi
        } > "$CMD_FILE"
        python3 "$REPO_ROOT/scripts/launch_konsole.py" "$CMD_FILE" "$REPO_ROOT"
    fi
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
    if is_true "${MANUAL_DRIVE:-false}"; then
        tmux new-window -t "$SESSION" -n "manual" \
            "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; $MANUAL_CMD"
        # A normal (cooked) shell so tmux/ROS commands can be typed while the
        # manual driver holds the terminal in the 'manual' window.
        tmux new-window -t "$SESSION" -n "shell" \
            "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; exec bash"
        tmux bind-key m select-window -t "$SESSION:manual"
        tmux bind-key s select-window -t "$SESSION:shell"
        tmux bind-key b select-window -t "$SESSION:bringup"
        # Ctrl+C tears down the whole stack instead of being forwarded to a pane.
        tmux bind-key -n C-c kill-session -t "$SESSION"
        # Start in the shell window so there is a usable terminal immediately;
        # switch to 'manual' (Ctrl+b m) to drive.
        tmux select-window -t "$SESSION:shell"
    else
        tmux bind-key -n C-c kill-session -t "$SESSION"
        tmux select-window -t "$SESSION:bringup"
    fi
    tmux attach -t "$SESSION" || true
    tmux kill-session -t "$SESSION" 2>/dev/null || true
else
    # ---------- running on the host ----------
    REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
    COMPOSE="$REPO_ROOT/docker/docker-compose.yml"
    HAS_DOCKER=true

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
    if is_true "${MANUAL_DRIVE:-false}"; then
        # The manual window needs the TTY that tmux gives it, so keyboard
        # input reaches the node (docker compose run auto-detects the TTY).
        run_in_ros2 "manual" "$MANUAL_CMD"
        # A normal (cooked) shell so tmux/ROS commands can be typed while the
        # manual driver holds its window's terminal in cbreak mode.
        run_in_ros2 "shell" "exec bash"
        tmux bind-key m select-window -t "$SESSION:manual"
        tmux bind-key s select-window -t "$SESSION:shell"
        tmux bind-key b select-window -t "$SESSION:bringup"
        # Ctrl+C tears down the whole stack instead of being forwarded to a pane.
        tmux bind-key -n C-c kill-session -t "$SESSION"
        # Start in the shell window so there is a usable terminal immediately;
        # switch to 'manual' (Ctrl+b m) to drive.
        tmux select-window -t "$SESSION:shell"
    else
        tmux bind-key -n C-c kill-session -t "$SESSION"
        tmux select-window -t "$SESSION:bringup"
    fi

    echo "Attaching to tmux session '$SESSION'. Detach with Ctrl-b d."
    if is_true "${MANUAL_DRIVE:-false}"; then
        echo "Manual mode: 'manual' window drives the robot; 'shell' window is a"
        echo "  normal terminal for tmux/ROS commands. Switch with Ctrl+b then"
        echo "  m (manual), s (shell), b (bringup), or w (window list)."
        echo "  Hold W/A/S/D to drive, Space to stop, Q to quit."
    fi
    tmux attach -t "$SESSION" || true
    tmux kill-session -t "$SESSION" 2>/dev/null || true
fi
