#!/usr/bin/env bash
# Launch the whole Dog Toy Finder stack in separate tmux panes.
#
# Usage:
#   On the host:        ./scripts/launch_all.sh
#   Inside the ros2 container:  ./scripts/launch_all.sh
#
# On the host this starts the micro-ROS agent container and one ros2 container
# per long-running component. Inside the container it runs everything in tmux
# panes sharing the same ROS 2 context.

set -euo pipefail

SESSION="dog_toy_finder"

if [ -f /opt/ros/humble/setup.bash ]; then
    # ---------- running inside the ROS 2 container ----------
    REPO=/workspace
    WS="$REPO/software/ros2_ws"
    source /opt/ros/humble/setup.bash
    source "$WS/install/setup.bash"

    tmux kill-session -t "$SESSION" 2>/dev/null || true
    tmux new-session -d -s "$SESSION" -n "bringup" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; ros2 launch dog_toy_finder_bringup robot.launch.py"
    tmux new-window -t "$SESSION" -n "stereo" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; ros2 launch dog_toy_finder_vision stereo.launch.py"
    tmux new-window -t "$SESSION" -n "vision" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; ros2 launch dog_toy_finder_vision vision.launch.py"
    tmux new-window -t "$SESSION" -n "mapping" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; ros2 launch dog_toy_finder_mapping mapping.launch.py"
    tmux new-window -t "$SESSION" -n "webmap" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; ros2 run dog_toy_finder_mapping map_webserver"
    tmux new-window -t "$SESSION" -n "rosa" \
        "source /opt/ros/humble/setup.bash; source $WS/install/setup.bash; cd $REPO/software/rosa_agent && python3 -m rosa_agent"
    tmux select-window -t "$SESSION:bringup"
    tmux attach -t "$SESSION"
else
    # ---------- running on the host ----------
    REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
    COMPOSE="$REPO_ROOT/docker/docker-compose.yml"

    tmux kill-session -t "$SESSION" 2>/dev/null || true

    # micro-ROS agent (host network)
    tmux new-session -d -s "$SESSION" -n "agent" \
        "cd $REPO_ROOT && docker compose -f $COMPOSE up micro_ros_agent"

    # ROS 2 container windows
    run_in_ros2() {
        local name="$1"; shift
        tmux new-window -t "$SESSION" -n "$name" \
            "cd $REPO_ROOT && docker compose -f $COMPOSE run --rm ros2 bash -lc 'source /opt/ros/humble/setup.bash; source /workspace/software/ros2_ws/install/setup.bash; $*'"
    }

    run_in_ros2 "bringup" "ros2 launch dog_toy_finder_bringup robot.launch.py"
    run_in_ros2 "stereo"  "ros2 launch dog_toy_finder_vision stereo.launch.py"
    run_in_ros2 "vision"  "ros2 launch dog_toy_finder_vision vision.launch.py"
    run_in_ros2 "mapping" "ros2 launch dog_toy_finder_mapping mapping.launch.py"
    run_in_ros2 "webmap"  "ros2 run dog_toy_finder_mapping map_webserver"
    run_in_ros2 "rosa"    "cd /workspace/software/rosa_agent && python3 -m rosa_agent"

    tmux select-window -t "$SESSION:bringup"
    echo "Attaching to tmux session '$SESSION'. Detach with Ctrl-b d."
    tmux attach -t "$SESSION"
fi
