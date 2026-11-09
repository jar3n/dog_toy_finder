#!/usr/bin/env bash
# Stop everything started by launch_all.sh.
#
# Usage:
#   On the host:        ./scripts/stop_all.sh
#   Inside container:   ./scripts/stop_all.sh

set -u

SESSION="dog_toy_finder"

if [ -f /opt/ros/humble/setup.bash ]; then
    # Inside the ROS 2 container: kill the tmux session only.
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux kill-session -t "$SESSION"
        echo "Killed tmux session '$SESSION'."
    else
        echo "No tmux session '$SESSION'."
    fi
else
    # On the host: kill tmux session and stop the agent container.
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux kill-session -t "$SESSION"
        echo "Killed tmux session '$SESSION'."
    else
        echo "No tmux session '$SESSION'."
    fi
    REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
    if docker ps --format '{{.Names}}' | grep -qx 'micro-ros-agent'; then
        docker stop micro-ros-agent >/dev/null
        echo "Stopped micro-ros-agent container."
    fi
    # Also stop any leftover ros2 containers started by the launcher.
    docker ps -q --filter "name=rosa-ros2" | xargs -r docker stop >/dev/null 2>&1 || true
fi
echo "Done."
