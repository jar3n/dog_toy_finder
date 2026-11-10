#!/usr/bin/env bash
# Run the keyboard manual drive node.
#
# Usage:
#   ./scripts/manual_drive.sh
#   ./scripts/manual_drive.sh --ros-args -p linear_speed:=0.10
#
# Start the robot stack first (micro-ROS agent + robot.launch.py). On the host
# this opens the ROS 2 container with an interactive TTY; inside the ROS 2
# container it runs the node directly.
#
# To start the whole stack and this node together, use:
#   ./scripts/launch_all.sh --manual
#
# Keys: W/S or Up/Down drive, A/D or Left/Right turn, Space stops, Q quits.
# The robot never drives and turns at the same time.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if [ -f /opt/ros/humble/setup.bash ]; then
    # Already inside the ROS 2 container.
    source /opt/ros/humble/setup.bash
    source "$REPO_ROOT/software/ros2_ws/install/setup.bash"
    exec ros2 run dog_toy_finder_motion manual_drive "$@"
fi

# On the host: run in the ROS 2 container with a TTY so key input works.
cd "$REPO_ROOT"
exec docker compose -f docker/docker-compose.yml run --rm -it ros2 bash -lc '
    source /opt/ros/humble/setup.bash
    source /workspace/software/ros2_ws/install/setup.bash
    exec ros2 run dog_toy_finder_motion manual_drive "$@"
' bash "$@"
