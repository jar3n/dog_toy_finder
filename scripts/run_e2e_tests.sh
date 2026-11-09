#!/usr/bin/env bash
# Run the end-to-end search workflow tests in the ROS 2 container.
#
# Usage:
#   ./scripts/run_e2e_tests.sh [scenario ...]
#
# With no arguments it runs every scenario. Or pick some, e.g.:
#   ./scripts/run_e2e_tests.sh note_found size distance
#
# The tests use a fake robot (integrates /cmd_vel into /odometry/filtered) and
# scripted fake vision, so no hardware, cameras, or VLMs are needed. The full
# suite takes several minutes (ROSA 'all'/'multi' drive 3 loops at full side).

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TESTS=("$@")
if [ ${#TESTS[@]} -eq 0 ]; then
    TESTS=(note_found size distance notfound findall multitarget
           rosa_one rosa_all rosa_multi)
fi

run_one() {
    local name="$1"
    case "$name" in
        note_found)
            docker run --rm --network host -v "$REPO_ROOT":/workspace \
                -w /workspace/software/ros2_ws docker-ros2:latest \
                bash -lc 'source /opt/ros/humble/setup.bash;
                          source install/setup.bash;
                          python3 -u /workspace/tests_e2e/note_found.py' ;;
        size|distance|notfound|findall|multitarget)
            docker run --rm --network host -v "$REPO_ROOT":/workspace \
                -w /workspace/software/ros2_ws docker-ros2:latest \
                bash -lc "source /opt/ros/humble/setup.bash;
                          source install/setup.bash;
                          SCENARIO=$name timeout 300 python3 -u /workspace/tests_e2e/search_workflows.py" ;;
        rosa_one|rosa_all|rosa_multi)
            local mode="${name#rosa_}"
            docker run --rm --network host -v "$REPO_ROOT":/workspace \
                -w /workspace/software/ros2_ws docker-ros2:latest \
                bash -lc "source /opt/ros/humble/setup.bash;
                          source install/setup.bash;
                          MODE=$mode ROSA_REQUIRE_APPROVAL=0 timeout 400 python3 -u /workspace/tests_e2e/rosa_search.py" ;;
        *)
            echo "run_e2e_tests.sh: unknown scenario '$name'" >&2
            exit 2 ;;
    esac
}

for name in "${TESTS[@]}"; do
    echo "===== $name ====="
    run_one "$name"
done
echo "===== all requested scenarios passed ====="
