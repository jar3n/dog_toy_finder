# ROS 2 workspace

Colcon workspace for the computer-side ROS 2 packages.

## Packages

| Package | Kind | Purpose |
|---|---|---|
| `dog_toy_finder_bringup` | `ament_cmake` | launch files, EKF config, robot geometry, `cmd_vel_mux` start |
| `dog_toy_finder_odometry` | `ament_python` | `/joint_states` -> `/wheel/odom` |
| `dog_toy_finder_motion` | `ament_python` | closed-loop moves, `cmd_vel_mux`, manual keyboard drive, `mover_test` |
| `dog_toy_finder_vision` | `ament_python` | stereo capture, rectification, disparity/point cloud, VLM |
| `dog_toy_finder_mapping` | `ament_python` | occupancy map, toy markers, browser map server |
| `dog_toy_finder_planning` | `ament_python` | autonomous search planner and path utilities |
| `dog_toy_finder_diagnostics` | `ament_python` | sensor and camera heartbeat monitor |
| `dog_toy_finder_data` | `ament_python` | dataset collection and summary |
| `dog_toy_finder_experiments` | `ament_python` | scripted search harness and run evaluation |

## Build

Run inside the ROS 2 container (`/workspace` is the repository root):

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Rebuild after editing Python or changing launch/config files. With
`--symlink-install`, Python and YAML edits take effect without a rebuild, but
new files still need a build.

## Run

The quickest path is the launcher, which reads every per-run value from
`scripts/full_system_eval.conf`:

```bash
/workspace/scripts/launch_all.sh
/workspace/scripts/launch_all.sh --manual     # plus keyboard drive
```

Individual pieces:

```bash
ros2 launch dog_toy_finder_bringup robot.launch.py
ros2 launch dog_toy_finder_vision stereo_vision.launch.py
ros2 launch dog_toy_finder_mapping mapping.launch.py
ros2 run dog_toy_finder_mapping map_webserver

ros2 run dog_toy_finder_motion mover_test straight 0.5
ros2 run dog_toy_finder_motion manual_drive
```

The ROSA agent is outside this workspace in `../rosa_agent/`; source this
workspace before starting it so `dog_toy_finder_*` Python modules are on
`PYTHONPATH`.

## Tests

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

cd src/dog_toy_finder_motion   && python3 -m unittest discover -s tests
cd src/dog_toy_finder_planning && python3 -m unittest discover -s tests
```

End-to-end search workflows (fake robot + scripted vision, no hardware) live in
`/workspace/tests_e2e/` and run with `/workspace/scripts/run_e2e_tests.sh`.
