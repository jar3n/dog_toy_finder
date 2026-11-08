# ROS 2 workspace

Colcon workspace for the computer-side ROS 2 packages.

## Packages

| Package | Kind | Purpose |
|---|---|---|
| `dog_toy_finder_bringup` | `ament_cmake` | launch files and YAML config |
| `dog_toy_finder_odometry` | `ament_python` | `/joint_states` -> `/wheel/odom` |
| `dog_toy_finder_motion` | `ament_python` | closed-loop moves and `mover_test` |

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

```bash
ros2 launch dog_toy_finder_bringup robot.launch.py
ros2 run dog_toy_finder_motion mover_test straight 0.5
```

The ROSA agent is outside this workspace in `../rosa_agent/`; source this
workspace before starting it so `dog_toy_finder_motion` is importable.
