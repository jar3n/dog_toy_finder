# Motion primitives

`dog_toy_finder_motion/mover.py` provides the closed-loop moves used by both
the command-line tests and the ROSA agent. It is deliberately separate from
ROSA: the ROS 2 graph owns motion, and the agent only decides which move to
request.

## Topics

| Topic | Type | Direction |
|---|---|---|
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | subscribe (feedback) |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | subscribe (comparison) |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | publish |

Every move stops when the fused estimate says the requested distance or angle
is complete. The Pico's 500 ms `/cmd_vel` watchdog is a second safety net if the
computer or Wi-Fi link stops.

## Moves

| Method | Meaning |
|---|---|
| `straight(m)` | drive forward by `m` metres (negative = backward) |
| `turn(rad)` | turn in place (positive = left) |
| `goto(x, y)` | drive to a world-frame point |
| `square(side)` | four sides with 90 degree turns |
| `stop()` | publish zero Twist commands |

`COAST_TIME` compensates for Wi-Fi and motor spin-down by stopping slightly
early. Tune it after measuring the real robot; Gazebo would use zero.

## Test moves

```bash
ros2 run dog_toy_finder_motion mover_test straight 1.0
ros2 run dog_toy_finder_motion mover_test turn 90
ros2 run dog_toy_finder_motion mover_test square 0.5 --cw
ros2 run dog_toy_finder_motion mover_test goto 1.0 0.5
```

The unit tests in `software/rosa_agent/tests/` cover the pure transforms.

## ROSA dependency

The ROSA tools import `Mover` from this package. Build the workspace and source
`install/setup.bash` before starting the agent, otherwise Python cannot import
`dog_toy_finder_motion`.
