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
| `/ir_obstacle/detected` | `std_msgs/msg/Bool` | subscribe (Pico failsafe) |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | publish |

Every move stops when the fused estimate says the requested distance or angle
is complete. The Pico's 500 ms `/cmd_vel` watchdog is a second safety net if the
computer or Wi-Fi link stops.

## Pico failsafe (IR reflex)

The Pico brakes and blocks forward/turn commands while its IR sensor sees an
obstacle, allowing only stepped reverse. ROS knows it happened because:

- The Pico publishes the debounced sensor state on `/ir_obstacle/detected`
  (10 Hz), and `Mover` subscribes to it.
- The Pico mirrors the reflex engage/clear messages (and the watchdog stop) to
  `/pico/log` as `rcl_interfaces/msg/Log`; check with
  `ros2 topic echo /pico/log`.

When `/ir_obstacle/detected` is true during a move, `straight()` (forward
only — backward uses the Pico's stepped reverse) and `turn()` abort promptly
instead of driving against the brake, log a warning, and latch the flag read
with `failsafe_hit()`. `obstacle_ahead()` returns the latest IR value, and
`clear_failsafe()` resets the flag (every move clears it at the start). Until
the first IR message arrives, `obstacle_ahead()` is false, so a missing sensor
does not block moves.

| Method | Meaning |
|---|---|
| `obstacle_ahead()` | latest `/ir_obstacle/detected` value |
| `failsafe_hit()` | true when the failsafe interrupted the last move |
| `clear_failsafe()` | reset the failsafe flag |

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

## Manual keyboard drive

`manual_drive` publishes `/cmd_vel` straight from keyboard input, for moving
the robot by hand while the bringup stack is running. The easiest launch is
the full-stack launcher with `--manual`, which starts every component and
selects a `manual` tmux window:

```bash
./scripts/launch_all.sh --manual
```

The node can also be started on its own:

```bash
ros2 run dog_toy_finder_motion manual_drive
# slower:
ros2 run dog_toy_finder_motion manual_drive --ros-args \
    -p linear_speed:=0.10 -p angular_speed:=0.4
```

On the host, `./scripts/manual_drive.sh` opens the ROS 2 container with a TTY
and runs the same node. When started through `launch_all.sh`, the speeds and
timeout come from the `MANUAL_*` values in `scripts/full_system_eval.conf`.

| Key | Action |
|---|---|
| `W` / Up | drive forward |
| `S` / Down | drive backward |
| `A` / Left | turn left |
| `D` / Right | turn right |
| Space | stop now |
| `Q` | stop and quit |

The robot never drives and turns at the same time. Every key sets exactly one
axis and clears the other, so a published `Twist` never has both a linear and
an angular component; pressing a turn key cancels a drive and vice versa.

A terminal has no key-release event, so the node relies on keyboard auto-repeat:
hold a key to keep moving, and `key_timeout` seconds (default 0.6 s) without a
drive key publishes a stop. This makes releasing the key a dead-man stop rather
than leaving the robot driving. If your keyboard repeat delay is longer than
the timeout, raise it with `-p key_timeout:=0.8`.

| Parameter | Default | Meaning |
|---|---|---|
| `linear_speed` | 0.12 | forward/backward speed (m/s) |
| `angular_speed` | 0.5 | turn rate (rad/s) |
| `key_timeout` | 0.6 | stop after this long without a key (s) |
| `publish_rate` | 20.0 | command publish rate (Hz) |
| `cmd_topic` | `/cmd_vel` | topic to publish |

The Pico's IR reflex and 500 ms `/cmd_vel` watchdog still apply: the Pico
brakes locally in front of an obstacle and stops the motors if the node or the
Wi-Fi link stops publishing.

The unit tests in `software/rosa_agent/tests/` cover the pure transforms.

## ROSA dependency

The ROSA tools import `Mover` from this package. Build the workspace and source
`install/setup.bash` before starting the agent, otherwise Python cannot import
`dog_toy_finder_motion`.
