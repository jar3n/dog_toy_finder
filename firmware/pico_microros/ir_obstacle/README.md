# Keyestudio KS0051 infrared obstacle avoidance sensor

`ir_obstacle.c` / `ir_obstacle.h` read the sensor's digital output with a
simple debounce filter. `ir_obstacle_ros.c` / `ir_obstacle_ros.h` publish the
result as `std_msgs/msg/Bool` on `/ir_obstacle/detected`.

The KS0051 is an infrared proximity sensor with an adjustable range of 2-40 cm.
It outputs a TTL signal (active high by default) when an obstacle is detected
and has an enable (EN) line. See the [module documentation](https://docs.keyestudio.com/projects/KS0051/en/latest/docs/KS0051.html).

## Wiring

| Sensor | Pico 2 W | Notes |
|---|---|---|
| VCC | 3V3 | 3.3-5 V supply |
| GND | GND | common ground |
| S | GPIO4 | digital signal, active high by default |
| EN | GPIO5 | high enables the sensor |

Free GPIOs on the Pico 2 W include 0-3, 10-13, 26-28.

## Configuration

Pins live in `../config/pinout.h`; timing and topic live in
`../config/ir_obstacle_config.h`.

| Setting | Default | Meaning |
|---|---|---|
| `IR_OBSTACLE_TOPIC` | `/ir_obstacle/detected` | ROS topic for the Bool result |
| `IR_OBSTACLE_DEBOUNCE_MS` | 50 | ignore changes shorter than this |
| `IR_OBSTACLE_ACTIVE_LEVEL` | 1 | 1 = obstacle when signal is HIGH |
| `IR_OBSTACLE_PUBLISH_PERIOD_MS` | 100 | publish rate (10 Hz) |

## Local reflex stop

The sensor is not only published to ROS 2. `motor/cmd_vel.c` also reads the
debounced result every control cycle and brakes both motors when it sees an
obstacle. That reflex runs on the Pico, so it still works if Wi-Fi or the ROS 2
computer stalls. While the reflex is active, forward commands are blocked.
Reverse is never unrestricted: it is allowed only in small steps at
`CMD_VEL_IR_REFLEX_REVERSE_SPEED_M_S` for
`CMD_VEL_IR_REFLEX_REVERSE_STEP_MS`, followed by a brake and a pause of
`CMD_VEL_IR_REFLEX_REVERSE_PAUSE_MS`. A turn-in-place command is allowed during
that pause, so the robot can back away a little and turn around between steps.
The behaviour is configured in `../config/cmd_vel_config.h`:

| Setting | Default | Meaning |
|---|---|---|
| `CMD_VEL_IR_REFLEX` | 1 | 1 = brake locally on an obstacle, 0 = disable |
| `CMD_VEL_IR_REFLEX_REVERSE_STEPS` | 1 | 1 = allow stepped reverse, 0 = block all commands while blocked |
| `CMD_VEL_IR_REFLEX_REVERSE_SPEED_M_S` | 0.10 | capped reverse speed during a step (m/s) |
| `CMD_VEL_IR_REFLEX_REVERSE_STEP_MS` | 400 | how long one reverse step drives (ms) |
| `CMD_VEL_IR_REFLEX_REVERSE_PAUSE_MS` | 300 | brake pause between steps (ms); turning is allowed here |

The reflex logs one message when it engages and one when the path clears, so a
steady obstacle does not flood the log.

## micro-ROS note

This driver adds one publisher. `libmicroros.a` is built with
`RMW_UXRCE_MAX_PUBLISHERS=4`, so the log, IMU, `/joint_states`, and IR
publishers all fit. The `/cmd_vel` subscriber uses one of the two
`RMW_UXRCE_MAX_SUBSCRIPTIONS` slots. If you add more ROS entities, rebuild the
library with `docker/microros_builder/microros/colcon.meta`.
