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
| S | GPIO10 | digital signal, active high by default |
| EN | GPIO11 | high enables the sensor |

These pins are free on the Pico 2 W and do not overlap the encoders
(GPIO6-9), the IMU (GPIO14/15), the motor controller (GPIO16-22), or the
wireless chip (GPIO23/24/25/29).

## Configuration

Pins live in `../config/pinout.h`; timing and topic live in
`../config/ir_obstacle_config.h` (created from the `.example.h` by the build).

| Setting | Default | Meaning |
|---|---|---|
| `IR_OBSTACLE_TOPIC` | `/ir_obstacle/detected` | ROS topic for the Bool result |
| `IR_OBSTACLE_DEBOUNCE_MS` | 50 | ignore changes shorter than this |
| `IR_OBSTACLE_ACTIVE_LEVEL` | 1 | 1 = obstacle when signal is HIGH |
| `IR_OBSTACLE_PUBLISH_PERIOD_MS` | 100 | publish rate (10 Hz) |

## micro-ROS note

This driver adds one publisher, bringing the total to four. The default
`libmicroros.a` is built with `RMW_UXRCE_MAX_PUBLISHERS=3`, so the publisher
init will fail until the library is rebuilt with `=4` (see
`docker/microros_builder/microros/colcon.meta`). The sensor still reads and
logs over Wi-Fi/serial in the meantime.