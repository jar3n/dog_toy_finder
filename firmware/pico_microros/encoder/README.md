# Wheel encoders

`quadrature_encoder.c` / `quadrature_encoder.h` decode the two hall channels of
a TT encoded gearmotor, and `encoder.c` / `encoder.h` convert the raw count
into output-shaft position and velocity. `encoder_ros.c` / `encoder_ros.h`
publish both wheels as one `sensor_msgs/msg/JointState`.

The reference motor is
[nulllaborg/tt-encoder-motor](https://github.com/nulllaborg/tt-encoder-motor):
a 12 PPR hall disk on a 1:90 gearbox. With full 4x quadrature decoding that is

```
CPR = 12 PPR * 90 gear ratio * 4 = 4320 counts per output-shaft revolution
```

The decoder uses every edge of both channels, so it also detects direction.
At the motor's 113 RPM no-load speed each encoder produces about 8.1 kcounts/s,
well within what the GPIO interrupt handler can service.

## Wiring

Encoder pins live in `../config/pinout.h`; geometry and ROS settings live in
`../config/encoder_config.h`.

| Encoder lead | Pico 2 W | Notes |
|---|---|---|
| VCC | 3V3 | encoder is rated 3.3-5 V |
| GND | GND | common ground with the motor driver |
| A | GPIO7 (left), GPIO9 (right) | hall channel A |
| B | GPIO6 (left), GPIO8 (right) | hall channel B |
| Motor + / - | motor driver | never connect the motor to the Pico |

This is physical pins 9-12 on the Pico 2 W: left A/B on pins 9/10 and right A/B
on pins 11/12.

The hall outputs are open-drain, so the driver enables the Pico's internal
pull-ups on all four channels. No external resistors are needed. The four pins support edge interrupts.

```c
#define ENCODER_LEFT_PIN_A 6
#define ENCODER_LEFT_PIN_B 7
#define ENCODER_RIGHT_PIN_A 8
#define ENCODER_RIGHT_PIN_B 9
```

A and B are decoded independently of the motor driver: the encoder still works
when the motor is coasting or being pushed by hand.

## Direction

The sign of the count depends on which channel is wired to A and how the motor
is mounted. To calibrate:

1. Flash the firmware and open the serial/UDP log.
2. Push the robot straight forward by hand.
3. Both `enc:` counts should increase. If a side decreases, set
   `ENCODER_LEFT_INVERT` or `ENCODER_RIGHT_INVERT` to `1` in
   `config/encoder_config.h`, or swap that side's A/B wires.

Because the left and right motors are physically mirrored, one side usually
needs to be inverted for both joints to report positive rotation when the robot
drives forward. Match the sign convention used by the URDF/joint names.

## Configuration

`config/encoder_config.h` is git-ignored and created from
`config/encoder_config.example.h` by the build when missing. Important values:

| Setting | Default | Meaning |
|---|---|---|
| `ENCODER_PPR` | 12 | hall disk pulses per motor revolution |
| `ENCODER_GEAR_RATIO` | 90.0 | gearbox reduction |
| `ENCODER_QUADRATURE_MULTIPLIER` | 4 | full quadrature decoding |
| `ENCODER_WHEEL_RADIUS_M` | 0.0325 | used only for the linear-speed log |
| `ENCODER_LEFT_INVERT` / `ENCODER_RIGHT_INVERT` | 0 | flip counting direction |
| `ENCODER_VELOCITY_FILTER_ALPHA` | 0.3f | 0-1 low-pass weight; 1 disables it |
| `ENCODER_ROS_TOPIC` | `/joint_states` | JointState topic |
| `ENCODER_LEFT_JOINT_NAME` | `left_wheel_joint` | must match the URDF |
| `ENCODER_RIGHT_JOINT_NAME` | `right_wheel_joint` | must match the URDF |
| `ENCODER_PUBLISH_PERIOD_MS` | 20 | 50 Hz publishing |
| `ENCODER_LOG_PERIOD_MS` | 1000 | 1 Hz serial/UDP summary |

If `ENCODER_PPR`, `ENCODER_GEAR_RATIO`, or the multiplier differ from the
motor datasheet, change them here. `ENCODER_COUNTS_PER_REVOLUTION` is derived
automatically.

`ENCODER_WHEEL_RADIUS_M` defaults to a common 65 mm TT wheel. Measure the real
wheel before trusting the logged linear speed.

## ROS 2 publishing

`encoder_ros_init()` configures the GPIO decoders first and then attaches a
best-effort `sensor_msgs/msg/JointState` publisher to the shared micro-ROS node
created by `ros/ros_node.c`. If the agent is unreachable the decoders still
work and the main loop keeps logging; only the ROS publisher is skipped.

The message contains, in this fixed order:

| Field | Left | Right |
|---|---|---|
| `name` | `ENCODER_LEFT_JOINT_NAME` | `ENCODER_RIGHT_JOINT_NAME` |
| `position` | output-shaft radians | output-shaft radians |
| `velocity` | filtered rad/s | filtered rad/s |
| `effort` | empty | empty |

`header.stamp` comes from `ros_node_time_ns()` (agent epoch when synchronized,
otherwise time since boot). Position is unwrapped, so it keeps increasing as
the wheel turns instead of wrapping at 2π. `frame_id` is empty, which is the
usual convention for `JointState`.

Inspect it on the computer with the agent running:

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states
```

The micro-ROS library is built with `RMW_UXRCE_MAX_PUBLISHERS=3`. Logs, the
IMU, and this encoder publisher use all three slots. Increasing that limit
requires rebuilding `libmicroros.a` with the Docker builder.

## Usage

```c
#include "encoder/encoder_ros.h"

if (!encoder_ros_init()) {
    // GPIO setup failed; check config/pinout.h
}

// Every 20 ms:
if (encoder_ros_ready()) {
    encoder_ros_publish();   // samples both encoders, then publishes
} else {
    encoder_ros_update();    // keeps position/velocity fresh for logging
}
```

The main loop in `dtf_pico_fw_main.c` publishes at 50 Hz and logs counts,
angular velocity, and linear velocity at 1 Hz.

## API

| Function | Purpose |
|---|---|
| `encoder_ros_init()` | configure decoders and the ROS publisher |
| `encoder_ros_ready()` | true when the JointState publisher exists |
| `encoder_ros_update()` | sample both encoders once |
| `encoder_ros_publish()` | sample and publish one JointState |
| `encoder_ros_*_position_rad()` | latest output-shaft position |
| `encoder_ros_*_velocity_rad_s()` | latest filtered angular velocity |
| `encoder_ros_*_linear_velocity_m_s()` | angular velocity times wheel radius |
| `encoder_ros_*_count()` | raw signed 4x count |

The lower-level `encoder_init()`, `encoder_update()`, `encoder_reset()`, and
`quadrature_encoder_*()` functions can be used without ROS.

## Resetting

`encoder_reset(&encoder, time_us_64())` zeroes the count, position, and
velocity and restarts the velocity interval so the reset does not create a
spike. The ROS layer currently resets only at boot; add a call from a control
node if the robot needs a runtime zero.

## Notes

- The RP2350 SDK has one `gpio_set_irq_callback()` slot per core. This module
  installs it once and dispatches by GPIO number. If another module needs GPIO
  interrupts, add its pins to that shared callback instead of installing a
  second callback, which would replace this one.
- The raw count is a signed 32-bit value and wraps after about 73 hours of
  continuous running at full speed. Position accumulates 32-bit deltas, so it
  stays continuous across that wrap; only `encoder_ros_*_count()` shows the
  wrapped raw value.
- Interrupt latency matters more than CPU time here: each edge only reads two
  pins and updates one counter. The handler returns immediately for GPIOs it
  does not own.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| count never changes | encoder VCC/GND not connected, or wrong GPIO pins |
| count moves in the wrong direction | set the side's `*_INVERT`, or swap A/B |
| position doubles or halves | wrong PPR/gear ratio in `encoder_config.h` |
| velocity is noisy | lower `ENCODER_VELOCITY_FILTER_ALPHA` (for example 0.15) |
| velocity lags | raise the alpha toward `1.0f` |
| publisher init fails | all three micro-ROS publisher slots are in use |
| `/joint_states` motion looks inverted | joint sign convention differs from the URDF |
