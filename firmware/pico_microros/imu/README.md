# BNO055 IMU

`bno055.c` / `bno055.h` drive a BNO055 9-DoF IMU over I2C. The driver resets
the sensor, verifies the chip ID, selects metric units, and puts the sensor in
**NDOF** mode, where the accelerometer, gyroscope, and magnetometer are fused
into an absolute orientation.

## Wiring

The bus and pins live in `../config/pinout.h`:

| BNO055 | Pico 2 W | Notes |
|---|---|---|
| VIN | 3V3 | 3.3 V only |
| GND | GND | common ground |
| SDA | GPIO4 | I2C0 SDA, physical pin 6 |
| SCL | GPIO5 | I2C0 SCL, physical pin 7 |
| ADDR | GND | address `0x28`; tie high for `0x29` |

```c
#define BNO055_I2C i2c0
#define BNO055_I2C_SDA_PIN 4
#define BNO055_I2C_SCL_PIN 5
#define BNO055_I2C_FREQUENCY_HZ 400000
#define BNO055_I2C_ADDRESS 0x28
```

Change those values (and the `ADDR` wiring / address) if the IMU is connected
elsewhere. The pins must be a valid pair for the selected I2C instance. Most
BNO055 breakouts include the required pull-ups.

## Usage

```c
#include "imu/bno055.h"

if (!bno055_init()) {
    // no sensor on the bus, or the reset/chip-ID check failed
}

bno055_euler_t euler;
if (bno055_read_euler(&euler)) {
    printf("heading=%.1f roll=%.1f pitch=%.1f\n",
           euler.heading, euler.roll, euler.pitch);
}
```

`bno055_init()` blocks for about a second while the sensor boots. It returns
false when the chip ID is not `0xA0`, which is the usual symptom of wrong
pins, a bad address, or missing pull-ups.

## ROS 2 publishing

`imu_ros.c` / `imu_ros.h` attach a `sensor_msgs/msg/Imu` publisher to the
shared micro-ROS node created by `ros/ros_node.c`. Only one node is allowed by
the micro-ROS configuration, so IMU data and log messages share it.

```c
#include "imu/imu_ros.h"

if (bno055_init() && imu_ros_init()) {
    imu_ros_publish();   // call at the desired rate, e.g. every 20 ms
}
```

`imu_ros_init()` calls `ros_node_init()`, so it also works when
`ros_log_init()` was not called. If the agent is unreachable, it returns false
and the rest of the firmware keeps running.

The message contains:

| Field | Source |
|---|---|
| `header.stamp` | `ros_node_time_ns()` (agent epoch, or time since boot) |
| `header.frame_id` | `IMU_ROS_FRAME_ID` |
| `orientation` | BNO055 fused quaternion (NDOF) |
| `angular_velocity` | gyro, converted from degrees/s to rad/s |
| `linear_acceleration` | linear acceleration, gravity removed, m/s^2 |
| `*_covariance` | diagonal from `IMU_ROS_*_VARIANCE` |

Settings live in `config/imu_config.h`, which is git-ignored and generated
from `config/imu_config.example.h` when missing:

```c
#define IMU_ROS_TOPIC "/imu/data"
#define IMU_ROS_FRAME_ID "imu_link"
#define IMU_ROS_QUEUE_DEPTH 10

#define IMU_ROS_ORIENTATION_VARIANCE 0.01
#define IMU_ROS_ANGULAR_VELOCITY_VARIANCE 0.01
#define IMU_ROS_LINEAR_ACCELERATION_VARIANCE 0.04
```

Inspect it on the computer with the agent running:

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data
```

The main loop in `dtf_pico_fw_main.c` publishes at 50 Hz and logs heading,
roll, pitch, and calibration at 2 Hz.

## API

| Function | Output |
|---|---|
| `bno055_read_euler()` | heading/roll/pitch in degrees |
| `bno055_read_quaternion()` | `w, x, y, z` unit quaternion |
| `bno055_read_acceleration()` | m/s^2, includes gravity |
| `bno055_read_linear_acceleration()` | m/s^2, gravity removed |
| `bno055_read_gravity()` | m/s^2 |
| `bno055_read_gyroscope()` | degrees/s |
| `bno055_read_magnetometer()` | microtesla |
| `bno055_read_temperature()` | degrees Celsius |
| `bno055_read_calibration()` | system/gyro/accel/mag levels, each 0-3 |
| `bno055_wait_for_calibration(ms)` | blocks until all levels reach 3 |
| `bno055_set_operation_mode(mode)` | switches mode (`BNO055_MODE_*`) |
| `bno055_is_connected()` | probes the bus without reconfiguring |
| `bno055_reset()` | hardware reset; leaves the sensor in CONFIG mode |

Every read returns `false` when the I2C transfer fails, so callers should check
the result instead of trusting a stale struct. `imu_ros_publish()` returns
false when either the BNO055 read or the ROS publish fails.

## Calibration

The BNO055 calibrates at runtime and does not keep calibration across power
cycles. Move the robot slowly through different orientations until
`bno055_read_calibration()` reports 3 for all four sensors, or call
`bno055_wait_for_calibration()` during setup. Until then, heading is the least
trustworthy value.

## Notes

- Reads are blocking I2C transactions at 400 kHz. A burst read of all Euler
  angles or a vector takes well under a millisecond on a healthy bus.
- NDOF mode runs at 100 Hz internally; polling at 50-100 Hz is plenty for the
  robot's needs.
- The magnetometer is sensitive to nearby motors and current-carrying wires.
  Mount the IMU away from the drive motors and keep its axes aligned with the
  robot's forward/left/up convention.
- `bno055_init()` performs a full reset, so call it once at boot rather than in
  the control loop.
