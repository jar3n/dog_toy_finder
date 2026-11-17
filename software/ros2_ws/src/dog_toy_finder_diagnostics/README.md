# Sensor heartbeat health monitor

`dog_toy_finder_diagnostics/health_monitor.py` subscribes to every hardware
sensor topic and logs an **ERROR** on `/rosout` when a sensor's heartbeat goes
silent, so a dead sensor, Pico, camera, or link shows up as a logged failure
in ROS instead of silently degrading the robot.

## Monitored sensors

| Sensor | Topic | Type | Expected rate | Default timeout |
|---|---|---|---|---|
| Pico IMU (BNO055) | `/imu/data` | `sensor_msgs/msg/Imu` | 50 Hz | 0.5 s |
| Wheel encoders | `/joint_states` | `sensor_msgs/msg/JointState` | 50 Hz | 0.5 s |
| IR obstacle sensor | `/ir_obstacle/detected` | `std_msgs/msg/Bool` | 10 Hz | 1.0 s |
| ESP32-CAM left eye | `/stereo/left/image_raw` | `sensor_msgs/msg/Image` | 2 Hz | 3.0 s |
| ESP32-CAM right eye | `/stereo/right/image_raw` | `sensor_msgs/msg/Image` | 2 Hz | 3.0 s |
| Fused odometry (EKF) | `/odometry/filtered` | `nav_msgs/msg/Odometry` | ~50 Hz | 2.0 s |

The Pico firmware only publishes `/imu/data` when the BNO055 read succeeds, and
the stereo node only publishes images when both eyes delivered a photo, so
topic silence reliably means a failure — no extra firmware heartbeat message is
needed.

## Log behaviour

- Each sensor is checked every `check_period` (default 0.5 s).
- **Failure (ERROR):** logged once when a sensor exceeds its timeout, or when
  it still has never published after `startup_grace` (default 10 s, so normal
  boot-up does not spam failures).
- **Recovery (INFO):** logged once when a failed sensor publishes again.
- **First message (INFO):** logged once when a sensor starts reporting.

Watch the failures with:

```bash
ros2 topic echo /rosout
# or filter for errors:
ros2 topic echo /rosout | grep -E "level: 40|FAILED"
```

(`rcl_interfaces/msg/Log` levels: 40 = ERROR, 20 = INFO.) The same messages
also print to the console of the node that runs the monitor.

## Run

The node starts with the bringup launch (`dog_toy_finder_bringup
robot.launch.py`). Standalone:

```bash
ros2 run dog_toy_finder_diagnostics health_monitor
ros2 launch dog_toy_finder_diagnostics health.launch.py \
    imu_timeout:=1.0 startup_grace:=15.0
```

## Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `check_period` | `0.5` | seconds between heartbeat checks |
| `startup_grace` | `10.0` | seconds to wait for a sensor's first message |
| `imu_timeout` | `0.5` | seconds without `/imu/data` before failure |
| `joint_states_timeout` | `0.5` | seconds without `/joint_states` before failure |
| `ir_timeout` | `1.0` | seconds without `/ir_obstacle/detected` before failure |
| `left_camera_timeout` | `3.0` | seconds without left images before failure |
| `right_camera_timeout` | `3.0` | seconds without right images before failure |
| `odometry_timeout` | `2.0` | seconds without `/odometry/filtered` before failure |

Set timeouts to a few times the expected publish period to allow for Wi-Fi
jitter. A timeout of `0` or less is not special-cased; use the defaults unless
a sensor publishes slower than expected.

## Programmatic access

`HealthMonitor.sensor_states()` returns `{sensor: "ok"|"fail"|"unknown"}` and
`all_ok()` returns true when every sensor is reporting. Other nodes can import
the class if they need the same state.
