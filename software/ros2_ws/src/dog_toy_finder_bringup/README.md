# Dog Toy Finder bringup

Launch files and shared YAML for the computer-side real-robot stack.

| File | Purpose |
|---|---|
| `launch/robot.launch.py` | odometry, IMU mounting transform, EKF, `cmd_vel_mux`, sensor heartbeat health monitor |
| `config/robot.yaml` | wheel radius, wheel separation, joint names, odometry tuning |
| `config/ekf.yaml` | `robot_localization` inputs and frame names |

## Topics

| Topic | Type | Direction | Source |
|---|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | in | Pico wheel encoders |
| `/imu/data` | `sensor_msgs/msg/Imu` | in | Pico BNO055 |
| `/wheel/odom` | `nav_msgs/msg/Odometry` | internal | `dog_toy_finder_odometry` |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | out | `robot_localization` EKF |
| `/cmd_vel_auto` | `geometry_msgs/msg/Twist` | in | ROSA / autonomous search / test scripts |
| `/cmd_vel_manual` | `geometry_msgs/msg/Twist` | in | keyboard `manual_drive` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | out | `cmd_vel_mux` (the only Pico publisher) |
| `/cmd_vel_source` | `std_msgs/msg/String` | out | `cmd_vel_mux` (`manual`/`auto`/`released`/`none`) |

The EKF also publishes `odom -> base_link` and `robot_state_publisher` (added
later) publishes the wheel joint transforms.

`cmd_vel_mux` prevents two controllers from publishing over each other: manual
commands win while fresh, and autonomous commands resume only after a short
hands-off release window. Pass `cmd_vel_mux:=false` only if an external mux
owns `/cmd_vel`. The timeout parameters are listed in the motion package
README.

## Run

Start the micro-ROS agent on the host (it is a separate container):

```bash
docker compose -f docker/docker-compose.yml up -d micro_ros_agent
```

Then, inside the ROS 2 container:

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch dog_toy_finder_bringup robot.launch.py
```

Check that data is flowing:

```bash
ros2 topic hz /joint_states
ros2 topic hz /imu/data
ros2 topic echo /wheel/odom --once
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
```

The health monitor (`dog_toy_finder_diagnostics`) watches every sensor
heartbeat and logs an ERROR on `/rosout` when one goes silent — check with
`ros2 topic echo /rosout`. Pass `health_monitor:=false` to skip it. See that
package's README for timeouts.

## Tuning

- `wheel_radius` and `wheel_separation` must match
  `firmware/pico_microros/config/encoder_config.h` and
  `config/cmd_vel_config.h`.
- If a wheel counts backwards while the robot drives forward, flip
  `invert_left` or `invert_right` in `config/robot.yaml` (or the matching
  firmware invert flag) instead of changing the wiring.
- The EKF trusts the gyroscope for turn rate and the wheels for forward speed.
  Adjust `linear_speed_std` / `turn_rate_std` in `robot.yaml` if the fused pose
  lags or oscillates.
