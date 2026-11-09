# Computer-side software

Everything that runs on the computer and talks to the robot over Wi-Fi. The
layout keeps the ROS 2 graph separate from the LLM agent, mirroring the firmware
convention of one folder per concern with its own README.

```text
software/
  ros2_ws/src/
    dog_toy_finder_bringup/    launch files, EKF config, robot geometry
    dog_toy_finder_odometry/   /joint_states -> /wheel/odom
    dog_toy_finder_motion/     closed-loop moves used by tests and ROSA
    dog_toy_finder_diagnostics/ sensor heartbeat health monitor (logged failures)
  rosa_agent/                  LLM agent, prompts, and agent-facing tools
  pico_ros2_led/               early LED connection test
```

## Data flow

```text
Pico firmware                         Computer
-------------                         --------
/joint_states  --Wi-Fi/UDP-->  micro-ROS agent
/imu/data      --Wi-Fi/UDP-->       |
                                    v
                       dog_toy_finder_odometry -> /wheel/odom
                                    |
                                    v
                        robot_localization EKF -> /odometry/filtered
                                    |
                                    v
                              ROSA tools -> /cmd_vel -> Pico motors
```

The simulation is expected to expose the same topics later, so the ROS 2 nodes
and ROSA tools work unchanged against Gazebo.

## Quick start

On the host, from the repository root, build the images once and start the
micro-ROS agent. The agent is its own container because the ROS 2 image does
not ship the `micro_ros_agent` package.

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up -d micro_ros_agent
docker compose -f docker/docker-compose.yml run --rm ros2
```

Inside the ROS 2 container:

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash

# odometry + EKF
ros2 launch dog_toy_finder_bringup robot.launch.py
```

In another shell in the same container:

```bash
source /workspace/software/ros2_ws/install/setup.bash

# test a move
ros2 run dog_toy_finder_motion mover_test straight 0.5

# natural-language control
cd /workspace/software/rosa_agent
python3 -m rosa_agent
```

Stop the agent with `docker compose -f docker/docker-compose.yml stop micro_ros_agent`.

## QoS compatibility

The Pico publishes `/joint_states`, `/imu/data`, and subscribes to `/cmd_vel`
with **best effort** reliability. The computer side matches it:

- `dog_toy_finder_odometry` reads `/joint_states` with `qos_profile_sensor_data`
  (best effort).
- `robot_localization` reads `odom0` and `imu0` with `rclcpp::SensorDataQoS`
  (best effort) in Humble, so the firmware IMU publisher connects directly.
- `Mover` reads `/odometry/filtered` and `/wheel/odom` best effort, which a
  reliable publisher also satisfies.

If you change a firmware publisher to reliable, the existing subscribers still
work. If you change a subscriber to reliable, the matching firmware publisher
must become reliable too, or the two will not connect.

## What is not here yet

- Vision and the arm: the ESP32-CAM firmware and arm control are not written
  yet, so ROSA has no seeing or grasping tools.
- Gazebo simulation: add a `dog_toy_finder_sim` package that publishes the same
  topics and accepts `/cmd_vel`.
- Robot description: `robot_state_publisher` and a URDF are needed to complete
  the TF tree (`base_link -> left_wheel_joint`, `right_wheel_joint`).

## Related firmware

The Pico side lives in `firmware/pico_microros/`. Its config values that must
match this code are:

| Computer value | Firmware file |
|---|---|
| wheel radius, wheel separation | `config/encoder_config.h`, `config/cmd_vel_config.h` |
| joint names | `config/encoder_config.h` |
| IMU topic and frame | `config/imu_config.h` |
| `/cmd_vel` topic | `config/cmd_vel_config.h` |
| micro-ROS agent IP/port | `config/ros_config.h` |
