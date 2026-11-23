# Computer-side software

Everything that runs on the computer and talks to the robot over Wi-Fi. The
layout keeps the ROS 2 graph separate from the LLM agent, mirroring the firmware
convention of one folder per concern with its own README.

```text
software/
  ros2_ws/src/
    dog_toy_finder_bringup/     launch files, EKF config, robot geometry
    dog_toy_finder_odometry/    /joint_states -> /wheel/odom
    dog_toy_finder_motion/      closed-loop moves, cmd_vel mux, manual keyboard drive
    dog_toy_finder_vision/      stereo capture, disparity/point cloud, Ollama VLM detector
    dog_toy_finder_mapping/     occupancy map, toy markers, browser map server
    dog_toy_finder_planning/    autonomous search planner
    dog_toy_finder_diagnostics/ sensor and camera heartbeat monitor
    dog_toy_finder_data/        dataset collection and summary
    dog_toy_finder_experiments/ scripted search harness and evaluation
  rosa_agent/                   LLM agent, prompts, and agent-facing tools
```

## Data flow

```text
Pico firmware                         Computer
-------------                         --------
/joint_states  --Wi-Fi/UDP-->  micro-ROS agent
/imu/data      --Wi-Fi/UDP-->       |
/ir_obstacle/detected --Wi-Fi/UDP-->|
                                    v
                       dog_toy_finder_odometry -> /wheel/odom
                                    |
                                    v
                        robot_localization EKF -> /odometry/filtered
                                    |
                                    v
        ROSA / search -> /cmd_vel_auto --\
                                        >-- cmd_vel_mux -> /cmd_vel -> Pico
        keyboard drive -> /cmd_vel_manual --/

ESP32-CAM left/right --HTTP--> stereo_camera_node -> image_proc
                                    -> stereo_image_proc -> /stereo/disparity, /stereo/points2
                                    -> rectified colour pair -> vision_node
                                        (Ollama vision models) -> /vision/result
                                             |                         |
                                             |                         +-> occupancy_mapper
                                             |                             -> /map, /detected_toys
                                             |                             -> map_webserver -> browser
                                             +-> vision_capture -> saved L/R JPEGs + meta.json
                                                                  -> vision_eval (offline)
                                                                      -> vision_eval_trials.csv
```

The vision node is request-driven: planners and ROSA publish a JSON request on
`/vision/ask` and read the matching answer on `/vision/result`. All answers use
a numeric `lateral_bearing_deg` (positive = robot-left). For fair model
comparison, capture rectified pairs once with `vision_capture` and replay them
offline with `vision_eval`; no live pose or Ollama server is needed at capture
time.

## Quick start

On the host, from the repository root, build the image once and start the
micro-ROS agent. The agent is its own container because the ROS 2 image does not
ship the `micro_ros_agent` package.

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

# whole stack, one pane per component:
/workspace/scripts/launch_all.sh
# plus keyboard manual drive:
/workspace/scripts/launch_all.sh --manual
```

Or run components individually:

```bash
# odometry + EKF + cmd_vel_mux + health monitor
ros2 launch dog_toy_finder_bringup robot.launch.py

# cameras + stereo depth + Ollama vision models
ros2 launch dog_toy_finder_vision stereo_vision.launch.py

# occupancy map and browser viewer
ros2 launch dog_toy_finder_mapping mapping.launch.py
ros2 run dog_toy_finder_mapping map_webserver

# test a move, drive with the keyboard, or use natural language
ros2 run dog_toy_finder_motion mover_test straight 0.5
ros2 run dog_toy_finder_motion manual_drive
cd /workspace/software/rosa_agent && python3 -m rosa_agent
```

Stop the agent with
`docker compose -f docker/docker-compose.yml stop micro_ros_agent`.

## QoS compatibility

The Pico publishes `/joint_states`, `/imu/data`, and subscribes to `/cmd_vel`
with **best effort** reliability. `/cmd_vel` is fed by `cmd_vel_mux`, the only
publisher on that topic; its reliable publisher is compatible with the Pico's
best-effort subscription. The computer side matches the rest:

- `dog_toy_finder_odometry` reads `/joint_states` with `qos_profile_sensor_data`
  (best effort).
- `robot_localization` reads `odom0` and `imu0` with `rclcpp::SensorDataQoS`
  (best effort) in Humble, so the firmware IMU publisher connects directly.
- `Mover` and `cmd_vel_mux` read `/odometry/filtered` best effort, which a
  reliable publisher also satisfies. `Mover` never subscribes to `/wheel/odom`;
  it is only the EKF's input.

If you change a firmware publisher to reliable, the existing subscribers still
work. If you change a subscriber to reliable, the matching firmware publisher
must become reliable too, or the two will not connect.

## Not implemented

- The robot has no arm/gripper, so ROSA has no grasping tool.
- There is no URDF, `robot_state_publisher`, or Gazebo simulation package; TF
  currently covers `odom -> base_link`, not the individual wheel joints.

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
