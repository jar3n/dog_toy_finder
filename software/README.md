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
    dog_toy_finder_vision/      stereo capture, disparity/point cloud, selectable vision pipeline
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
                                        (open_vocab: two-eye triangulation;
                                         stereo-net: one-eye detection + dense
                                         stereo depth) -> /vision/result
                                             |                         |
                                             |                         +-> occupancy_mapper
                                             |                             -> /map, /detected_toys
                                             |                             -> map_webserver -> browser
                                             +-> vision_capture -> saved L/R shots + meta.json
                                                                  -> vision_eval (offline)
                                                                      -> vision_eval_trials.csv
```

The vision node is request-driven: planners and ROSA publish a JSON request on
`/vision/ask` and read the matching answer on `/vision/result`. Select the
pipeline with `pipeline:=open_vocab|stereo-net` at launch, `VISION_PIPELINE`, or the
top-level `pipeline` key in `vision_detectors.yaml`. `open_vocab` triangulates
two-eye detections; `stereo-net` detects in the left eye and reads depth from a
dense stereo map. All answers use a numeric `lateral_bearing_deg`
(`atan((cx - u) / fx)`, positive = robot-left). For fair comparison, capture
several rectified shots per scene with `vision_capture` and replay them offline
with `vision_eval`; no live pose or detector is needed at capture time.

## Quick start

On the host, from the repository root, build the image once and start the
micro-ROS agent. The agent is its own container because the ROS 2 image does not
ship the `micro_ros_agent` package.

```bash
docker compose -f docker/docker-compose.yml build
docker compose -f docker/docker-compose.yml up -d micro_ros_agent
docker compose -f docker/docker-compose.yml run --rm ros2
```

On the host, before starting the vision node, start the detector service
(`./scripts/start_yolo_service.sh`, one detector at a time via
`DETECTOR_MODEL=yolo_world|yoloe_26|grounding_dino`). For the `stereo-net` pipeline
also start the dense-stereo depth service
(`./scripts/start_stereo_service.sh`, one model at a time via
`DEPTH_MODEL=lite_any_stereo|fast_foundation_stereo|rt_igevpp`). Both keep torch
and weights out of the container; see
[`software/ros2_ws/src/dog_toy_finder_vision/README.md`](ros2_ws/src/dog_toy_finder_vision/README.md)
for the full host setup.

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

# cameras + stereo depth + open-vocabulary vision pipeline
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    pipeline:=open_vocab

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
