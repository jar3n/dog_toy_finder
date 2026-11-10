# Dog Toy Finder Robot

A ROS 2 robot that finds dog toys: a robot arm mounted on a differential-drive
platform.

- **Local brain:** Raspberry Pi Pico (micro-ROS)
- **Link:** WiFi (UDP) to a computer running the micro-ROS agent
- **High-level brain:** ROSA (LLM agent) + ROS 2 nodes on the computer
- **Simulation:** Gazebo

## Layout

| Folder | What lives there | README |
|---|---|---|
| `firmware/pico_microros/` | Pico firmware: drivers, micro-ROS app, WiFi transport, build config | [README](firmware/pico_microros/README.md) |
| `firmware/esp_cam/` | ESP32-CAM stereo-vision firmware (left/right eye) | [README](firmware/esp_cam/README.md) |
| `software/ros2_ws/src/` | Computer-side ROS 2 packages | [README](software/ros2_ws/README.md) |
| `software/rosa_agent/` | ROSA agent, tools, prompts, tests | [README](software/rosa_agent/README.md) |
| `software/pico_ros2_led/` | Early LED connection test | |
| `docker/` | Containers: `ros2`, `gazebo`, `microros_builder` | [README](docker/README.md) |
| `scripts/` | Setup, build, flash, launch/stop helpers | |
| `config/` | Shared, non-secret configuration | |
| `data/` | Test images and logs | |
| `docs/` | Architecture, setup, design notes | [README](docs/README.md) |
| `hardware/` | CAD, electronics, BOM | |
| `simulation/` | Gazebo worlds, models, launch | |

### ROS 2 packages (`software/ros2_ws/src/`)

| Package | Purpose | README |
|---|---|---|
| `dog_toy_finder_bringup` | Launch files, EKF config, robot geometry | [README](software/ros2_ws/src/dog_toy_finder_bringup/README.md) |
| `dog_toy_finder_odometry` | `/joint_states` -> `/wheel/odom` | [README](software/ros2_ws/src/dog_toy_finder_odometry/README.md) |
| `dog_toy_finder_motion` | Closed-loop moves, `cmd_vel_mux`, manual keyboard drive + `mover_test` | [README](software/ros2_ws/src/dog_toy_finder_motion/README.md) |
| `dog_toy_finder_vision` | Stereo camera, VLM vision | [README](software/ros2_ws/src/dog_toy_finder_vision/README.md) |
| `dog_toy_finder_mapping` | Occupancy map + 3D web viewer | [README](software/ros2_ws/src/dog_toy_finder_mapping/README.md) |
| `dog_toy_finder_planning` | Autonomous search planner + path utilities | [README](software/ros2_ws/src/dog_toy_finder_planning/README.md) |
| `dog_toy_finder_data` | Data collection + evaluation | [README](software/ros2_ws/src/dog_toy_finder_data/README.md) |
| `dog_toy_finder_experiments` | Full-system search tests + evaluation | [README](software/ros2_ws/src/dog_toy_finder_experiments/README.md) |

### Firmware modules (`firmware/pico_microros/`)

| Module | Purpose | README |
|---|---|---|
| `imu/` | BNO055 IMU driver + `/imu/data` publisher | [README](firmware/pico_microros/imu/README.md) |
| `encoder/` | Quadrature encoders + `/joint_states` publisher | [README](firmware/pico_microros/encoder/README.md) |
| `motor/` | TB6612FNG driver + `/cmd_vel` subscriber | [README](firmware/pico_microros/motor/README.md) |
| `ir_obstacle/` | KS0051 IR obstacle sensor + `/ir_obstacle/detected` | [README](firmware/pico_microros/ir_obstacle/README.md) |
| `ros/` | micro-ROS session + ROS log publisher | [README](firmware/pico_microros/ros/README.md) |
| `wifi/` | Wi-Fi station + serial/UDP log mirror | [README](firmware/pico_microros/wifi/README.md) |

## Data flow

```
ROSA agent -> ROS 2 nodes (computer) -> micro-ROS agent --WiFi/UDP--> Pico firmware -> motors / arm / sensors
                    ^
                    +-- Gazebo simulation (stands in for the Pico + hardware)
```

The simulation exposes the same topics as the real robot, so the computer-side
code and ROSA work unchanged against either.

## Running the full system

See the full guide: [docs/setup/full_system_test.md](docs/setup/full_system_test.md).

Quick start (host):

```bash
cd docker
docker compose build
docker compose up -d micro_ros_agent
docker compose run --rm ros2
```

Inside the container:

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash

# one command to launch everything in tmux panes:
/workspace/scripts/launch_all.sh
# same, plus a keyboard manual-drive window (W/A/S/D):
/workspace/scripts/launch_all.sh --manual
# stop everything:
/workspace/scripts/stop_all.sh
```

Or run components individually:

```bash
ros2 launch dog_toy_finder_bringup robot.launch.py
ros2 launch dog_toy_finder_vision stereo.launch.py \
    left_url:=http://esp32cam-left.local/capture \
    right_url:=http://esp32cam-right.local/capture
ros2 launch dog_toy_finder_vision vision.launch.py models:="gemma3,llava"
ros2 launch dog_toy_finder_mapping mapping.launch.py
ros2 run dog_toy_finder_mapping map_webserver   # live map at http://<ip>:8765 (Save PNG button)
cd /workspace/software/rosa_agent && python3 -m rosa_agent
```

## Building firmware

- Pico: `firmware/scripts/pico_build/build.sh` -> UF2 in `firmware/pico_microros/build/artifacts/`.
- ESP32-CAM: `cd firmware/esp_cam && pio run -e esp32cam_left` (and `_right`).

## Testing

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install && source install/setup.bash
cd /workspace/software/rosa_agent && python3 -m unittest discover -s tests
cd /workspace/software/ros2_ws/src/dog_toy_finder_planning && python3 -m unittest discover -s tests
```

## Flashing

See [docs/setup/full_system_test.md](docs/setup/full_system_test.md) for flashing
the Pico (UF2) and both ESP32-CAM eyes, plus Wi-Fi configuration.
