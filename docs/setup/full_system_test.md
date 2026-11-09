# Full system test and evaluation guide

This guide covers flashing every firmware, bringing up the ROS 2 stack, and
running a complete evaluation of the robot finding a given pet toy. Both the
LLM (ROSA) planner and the autonomous planner are exercised and compared
using the shared CSV/evaluation tools.

## 1. Prerequisites

- Docker + Docker Compose (the `ros2` image is built from `docker/ros2/Dockerfile`).
- PlatformIO (`pio`) for the ESP32-CAM, Pico SDK toolchain for the Pico.
- A 2.4 GHz Wi-Fi network reachable by the Pico, both ESP32-CAMs and the
  computer.
- A measured/estimated toy (e.g. a red ball) and a clear floor area.

Build the ROS 2 image once:

```bash
cd docker
docker compose build
```

## 2. Flash the Pico firmware

```bash
cd firmware/pico_microros
# Create the local Wi-Fi config (git-ignored) from its template:
../../scripts/setup_firmware_config.sh
# Set the SSID/password in config/wifi_config.h. Then edit the committed
# configs in place: ros_config.h (agent IP/port), encoder_config.h,
# motor_config.h, cmd_vel_config.h, imu_config.h, ir_obstacle_config.h,
# and pinout.h.
```

Build:

```bash
cd ../../   # repo root
firmware/scripts/pico_build/build.sh
```

The UF2 is staged at `firmware/pico_microros/build/artifacts/dtf_pico_fw.uf2`.
Hold BOOTSEL on the Pico, plug it in, then copy the UF2 to the mounted drive.

Geometry defaults (Module 8 course robot): wheel diameter 0.065 m, wheel
separation 0.133 m, 4320 counts/rev. Refine in `encoder_config.h` and
`cmd_vel_config.h` for your chassis.

## 3. Flash the ESP32-CAM (stereo eyes)

```bash
cd firmware/esp_cam
cp wifi_config.example.h wifi_config.h   # fill in SSID/password (2.4 GHz only)
```

Flash the left eye and the right eye (PlatformIO):

```bash
pio run -e esp32cam_left  -t upload
pio run -e esp32cam_right -t upload
```

Or with the Arduino IDE: open `esp32_cam_stereo.ino`, select board
"AI Thinker ESP32-CAM", set `CAMERA_SIDE` to `"left"` / `"right"`, and upload.

Note the printed IPs / mDNS names (`esp32cam-left.local`, `esp32cam-right.local`).

## 4. Start the ROS 2 stack

A tmux-based launcher opens one pane per component:

```bash
# on the host:
./scripts/launch_all.sh
# or, already inside the ros2 container:
./scripts/launch_all.sh
```

Stop everything with:

```bash
./scripts/stop_all.sh
```

On the host:

```bash
cd docker
docker compose up -d micro_ros_agent
```

In the ROS 2 container:

```bash
docker compose run --rm ros2
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Terminal A: robot stack (odometry + EKF)
ros2 launch dog_toy_finder_bringup robot.launch.py

# Terminal B: stereo vision (images + disparity + point cloud)
ros2 launch dog_toy_finder_vision stereo.launch.py \
    left_url:=http://esp32cam-left.local/capture \
    right_url:=http://esp32cam-right.local/capture

# Terminal C: VLM vision (optional, for find_toy)
ros2 launch dog_toy_finder_vision vision.launch.py \
    models:="gemma3,llava"   # or a single model
```

Check data flow:

```bash
ros2 topic hz /joint_states
ros2 topic hz /imu/data
ros2 topic hz /stereo/disparity
ros2 topic echo /odometry/filtered --once
```

## 5. Run the full test evaluation

### 5.1 LLM (ROSA) planner

```bash
cd /workspace/software/rosa_agent
# compare several models on one prompt:
python3 -m rosa_agent --compare llama3.2,mistral,qwen2.5 --prompt "Find the red ball."
# or interactive:
python3 -m rosa_agent
```

Or drive a search with the test harness (uses ROSA tools under the hood is not
automatic; use the autonomous node for deterministic runs, or drive manually
and log with the data collector).

### 5.2 Autonomous (deterministic) planner

```bash
ros2 launch dog_toy_finder_planning autonomous_search.launch.py \
    target:="red ball" side:=0.5 loops:=3 max_time:=120 \
    output_csv:=/tmp/auto1.csv run_id:=auto1
```

### 5.3 LLM-driven search via toy_search harness (baseline)

```bash
ros2 launch dog_toy_finder_experiments toy_search.launch.py \
    target:="red ball" side:=0.5 loops:=3 max_time:=120 \
    output_csv:=/tmp/llm1.csv run_id:=llm1
```

### 5.4 Collect richer data (optional)

```bash
ros2 launch dog_toy_finder_data collect.launch.py \
    output_dir:=/tmp/dog_toy_dataset run_id:=run1 image_every:=5
```

## 6. Evaluate

```bash
ros2 run dog_toy_finder_experiments evaluate_runs /tmp/llm1.csv /tmp/auto1.csv
ros2 run dog_toy_finder_data dataset_summary /tmp/dog_toy_dataset/run1
```

Metrics reported: success, time/steps to find, path length, vision call count
and average latency, target-visible rate, final pose.

## 6.5 Viewing the map

The occupancy mapper publishes `/map` (nav_msgs/OccupancyGrid) and
`/tracked_path` (nav_msgs/Path). Start the browser viewer:

```bash
ros2 run dog_toy_finder_mapping map_webserver
# next line of the log prints the URL, e.g. http://192.168.1.50:8765
```

Open that URL on any computer or phone on the same network. The 3D view
updates automatically. Click **Save PNG** to download the current view to the
computer running the browser; **Reset view** restores the camera angle.

The old `save_map` and `view_map` commands have been removed. If you need a
one-off screenshot on the robot computer, use the browser's download instead
of writing files on the robot.

## 7. Unit tests

```bash
cd /workspace/software/rosa_agent && python3 -m unittest discover -s tests
cd /workspace/software/ros2_ws/src/dog_toy_finder_planning && python3 -m unittest discover -s tests
```

## Notes

- Keep `ROSA_REQUIRE_APPROVAL=1` during development to avoid unintended motion.
- The Pico's 500 ms `/cmd_vel` watchdog stops the robot if the computer or
  Wi-Fi link stalls.
- All geometry values must agree between `encoder_config.h`,
  `cmd_vel_config.h`, `robot.yaml`, and the launch defaults.
