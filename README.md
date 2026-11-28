# Dog Toy Finder Robot

A differential-drive ROS 2 robot that searches for dog toys with a stereo camera
pair and a selectable vision pipeline: `triangulation` (two-eye detection and
disparity triangulation) or `stereo-net` (one-eye detection plus dense-stereo
depth).

- **Local brain:** Raspberry Pi Pico 2 W running micro-ROS
- **Link:** Wi-Fi (UDP) to a computer running the micro-ROS agent
- **High-level brain:** ROS 2 nodes plus the ROSA LLM agent on the computer
- **Eyes:** two ESP32-CAM boards providing a stereo pair
- **Map:** browser-based live occupancy view with distinct toy/IR/vision layers

## What it does

- Captures a stereo pair, rectifies it, computes disparity and a point cloud,
  then runs a selectable vision pipeline. `triangulation` uses one host-side
  open-vocabulary detector (YOLO-World, YOLOE-26, or Grounding DINO), matches
  detections between the eyes, and triangulates the target's location from
  disparity using `fx` and the stereo baseline. `stereo-net` uses the same detector
  in the left eye only and reads depth from a dense disparity map produced by a
  host stereo model (Lite Any Stereo, Fast Foundation Stereo, or RT-IGEV++),
  taking the median over the object's mask pixels (YOLOE-26) or a small window
  around its box centre (box-only detectors). YOLOE-26-seg reports the centroid
  of its instance mask; the box-only models report their box centre.
- Runs either a deterministic expanding-square search or a ROSA/LLM-driven
  search, for one toy or a comma-separated list of toy kinds.
- Records poses/vision results to CSV and evaluates success, time, path length,
  and vision latency. A shared trial schema scores vision and planning runs
  side by side against ground truth.
- Serves a live 3D map in any browser on the network, with **Save PNG** writing
  the image to the browser's computer.
- Keeps the robot safe locally: the Pico brakes on its IR obstacle sensor and
  stops if `/cmd_vel` goes quiet. ROS learns when either failsafe fires.

## Layout

| Folder | What lives there | README |
|---|---|---|
| `firmware/pico_microros/` | Pico firmware: drivers, micro-ROS app, Wi-Fi transport, build config | [README](firmware/pico_microros/README.md) |
| `firmware/esp_cam/` | ESP32-CAM stereo-vision firmware (left/right eye) | [README](firmware/esp_cam/README.md) |
| `firmware/scripts/` | Pico build/flash helpers and the Wi-Fi log listener | |
| `software/ros2_ws/src/` | Computer-side ROS 2 packages | [README](software/ros2_ws/README.md) |
| `software/rosa_agent/` | ROSA agent, tools, prompts, tests | [README](software/rosa_agent/README.md) |
| `tests_e2e/` | End-to-end search tests against a fake robot and scripted vision | [README](tests_e2e/README.md) |
| `docker/` | Containers: `ros2`, `microros_builder`, and the micro-ROS agent | [README](docker/README.md) |
| `scripts/` | Setup, launcher, manual drive, vision test, E2E runner | |
| `docs/` | Install and full-system test/evaluation guides | [README](docs/README.md) |
| `data/` | Generated vision and planning comparison trials | [README](data/README.md) |
| `hardware/` | Text wiring reference and pin summary | [Wiring](hardware/wiring.md) |

### ROS 2 packages (`software/ros2_ws/src/`)

| Package | Purpose | README |
|---|---|---|
| `dog_toy_finder_bringup` | Launch files, EKF config, robot geometry, `cmd_vel_mux` start | [README](software/ros2_ws/src/dog_toy_finder_bringup/README.md) |
| `dog_toy_finder_odometry` | `/joint_states` -> `/wheel/odom` | [README](software/ros2_ws/src/dog_toy_finder_odometry/README.md) |
| `dog_toy_finder_motion` | Closed-loop moves, `cmd_vel_mux`, manual keyboard drive, `mover_test` | [README](software/ros2_ws/src/dog_toy_finder_motion/README.md) |
| `dog_toy_finder_vision` | Stereo rectification, selectable vision pipeline behind `/vision/ask`, capture-first `vision_capture`/`vision_eval` | [README](software/ros2_ws/src/dog_toy_finder_vision/README.md) |
| `dog_toy_finder_mapping` | Occupancy map, toy markers, browser map server | [README](software/ros2_ws/src/dog_toy_finder_mapping/README.md) |
| `dog_toy_finder_planning` | Autonomous search planner + path utilities | [README](software/ros2_ws/src/dog_toy_finder_planning/README.md) |
| `dog_toy_finder_diagnostics` | Sensor/camera heartbeat monitor with logged failures | [README](software/ros2_ws/src/dog_toy_finder_diagnostics/README.md) |
| `dog_toy_finder_data` | Dataset collection plus LLM vision/planning comparison trials | [README](software/ros2_ws/src/dog_toy_finder_data/README.md) |
| `dog_toy_finder_experiments` | Scripted search harness and run evaluation | [README](software/ros2_ws/src/dog_toy_finder_experiments/README.md) |

### Firmware modules (`firmware/pico_microros/`)

| Module | Purpose | README |
|---|---|---|
| `imu/` | BNO055 IMU driver + `/imu/data` publisher | [README](firmware/pico_microros/imu/README.md) |
| `encoder/` | Quadrature encoders + `/joint_states` publisher | [README](firmware/pico_microros/encoder/README.md) |
| `motor/` | TB6612FNG driver, `/cmd_vel` subscriber, watchdog, IR reflex | [README](firmware/pico_microros/motor/README.md) |
| `ir_obstacle/` | KS0051 IR obstacle sensor + `/ir_obstacle/detected` | [README](firmware/pico_microros/ir_obstacle/README.md) |
| `status_led/` | Built-in LED: blinking = booting, solid = ready, off = failed | [README](firmware/pico_microros/status_led/README.md) |
| `ros/` | micro-ROS session + ROS log publisher | [README](firmware/pico_microros/ros/README.md) |
| `wifi/` | Wi-Fi station + serial/UDP log mirror | [README](firmware/pico_microros/wifi/README.md) |
| `udp_transport/` | Custom micro-ROS UDP transport over Wi-Fi | |
| `microros_time/` | Agent time synchronization helper | |

## Data flow

```text
ROS 2 nodes / ROSA (computer)
        |
        +--> /cmd_vel_auto ----\
        |                       >-- cmd_vel_mux --> /cmd_vel
        +--> /cmd_vel_manual ---/                      |
                                                       v
                                        micro-ROS agent --Wi-Fi/UDP--> Pico
                                                       |
                                          motors, encoders, IMU, IR sensor
                                                       |
                             /joint_states, /imu/data, /ir_obstacle/detected
                                                       |
                                                       v
                                 dog_toy_finder_odometry -> EKF -> /odometry/filtered
```

Vision runs on the computer: the two ESP32-CAMs serve still frames over HTTP,
`stereo_vision.launch.py` turns them into images, disparity and a point cloud,
and `vision_node` answers `/vision/ask` requests from the planner, the search
harness, or ROSA using the selected vision pipeline. Choose it with the launch
arg `pipeline:=triangulation|stereo-net`, the `VISION_PIPELINE` environment variable,
or the top-level `pipeline` key in `vision_detectors.yaml`. Both pipelines are
implemented; `stereo-net` uses one-image detection plus dense-stereo depth so
the two approaches can be compared. Vision answers report `distance_m` along
the optical axis, signed `center_offset_m` from the optical-axis center, and
`lateral_bearing_deg` = `atan(center_offset_m / distance_m)` in degrees (positive
= robot-left / ROS `+y`); no coarse side label is emitted or accepted. For
comparison, `vision_capture` saves several rectified shots per scene with
operator-measured ground truth, and `vision_eval` replays every shot offline
through both pipelines in one command, writing separate per-pipeline outputs
plus a combined CSV. Host models are unloaded between pipelines so the next
configuration loads fresh weights. A seed-recorded order shuffles scenes and the
shots within each scene; `--pipeline NAME` evaluates only one pipeline.

## Running the full system

See [docs/setup/full_system_test.md](docs/setup/full_system_test.md) for
flashing, the complete evaluation flow, and reset instructions, and
[docs/setup/camera_calibration.md](docs/setup/camera_calibration.md) for stereo
intrinsics and baseline calibration.

Quick start (host):

```bash
cd docker
docker compose build
docker compose up -d micro_ros_agent
docker compose run --rm ros2
```

The `ros2` image has only Pillow (to convert ROS images to JPEG); detector and
dense-stereo inference run on the computer, not in Docker. Start the host vision
services in the background once with `./scripts/start_host_vision_services.sh`
(or start them individually with `./scripts/start_yolo_service.sh` and
`./scripts/start_stereo_service.sh`). On first run they create a local venv,
install CPU torch/ultralytics/transformers, and download all supported weights.
No model variable is needed: the vision node or `vision_eval` selects the
backend with `POST /configure` using `open_vocab.detector` and `stereo-net.model`
in `software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml`.
Stop both services with `./scripts/stop_host_vision_services.sh`. No vision
Ollama server is needed; Ollama is used separately by the ROSA planner.

Inside the container:

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash

# launch the whole stack in Konsole split panes (tmux fallback):
/workspace/scripts/launch_all.sh
# same, plus a keyboard manual-drive window (W/A/S/D):
/workspace/scripts/launch_all.sh --manual
# stop everything: Ctrl+C in the launch_all.sh terminal
```

Per-run launch values (camera URLs, vision pipeline choice and detector name,
map size, search target, CSV paths, manual-drive speeds) come from
`scripts/full_system_eval.conf`. Camera geometry is the one exception: that is
read from `vision_detectors.yaml` when the config is sourced, so there is no
second copy to keep in sync. Edit those files, pass another config file, or
override a value in the environment:

```bash
WEBMAP_PORT=9000 ./scripts/launch_all.sh
```

Or run components individually:

```bash
ros2 launch dog_toy_finder_bringup robot.launch.py
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    left_url:=http://esp32cam-left.local/capture \
    right_url:=http://esp32cam-right.local/capture
ros2 launch dog_toy_finder_mapping mapping.launch.py
ros2 run dog_toy_finder_mapping map_webserver   # live map (prints its URL; Save PNG works there)
cd /workspace/software/rosa_agent && python3 -m rosa_agent
```

Test the vision pipeline without ROSA:

```bash
python3 /workspace/scripts/vision_test.py "red ball"
```

## Building firmware

- Pico: `firmware/scripts/pico_build/build.sh` -> UF2 in
  `firmware/pico_microros/build/artifacts/`.
- ESP32-CAM: `cd firmware/esp_cam && pio run -e esp32cam_left` (and `_right`).

## Testing

```bash
# rebuild, then run the unit suites
cd /workspace/software/ros2_ws && colcon build --symlink-install && source install/setup.bash
cd /workspace/software/rosa_agent && python3 -m unittest discover -s tests
cd /workspace/software/ros2_ws/src/dog_toy_finder_planning && python3 -m unittest discover -s tests
cd /workspace/software/ros2_ws/src/dog_toy_finder_motion && python3 -m unittest discover -s tests

# full search workflows against a fake robot and scripted vision
/workspace/scripts/run_e2e_tests.sh
```

## Flashing

See [docs/setup/full_system_test.md](docs/setup/full_system_test.md) for flashing
the Pico (UF2) and both ESP32-CAM eyes, plus Wi-Fi configuration.


## AI Disclosure

This code was written using freely available models in OpenCode and the Zed Editor. Models used include Big Pickle, Beam Free, Flint 40B Free, and Crouton Alpha Free.
