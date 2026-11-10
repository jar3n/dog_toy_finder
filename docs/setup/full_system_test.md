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
# status_led_config.h, and pinout.h.
```

Build:

```bash
cd ../../   # repo root
firmware/scripts/pico_build/build.sh
```

The UF2 is staged at `firmware/pico_microros/build/artifacts/dtf_pico_fw.uf2`.
Hold BOOTSEL on the Pico, plug it in, then copy the UF2 to the mounted drive.
The built-in LED blinks while the firmware starts up, turns solid once the
control loop is running with every required device up, and stays off if any
initialization step failed (see `config/status_led_config.h`).

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

All software parameters for the evaluation run (bringup geometry, health
timeouts, camera URLs/intrinsics, VLM models, map/toy settings, web-map port,
and the ROSA environment) live in `scripts/full_system_eval.conf`.
`launch_all.sh` sources that file and passes each value to its launch file.
Edit the file for a run, pass a different file as the first argument, or
override one value from the environment (`WEBMAP_PORT=9000 ./scripts/launch_all.sh`).
`./scripts/launch_all.sh --dry-run` prints the exact commands without starting tmux.

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

If any sensor (Pico IMU, encoders, IR, either camera eye, or the fused
odometry) stops publishing, the health monitor started by the bringup launch
logs an ERROR on `/rosout`. Watch for failures with:

```bash
ros2 topic echo /rosout | grep FAILED
```

### 4.1 Manual drive mode (keyboard)

With the bringup stack running (micro-ROS agent + `robot.launch.py`), drive the
robot by hand with:

```bash
# on the host, opens the ros2 container with a TTY:
./scripts/manual_drive.sh

# or inside the ROS 2 container:
ros2 run dog_toy_finder_motion manual_drive
```

| Key | Action |
|---|---|
| `W` / Up | drive forward |
| `S` / Down | drive backward |
| `A` / Left | turn left |
| `D` / Right | turn right |
| Space | stop now |
| `Q` | stop and quit |

Manual mode never drives and turns at the same time: each command sets one
axis and clears the other, so a `Twist` cannot contain both. Hold a key to keep
moving (keyboard auto-repeat); releasing it stops the robot after
`key_timeout` (default 0.6 s). If a longer keyboard repeat delay causes
stutter, raise it:

```bash
ros2 run dog_toy_finder_motion manual_drive --ros-args -p key_timeout:=0.8
# also available: -p linear_speed:=0.10 -p angular_speed:=0.4
```

The Pico's IR reflex still brakes in front of an obstacle and the 500 ms
`/cmd_vel` watchdog still stops the motors if key publishing stops.

## 5. Run the full test evaluation

All evaluation parameters (target, search sizes, run IDs, CSV paths, data
collection) live in `scripts/full_system_eval.conf`. Source it first so the
commands below expand to the configured values:

```bash
cd /workspace
source scripts/full_system_eval.conf
```

### 5.1 LLM (ROSA) planner

ROSA can command both workflows with its `run_search` tool: mode `one` runs
the single-toy evaluation search (stops when the toy is found) and mode `all`
runs the find-all demo search. The vision node must be running first
(sections 4/stereo + vision), otherwise each vision check times out. The
search writes `/tmp/rosa_search_<timestamp>.csv` and reports every toy found
with its position.

```bash
cd /workspace/software/rosa_agent
# compare several models on one prompt:
python3 -m rosa_agent --compare "$ROSA_COMPARE" --prompt "$ROSA_PROMPT"
# or interactive:
python3 -m rosa_agent
```

In the interactive session, ask for a search ("Search for the red ball and
stop when you find it") or a demo find-all ("Find every red ball in the
area"); the agent calls `run_search` with the matching mode. Approval gates
the whole search, and Ctrl-C stops it.

Or drive a search with the test harness (uses ROSA tools under the hood is not
automatic; use the autonomous node for deterministic runs, or drive manually
and log with the data collector).

### 5.2 Autonomous (deterministic) planner

```bash
ros2 launch dog_toy_finder_planning autonomous_search.launch.py \
    target:="$AUTO_TARGET" side:=$AUTO_SIDE \
    side_increment:=$AUTO_SIDE_INCREMENT loops:=$AUTO_LOOPS \
    max_time:=$AUTO_MAX_TIME output_csv:=$AUTO_OUTPUT_CSV \
    run_id:=$AUTO_RUN_ID found_distance:=$AUTO_FOUND_DISTANCE \
    find_all:=$AUTO_FIND_ALL toy_side_offset:=$AUTO_TOY_SIDE_OFFSET \
    toy_forward_distance:=$AUTO_TOY_FORWARD_DISTANCE \
    toy_merge_radius:=$AUTO_TOY_MERGE_RADIUS \
    escape_distance:=$AUTO_ESCAPE_DISTANCE
```

### 5.3 LLM-driven search via toy_search harness (baseline)

```bash
ros2 launch dog_toy_finder_experiments toy_search.launch.py \
    target:="$TOY_TARGET" pattern:=$TOY_PATTERN side:=$TOY_SIDE \
    side_increment:=$TOY_SIDE_INCREMENT loops:=$TOY_LOOPS \
    max_time:=$TOY_MAX_TIME output_csv:=$TOY_OUTPUT_CSV \
    run_id:=$TOY_RUN_ID found_distance:=$TOY_FOUND_DISTANCE \
    find_all:=$TOY_FIND_ALL toy_side_offset:=$TOY_TOY_SIDE_OFFSET \
    toy_forward_distance:=$TOY_TOY_FORWARD_DISTANCE \
    toy_merge_radius:=$TOY_TOY_MERGE_RADIUS \
    escape_distance:=$TOY_ESCAPE_DISTANCE
```

### 5.3.1 Demo: find all toys

The demo workflow searches for every toy in the environment instead of
stopping at the first one. Set `AUTO_FIND_ALL=true` (or `TOY_FIND_ALL=true`)
in `scripts/full_system_eval.conf` and run section 5.2 or 5.3 as above. The
`target` accepts a comma-separated list, so several toy kinds can be searched
in one run — e.g. `EVAL_TARGET=red ball,blue bone`. The robot sweeps the same
expanding-square coverage, homes on each visible toy, records it, backs up
`escape_distance` (default 0.3 m), and keeps searching until the loops/time
budget end. Each found toy is labelled with the target that found it. The
summary prints `toys_found` with every estimated position, and the map shows
each toy as an occupied cell plus a sphere. "All toys" means all toys inside
the swept area — coverage sized with `side`/`side_increment`/`loops` decides
what area that is.

To re-run the demo, reset as described in section 6 (place the robot back at
the start, restart bringup/mapping, new `run_id`/`output_csv`).

### 5.4 Collect richer data (optional)

```bash
ros2 launch dog_toy_finder_data collect.launch.py \
    output_dir:=$COLLECT_OUTPUT_DIR run_id:=$COLLECT_RUN_ID \
    record_images:=$COLLECT_RECORD_IMAGES \
    image_every:=$COLLECT_IMAGE_EVERY \
    record_disparity:=$COLLECT_RECORD_DISPARITY
```

## 6. Reset and run again

The autonomous and toy-search nodes are one-shot: each launch runs one search
and exits. The search stops by itself when the target is found (visible and
medium/large, or within `found_distance`), printing `found: True` and writing
its CSV. The ROSA planner (section 5.1) is interactive instead — the operator
drives and stops it.

To run the test again from the same starting spot:

1. Physically place the robot back at the start mark, facing the same way.
2. **Restart the bringup and mapping nodes** (in `launch_all.sh`, the
   `bringup`, `mapping`, and `webmap` panes). This zeroes the EKF/odometry
   frame at the new pose and clears the occupancy map and toy markers.
   Without this, poses and the map keep accumulating from the previous run.
   The stereo and vision panes can stay up.
3. Give each run its own `run_id` and `output_csv` in
   `scripts/full_system_eval.conf` — the search nodes overwrite the CSV file
   on every run, so reusing a path destroys the previous run's log.
4. Run section 5.2 or 5.3 again. The search starts from `x=0, y=0` in the
   fresh odometry frame.

## 7. Evaluate

```bash
ros2 run dog_toy_finder_experiments evaluate_runs "$TOY_OUTPUT_CSV" "$AUTO_OUTPUT_CSV"
ros2 run dog_toy_finder_data dataset_summary "$COLLECT_OUTPUT_DIR/$COLLECT_RUN_ID"
```

Metrics reported: success, time/steps to find, path length, vision call count
and average latency, target-visible rate, final pose.

## 6.5 Viewing the map

The occupancy mapper publishes `/map` (`nav_msgs/OccupancyGrid`),
`/tracked_path` (`nav_msgs/Path`), and `/detected_toys`
(`geometry_msgs/PoseArray`). Start the browser viewer:

```bash
ros2 run dog_toy_finder_mapping map_webserver
# next line of the log prints the URL, e.g. http://192.168.1.50:8765
```

Open that URL on any computer or phone on the same network. The 3D view
updates automatically. Click **Save PNG** to download the current view to the
computer running the browser; **Reset view** restores the camera angle.

The map distinguishes where each obstacle came from:

| Colour | Occupancy value | Source |
|---|---|---|
| red | `100` | IR only |
| blue | `70` | vision only |
| orange | `85` | both IR and vision |
| green cells | `60` | occupied by a detected toy |
| bright green sphere | `/detected_toys` pose | VLM toy detection (centre) |
| white floor | `0` | driven-through free space |

Vision obstacles expire after `vision_timeout` (default 5 s), so blue/orange
blocks reflect recent VLM results rather than old ones. Toy cells are a
separate occupied value (`60`) so they stay visually distinct from obstacles
and are never mistaken for an IR or vision detection. Toy spheres use the
VLM's `distance_m` as the distance ahead and `toy_side_offset` metres to the
left or right (default 0.25 m); if `distance_m` is missing or zero, the
fallback `toy_forward_distance` (default 0.6 m) is used. Markers are merged
within `toy_merge_radius` (default 0.15 m), and each toy marks a
`(2 * toy_occupancy_radius + 1)` square of cells (default radius `1`, a 3x3
patch). Tune these with:

```bash
ros2 launch dog_toy_finder_mapping mapping.launch.py \
    toy_side_offset:=0.2 toy_forward_distance:=0.4 \
    toy_occupancy_radius:=2 vision_timeout:=3.0
```

## 7. Unit tests

```bash
cd /workspace/software/rosa_agent && python3 -m unittest discover -s tests
cd /workspace/software/ros2_ws/src/dog_toy_finder_planning && python3 -m unittest discover -s tests
cd /workspace/software/ros2_ws/src/dog_toy_finder_motion && python3 -m unittest discover -s tests
```

End-to-end search workflow tests (fake robot + scripted vision, no hardware
needed) live in `tests_e2e/`:

```bash
./scripts/run_e2e_tests.sh                 # everything (several minutes)
./scripts/run_e2e_tests.sh note_found size # just two scenarios
```

## Notes

- Keep `ROSA_REQUIRE_APPROVAL=1` during development to avoid unintended motion.
- The Pico's 500 ms `/cmd_vel` watchdog stops the robot if the computer or
  Wi-Fi link stalls.
- The Pico also runs a local IR reflex: when the obstacle sensor sees something
  within range it brakes and blocks forward commands. Reverse is only allowed
  in small steps (about 4 cm per step by default), with a brake pause between
  steps where a turn-in-place is allowed so the robot can back away and turn
  around. There is no unrestricted reverse. This works without Wi-Fi.
  Configure it with `CMD_VEL_IR_REFLEX`,
  `CMD_VEL_IR_REFLEX_REVERSE_STEPS`,
  `CMD_VEL_IR_REFLEX_REVERSE_SPEED_M_S`,
  `CMD_VEL_IR_REFLEX_REVERSE_STEP_MS`, and
  `CMD_VEL_IR_REFLEX_REVERSE_PAUSE_MS` in
  `firmware/pico_microros/config/cmd_vel_config.h`.
- ROS knows when the failsafe fires: the Pico mirrors the reflex engage/clear
  messages and the watchdog stop to `/pico/log`
  (`ros2 topic echo /pico/log`) and publishes `/ir_obstacle/detected`, which
  the motion layer subscribes to. Forward moves and turns abort on that signal
  and report it via `failsafe_hit()` / the ROSA `check_failsafe` tool, instead
  of driving against the brake until the deadline.
- Toy cells in `/map` are map memory for display; the local IR reflex is what
  prevents the robot from driving into an obstacle or toy. Do not rely on the
  map alone for collision avoidance.
- All geometry values must agree between `encoder_config.h`,
  `cmd_vel_config.h`, `robot.yaml`, and the launch defaults.
