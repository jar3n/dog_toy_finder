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
- A toy (e.g. a red ball) and a clear floor area.

Build the ROS 2 image once:

```bash
cd docker
docker compose build
```

The image includes Pillow for JPEG conversion, but open-vocabulary and
dense-stereo inference run on the computer, not in Docker. Start the host
detector service once before the vision node:

```bash
cd /workspace   # or the repository root on the host
./scripts/start_yolo_service.sh                        # YOLO-World (default)
DETECTOR_MODEL=yoloe_26 ./scripts/start_yolo_service.sh
DETECTOR_MODEL=grounding_dino ./scripts/start_yolo_service.sh
```

One detector runs at a time. The defaults are `yolov8s-world.pt`,
`yoloe-26s-seg.pt`, and `IDEA-Research/grounding-dino-tiny`; override them with
`YOLO_MODEL_PATH`, `YOLOE_MODEL_PATH`, or `GROUNDING_DINO_MODEL`. The script
creates `software/host_vision/.venv`, installs CPU torch/ultralytics/
transformers, downloads the selected weights, and serves HTTP on
`YOLO_SERVICE_URL` (default `http://localhost:5006`). `YOLO_DEVICE` selects the
torch device; `GROUNDING_DINO_TEXT_THRESHOLD` sets its text threshold.

For the `stereo-net` pipeline also start the host dense-stereo depth service:

```bash
./scripts/start_stereo_service.sh                          # Lite Any Stereo (default)
DEPTH_MODEL=fast_foundation_stereo ./scripts/start_stereo_service.sh
DEPTH_MODEL=rt_igevpp ./scripts/start_stereo_service.sh
```

It reuses `software/host_vision/.venv`, serves `POST /depth` on
`STEREO_SERVICE_URL` (default `http://localhost:5007`), and downloads the
selected checkpoint and upstream repository under `software/host_vision/models/`.
One depth model runs at a time; `STEREO_DEVICE` selects the torch device and
per-model `*_MODEL_PATH` variables override checkpoint paths (see
`software/ros2_ws/src/dog_toy_finder_vision/README.md`). Keep the same
`DETECTOR_MODEL` for both pipelines: `open_vocab` needs only the detector
service, while `stereo-net` also needs the depth service. Run one detector and one
depth model at a time. Ollama is only needed for the ROSA/planning comparisons;
vision does not use it.

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

The Pico waits indefinitely for Wi-Fi and for the micro-ROS agent, logging one
connect message for each. While it waits the LED keeps blinking; a power cycle
is the only way to reboot. If `docker compose logs micro_ros_agent` shows
repeated 23-byte pings from `client_key: 0x00000000` but no `/dtf_pico` node,
the custom transport is framed and the agent cannot answer it; see
`firmware/pico_microros/udp_transport/README.md`.

After flashing/rebooting the Pico, verify that exactly one micro-ROS session is
in the graph before starting any motion test:

```bash
docker exec -it rosa-ros2 bash -lc \
  'source /opt/ros/humble/setup.bash; /workspace/firmware/scripts/check_pico_graph.sh'
```

The firmware uses a stable client key so reboots reuse one agent session. If
the check reports duplicates from an older firmware, restart the micro-ROS
agent once to discard the stale sessions, then rerun the check.

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

A launcher opens one pane per component. On a desktop with Konsole
available it uses a single Konsole window with split panes; otherwise it falls
back to tmux:

```bash
# on the host:
./scripts/launch_all.sh
# or, already inside the ros2 container:
./scripts/launch_all.sh

# start the stack and open keyboard manual drive in the same terminal session:
./scripts/launch_all.sh --manual
```

All software parameters for the evaluation run (bringup geometry, health
timeouts, camera URLs/intrinsics, the detector config path, vision pipeline
choice, map/toy settings, web-map port, manual drive speeds, and
the ROSA environment) live in
`scripts/full_system_eval.conf`. Set `VISION_PIPELINE` there to `open_vocab`
(two-eye detection and triangulation) or `stereo-net` (one-eye detection plus dense
stereo depth; start both host services first). The pipeline can
also be set with the top-level `pipeline` key in `vision_detectors.yaml`;
explicit selections (launch arg `pipeline:=...` or `vision_eval --pipeline`) win
over `VISION_PIPELINE`, which wins over the YAML key. `launch_all.sh` sources
that file and passes each value to its launch file. Edit the file for a run,
pass a different file as the first argument, or override one value from the
environment (`WEBMAP_PORT=9000 ./scripts/launch_all.sh`). `--manual` adds and
selects a `manual` pane/window without editing the config; setting
`MANUAL_DRIVE=true` in the config does the same for every launch.
`./scripts/launch_all.sh --dry-run` prints the exact commands without starting
anything. `--konsole` / `--tmux` force the launcher; `KONSOLE=true` in the
config makes Konsole the default. Set `LLM_TRIALS=true` to add a `trials`
pane/window that records vision comparison rows (section 5.5).

Konsole mode needs the security-sensitive D-Bus API enabled so panes can run
their commands:

```bash
mkdir -p ~/.config
cat > ~/.config/konsolerc <<'EOF'
[KonsoleWindow]
EnableSecuritySensitiveDBusAPI=true
EOF
```

Stop everything by pressing Ctrl+C in the `launch_all.sh` terminal; that
tears down the whole stack (tmux/Konsole panes and docker containers).

Alternatively, start the pieces by hand. On the host:

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

# Terminal B: stereo vision + live detector (rectified pair, /vision/ask)
# For capture-only comparison, add vision:=false; vision_capture needs
# only /stereo/{left,right}/image_rect_color and waits for a fresh pair.
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    left_url:=http://esp32cam-left.local/capture \
    right_url:=http://esp32cam-right.local/capture \
    config_path:=$VISION_DETECTOR_CONFIG \
    pipeline:="open_vocab"   # or set VISION_PIPELINE in the config file
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

The easiest way is to start (or restart) the whole stack with manual mode
already open:

```bash
./scripts/launch_all.sh --manual
```

That starts every component and creates a `manual` pane/window (the keyboard
driver, terminal in cbreak mode) plus a `shell` pane/window with a normal
terminal. The shell is selected on attach so ROS commands can be typed freely.
Switch to `manual` to drive, then press `Q` there to stop driving.

- **tmux:** `Ctrl-b m` selects `manual`; `Ctrl-b s` selects `shell`,
  `Ctrl-b b` selects `bringup`, and `Ctrl-b w` lists windows.
- **Konsole:** use the split-view focus shortcut (Shift+Right / Shift+Left by
  default) to move between the `shell` and `manual` panes.

With only the bringup stack running (micro-ROS agent + `robot.launch.py`), the
manual node can also be started on its own:

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

When manual drive is started by `launch_all.sh`, set the same values through
`MANUAL_LINEAR_SPEED`, `MANUAL_ANGULAR_SPEED`, `MANUAL_KEY_TIMEOUT`, and
`MANUAL_PUBLISH_RATE` in `scripts/full_system_eval.conf` instead.

Manual drive publishes `/cmd_vel_manual`, not `/cmd_vel`. The bringup
`cmd_vel_mux` owns `/cmd_vel` and gives manual commands priority over
autonomous ones, so no second publisher can fight the keyboard. Check which
source currently has control with:

```bash
ros2 topic echo /cmd_vel_source
```

The mux holds zero for a short release window after manual input stops before
autonomous commands may resume; tune it with the `CMD_VEL_*` values in
`scripts/full_system_eval.conf`. It also blocks non-zero manual or autonomous
commands while `/odometry/filtered` is missing or stale
(`CMD_VEL_ODOMETRY_TIMEOUT`), so the EKF must be healthy before the robot moves.

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
(see section 4), otherwise each vision check times out. The search writes
`/tmp/rosa_search_<timestamp>.csv` and reports every toy found with its
position.

```bash
cd /workspace/software/rosa_agent
# compare several models on one prompt and save the comparison rows:
python3 -m rosa_agent --compare "$ROSA_COMPARE" --prompt "$ROSA_PROMPT" \
    --output-csv "$ROSA_OUTPUT_CSV" --run-id "$ROSA_RUN_ID"
# add planning ground truth now (optional):
python3 -m rosa_agent --compare "$ROSA_COMPARE" --prompt "$ROSA_PROMPT" \
    --output-csv "$ROSA_OUTPUT_CSV" --run-id "$ROSA_RUN_ID" \
    --gt-json "$ROSA_GT_JSON"
# or interactive:
python3 -m rosa_agent --output-csv "$ROSA_OUTPUT_CSV" --run-id "$ROSA_RUN_ID"
```

In the interactive session, ask for a search ("Search for the red ball and
stop when you find it") or a demo find-all ("Find every red ball in the
area"); the agent calls `run_search` with the matching mode. Approval gates
the whole search, and Ctrl-C stops it.

For repeatable runs use the autonomous node (5.2). The toy-search harness
(5.3) runs the same search loop through the experiments launch file, and the
data collector (5.4) records poses and images for offline analysis.

### 5.2 Autonomous (deterministic) planner

```bash
ros2 launch dog_toy_finder_planning autonomous_search.launch.py \
    target:="$AUTO_TARGET" side:=$AUTO_SIDE \
    side_increment:=$AUTO_SIDE_INCREMENT loops:=$AUTO_LOOPS \
    max_time:=$AUTO_MAX_TIME output_csv:=$AUTO_OUTPUT_CSV \
    run_id:=$AUTO_RUN_ID found_distance:=$AUTO_FOUND_DISTANCE \
    find_all:=$AUTO_FIND_ALL \
    toy_forward_distance:=$AUTO_TOY_FORWARD_DISTANCE \
    toy_merge_radius:=$AUTO_TOY_MERGE_RADIUS \
    escape_distance:=$AUTO_ESCAPE_DISTANCE
```

### 5.3 Toy-search harness (baseline)

```bash
ros2 launch dog_toy_finder_experiments toy_search.launch.py \
    target:="$TOY_TARGET" pattern:=$TOY_PATTERN side:=$TOY_SIDE \
    side_increment:=$TOY_SIDE_INCREMENT loops:=$TOY_LOOPS \
    max_time:=$TOY_MAX_TIME output_csv:=$TOY_OUTPUT_CSV \
    run_id:=$TOY_RUN_ID found_distance:=$TOY_FOUND_DISTANCE \
    find_all:=$TOY_FIND_ALL \
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

### 5.5 LLM comparison trials

Vision and planning comparisons share the `dog_toy_finder_data.llm_trials` CSV
schema and scoring tools. **Vision comparison is capture-first**: save each
rectified left/right pair with operator-measured ground truth, then replay all
captures offline. See [`llm_comparison_guide.md`](llm_comparison_guide.md) for
the full step-by-step workflow.

Trial data is stored in the repository's `data/` folder, next to `hardware/`
and `software/`: `data/vision/captures/`, `data/vision/vision_eval_*.csv/json`
and `data/planning/<run_id>/`. Inside the ROS 2 container those paths are under
`/workspace/data`.

Calibrate the stereo pair first with
[camera_calibration.md](camera_calibration.md); put the measured intrinsics and
baseline in the detector YAML. `scripts/full_system_eval.conf` reads them from
that file for the launch arguments.

For vision, set `VISION_DETECTOR_CONFIG` to your measured camera YAML and
`VISION_PIPELINE` to `open_vocab` or `stereo-net` (or select it with
`pipeline:=...` on the launch command). Use the same `DETECTOR_MODEL` for both
pipelines: `open_vocab` needs only the host detector service; `stereo-net` also
needs the host dense-stereo depth service, with one depth model at a time. The
live vision node is not needed for capture; terminate it (or launch
`stereo_vision.launch.py` with `vision:=false`) and leave the rectified stereo
pipeline running. Then save a labelled pair:

```bash
cd /workspace
source scripts/full_system_eval.conf

ros2 run dog_toy_finder_vision vision_capture \
    --target "$VISION_CAPTURE_TARGET" \
    --size medium \
    --run-id "$VISION_CAPTURE_RUN_ID" \
    --capture-id ball_115m_b12 \
    --output-dir "$VISION_CAPTURE_DIR" \
    --config "$VISION_DETECTOR_CONFIG" \
    --timeout "$VISION_CAPTURE_TIMEOUT"
```

`vision_capture` waits for a fresh matched `/stereo/{left,right}/image_rect_color`
pair. Measure depth from the cameras with a tape measure and lateral bearing
from the optical axis; positive bearing means robot-left (ROS `+y`). The command
above is interactive: it prompts for `distance_m (metres)` and
`lateral_bearing_deg (+left)`; type the measured values. Use `--not-visible`
for an empty frame (no prompts), plus `--obstacle` and `--notes` where needed.
For scripts or non-interactive shells, pass `--depth-m` and `--bearing-deg`
explicitly instead. Each call writes:

```text
$VISION_CAPTURE_DIR/<run_id>/<capture_id>/left.jpg
$VISION_CAPTURE_DIR/<run_id>/<capture_id>/right.jpg
$VISION_CAPTURE_DIR/<run_id>/<capture_id>/meta.json
```

`meta.json` contains the operator's ground truth, including
`lateral_bearing_deg`, the sole lateral measurement. Repeat with a new
`--capture-id` for each scene. After the capture phase, replay every
saved pair offline with no camera topics or ROS graph. Run **one pipeline per
invocation**. To compare both, use distinct output paths and pass the same
`--seed` to each run so they process the randomized captures in the same order;
a seed is printed and recorded in the summary when omitted:

```bash
# Add --seed N to both commands (use a recorded seed for identical order).
ros2 run dog_toy_finder_vision vision_eval \
    --captures-dir "$VISION_CAPTURE_DIR" \
    --pipeline open_vocab \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE" \
    --output-csv "$DATA_DIR/vision/open_vocab_trials.csv" \
    --results-json "$DATA_DIR/vision/open_vocab_results.jsonl" \
    --summary-json "$DATA_DIR/vision/open_vocab_summary.json"

# detector + depth service must be running for this second invocation
ros2 run dog_toy_finder_vision vision_eval \
    --captures-dir "$VISION_CAPTURE_DIR" \
    --pipeline stereo-net \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE" \
    --output-csv "$DATA_DIR/vision/stereo-net_trials.csv" \
    --results-json "$DATA_DIR/vision/stereo-net_results.jsonl" \
    --summary-json "$DATA_DIR/vision/stereo-net_summary.json"
```

One scored row is written per capture per invocation; the trial CSV schema is
unchanged. For `open_vocab`, `model` is the detector name and matched detections
carry `method=open_vocab` plus `distance_source=stereo_triangulation`. For
`stereo-net`, `model` is the dense depth model name, `detector_model` records the
one-image detector, and answers carry `method=stereo-net` plus
`distance_source=dense_stereo_depth`. `method=ml` is only used by legacy rows.
Ground truth is copied from `meta.json`, and scored columns are written
immediately. Each summary manifest (`vision_eval_summary.json`) records the
pipeline, `shuffle_seed`, and `capture_order` (paths relative to
`--captures-dir`) so an order can be reproduced.

Planning rows are written by ROSA (section 5.1) to `$ROSA_OUTPUT_CSV`. The
`gt_*` columns stay blank unless `ROSA_GT_JSON` points at a JSON file. A flat
object applies to every model; the per-model/prompt form is documented in
`software/rosa_agent/README.md`.

Score a trial CSV after any manual ground-truth annotation:

```bash
ros2 run dog_toy_finder_data llm_trials score "$VISION_EVAL_CSV"
ros2 run dog_toy_finder_data llm_trials score "$ROSA_OUTPUT_CSV"
ros2 run dog_toy_finder_data llm_trials aggregate "$ROSA_OUTPUT_CSV" --json
```

`score` writes `<name>_scored.csv` and prints per-model latency, error count,
ground-truthed row count, per-field accuracy, distance/bearing error, and mean
score. Repeat the score/analyze commands for each pipeline CSV when comparing
both. Create an empty template for data collected another way with
`llm_trials template --kind vision|planning --models ... --output ...`: for
vision use the detector name for `open_vocab` rows or the dense depth model name
for `stereo-net` rows.

## 6. Reset and run again

The autonomous and toy-search nodes are one-shot: each launch runs one search
and exits. The search stops by itself when the target is found (visible and
within `found_distance`), printing `found: True` and writing
its CSV. The ROSA planner (section 5.1) is interactive instead — the operator
drives and stops it.

To run the test again from the same starting spot:

1. Physically place the robot back at the start mark, facing the same way.
2. **Restart the bringup and mapping nodes** (in `launch_all.sh`, the
   `bringup`, `mapping`, and `webmap` panes). This zeroes the EKF/odometry
   frame at the new pose and clears the occupancy map and toy markers.
   Without this, poses and the map keep accumulating from the previous run.
   The vision pane can stay up.
3. Give each run its own `run_id` and `output_csv` in
   `scripts/full_system_eval.conf` — the search nodes overwrite the CSV file
   on every run, so reusing a path destroys the previous run's log.
4. Run section 5.2 or 5.3 again. The search starts from `x=0, y=0` in the
   fresh odometry frame.

## 7. Evaluate

```bash
ros2 run dog_toy_finder_experiments evaluate_runs "$TOY_OUTPUT_CSV" "$AUTO_OUTPUT_CSV"
ros2 run dog_toy_finder_data dataset_summary "$COLLECT_OUTPUT_DIR/$COLLECT_RUN_ID"
ros2 run dog_toy_finder_data llm_trials score "$ROSA_OUTPUT_CSV"
ros2 run dog_toy_finder_data llm_trials score "$VISION_EVAL_CSV"

# Post-collection statistics (paired comparisons across models)
ros2 run dog_toy_finder_data llm_trials analyze "$ROSA_OUTPUT_CSV" \
    --confidence "$LLM_TRIAL_CONFIDENCE" --output "$PLANNING_ANALYSIS_JSON"
ros2 run dog_toy_finder_data llm_trials analyze "$VISION_EVAL_CSV" \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE" \
    --confidence "$LLM_TRIAL_CONFIDENCE" --output "$VISION_EVAL_ANALYSIS_JSON"

# repeat the vision score/analyze commands for the other pipeline CSV
```

Metrics reported: success, time/steps to find, path length, vision call count
and average latency, target-visible rate, final pose. The LLM trial tool reports
vision accuracy per `model` value (detector name for `open_vocab`, dense depth
model name for `stereo-net`), distance/bearing error, planning action/argument
matches, latency, errors, and a mean score. `llm_trials analyze` adds confidence
intervals and paired comparisons so differences can be judged after all trials
are collected. For captured trials, `ask_id` is the capture ID, so each pipeline
is compared on the same saved images; use the recorded `shuffle_seed` and
`capture_order` to keep replay order identical across pipelines.

## 8. Viewing the map

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
| bright green sphere | `/detected_toys` pose | vision toy detection (vision pipeline answers fused by median) |
| white floor | `0` | driven-through free space |

Vision obstacles expire after `vision_timeout` (default 5 s), so blue/orange
blocks reflect recent detector results rather than old ones. Toy cells are a
separate occupied value (`60`) so they stay visually distinct from obstacles
and are never mistaken for an IR or vision detection. Toy spheres use the
fused detectors' median `distance_m` as the distance ahead and the median
`lateral_bearing_deg` for the lateral angle (degrees from the optical axis,
positive = robot-left / ROS `+y`). No coarse side label is emitted or accepted.
If `distance_m` is missing or zero, the fallback `toy_forward_distance`
(default 0.6 m) is used. Markers are merged
within `toy_merge_radius` (default 0.15 m), and each toy marks a
`(2 * toy_occupancy_radius + 1)` square of cells (default radius `1`, a 3x3
patch). Tune these with:

```bash
ros2 launch dog_toy_finder_mapping mapping.launch.py \
    toy_forward_distance:=0.4 toy_merge_radius:=0.2 \
    toy_occupancy_radius:=2 vision_timeout:=3.0
```

## 9. Unit tests

```bash
cd /workspace/software/rosa_agent && python3 -m unittest discover -s tests
cd /workspace/software/ros2_ws/src/dog_toy_finder_planning && python3 -m unittest discover -s tests
cd /workspace/software/ros2_ws/src/dog_toy_finder_motion && python3 -m unittest discover -s tests
cd /workspace/software/ros2_ws/src/dog_toy_finder_data && python3 -m unittest discover -s tests
cd /workspace/software/ros2_ws/src/dog_toy_finder_vision && python3 -m unittest discover -s tests
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
