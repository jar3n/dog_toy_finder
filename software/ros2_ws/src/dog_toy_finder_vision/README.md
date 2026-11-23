# Stereo vision

`dog_toy_finder_vision` turns the two ESP32-CAM eyes into a ROS 2 stereo
pipeline and asks **one or more Ollama vision models about the same rectified
colour pair**. Models are loaded and unloaded one at a time, so each gets all
available VRAM during comparison runs.

The node subscribes only to `/stereo/{left,right}/image_rect_color`. Raw images
still exist for rectification and the point-cloud pipeline, but model inputs
are rectified.

Model comparison is **capture-first**. `vision_capture` saves a rectified
left/right pair with operator-measured ground truth, and `vision_eval` later
replays those saved pairs through the configured models. Lateral position is a
numeric `lateral_bearing_deg` (degrees from the optical axis, positive =
robot-left / ROS `+y`); the legacy `left`/`center`/`right` label is derived from
it for compatibility.

## Topics

### Stereo pipeline
| Topic | Type | Source |
|---|---|---|
| `/stereo/left/image_raw` | `sensor_msgs/Image` (rgb8) | `stereo_camera_node` |
| `/stereo/right/image_raw` | `sensor_msgs/Image` (rgb8) | `stereo_camera_node` |
| `/stereo/left/image_mono` | `sensor_msgs/Image` (mono8) | `stereo_camera_node` |
| `/stereo/right/image_mono` | `sensor_msgs/Image` (mono8) | `stereo_camera_node` |
| `/stereo/left/camera_info` | `sensor_msgs/CameraInfo` | `stereo_camera_node` |
| `/stereo/right/camera_info` | `sensor_msgs/CameraInfo` | `stereo_camera_node` |
| `/stereo/{left,right}/image_rect` | `sensor_msgs/Image` (mono8) | `image_proc` rectify |
| `/stereo/{left,right}/image_rect_color` | `sensor_msgs/Image` (rgb8) | `image_proc` rectify |
| `/stereo/disparity` | `stereo_msgs/DisparityImage` | `stereo_image_proc` |
| `/stereo/points2` | `sensor_msgs/PointCloud2` | `stereo_image_proc` |

### Detector interface
| Topic | Type | Source |
|---|---|---|
| `/stereo/left/image_rect_color` | `sensor_msgs/Image` | `image_proc` rectify |
| `/stereo/right/image_rect_color` | `sensor_msgs/Image` | `image_proc` rectify |
| `/vision/ask` | `std_msgs/String` (JSON) | user / planner |
| `/vision/result` | `std_msgs/String` (JSON) | `vision_node` |

## Physical measurement config

The node reads the same YAML file:

```text
software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml
```

The path is set once in `scripts/full_system_eval.conf` as
`VISION_DETECTOR_CONFIG` and forwarded through the launch file. Put your
measured camera and toy values there:

- `camera.fx`, `fy`, `cx`, `cy` — calibrated intrinsics in pixels
- `camera.baseline_m` — spacing between the two optical centres
- `camera.depth_min_m`, `depth_max_m` — valid metric depth range
- `camera.{left,right}_rectified_topic` — model image inputs
- `target.known_width_m`, `known_height_m` — real toy measurements
- `target.position_split_left/right` — image thirds used to derive the
  compatibility `left`/`center`/`right` label
- `target.position_bearing_deg` — fallback bearing (default `15°`) only for old
  answers that have a label but no numeric `lateral_bearing_deg`

The camera values should match the `CAMERA_*` and `STEREO_BASELINE` values in
`full_system_eval.conf`. The defaults are placeholders until you calibrate the
cameras and measure the toy.

The bearing convention is `atan((cx - u) / fx)` in degrees, where `u` is the
target's horizontal pixel centre. Positive values mean the target is left of
the optical axis; zero means centred. This same field is used by live control,
mapping, and offline scoring.

## Vision model path

The vision node sends the same two rectified images to Ollama together with a
short physical-measurement summary from the YAML config. Each model returns:

```json
{
  "description": "...",
  "target_visible": true,
  "position": "left | center | right | none",
  "lateral_bearing_deg": 12.0,
  "size": "small | medium | large | none",
  "distance_m": 0.0,
  "obstacle_ahead": "..."
}
```

Set `models:="gemma4:latest,minicpm-v4.5:latest,qwen3.5:4b"` to compare models
on every ask. Each answer gets `method: "ml"`, its model name, and timing.

## Result fields

`/vision/result` contains a `results` array. Every entry has:

| Field | Meaning |
|---|---|
| `model` | Ollama model name |
| `method` | `ml` |
| `target_visible` | whether the requested target was found |
| `lateral_bearing_deg` | numeric bearing from the optical axis; positive = robot-left |
| `position` | derived compatibility label: `left`, `center`, `right`, or `none` |
| `size` | `small`, `medium`, `large`, or `none` |
| `distance_m` | distance to the target in metres, `0` if unknown/not visible |
| `obstacle_ahead` | object blocking the centre of the path, or `none` |
| `path_clear` | `true` when `obstacle_ahead` is `none` |
| `description` | human-readable model summary |
| `seconds` | model latency |
| `distance_source` | how the metric distance was derived (`vlm_estimate`) |

The mapping node consumes all entries and fuses visible toys by median bearing
and distance. The trial collector writes one CSV row per entry. For fair
offline comparison, prefer `vision_capture` + `vision_eval` below: every model
sees the exact same saved images.

## Capture-first model comparison

With the rectified stereo pipeline running, save a labelled pair. No Ollama
model or `/vision/ask` is involved:

```bash
cd /workspace
source scripts/full_system_eval.conf

ros2 run dog_toy_finder_vision vision_capture \
    --target "$VISION_CAPTURE_TARGET" \
    --depth-m 1.15 --bearing-deg +12 \
    --run-id "$VISION_CAPTURE_RUN_ID" \
    --capture-id ball_115m_left12 \
    --output-dir "$VISION_CAPTURE_DIR"
```

Each capture writes `left.jpg`, `right.jpg`, and `meta.json` under
`<output_dir>/<run_id>/<capture_id>/`. `meta.json` holds the operator's ground
truth, including `lateral_bearing_deg`; use `--not-visible` for an empty frame.
`vision_capture` waits for a fresh matched rectified pair, so it works with the
non-hardware-synchronised ESP32-CAMs.

After capturing all scenes, replay them offline:

```bash
ros2 run dog_toy_finder_vision vision_eval \
    --captures-dir "$VISION_CAPTURE_DIR" \
    --models "$VISION_EVAL_MODELS" \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE" \
    --output-csv "$VISION_EVAL_CSV" \
    --results-json "$VISION_EVAL_JSONL" \
    --summary-json "$VISION_EVAL_SUMMARY_JSON"
```

`vision_eval` needs no ROS graph. It writes one scored row per capture and
model, plus a per-model summary. Multiple models are unloaded after each answer;
use `--compare-models` to force that for one model. Useful flags include
`--run-id`, `--capture`, and `--limit`.
See [`docs/setup/llm_comparison_guide.md`](../../../../docs/setup/llm_comparison_guide.md)
for the full experiment workflow.

## Run

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash

# full stack: cameras + rectification + depth + Ollama vision models
ros2 launch dog_toy_finder_vision stereo_vision.launch.py

# compare several Ollama models on the same rectified pair
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    models:="gemma4:latest,minicpm-v4.5:latest,qwen3.5:4b"

# use a different physical measurement config
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    config_path:=/workspace/data/vision/my_calibration.yaml

# skip the vision models (keep cameras and depth)
ros2 launch dog_toy_finder_vision stereo_vision.launch.py ml:=false

# depth pipeline only, no detectors
ros2 launch dog_toy_finder_vision stereo_vision.launch.py vision:=false
```

## Test without ROSA

With the launch above running:

```bash
python3 /workspace/scripts/vision_test.py "red ball"
python3 /workspace/scripts/vision_test.py ""   # describe the scene
```

It waits for `/vision/ask` to be discovered, publishes one request, and prints
the full `/vision/result` JSON.

## Tuning

- `rate` controls how often each eye is sampled (default 2 Hz). Higher rates
  need a stronger Wi-Fi link.
- `stereo:=false` skips cameras/depth pipeline, `stereo_proc:=false` skips
  disparity/point cloud, `vision:=false` skips the vision node, and
  `ml:=false` keeps the node up but skips the models.
- `approximate_sync:=true` is the default because the eyes are not
  hardware-synchronised.
- At 640x480 the ~900 KB colour frames are often dropped by DDS. If you raise
  width/height, scale `fx`, `fy`, `cx`, `cy`, and every physical measurement in
  the detector YAML accordingly.
