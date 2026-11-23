# Stereo vision

`dog_toy_finder_vision` turns the two ESP32-CAM eyes into a ROS 2 stereo
pipeline and runs a **selectable vision pipeline on the same rectified colour
pair**. `open_vocab` is implemented: one host-side open-vocabulary detector
runs on both rectified images, detections are matched between the eyes, and the
target location is triangulated from disparity using `fx` and the stereo
baseline. The detector runs on the computer (YOLO-World, YOLOE-26, or Grounding
DINO); `second` is a reserved placeholder that is not configured yet.
Selecting it reports a clear error.

The pipeline is chosen by the launch arg `pipeline:=open_vocab|second`, the
`VISION_PIPELINE` environment variable, or the top-level `pipeline` key in
`vision_detectors.yaml` (in that order of precedence); `vision_eval` also has
`--pipeline`.

The node subscribes only to `/stereo/{left,right}/image_rect_color`. Raw images
still exist for rectification and the point-cloud pipeline, but detector inputs
are rectified.

Comparison is **capture-first**. `vision_capture` saves a rectified
left/right pair with operator-measured ground truth, and `vision_eval` later
replays those saved pairs through the configured pipeline. Lateral position is a
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

- `pipeline` — `open_vocab` (default) or the reserved `second`
- `open_vocab.service_url` — host detector service URL (default
  `http://localhost:5006`; `YOLO_SERVICE_URL` overrides it)
- `open_vocab.timeout_s` — HTTP timeout in seconds
- `open_vocab.name` — fallback model name recorded in results and trial CSVs
  (the host service may report its own name)
- `open_vocab.confidence_threshold`, `max_detections`, `min_disparity`,
  `max_disparity`, `max_vertical_delta_px` — detector and stereo-matching limits
- The detector backend, weights, and torch device are **not** in the YAML:
  configure them on the host service with `DETECTOR_MODEL`,
  `YOLO_MODEL_PATH` / `YOLOE_MODEL_PATH` / `GROUNDING_DINO_MODEL`, and
  `YOLO_DEVICE` (see below)
- `camera.fx`, `fy`, `cx`, `cy` — calibrated intrinsics in pixels
- `camera.baseline_m` — spacing between the two optical centres
- `camera.depth_min_m`, `depth_max_m` — valid metric depth range
- `camera.{left,right}_rectified_topic` — detector image inputs
- `target.known_width_m`, `known_height_m` — retained for older tooling;
  `open_vocab` triangulates from disparity rather than apparent size
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

## Open-vocabulary pipeline

`open_vocab` sends both rectified JPEGs and the text target to a detector
service running **on the computer**, not in the ROS container. Start it once
from the repository root:

```bash
./scripts/start_yolo_service.sh                        # YOLO-World
DETECTOR_MODEL=yoloe_26 ./scripts/start_yolo_service.sh
DETECTOR_MODEL=grounding_dino ./scripts/start_yolo_service.sh
```

One detector runs at a time. The script creates `software/host_vision/.venv`,
installs CPU torch/ultralytics/transformers, downloads the selected weights
(`yolov8s-world.pt`, `yoloe-26s-seg.pt`, or `IDEA-Research/grounding-dino-tiny`),
and serves `POST /detect` on `http://localhost:5006` (host networking lets the
container reach it). Override weights with `YOLO_MODEL_PATH`, `YOLOE_MODEL_PATH`,
or `GROUNDING_DINO_MODEL`; pass `--device` to change the torch device.
`GET /health` reports the selected detector and model name.

The service detects the text target independently in each eye. The container
pairs detections by label, image row, and horizontal disparity; distance is
`fx * baseline_m / disparity`, and lateral bearing comes from the left-image
pixel centre. The service's model name (or `open_vocab.name`) is reported as
`model`, with `method: "open_vocab"` and
`distance_source: "stereo_triangulation"` for matched detections. Selecting
`second` instead raises a clear "reserved but not configured yet" error.

Each answer uses the existing schema:

```json
{
  "model": "yolov8s-world",
  "method": "open_vocab",
  "distance_source": "stereo_triangulation",
  "description": "...",
  "target_visible": true,
  "position": "left | center | right | none",
  "lateral_bearing_deg": 12.0,
  "size": "small | medium | large | none",
  "distance_m": 1.15,
  "obstacle_ahead": "none",
  "bbox_px": [140, 96, 176, 132],
  "bbox_right_px": [120, 96, 156, 132],
  "disparity_px": 20.0
}
```

## Result fields

`/vision/result` contains a `results` array plus the selected `pipeline` name.
Every entry has:

| Field | Meaning |
|---|---|
| `model` | configured detector name from the pipeline |
| `method` | `open_vocab` for new vision rows (`ml` only on legacy rows) |
| `target_visible` | whether the requested target was found |
| `lateral_bearing_deg` | numeric bearing from the optical axis; positive = robot-left |
| `position` | derived compatibility label: `left`, `center`, `right`, or `none` |
| `size` | `small`, `medium`, `large`, or `none` |
| `distance_m` | distance to the target in metres, `0` if unknown/not visible |
| `obstacle_ahead` | object blocking the centre of the path, or `none` |
| `path_clear` | `true` when `obstacle_ahead` is `none` |
| `description` | human-readable detector summary |
| `seconds` | detector latency |
| `distance_source` | `stereo_triangulation` for matched detections, `none` when no target was found |
| `bbox_px` | left rectified-image box `[x1, y1, x2, y2]` in pixels |
| `bbox_right_px` | matching box in the right rectified image |
| `disparity_px` | horizontal difference between the two box centres, used for depth |

The mapping node consumes all entries and fuses visible toys by median bearing
and distance. The trial collector writes one CSV row per entry. For fair
offline comparison, prefer `vision_capture` + `vision_eval` below: every
configured detector name sees the exact same saved images.

## Capture-first comparison

With the rectified stereo pipeline running, save a labelled pair. No detector
or `/vision/ask` is involved:

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
    --pipeline "$VISION_PIPELINE" \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE" \
    --output-csv "$VISION_EVAL_CSV" \
    --results-json "$VISION_EVAL_JSONL" \
    --summary-json "$VISION_EVAL_SUMMARY_JSON"
```

`vision_eval` needs no ROS graph. It writes one scored row per capture per
configured model name produced by the pipeline (`open_vocab.name` or the weight
filename) and records the chosen pipeline in the summary manifest
(`vision_eval_summary.json`). Useful flags include `--run-id`, `--capture`, and
`--limit`.
See [`docs/setup/llm_comparison_guide.md`](../../../../docs/setup/llm_comparison_guide.md)
for the full experiment workflow.

## Run

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash

# full stack: cameras + rectification + depth + open-vocabulary detector
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    pipeline:=open_vocab

# same pipeline selected via the environment (VISION_PIPELINE in the config)
VISION_PIPELINE=open_vocab ros2 launch dog_toy_finder_vision stereo_vision.launch.py

# use a different physical measurement config (including its pipeline key)
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    config_path:=/workspace/data/vision/my_calibration.yaml

# depth pipeline only: keep cameras and depth, skip the detector node
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
  disparity/point cloud, and `vision:=false` skips the vision node.
- Tune `open_vocab.confidence_threshold`, `min_disparity`, `max_disparity`, and
  `max_vertical_delta_px` in the YAML when calibration or lighting makes
  left/right matches noisy.
- `approximate_sync:=true` is the default because the eyes are not
  hardware-synchronised.
- At 640x480 the ~900 KB colour frames are often dropped by DDS. If you raise
  width/height, scale `fx`, `fy`, `cx`, `cy`, and every physical measurement in
  the detector YAML accordingly.
