# Stereo vision

`dog_toy_finder_vision` turns the two ESP32-CAM eyes into a ROS 2 stereo
pipeline and runs a **selectable vision pipeline on the same rectified colour
pair**. Both pipelines are implemented:

- `open_vocab`: one host-side open-vocabulary detector runs on both rectified
  images, detections are matched between the eyes, and the target location is
  triangulated from disparity using `fx` and the stereo baseline.
- `stereo-net`: the same detector runs on the left image only; a dense stereo model
  on the host produces a disparity map from the full pair and its median over
  the object's mask pixels (YOLOE-26) or a small window around the box centre
  (YOLO-World / Grounding DINO) gives the depth.

The detector runs on the computer (YOLO-World, YOLOE-26, or Grounding DINO).
YOLOE-26-seg supplies instance masks, so the reported centre is the mean of the
mask pixels; YOLO-World and Grounding DINO are box-only, so they use the box
centre. All model weights, torch, and the dense stereo repositories stay on the
computer, never in the ROS container.

The pipeline is chosen by the launch arg `pipeline:=open_vocab|stereo-net`, the
`VISION_PIPELINE` environment variable, or the top-level `pipeline` key in
`vision_detectors.yaml` (in that order of precedence); `vision_eval` also has
`--pipeline`.

The node subscribes only to `/stereo/{left,right}/image_rect_color`. Raw images
still exist for rectification and the point-cloud pipeline, but detector inputs
are rectified.

Comparison is **capture-first**. `vision_capture` saves several rectified
left/right shots per scene with operator-measured ground truth, and `vision_eval`
later replays every saved shot through the configured pipeline. Each live answer
reports `distance_m` along the optical axis, `center_offset_m` as the signed
lateral distance from the optical-axis center (positive = robot-left / ROS
`+y`), and `lateral_bearing_deg` in degrees from the optical axis. They satisfy
`center_offset_m = distance_m * tan(lateral_bearing_deg)`. No coarse side label
is emitted or accepted.

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
measured camera values there:

- `pipeline` — `open_vocab` (default) or `stereo-net`
- `open_vocab.service_url` — host detector service URL (default
  `http://localhost:5006`; `YOLO_SERVICE_URL` overrides it)
- `open_vocab.timeout_s` — HTTP timeout in seconds
- `open_vocab.name` — fallback model name recorded in results and trial CSVs
  (the host service may report its own name)
- `open_vocab.confidence_threshold`, `max_detections`, `min_disparity`,
  `max_disparity`, `max_vertical_delta_px` — detector and stereo-matching limits
- `stereo-net.service_url` — host dense-stereo depth service URL (default
  `http://localhost:5007`; `STEREO_SERVICE_URL` overrides it)
- `stereo-net.timeout_s`, `stereo-net.window_radius_px` — HTTP timeout and the
  `(2r+1)^2` box-centre window used by box-only detectors
- `stereo-net.name`, `stereo-net.max_disparity` — fallback depth model name and the
  upper bound on sampled disparity
- The detector backend, weights, and torch device are **not** in the YAML:
  configure them on the host services with `DETECTOR_MODEL`,
  `YOLO_MODEL_PATH` / `YOLOE_MODEL_PATH` / `GROUNDING_DINO_MODEL`,
  `YOLO_DEVICE`, and `DEPTH_MODEL` / `STEREO_DEVICE` (see below)
- `camera.fx`, `fy`, `cx`, `cy` — calibrated intrinsics in pixels
- `camera.baseline_m` — spacing between the two optical centres
- `camera.depth_min_m`, `depth_max_m` — valid metric depth range
- `camera.{left,right}_rectified_topic` — detector image inputs

`full_system_eval.conf` reads `camera.width`, `height`, `fx`, `fy`, `cx`, `cy`,
and `baseline_m` from this YAML into the launch arguments, so edit them here
only. The defaults are placeholders until you calibrate the cameras.

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

The service detects the text target independently in each eye. It uses each
model's own output for the centre: **YOLOE-26-seg** returns instance masks, so
the reported `u`/`v` is the mean of the mask pixels (`center_source:
"mask_centroid"`), and the box/area fields are derived from that mask.
**YOLO-World** and **Grounding DINO** are box-only models, so they report the
box centre (`center_source: "box_center"`). No masks cross HTTP — only the
centroid, mask-derived box, and area. Full masks stay on the computer.

The container pairs detections by label, image row, and horizontal disparity;
distance is `fx * baseline_m / disparity`, and lateral bearing comes from the
left-image centre described above. The service's model name (or
`open_vocab.name`) is reported as `model`, with `method: "open_vocab"` and
`distance_source: "stereo_triangulation"` for matched detections.

Each answer uses this schema:

```json
{
  "model": "yolov8s-world",
  "method": "open_vocab",
  "distance_source": "stereo_triangulation",
  "description": "...",
  "target_visible": true,
  "lateral_bearing_deg": 12.0,
  "center_offset_m": 0.244,
  "distance_m": 1.15,
  "obstacle_ahead": "none",
  "bbox_px": [140, 96, 176, 132],
  "bbox_right_px": [120, 96, 156, 132],
  "center_source": "mask_centroid | box_center | mixed | none",
  "mask_centroid_px": [158.0, 114.0],
  "mask_area_px": 900,
  "disparity_px": 20.0
}
```

## Stereo-net pipeline (dense stereo depth)

`stereo-net` reuses the **same detector service** as `open_vocab` (configured with
`DETECTOR_MODEL`), but calls `POST /detect` with a single rectified left image
and `include_depth_samples: true`. A second host service computes a dense
disparity map from the full pair; start it once from the repository root:

```bash
./scripts/start_stereo_service.sh                          # Lite Any Stereo (default)
DEPTH_MODEL=fast_foundation_stereo ./scripts/start_stereo_service.sh
DEPTH_MODEL=rt_igevpp ./scripts/start_stereo_service.sh
```

One depth model runs at a time:

| `DEPTH_MODEL` | Model | Default weights |
|---|---|---|
| `lite_any_stereo` | Lite Any Stereo V2 (small) | `LAS2_S.pth` |
| `fast_foundation_stereo` | NVIDIA Fast-FoundationStereo 23-36-37 | `model_best_bp2_serialize.pth` |
| `rt_igevpp` | IGEV++ real-time (`core_rt`, 6 iterations) | `sceneflow.pth` |

The script creates the same `software/host_vision/.venv`, clones each upstream
repository over HTTPS into `software/host_vision/models/repos/`, and downloads
weights from Hugging Face (no file-sharing drives). It serves `POST /depth` on
`http://localhost:5007` and `GET /health` (which reports the selected `model` and
weights). The container reaches it via `STEREO_SERVICE_URL` (default
`http://localhost:5007`, also settable through `stereo-net.service_url`).

Host-only environment variables for the depth service:

| Variable | Meaning |
|---|---|
| `DEPTH_MODEL` | `lite_any_stereo` (default), `fast_foundation_stereo`, or `rt_igevpp` |
| `STEREO_DEVICE` | torch device (`cpu` or `cuda:0`); auto-selects CUDA when available |
| `LITE_ANY_STEREO_MODEL_PATH` | override the Lite Any Stereo checkpoint path |
| `FAST_FOUNDATION_STEREO_MODEL_PATH` | override the Fast Foundation Stereo checkpoint path |
| `RT_IGEVPP_MODEL_PATH` | override the RT-IGEV++ checkpoint path |
| `LITE_ANY_STEREO_REPO`, `FAST_FOUNDATION_STEREO_REPO`, `IGEV_PLUSPLUS_REPO` | override upstream source checkout locations |

The depth sampler uses every instance-mask pixel from YOLOE-26, or the
`(2 * stereo-net.window_radius_px + 1)^2` window around the box centre for
YOLO-World and Grounding DINO, and takes the median of the valid disparities.
Depth is `fx * baseline_m / disparity_px`; bearing uses the same left-image
centre and convention as `open_vocab`. The answer reports `method: "stereo-net"`,
`distance_source: "dense_stereo_depth"`, the dense model name in `model`, the
detector name in `detector_model`, and `depth_sample_source`,
`depth_sample_count`, `depth_valid_count`, and `depth_disparity_px` describing
the depth read. The trial CSV fields are the same for both pipelines: the
`model` column (dense model name) distinguishes Lite Any Stereo, Fast
Foundation Stereo, and RT-IGEV++ runs.

## Result fields

`/vision/result` contains a `results` array plus the selected `pipeline` name.
Every entry has:

| Field | Meaning |
|---|---|
| `model` | detector name for `open_vocab`; dense depth model name for `stereo-net` |
| `method` | `open_vocab` or `stereo-net` (`ml` only on legacy rows) |
| `detector_model` | detector used for the single-image detection (`stereo-net` only) |
| `target_visible` | whether the requested target was found |
| `distance_m` | optical-axis distance (normal to the optical plane) in metres, `0` if unknown/not visible |
| `center_offset_m` | signed lateral distance from the optical-axis center in metres, positive = robot-left / ROS `+y`, `0` if not visible |
| `lateral_bearing_deg` | numeric bearing from the optical axis in degrees, positive = robot-left / ROS `+y`; `atan(center_offset_m / distance_m)` |
| `obstacle_ahead` | object blocking the centre of the path, or `none` |
| `path_clear` | `true` when `obstacle_ahead` is `none` |
| `description` | human-readable detector summary |
| `seconds` | detector latency |
| `distance_source` | `stereo_triangulation` (`open_vocab`), `dense_stereo_depth` (`stereo-net`), or `none` |
| `bbox_px` | left rectified-image box `[x1, y1, x2, y2]` in pixels (mask bounds for YOLOE-26) |
| `bbox_right_px` | matching box in the right rectified image (`open_vocab` only) |
| `center_source` | `mask_centroid` (YOLOE-26), `box_center` (YOLO-World/Grounding DINO), `mixed`, or `none` |
| `mask_centroid_px` | `[u, v]` mean of mask pixels on the left image, or `null` for box-only models |
| `mask_area_px` | mask area in left-image pixels, `0` for box-only models |
| `disparity_px` | horizontal difference between the two reported centres (`open_vocab` only) |
| `depth_sample_source` | `mask_pixels` or `box_window` used by `stereo-net` |
| `depth_sample_count` | number of disparity samples requested by `stereo-net` |
| `depth_valid_count` | number of valid samples in the median |
| `depth_disparity_px` | median disparity used for dense depth (`stereo-net` only) |

The mapping node consumes all entries and fuses visible toys by median bearing
and distance. For fair offline comparison, use `vision_capture` + `vision_eval`
below: each selected pipeline sees the exact same saved images.

## Capture-first comparison

With the rectified stereo pipeline running, save several labelled shots of one
scene. No detector or `/vision/ask` is involved:

```bash
cd /workspace

ros2 run dog_toy_finder_vision vision_capture \
    --target "red ball" \
    --shots "3" \
    --run-id "capture_run1" \
    --output-dir "/workspace/data/vision/captures" \
    --config "/workspace/software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml"
```

In a TTY, `vision_capture` prompts (unless the flags are given) once per scene
for `center_offset_m along optical plane (+left)` and `distance_m along optical
axis (normal)`, computes `lateral_bearing_deg`, then suggests a `capture_id`
from the toy name and measurements, such as `red_ball_d1p15_op0p244`. Press
Enter to accept the suggestion or type a custom ID; absent targets suggest
`<toy>_absent`. `--capture-id` is not a command-line flag: non-interactive
callers get the same generated ID and must pass `--depth-m` plus
`--center-offset-m` (or, for older scripts, `--bearing-deg`).

The default `--shots "3"` saves three fresh matched pairs under
`<output_dir>/<run_id>/<capture_id>/shot_01/` through `shot_03/`, plus
`scene.json` for the shared ground truth. Each shot directory holds
`left.jpg`, `right.jpg`, and `meta.json`; each `meta.json` holds the operator's
ground truth, including the entered `center_offset_m` and computed
`lateral_bearing_deg`. `--shots "1"` keeps the older flat layout directly in
`<capture_id>/`. Use `--not-visible` for an empty scene. `vision_capture` waits
for a new matched rectified pair for every shot, so consecutive shots cannot
reuse the same camera frames from the non-hardware-synchronised ESP32-CAMs.

After capturing all scenes, replay them offline. Select **one** pipeline per
invocation with `--pipeline open_vocab` or `--pipeline stereo-net` (or set
`VISION_PIPELINE` / the YAML `pipeline` key and omit the flag); the other
pipeline is not run. Keep the same host `DETECTOR_MODEL` for both: `open_vocab`
uses the detector service only, while `stereo-net` also needs the depth service
(one depth model at a time). Use the same `--seed` for both runs so they process
the randomized captures in identical order. To compare both, use different
output paths for each run so the first result is not overwritten:

```bash
ros2 run dog_toy_finder_vision vision_eval \
    --captures-dir "/workspace/data/vision/captures" \
    --pipeline open_vocab \
    --output-csv "/workspace/data/vision/vision_eval_trials.csv" \
    --results-json "/workspace/data/vision/vision_eval_results.jsonl" \
    --summary-json "/workspace/data/vision/vision_eval_summary.json" \
    --tolerance "0.25" \
    --bearing-tolerance "5.0"

# start the host depth service, then replay the same captures with:
#   --pipeline stereo-net --output-csv .../stereo-net_trials.csv ...
```

`vision_eval` needs no ROS graph. It writes one scored row per model per shot
using the model name produced by the selected pipeline (detector name for
`open_vocab`, dense depth model name for `stereo-net`) and records the chosen
pipeline in the summary manifest. Multi-shot `ask_id` values are
`<capture_id>_shot_NN`, so every shot pairs across pipelines while sharing its
scene's operator-measured ground truth. Shots are processed in shuffled order
(`--limit` applies after the shuffle); a random `shuffle_seed` is generated
unless `--seed` is given, and the seed, `shot_order`, `scene_order`, and
`shots_per_scene` are saved in the summary manifest so the order can be
reproduced. `--capture` accepts either one shot directory or a whole scene
directory. Other useful flags include `--run-id`, `--limit`, and `--seed`. See
the
[vision comparison guide](../../../../docs/setup/vision_comparison_guide.md)
for the full experiment workflow.

## Run

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash

# full stack: cameras + rectification + depth + open-vocabulary detector
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    pipeline:=open_vocab

# one-eye detection + dense stereo depth (start both host services first)
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    pipeline:=stereo-net

# same pipeline selected via the environment (VISION_PIPELINE in the config)
VISION_PIPELINE=open_vocab ros2 launch dog_toy_finder_vision stereo_vision.launch.py

# point at a non-default host depth service
STEREO_SERVICE_URL=http://localhost:5007 ros2 launch \
    dog_toy_finder_vision stereo_vision.launch.py pipeline:=stereo-net

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
