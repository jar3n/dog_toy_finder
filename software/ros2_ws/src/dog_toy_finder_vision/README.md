# Stereo vision

`dog_toy_finder_vision` turns the two ESP32-CAM eyes into a ROS 2 stereo
pipeline and runs **two independent toy detectors on the same rectified colour
pair**:

1. **Classical path** — deterministic colour thresholding, connected
   components, edge checks, and rectified-stereo patch matching. No learned
   model is involved.
2. **ML path** — one or more Ollama vision models. Models are loaded and
   unloaded one at a time, so each gets all available VRAM during comparison
   runs.

Both paths subscribe only to `/stereo/{left,right}/image_rect_color`. Raw images
still exist for rectification and the point-cloud pipeline, but detector inputs
are rectified.

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

Both paths read the same YAML file:

```text
software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml
```

The path is set once in `scripts/full_system_eval.conf` as
`VISION_DETECTOR_CONFIG` and forwarded through the launch file. Put your
measured camera and toy values there:

- `camera.fx`, `fy`, `cx`, `cy` — calibrated intrinsics in pixels
- `camera.baseline_m` — spacing between the two optical centres
- `camera.depth_min_m`, `depth_max_m` — valid metric depth range
- `camera.{left,right}_rectified_topic` — detector image inputs
- `target.known_width_m`, `known_height_m` — real toy measurements
- `target.position_split_left/right` — image thirds used for left/center/right
- `color_ranges` — HSV limits per colour word in the ask target
- `classical.*` — area, shape, edge, disparity search, and depth fusion limits

The camera values should match the `CAMERA_*` and `STEREO_BASELINE` values in
`full_system_eval.conf`. The defaults are placeholders until you calibrate the
cameras and measure the toy.

## Classical path

`classical_detector.py` uses only NumPy:

1. Pick HSV ranges from the target text (`"red ball"` → red ranges). Unknown
   colours fall back to `target.default_color`.
2. Threshold, open/close, and find connected components.
3. Reject candidates by area, bbox size, aspect ratio, fill ratio, and mean
   Sobel edge strength.
4. Match small greyscale patches along image rows in the rectified right image
   to get disparity.
5. Convert disparity to depth with `depth = fx * baseline / disparity`, then
   fuse it with `fx * real_toy_width / bbox_width_px` using the configured
   weights. The two estimates are also compared for consistency.

It returns a `bbox_px`, a `confidence`, a `distance_source`, and a conservative
edge-density obstacle check in the centre of the lower image.

## ML path

The ML path sends the same two rectified images to Ollama together with a short
physical-measurement summary from the YAML config. It returns the same semantic
answer fields as before:

```json
{
  "description": "...",
  "target_visible": true,
  "position": "left | center | right | none",
  "size": "small | medium | large | none",
  "distance_m": 0.0,
  "obstacle_ahead": "..."
}
```

Set `models:="gemma4:latest,minicpm-v4.5:latest,qwen3.5:4b"` to compare models
on every ask. Each answer gets `method: "ml"`, its model name, and timing. The
classical answer gets `method: "classical"` and model name `classical`.

## Result fields

`/vision/result` contains a `results` array. Every entry has:

| Field | Meaning |
|---|---|
| `model` | Ollama model name or `classical` |
| `method` | `classical` or `ml` |
| `target_visible` | whether the requested target was found |
| `position` | target in the image: `left`, `center`, `right`, or `none` |
| `size` | `small`, `medium`, `large`, or `none` |
| `distance_m` | distance to the target in metres, `0` if unknown/not visible |
| `obstacle_ahead` | object blocking the centre of the path, or `none` |
| `path_clear` | `true` when `obstacle_ahead` is `none` |
| `description` | human-readable detector/model summary |
| `seconds` | detector/model latency |
| `distance_source` | how the metric distance was derived |
| `confidence` | detector confidence (classical path) |

The mapping node consumes all entries and fuses visible toy positions and
distances. The trial collector writes one CSV row per entry, including a
`method` column so classical and ML results can be compared directly.

## Run

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash

# full stack: cameras + rectification + depth + both detectors
ros2 launch dog_toy_finder_vision stereo_vision.launch.py

# compare several Ollama models on the same rectified pair
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    models:="gemma4:latest,minicpm-v4.5:latest,qwen3.5:4b"

# use a different physical measurement config
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    config_path:=/workspace/data/vision/my_calibration.yaml

# run only one path
ros2 launch dog_toy_finder_vision stereo_vision.launch.py ml:=false
ros2 launch dog_toy_finder_vision stereo_vision.launch.py classical:=false

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
  disparity/point cloud, `vision:=false` skips both detectors, and
  `classical:=false` / `ml:=false` select one detector path.
- `approximate_sync:=true` is the default because the eyes are not
  hardware-synchronised.
- At 640x480 the ~900 KB colour frames are often dropped by DDS. If you raise
  width/height, scale `fx`, `fy`, `cx`, `cy`, and every physical measurement in
  the detector YAML accordingly.
