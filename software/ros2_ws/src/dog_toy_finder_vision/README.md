# Stereo vision

`dog_toy_finder_vision` turns the two ESP32-CAM eyes into a ROS 2 stereo
pipeline: raw colour + mono images and camera info from `stereo_camera_node`,
rectification from `image_proc`, disparity and a coloured point cloud from
`stereo_image_proc`, and a VLM answering `/vision/ask`. One launch file,
`stereo_vision.launch.py`, starts the whole stack.

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

### VLM vision (stereo)
| Topic | Type | Source |
|---|---|---|
| `/stereo/left/image_raw` | `sensor_msgs/Image` | `stereo_camera_node` |
| `/stereo/right/image_raw` | `sensor_msgs/Image` | `stereo_camera_node` |
| `/vision/ask` | `std_msgs/String` (JSON) | user / planner |
| `/vision/result` | `std_msgs/String` (JSON) | `vision_node` |

## Comparing LLMs

The vision node accepts a comma-separated `models` parameter, e.g.
`models:="gemma3,llava,moondream"`. It runs every model on the same photo and
publishes a `/vision/result` whose `results` array contains one answer plus
timing per model, so the models can be compared directly.

The ROSA planner supports the same idea via
`python3 -m rosa_agent --compare llama3.2,mistral,qwen2.5`.

## Result fields

Each entry in `/vision/result`'s `results` array contains:

| Field | Meaning |
|---|---|
| `target_visible` | whether the requested target appears in the image |
| `position` | target in the image: `left`, `center`, `right`, or `none` |
| `size` | `small`, `medium`, `large`, or `none` |
| `distance_m` | estimated distance to the target in metres, `0` if unknown/not visible |
| `obstacle_ahead` | object blocking the centre of the path, or `none` |
| `path_clear` | `true` when `obstacle_ahead` is `none` |
| `description` | free-text scene description |

`position` and `distance_m` are combined by the mapping node to place toy
markers: distance ahead, then a configurable offset to the left or right.

## Run

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash

# full stack: cameras + rectification + disparity + VLM
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    left_url:=http://esp32cam-left.local/capture \
    right_url:=http://esp32cam-right.local/capture

# compare several VLMs on the same stereo pair
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    models:="gemma3,llava,moondream"

# depth only, no VLM (useful to debug the stereo pipeline)
ros2 launch dog_toy_finder_vision stereo_vision.launch.py vision:=false
```

The default capture size is 320x240 (`fx=fy=250`, `cx=160`, `cy=120`). The
intrinsics and `baseline` are placeholders until the cameras are calibrated; run
a stereo calibration (e.g. with `camera_calibration` or OpenCV) and pass the
measured values. If you raise `width`/`height`, scale `fx`, `fy`, `cx` and `cy`
with them: at 640x480 the ~900 KB rgb8 frames are often dropped by the DDS
transport, which produces "do not appear to be synchronized" warnings and
stalls the point cloud.

## Tuning

- `rate` controls how often each eye is sampled (default 2 Hz). Higher rates
  need a stronger Wi-Fi link.
- `stereo:=false` skips the cameras and depth pipeline, `stereo_proc:=false`
  skips disparity/point cloud, and `vision:=false` skips the VLM server.
- `stereo_image_proc` exposes `stereo_algorithm`, `min_disparity`,
  `disparity_range`, `use_color`, and `approximate_sync` through the launch
  file.
- The two eyes are not hardware-synchronised, so `approximate_sync:=true` is
  set by default.
- Disparity is published on `/stereo/disparity` and the point cloud on
  `/stereo/points2`; the launch file remaps `stereo_image_proc`'s default
  `/disparity` and `/points2` names.

## Pipeline stages

`stereo_vision.launch.py` runs the camera node, one mono and one colour
rectifier per eye, then `stereo_image_proc`, then the VLM. The cameras output
`rgb8` and `mono8` directly, so there is no debayer stage: feeding a colour
image to `debayer_node` would produce no output at all.
