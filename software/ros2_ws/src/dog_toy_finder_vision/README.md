# Stereo vision

`dog_toy_finder_vision` turns the two ESP32-CAM eyes into a ROS 2 stereo
pipeline: raw images and camera info from `stereo_camera_node`, then disparity
and a coloured point cloud from `stereo_image_proc`.

## Topics

### Stereo pipeline
| Topic | Type | Source |
|---|---|---|
| `/stereo/left/image_raw` | `sensor_msgs/Image` | `stereo_camera_node` |
| `/stereo/right/image_raw` | `sensor_msgs/Image` | `stereo_camera_node` |
| `/stereo/left/camera_info` | `sensor_msgs/CameraInfo` | `stereo_camera_node` |
| `/stereo/right/camera_info` | `sensor_msgs/CameraInfo` | `stereo_camera_node` |
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

## Run

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source install/setup.bash

# stereo pipeline (images + disparity + point cloud)
ros2 launch dog_toy_finder_vision stereo.launch.py \
    left_url:=http://esp32cam-left.local/capture \
    right_url:=http://esp32cam-right.local/capture

# VLM vision on the stereo pair (separate shell)
ros2 launch dog_toy_finder_vision vision.launch.py
ros2 launch dog_toy_finder_vision vision.launch.py models:="gemma3,llava,moondream"
```

The intrinsics (`fx`, `fy`, `cx`, `cy`) and `baseline` are placeholders until
the cameras are calibrated; run a stereo calibration (e.g. with
`camera_calibration` or OpenCV) and pass the measured values.

## Tuning

- `rate` controls how often each eye is sampled (default 2 Hz). Higher rates
  need a stronger Wi-Fi link.
- `stereo_image_proc` parameters (disparity range, window size, uniqueness
  ratio, ...) can be overridden through its launch arguments.
- The two eyes are not hardware-synchronised, so `approximate_sync:=True` is
  set by default.