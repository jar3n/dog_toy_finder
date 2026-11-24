# Stereo camera calibration

The shipped camera values are starting estimates, not measurements. Both vision
pipelines depend on them:

```text
distance     = fx * baseline_m / disparity
lateral bearing = atan((cx - u) / fx)     # degrees, positive to robot-left (+y)
```

If `fx` or `baseline_m` is wrong, every reported distance is wrong by the same
scale factor. If `cx` is wrong, every bearing has a constant offset. Calibrate
once for a fixed camera mount, then record the numbers where both the live node
and the offline evaluator can read them.

This procedure uses the cameras' HTTP `/capture` endpoints and OpenCV on the
**host**. The ROS 2 image has `camera_calibration_parsers`, `image_proc`, and
`stereo_image_proc`, but not the interactive `camera_calibration` package, and
its system `cv2` does not import with the image's NumPy 2. Do not try to run the
calibration Python below inside the container.

## Where the calibrated numbers go

| Measurement | `scripts/full_system_eval.conf` | `vision_detectors.yaml` | Direct launch argument |
|---|---|---|---|
| Image width / height | `IMAGE_WIDTH`, `IMAGE_HEIGHT` | `camera.width`, `camera.height` | `width:=`, `height:=` |
| Focal lengths | `CAMERA_FX`, `CAMERA_FY` | `camera.fx`, `camera.fy` | `fx:=`, `fy:=` |
| Principal point | `CAMERA_CX`, `CAMERA_CY` | `camera.cx`, `camera.cy` | `cx:=`, `cy:=` |
| Stereo baseline | `STEREO_BASELINE` | `camera.baseline_m` | `baseline:=` |
| Valid depth window | (not present) | `camera.depth_min_m`, `camera.depth_max_m` | (not present) |

`launch_all.sh` forwards the `full_system_eval.conf` values as launch arguments,
so those win for the live node. `vision_eval` and `vision_capture` read only the
YAML. **Keep both in sync**, or captures and scores will be computed with
different geometry.

## 1. Prepare the host calibration environment

If you have already started a host vision service, its venv exists and already
contains OpenCV. Otherwise create it and install just the calibration
dependencies:

```bash
cd /path/to/dog_toy_finder
python3 -m venv software/host_vision/.venv
software/host_vision/.venv/bin/pip install numpy opencv-python-headless
```

The host services install the full `software/host_vision/requirements.txt` into
the same venv later; that file already includes `opencv-python-headless`.

## 2. Print and measure a checkerboard

- Use a flat, rigid checkerboard with **8 columns and 7 rows of squares**, which
  gives **7 x 6 inner corners**. The script below defaults to that pattern.
- Measure one square with a ruler. For a 25 mm square, use `--square 0.025`.
  The absolute scale matters: it sets the baseline and therefore all depths.
- Tape the board to something rigid and light it evenly. Avoid glare and motion
  blur.

## 3. Capture a calibration set

Use the same resolution the node publishes (by default 320 x 240; the ESP32-CAMs
may stream a larger JPEG and the node resizes it). Capture 12-20 left/right
pairs from both eyes:

```bash
mkdir -p /tmp/dtf_calib/left /tmp/dtf_calib/right
for i in $(seq -w 1 15); do
    curl -fsS http://esp32cam-left.local/capture  -o /tmp/dtf_calib/left/$i.jpg
    curl -fsS http://esp32cam-right.local/capture -o /tmp/dtf_calib/right/$i.jpg
    sleep 0.5
done
```

Between pairs, move the board to different distances and positions and tilt it
moderately in both axes. Keep the whole board visible in **both** images and
hold it still while the two captures are taken; the ESP32-CAMs are not
hardware-synchronised, so each pair is only approximately simultaneous. Skip
and recapture any pair where a corner is cut off or blurred.

## 4. Solve for intrinsics and baseline

This uses a pinhole model with zero distortion, matching the `CameraInfo` that
`stereo_camera_node.py` publishes (it has no distortion coefficients and assumes
both eyes share the same intrinsics). It resizes each capture to the launch
resolution first, so the result describes the images the node actually
processes.

```bash
software/host_vision/.venv/bin/python - \
    --left /tmp/dtf_calib/left --right /tmp/dtf_calib/right \
    --pattern 7x6 --square 0.025 --width 320 --height 240 <<'PY'
import argparse
import glob
import os
import sys

import cv2
import numpy as np

p = argparse.ArgumentParser()
p.add_argument("--left", default="/tmp/dtf_calib/left")
p.add_argument("--right", default="/tmp/dtf_calib/right")
p.add_argument("--pattern", default="7x6", help="inner corners: cols x rows")
p.add_argument("--square", type=float, default=0.025, help="square size, metres")
p.add_argument("--width", type=int, default=320)
p.add_argument("--height", type=int, default=240)
a = p.parse_args()

cols, rows = (int(v) for v in a.pattern.lower().split("x"))
object_template = np.zeros((cols * rows, 3), np.float32)
object_template[:, :2] = (
    np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * a.square)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5)


def grey(path):
    image = cv2.imread(path)
    if image is None:
        return None
    image = cv2.resize(image, (a.width, a.height), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


object_points, left_points, right_points = [], [], []
left_paths = sorted(glob.glob(os.path.join(a.left, "*.jpg")))
right_paths = sorted(glob.glob(os.path.join(a.right, "*.jpg")))
if len(left_paths) != len(right_paths) or not left_paths:
    sys.exit("capture the same number of left and right JPEGs first")

for left_path, right_path in zip(left_paths, right_paths):
    gl, gr = grey(left_path), grey(right_path)
    if gl is None or gr is None:
        continue
    found_l, corners_l = cv2.findChessboardCorners(gl, (cols, rows), None)
    found_r, corners_r = cv2.findChessboardCorners(gr, (cols, rows), None)
    if not (found_l and found_r):
        print("skip", os.path.basename(left_path), os.path.basename(right_path))
        continue
    corners_l = cv2.cornerSubPix(gl, corners_l, (5, 5), (-1, -1), criteria)
    corners_r = cv2.cornerSubPix(gr, corners_r, (5, 5), (-1, -1), criteria)
    object_points.append(object_template)
    left_points.append(corners_l)
    right_points.append(corners_r)

if len(object_points) < 8:
    sys.exit("need at least 8 valid pairs; recapture with the board fully visible")

size = (a.width, a.height)
guess_k = np.array([[500.0, 0.0, size[0] / 2.0],
                    [0.0, 500.0, size[1] / 2.0],
                    [0.0, 0.0, 1.0]], dtype=np.float64)
zero_d = np.zeros((5, 1), dtype=np.float64)
# The node publishes zero distortion and one shared K for both eyes.
fixed_zero_distortion = (
    cv2.CALIB_FIX_K1 | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3 |
    cv2.CALIB_FIX_TANGENT_DIST)

rms, k1, d1, k2, d2, rotation, translation, _, _ = cv2.stereoCalibrate(
    object_points, left_points, right_points,
    guess_k.copy(), zero_d.copy(), guess_k.copy(), zero_d.copy(), size,
    criteria=criteria, flags=fixed_zero_distortion)

baseline = abs(float(translation[0]))
print("valid pairs: %d, reprojection RMS: %.2f px" % (len(object_points), rms))
print("CAMERA_FX=%.2f" % k1[0, 0])
print("CAMERA_FY=%.2f" % k1[1, 1])
print("CAMERA_CX=%.2f" % k1[0, 2])
print("CAMERA_CY=%.2f" % k1[1, 2])
print("STEREO_BASELINE=%.4f" % baseline)
PY
```

Aim for a reprojection RMS of a couple of pixels or less at 320 x 240. If it is
much larger, discard blurry pairs, add more viewpoints, and recheck the measured
square size. If the printed baseline is implausible for your mount, re-measure
the distance between the two lens optical centres; depth errors scale directly
with this number.

## 5. Record the values

Because `full_system_eval.conf` uses `:-` defaults, exported values win. Put the
printed numbers in the shell environment **before** sourcing it, or edit the
defaults in the file directly:

```bash
export IMAGE_WIDTH=320
export IMAGE_HEIGHT=240
export CAMERA_FX=250.0      # replace with the printed numbers
export CAMERA_FY=250.0
export CAMERA_CX=160.0
export CAMERA_CY=120.0
export STEREO_BASELINE=0.08
source /workspace/scripts/full_system_eval.conf
```

Then set the same values in `vision_detectors.yaml`, which the offline tools
read:

```yaml
camera:
  width: 320
  height: 240
  fx: 250.0
  fy: 250.0
  cx: 160.0
  cy: 120.0
  baseline_m: 0.08
  depth_min_m: 0.25
  depth_max_m: 3.0
```

`depth_min_m` and `depth_max_m` are not solved by the checkerboard. Choose them
from the useful range of your rig: `depth_min_m` should be above the distance
where `fx * baseline_m / depth` exceeds the launch `disparity_range` (default
64 px), and `depth_max_m` below where disparity approaches the noise floor.
The depth service also uses these bounds, so the same YAML applies to both
pipelines.

If you launch `stereo_vision.launch.py` directly instead of through
`launch_all.sh`, pass the geometry explicitly:

```bash
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    width:=320 height:=240 fx:=250.0 fy:=250.0 \
    cx:=160.0 cy:=120.0 baseline:=0.08
```

## 6. Verify on the robot

Restart the stereo pipeline with the new values, then check the published camera
info and rectified images:

```bash
ros2 topic echo /stereo/left/camera_info --once
ros2 topic echo /stereo/right/camera_info --once
ros2 topic hz /stereo/left/image_rect_color
ros2 topic hz /stereo/right/image_rect_color
ros2 topic echo /stereo/disparity --once
```

The `k` matrix should contain the new `fx`/`cx`, and the right camera info
should show `p[3]` near `-fx * baseline`. Then do a physical sanity check with
a tape measure (no motion stack or Pico needed):

1. Put the toy at a known distance, for example 1.00 m from the camera baseline,
   and near the optical axis.
2. Save a capture with the interactive `vision_capture` procedure in
   [llm_comparison_guide.md](llm_comparison_guide.md#a2-capture-a-vision-dataset-step-by-step).
3. Replay it with `vision_eval` and compare the reported depth with the tape
   measure. A systematic ratio means `baseline_m` or `fx` is still wrong; a
   constant angular offset means `cx` is wrong.
4. Repeat with the toy to the left and right of the optical axis. Left-side
   positions must give **positive** `lateral_bearing_deg`.

Re-run this calibration whenever a camera is moved or refocused, the baseline
changes, the streaming resolution changes, or a depth sanity check drifts by
more than about 10%.
