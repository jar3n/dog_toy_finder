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
its system `cv2` does not import with the image's NumPy 2. Do not try to run
`scripts/calibrate_stereo.py` inside the container.

## Where the calibrated numbers go

| Measurement | `vision_detectors.yaml` | Direct launch argument |
|---|---|---|
| Image width / height | `camera.width`, `camera.height` | `width:=`, `height:=` |
| Focal lengths | `camera.fx`, `camera.fy` | `fx:=`, `fy:=` |
| Principal point | `camera.cx`, `camera.cy` | `cx:=`, `cy:=` |
| Stereo baseline | `camera.baseline_m` | `baseline:=` |
| Valid depth window | `camera.depth_min_m`, `camera.depth_max_m` | (not present) |

The detector YAML is the single source of truth. When `launch_all.sh` sources
`scripts/full_system_eval.conf`, that config reads these values from the YAML
into `IMAGE_WIDTH`, `CAMERA_FX`, `STEREO_BASELINE`, etc. and forwards them as
launch arguments. `vision_capture` and `vision_eval` read the same YAML, so
live geometry and offline scoring cannot drift apart. An explicit launch
argument or an exported environment variable still overrides a value for a
single run.

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

- Use a flat, rigid checkerboard with **9 columns and 12 rows of squares**,
  which gives **8 x 11 inner corners**. At 20 mm per square the pattern is
  180 x 240 mm, centered on US Letter paper (216 x 279 mm). That leaves 18 mm
  side margins and 19.5 mm top/bottom margins; a 20 mm grid cannot make all
  four margins exactly 18 mm. The script defaults to that block.
- Measure a run of ten squares with a ruler and divide by ten. For a 20 mm
  square, use `--square 0.020`. Print at 100% / actual size and do not let the
  printer scale to fit. The absolute scale matters: it sets the baseline and
  therefore all depths.
- Keep the margin white and clear. A black frame that merges with the outer
  black squares can shift or break corner detection, so cover or crop any
  touching border.
- Tape the board to something rigid and light it evenly. Avoid glare and motion
  blur.

## 3. Capture a calibration set

Use the same resolution the node publishes (by default 320 x 240; the ESP32-CAMs
may stream a larger JPEG and the node resizes it). The capture script prompts
before each pose so you can hold the sheet still, then captures both eyes when
you press Enter:

```bash
scripts/capture_calibration_pairs.sh
```

It writes `01.jpg` through `15.jpg` under `/tmp/dtf_calib/left` and
`/tmp/dtf_calib/right`. Use `scripts/capture_calibration_pairs.sh 15 /path/to/dir`
for a different directory, and set `DTF_LEFT_CAPTURE_URL` /
`DTF_RIGHT_CAPTURE_URL` if the camera URLs differ. Type `q` at a prompt to stop
early; a failed capture is retried at the same pair number.

After each pair, the script opens a temporary side-by-side view of the left and
right JPEGs. Check that both eyes show the full board with no glare, blur, or
clipped corners, then press Enter to accept, `r` to recapture that pair, or `q`
to quit. Set `DTF_CALIB_PREVIEW=0` to disable the preview on a headless host.

To recapture one numbered pose, use `scripts/capture_one_pair.sh 4` for pair 4,
or `scripts/capture_one_pair.sh 9 /path/to/dir` for pair 9 in another directory.
The number matches the table below; the script overwrites that pair's JPEGs and
uses the same camera URL and preview settings.

If a pair needs a different distance or tilt, pass overrides:

```bash
scripts/capture_one_pair.sh 4 --distance 1.2
scripts/capture_one_pair.sh 9 --yaw 15 --pitch -10
scripts/capture_one_pair.sh 10 --distance 0.8 --yaw -15 --pitch 10
```

The overrides change the displayed pose only; the pair number still selects
the output filenames. After recapturing, rerun `scripts/calibrate_stereo.py` as
normal: it reads the numbered JPEGs and does not need the override values.

Between pairs, move the board to different distances and positions and tilt it
moderately in both axes. Keep the whole board visible in **both** images and
hold it still while the two captures are taken; the ESP32-CAMs are not
hardware-synchronised, so each pair is only approximately simultaneous.

How to hold the sheet:

- Keep the printed side facing the cameras and hold it in **portrait** orientation:
  9 columns across, 12 rows down. The default `8x11` inner-corner pattern
  assumes this. Cover or crop any extra rows, columns, or border decorations so
  only the 9 x 12 square block is visible. A 90 degree rotation will not be
  detected unless you pass `--pattern 11x8`.
- Tape or clip the paper to a flat, rigid backing such as cardboard or foam
  board, and hold the backing rather than the paper corners. A curled sheet
  gives unreliable corners.
- Keep the top edge up and the grid roughly aligned with the image rows and
  columns. Do not let one edge of the board hide another.
- Leave a visible border around the whole pattern in **each** eye. If any outer
  corner is closer than about 10 px to an image edge, move the sheet back.
- Capture one pair for each pose below. All sheet poses are defined in the
  **camera frame**, from the cameras' point of view (POV), not in the robot or
  world frame. Put the origin at the midpoint between the two lens optical
  centres. Distance is along the cameras' optical axis away from the cameras,
  and left/right/up/down mean image-left/image-right/image-up/image-down in
  the camera view. The floor or table is not part of this frame. If a thick
  backing leaves the bottom square row about 1 cm above the ground, that is
  fine: measure to the sheet centre, not the backing edge or the floor, and
  do not change `--square` for the backing thickness. Just make sure the
  backing does not hide any square and the full pattern is visible in both
  eyes.
- The angle signs are camera-relative too: yaw rotates about the image-vertical
  axis (`+` = right edge farther from the cameras) and pitch tilts about the
  image-horizontal axis (`+` = top edge farther). Offsets move the sheet centre
  within the camera image plane.

  The table is grouped: a distance sweep at neutral pose, an angle sweep at
  1.0 m covering all four yaw/pitch sign combinations, then an offset sweep
  at 1.0 m.

  | # | Set | Distance | Offset | Yaw | Pitch |
  |---:|---|---:|---|---:|---:|
  | 1 | Distance | 0.6 m | center | 0° | 0° |
  | 2 | Distance | 0.8 m | center | 0° | 0° |
  | 3 | Distance | 1.0 m | center | 0° | 0° |
  | 4 | Distance | 1.5 m | center | 0° | 0° |
  | 5 | Angle | 1.0 m | center | +20° | 0° |
  | 6 | Angle | 1.0 m | center | -20° | 0° |
  | 7 | Angle | 1.0 m | center | 0° | +15° |
  | 8 | Angle | 1.0 m | center | 0° | -15° |
  | 9 | Angle | 1.0 m | center | +20° | -15° |
  | 10 | Angle | 1.0 m | center | -20° | +15° |
  | 11 | Angle | 1.0 m | center | +20° | +15° |
  | 12 | Angle | 1.0 m | center | -20° | -15° |
  | 13 | Offset | 1.0 m | 0.20 m left | 0° | 0° |
  | 14 | Offset | 1.0 m | 0.20 m right | 0° | 0° |
  | 15 | Offset | 1.0 m | 0.20 m up | 0° | 0° |

  If any corner comes within about 10 px of an image edge at a listed pose,
  move that pose farther away and recapture it rather than cropping or editing
  the JPEG.

  A downward offset is intentionally omitted: a thick backing resting on the
  floor cannot move lower without lifting the cameras. The angle sweep covers
  that variation instead.
- Freeze the sheet before pressing Enter at the capture prompt and keep it
  still until both JPEGs are saved. Moving between the two captures breaks the
  stereo correspondence because the cameras are not hardware-synchronised.
- Avoid glare and hard shadows. If squares are washed out, change the sheet
  angle or the lighting rather than editing the JPEGs.
- Use the preview after each capture to confirm both eyes show the full board.
  Recapture any pair where an inner corner is cut off, out of focus, blurred,
  or where the board is visible in only one eye.

## 4. Solve for intrinsics and baseline

This uses a pinhole model with zero distortion, matching the `CameraInfo` that
`stereo_camera_node.py` publishes (it has no distortion coefficients and assumes
both eyes share the same intrinsics). It resizes each capture to the launch
resolution first, so the result describes the images the node actually
processes.

From the repository root, run `scripts/calibrate_stereo.py` with the host venv:

```bash
software/host_vision/.venv/bin/python scripts/calibrate_stereo.py \
    --left /tmp/dtf_calib/left --right /tmp/dtf_calib/right \
    --pattern 8x11 --square 0.020 --width 320 --height 240
```

Aim for a reprojection RMS of a couple of pixels or less at 320 x 240. If it is
much larger, discard blurry pairs, add more viewpoints, and recheck the measured
square size. If the printed baseline is implausible for your mount, re-measure
the distance between the two lens optical centres; depth errors scale directly
with this number.

## 5. Record the values

Put the printed numbers in `vision_detectors.yaml`, then source
`full_system_eval.conf` as usual. The config reads them from the YAML; there is
no second camera-geometry block to edit:

```yaml
camera:
  width: 320
  height: 240
  fx: 250.0       # replace with the printed numbers
  fy: 250.0
  cx: 160.0
  cy: 120.0
  baseline_m: 0.08
  depth_min_m: 0.25
  depth_max_m: 3.0
```

For a one-off override, export `CAMERA_FX=...` (or any of the other `CAMERA_*`
variables) before sourcing `full_system_eval.conf`; exported values still win
over the YAML.

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
2. Save a scene of shots with the interactive `vision_capture` procedure in the
   [vision comparison guide](vision_comparison_guide.md#capture-a-vision-dataset-step-by-step).
3. Replay them with `vision_eval` and compare the reported depth with the tape
   measure. A systematic ratio means `baseline_m` or `fx` is still wrong; a
   constant angular offset means `cx` is wrong.
4. Repeat with the toy to the left and right of the optical axis. Left-side
   positions must give **positive** `lateral_bearing_deg`.

Re-run this calibration whenever a camera is moved or refocused, the baseline
changes, the streaming resolution changes, or a depth sanity check drifts by
more than about 10%.
