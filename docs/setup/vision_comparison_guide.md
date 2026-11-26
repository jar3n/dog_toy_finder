# Vision pipeline comparison guide

This guide covers **capture-first** comparison of the two vision pipelines.
Save rectified left/right images with operator-entered ground truth once, then
replay every saved capture offline through the configured pipeline. Planning
model comparison is separate; see the
[planning comparison guide](planning_comparison_guide.md).

The vision pipeline is selected with `pipeline:=open_vocab|stereo-net` on the launch
command, `VISION_PIPELINE` in `scripts/full_system_eval.conf`, the top-level
`pipeline` key in `vision_detectors.yaml`, or `vision_eval --pipeline` (explicit
selections win). Both are implemented:

- `open_vocab`: one host-side open-vocabulary detector (YOLO-World, YOLOE-26,
  or Grounding DINO) runs on both rectified images, matching detections are
  paired, and distance comes from stereo triangulation using `fx` and the stereo
  baseline.
- `stereo-net`: the same detector runs on the left image only and a host dense-stereo
  model (Lite Any Stereo, Fast Foundation Stereo, or RT-IGEV++) supplies depth
  from a disparity map, sampled over the object's mask pixels (YOLOE-26) or a
  small box-centre window (box-only detectors).

Run **one pipeline per invocation**. Each pipeline produces one answer per
capture, so rows stay comparable across runs.

Vision answers report three numeric measurements: `distance_m` along the
optical axis, `center_offset_m` as the signed lateral distance from the
optical-axis center (positive = robot-left / ROS `+y`), and
`lateral_bearing_deg` in degrees from the optical axis. They satisfy
`center_offset_m = distance_m * tan(lateral_bearing_deg)`. No coarse side label
is emitted or accepted. The same convention is used by live control, mapping,
and the offline evaluator.

| Tool | Purpose |
|---|---|
| `vision_capture` | Saves several fresh rectified stereo shots of one scene with operator-entered depth and center offset; computes bearing. |
| `vision_eval` | Replays every saved shot through the configured vision pipeline, then writes a scored CSV. |
| `llm_trials` CLI | `template`, `score`, `aggregate`, and `analyze` for vision CSVs. |

The capture/eval commands write the automated columns and copy the ground truth
from each capture's `meta.json` into the `gt_*` columns. `score` fills the metric
columns when a CSV is annotated later.

## Where vision trial data lives

Generated data lives under the repository's `data/` folder, next to `hardware/`
and `software/`:

```text
data/
  vision/
    captures/<run_id>/<capture_id>/scene.json       # shot_count and shared ground truth
    captures/<run_id>/<capture_id>/shot_01/left.jpg
    captures/<run_id>/<capture_id>/shot_01/right.jpg
    captures/<run_id>/<capture_id>/shot_01/meta.json
    captures/<run_id>/<capture_id>/shot_02/...       # one directory per shot
    captures/<run_id>/<capture_id>/shot_03/...
    vision_eval_trials.csv                     # default generic output
    vision_eval_results.jsonl
    vision_eval_summary.json
    vision_eval_analysis.json
    open_vocab_trials.csv                      # example per-pipeline outputs
    open_vocab_results.jsonl
    open_vocab_summary.json
    stereo-net_trials.csv
    stereo-net_results.jsonl
    stereo-net_summary.json
```

A scene captured with `--shots 1` keeps the older flat layout with `left.jpg`,
`right.jpg`, and `meta.json` directly in `<capture_id>/`; `vision_eval` discovers
both layouts.

Inside the ROS 2 container the repository is mounted at `/workspace`, so the
commands below use `/workspace/data/...`. On the host the same files appear
under `<repo>/data/...`. The `vision/` folder is tracked, but its generated
contents are ignored by git.

## One-time setup

Inside the ROS 2 container:

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

The `ros2` image ships Pillow for JPEG conversion. Open-vocabulary detection
itself runs on the computer: start `./scripts/start_yolo_service.sh` once, which
creates a host venv with CPU torch/ultralytics/transformers and downloads the
selected weights to `software/host_vision/models/`. One detector runs at a
time; choose it with `DETECTOR_MODEL=yolo_world|yoloe_26|grounding_dino`
(defaults: `yolov8s-world.pt`, `yoloe-26s-seg.pt`, and
`IDEA-Research/grounding-dino-tiny`). The container sends rectified JPEGs to
that service at `YOLO_SERVICE_URL` (default `http://localhost:5006`).

For the `stereo-net` pipeline also start `./scripts/start_stereo_service.sh` on the
host. It reuses the same venv and serves dense disparity at
`STEREO_SERVICE_URL` (default `http://localhost:5007`). One depth model runs at
a time, selected with
`DEPTH_MODEL=lite_any_stereo|fast_foundation_stereo|rt_igevpp`; weights and
upstream repositories are prepared under `software/host_vision/models/`. The
depth model name, not the detector name, is recorded in the `model` column so
runs can be compared. `STEREO_DEVICE` selects the torch device (auto-detects
CUDA); per-model `*_MODEL_PATH` variables override checkpoint locations.

No vision Ollama server is needed. Ollama is only used by the
[planning comparison guide](planning_comparison_guide.md); see
[`software/rosa_agent/README.md`](../../software/rosa_agent/README.md) for its
setup.

Confirm the data package is available:

```bash
ros2 run dog_toy_finder_data llm_trials --help
```

## Set the physical measurements and bearing convention

Edit
`/workspace/software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml`
(or point `VISION_DETECTOR_CONFIG` in `scripts/full_system_eval.conf` at your
own copy). At minimum, set the calibrated `fx`, `fy`, `cx`, `cy`, `baseline_m`,
and the valid `camera.depth_min_m`/`depth_max_m`. The top-level `pipeline` key
selects the vision pipeline (`open_vocab` by default); `open_vocab.service_url`
points at the host detector service and `open_vocab.name` is the fallback model
name recorded in results and trial CSVs. The `stereo-net.service_url`,
`stereo-net.window_radius_px`, and `stereo-net.max_disparity` keys tune the dense-depth
path. The detector backend, weights, and torch device are configured on the
host service via `DETECTOR_MODEL`, `YOLO_MODEL_PATH` / `YOLOE_MODEL_PATH` /
`GROUNDING_DINO_MODEL`, and `YOLO_DEVICE`; the dense model is selected on the
host with `DEPTH_MODEL` and `STEREO_DEVICE`. `fx` and `baseline_m` drive both
triangulation and dense depth (`distance = fx * baseline / disparity`), and
`cx`/`fx` set the bearing. Use
[camera_calibration.md](camera_calibration.md) to measure these values and the
stereo baseline for your rig before starting capture.

`lateral_bearing_deg` is measured from the camera's optical axis in degrees:

```text
bearing = atan((cx - u) / fx)
```

where `u` is the target's horizontal pixel centre. For **YOLOE-26-seg** that is
the mean of the instance-mask pixels (`center_source: "mask_centroid"`); for
**YOLO-World** and **Grounding DINO**, which are box-only models, it is the
detected box centre (`center_source: "box_center"`). Only centroid/box/area
numbers cross HTTP; full masks stay on the computer. The same centre drives
disparity and depth.

Positive bearing means the toy is to the robot's **left** (image column less
than `cx`, ROS `+y`), negative means right, and zero means centred. The
pipeline also reports the lateral distance from the optical-axis center:

```text
center_offset_m = distance_m * (cx - u) / fx
```

These three values are consistent by construction:
`lateral_bearing_deg = atan(center_offset_m / distance_m)`. `center_offset_m`
is signed, positive to the robot's left; there is no coarse side label.

## Capture a vision dataset (step by step)

Use two shells inside the ROS 2 container: one keeps the stereo pipeline
running, the other runs `vision_capture`. No host detector or depth service is
needed for capture; those are started later for `vision_eval`.

### Step 1 - Use the default settings in both shells

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/software/ros2_ws/install/setup.bash
```

The default capture settings used below are:

```text
captures=/workspace/data/vision/captures
config=/workspace/software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml
run=capture_run1
target=red ball
```

`scripts/full_system_eval.conf` defines these defaults. The YAML at `config`
must be the same file whose `fx`, `cx`, and baseline you checked above;
`vision_capture` copies those values into each `meta.json`. The config also
reads `camera.width`, `height`, `fx`, `fy`, `cx`, `cy`, and `baseline_m` from
that YAML into the launch arguments below, so the live pipeline and the capture
metadata share one set of numbers.

### Step 2 - Start a capture-only stereo pipeline (first shell)

```bash
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    left_url:="http://esp32cam-left.local/capture" \
    right_url:="http://esp32cam-right.local/capture" \
    rate:="2.0" \
    width:="320" \
    height:="240" \
    fx:="299.72" \
    fy:="300.92" \
    cx:="175.61" \
    cy:="127.57" \
    baseline:="0.0431" \
    config_path:="/workspace/software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml" \
    stereo_proc:=false \
    vision:=false
```

The `width`/`height`/`fx`/`fy`/`cx`/`cy`/`baseline` arguments are shown for
clarity; the launch file reads the same values from `config_path` when they are
omitted, and an explicit argument still wins.

`vision:=false` skips the live detector node, and `stereo_proc:=false` skips
disparity/point-cloud generation. Capture only needs the rectified colour
topics. Leave this launch running. Capturing while the full stack is running is
also fine; `vision_capture` never publishes `/vision/ask`.

### Step 3 - Confirm both rectified topics are publishing (second shell)

```bash
ros2 topic hz /stereo/left/image_rect_color
ros2 topic hz /stereo/right/image_rect_color
```

Each should report roughly `2.0` Hz. Press Ctrl+C after each check.
If either topic is silent, fix the camera URL or rectification before
capturing; `vision_capture` will otherwise time out.

### Step 4 - Position the toy and measure ground truth

For each scene, which becomes one directory containing several shots:

1. Put the toy where you want it and leave it still.
2. Measure `distance_m` along the optical axis (the normal to the optical
   plane) from the camera baseline to the toy's depth plane. This is
   operator-entered ground truth, never an estimate from the pipeline under
   test.
3. Measure `center_offset_m`: the signed lateral distance from the optical-axis
   center in that plane. Positive means the toy is to the robot's left (ROS
   `+y`); negative means the opposite side; zero means on the optical axis.
   `vision_capture` computes `lateral_bearing_deg = atan(center_offset_m /
   distance_m)` using the convention above.
4. Confirm the toy name; the tool will suggest a `capture_id` from the name and
   these measurements, which you can accept or replace.

Keep both measurements handy: Step 5 asks for them once, then uses the same
ground truth for every shot of that scene and offers the generated ID.

### Step 5 - Save several shots of one scene interactively

```bash
ros2 run dog_toy_finder_vision vision_capture \
    --target "red ball" \
    --shots "3" \
    --run-id "capture_run1" \
    --output-dir "/workspace/data/vision/captures" \
    --config "/workspace/software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml" \
    --timeout "20.0"
```

Leave out `--depth-m` and `--center-offset-m`. In this interactive (TTY) shell
the tool prompts for the values you chose in Step 4, then offers an ID generated
from the toy name and those measurements:

```text
Enter ground truth measured for this scene:
center_offset_m along optical plane (+left): 0.244
distance_m along optical axis (normal): 1.15
Enter capture details for this capture:
capture_id [red_ball_d1p15_op0p244]:
```

Type each measurement and press Enter. At the `capture_id` prompt, press Enter
again to accept the generated ID or type a custom one. In the generated ID, `d`
marks distance, `o` marks the signed lateral offset, `p` replaces the decimal
point, and `m` marks a negative offset. The tool computes and
stores `lateral_bearing_deg` (about +12° for these numbers). `--obstacle` and
`--notes` stay on the command line; they are not prompted for. For a script or
other non-interactive shell, `--capture-id` is not accepted; the generated ID is
used automatically, and `--depth-m` plus `--center-offset-m` are required for a
visible target (unless using `--not-visible`).

One invocation saves `--shots "3"` distinct fresh matched stereo pairs under
one scene, so the same measured ground truth is used for all three shots. Each
shot must contain camera frames newer than the previous shot; the tool waits up
to `--timeout` seconds for each pair. Use a different generated or custom
`capture_id` for every scene; reusing an ID overwrites that scene directory.
The command prints JSON with `capture_dir`, `shots`, and one `captures` entry
per saved shot when it succeeds. Use `--shots "1"` for the older flat layout if
a script needs exactly one pair.

If it times out, return to Step 3 and check the two rectified topics.

### Step 6 - Verify the saved scene

```bash
ls -l /workspace/data/vision/captures/capture_run1/red_ball_d1p15_op0p244
cat /workspace/data/vision/captures/capture_run1/red_ball_d1p15_op0p244/scene.json
cat /workspace/data/vision/captures/capture_run1/red_ball_d1p15_op0p244/shot_01/meta.json
```

You should see `scene.json` plus `shot_01`, `shot_02`, and `shot_03`; each shot
directory has `left.jpg`, `right.jpg`, and `meta.json`. The metadata records the
target, camera intrinsics, image topics/timestamps, and the operator's ground
truth:

```json
{
  "target": "red ball",
  "shot_count": 3,
  "shot_index": 1,
  "ground_truth": {
    "target_visible": true,
    "center_offset_m": 0.244,
    "lateral_bearing_deg": 12.0,
    "distance_m": 1.15,
    "obstacle_ahead": "none",
    "notes": ""
  }
}
```

The center offset, bearing, and distance in these files are the values
`vision_eval` will compare with every pipeline answer for that shot.

### Step 7 - Capture the other scene cases

Move the toy or change the scene, then run Step 5 again, enter the new
measurements, and accept the newly suggested `capture_id` (or type a custom
one). Keep the same `--run-id` for the whole dataset. Common variants:

```bash
# Target absent: stores target_visible=false, distance=0, bearing=0; ID suggests red_ball_absent
ros2 run dog_toy_finder_vision vision_capture \
    --target "red ball" \
    --shots "3" \
    --not-visible \
    --run-id "capture_run1" \
    --output-dir "/workspace/data/vision/captures" \
    --config "/workspace/software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml" \
    --timeout "20.0"

# Obstacle plus notes: distance and center offset are still entered at the prompts
ros2 run dog_toy_finder_vision vision_capture \
    --target "red ball" \
    --shots "3" \
    --obstacle chair \
    --notes "chair between cameras and toy" \
    --run-id "capture_run1" \
    --output-dir "/workspace/data/vision/captures" \
    --config "/workspace/software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml" \
    --timeout "20.0"

# Non-interactive shells/scripts: supply measurements as flags; ID is generated
ros2 run dog_toy_finder_vision vision_capture \
    --target "red ball" \
    --shots "3" \
    --depth-m 0.90 --center-offset-m -0.1265 \
    --run-id "capture_run1" \
    --output-dir "/workspace/data/vision/captures" \
    --config "/workspace/software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml" \
    --timeout "20.0"
```

`--depth-m` and either `--center-offset-m` or `--bearing-deg` are required in a
non-interactive shell unless `--not-visible` is set; the `capture_id` is generated
from the toy name and ground-truth distances there as well. `--bearing-deg`
remains available for older scripts;
do not pass both it and `--center-offset-m`. Use `--notes` for anything the
numeric fields do not capture.

### Step 8 - Aim for a varied set

A useful comparison covers numeric variety **and** per-eye visibility. The
distance and center-offset values are typed at the Step 5 prompts (bearing is
computed); other flags stay on the command line. The tool always suggests a
base ID from the toy name and measurements; for the special cases below, type
the shown suffixed ID at the prompt. Use `--notes` for per-eye visibility,
because the numeric fields describe one visible toy centre, and `--obstacle`
to label an occluder.

| Use case | Example scene | Capture ID to use | Ground truth / other options |
|---|---|---|---|
| Normal object in both images | On axis, 1 m | `red_ball_d1_op0` | distance 1.00 m, offset 0 m; visible in both eyes |
| Normal object in both images | Positive bearing | `red_ball_d1p15_op0p244` | distance 1.15 m, offset +0.244 m (+12°); visible in both eyes |
| Normal object in both images | Negative bearing | `red_ball_d0p9_om0p126` | distance 0.90 m, offset -0.126 m (-8°); visible in both eyes |
| Normal object in both images | Near / far | `red_ball_d0p4_op0p35`, `red_ball_d2_om0p105` | distance 0.40 m / 2.00 m; measure each visible centre |
| Object in one image only | Toy outside the right camera's field of view | `red_ball_d1p15_op0p244_left_only` | enter measurements for the visible left-eye centre; `--notes "visible left image only"` |
| Object in one image only | Toy outside the left camera's field of view | `red_ball_d1p15_op0p244_right_only` | enter measurements for the visible right-eye centre; `--notes "visible right image only"` |
| Object in neither image | Target absent | `red_ball_absent` | `--not-visible` (no measurement prompts) |
| Partially visible in one image | Toy cut by the left frame edge | `red_ball_d1p15_op0p244_partial_left` | measure the visible part's centre; `--notes "partial in left image only"` (use `_partial_right` when reversed) |
| Partially visible in both images, occluded | Chair blocks part of the toy in both eyes | `red_ball_d0p9_om0p126_partial_both_occluded` | distance 0.90 m, offset -0.126 m; `--obstacle chair --notes "partial in both images, occluded by chair"` |
| Different target | Dog bone | `dog_bone_d1p1_op0p194` | distance 1.10 m, offset +0.194 m (+10°), `--target "dog bone"` |

### Step 9 - Finish capture

When all scenes are saved, press Ctrl+C in the first shell to stop the stereo
launch. The captures can now be replayed offline; no camera topics, Pico, or
ROS graph are needed for `vision_eval`.

## Replay every shot offline

Run **one pipeline per invocation**. `--pipeline` (or `VISION_PIPELINE`, then
the YAML `pipeline` key when the flag is omitted) selects exactly one pipeline
for the whole run; the other is not touched. Keep the same host `DETECTOR_MODEL`
for both pipelines: `open_vocab` uses the detector service only, while `stereo-net`
also uses the depth service (one depth model at a time). To compare both, replay
the same shots twice with different output paths so the first run is not
overwritten. Every shot of every scene is replayed; scenes are shuffled first,
then the shots inside each scene are shuffled, so directory names do not
determine the processing order. `--limit` applies after this hierarchical
shuffle.

```bash
# Optional: add --seed N to both commands; reuse a recorded seed for identical order.
ros2 run dog_toy_finder_vision vision_eval \
    --captures-dir "/workspace/data/vision/captures" \
    --pipeline open_vocab \
    --output-csv /workspace/data/vision/open_vocab_trials.csv \
    --results-json /workspace/data/vision/open_vocab_results.jsonl \
    --summary-json /workspace/data/vision/open_vocab_summary.json \
    --tolerance "0.25" \
    --bearing-tolerance "5.0"

# start the host depth service, then replay the same captures with `stereo-net`
ros2 run dog_toy_finder_vision vision_eval \
    --captures-dir "/workspace/data/vision/captures" \
    --pipeline stereo-net \
    --output-csv /workspace/data/vision/stereo-net_trials.csv \
    --results-json /workspace/data/vision/stereo-net_results.jsonl \
    --summary-json /workspace/data/vision/stereo-net_summary.json \
    --tolerance "0.25" \
    --bearing-tolerance "5.0"
```

Without the output overrides, each run writes to the same default paths:

```text
/workspace/data/vision/vision_eval_trials.csv
/workspace/data/vision/vision_eval_results.jsonl
/workspace/data/vision/vision_eval_summary.json
```

One scored row is written per model per shot (one answer per shot), so a scene
captured with `--shots "3"` contributes three rows. Its `ask_id` is
`<capture_id>_shot_01` through `_shot_03`, keeping shots of the same scene
distinct when models are paired. For `open_vocab` the `model` column is reported
by the host detector service (falling back to `open_vocab.name`), answers carry
`method=open_vocab`, and matched detections report
`distance_source=stereo_triangulation`. For `stereo-net` the `model` column is
the dense depth model name from the host depth service, answers carry
`method=stereo-net` and `detector_model` for the one-eye detector, and use
`distance_source=dense_stereo_depth` plus `depth_sample_*` fields. Legacy rows
from the retired path use `method=ml`. Ground truth is copied from each shot's
`meta.json`, so no spreadsheet annotation is needed. The summary manifest
(`vision_eval_summary.json`) records the pipeline that produced the rows, the
`shuffle_seed` used for scene/shot order, the resulting `shot_order` (paths
relative to `--captures-dir`), `scene_order`, and `shots_per_scene`. When
`--seed` is omitted, a random seed is generated and printed, then recorded in
that manifest; pass the same seed to another invocation to replay the exact
scene and per-scene shot order. Use the same seed for both pipelines when
comparing them on identical order.

Useful options:

```bash
# one run, one whole scene (all of its shots), or one shot directory only
ros2 run dog_toy_finder_vision vision_eval \
    --capture /workspace/data/vision/captures/capture_run1/red_ball_d1p15_op0p244

# explicit pipeline and output paths; --limit applies after the scene/shot shuffle
ros2 run dog_toy_finder_vision vision_eval --captures-dir "/workspace/data/vision/captures" \
    --pipeline open_vocab --output-csv "/workspace/data/vision/vision_eval_trials.csv" --limit 10

# replay the exact order recorded in a previous summary
ros2 run dog_toy_finder_vision vision_eval --captures-dir "/workspace/data/vision/captures" \
    --pipeline open_vocab --seed 1234 --output-csv "/workspace/data/vision/vision_eval_trials.csv"
```

## Score and analyse the replay

`vision_eval` already writes scored columns and a per-name summary. For a
machine-readable summary or paired statistics (shots, not just scenes, are the
paired observations):

```bash
ros2 run dog_toy_finder_data llm_trials aggregate "/workspace/data/vision/vision_eval_trials.csv" --json

ros2 run dog_toy_finder_data llm_trials analyze "/workspace/data/vision/vision_eval_trials.csv" \
    --tolerance "0.25" \
    --bearing-tolerance "5.0" \
    --confidence "0.95" \
    --output "/workspace/data/vision/vision_eval_analysis.json"
```

Vision rows are paired by `run_id` + `ask_id`; for offline captures, `ask_id` is
one shot's ID (the scene ID plus `_shot_NN` for multi-shot scenes), so each
pipeline's `model` value is compared on the same saved shot. Keep the same
capture directories and `--seed` for both replay commands.

The live `/vision/ask` path remains available for robot control, but it does not
record comparison trials. Capture-first comparisons use the same answer schema
while keeping images and ground truth fixed across pipelines.

## Statistical analysis (after collection)

`analyze` scores the CSV in memory, then reports descriptive statistics for
every model and paired comparisons between models on matched trials. Vision
rows are matched by `run_id` + `ask_id` (the per-shot ID for `vision_eval`), so
all shots of a scene contribute separate paired observations.

```bash
ros2 run dog_toy_finder_data llm_trials analyze "/workspace/data/vision/vision_eval_trials.csv" \
    --tolerance "0.25" \
    --bearing-tolerance "5.0" \
    --confidence "0.95" \
    --output "/workspace/data/vision/vision_eval_analysis.json" \
    --csv-output /workspace/data/vision/vision_eval_analysis.csv
```

The JSON report contains, for each model/metric: `n`, `mean`, `median`,
`std` (sample), `sem`, `ci_low`, `ci_high`, `min`, and `max`. Pairwise entries
report `mean_diff` (`model_a - model_b`), its confidence interval, a two-sided
paired t-test `p_value`, a Holm-adjusted `p_value_holm`, `effect_dz`, and
win/tie/loss counts. The CSV option writes one tidy table containing both
descriptive and comparison rows.

Useful options:

```bash
# 99% confidence intervals, and a different matching field
ros2 run dog_toy_finder_data llm_trials analyze "/workspace/data/vision/vision_eval_trials.csv" \
    --confidence 0.99 --pair-field run_id,scene --json
```

Set `LLM_TRIAL_CONFIDENCE` in `scripts/full_system_eval.conf` for the confidence
level used in evaluation runs. Pairs with fewer than two observations get no
p-value; rows with different `ask_id` values are never compared.

## Collecting with a template instead

If another process produces the answers, create an empty CSV with the
correct columns and ground-truth fields. Use the detector name configured for
the pipeline in the `model` column:

```bash
ros2 run dog_toy_finder_data llm_trials template \
    --kind vision \
    --models "yolov8s-world" \
    --target "red ball" \
    --run-id capture_run1 \
    --output /workspace/data/vision/manual_vision_trials.csv
```

Fill in the answer and ground-truth columns (including
`gt_lateral_bearing_deg` and `gt_center_offset_m`), then run
`llm_trials score` as described above.

## How to read the results

| Column | Meaning |
|---|---|
| `visible_correct` | Detector's visible/not-visible answer matches ground truth. |
| `distance_m` | Optical-axis distance in metres (`0` when not visible). |
| `center_offset_m` | Signed lateral distance from the optical-axis center in metres, positive to the robot's left. |
| `lateral_bearing_deg` | Detector's numeric bearing in degrees, positive to the robot's left. |
| `bearing_error_deg` | Absolute bearing error in degrees against the operator's measurement. |
| `bearing_within_tolerance` | Error is within `--bearing-tolerance` (default `5°`). |
| `distance_error_m` | Absolute error in metres, against the mean when several ground-truth measurements are recorded. |
| `distance_within_tolerance` | Error is within `--tolerance`. |
| `obstacle_correct` | Obstacle description matches. |
| `score` | Mean of the checks that apply to that row. |

The summary manifest also reports `bearing_mae_deg` and
`bearing_within_rate` for each `model` value reported by the selected pipeline
(detector name for `open_vocab`, dense depth model name for `stereo-net`). Rows where
the toy is not visible only score visibility and obstacle checks. Blank
ground-truth fields are ignored, so partial annotation still produces useful
results.

## Experiment discipline

1. Keep the pipeline and its host detector/depth model selection identical
   across a comparison run.
2. Use one `run_id` per experiment and a new run ID when the setup changes.
3. Save several shots of each scene with `vision_capture` and replay those same
   shots with `vision_eval` once per pipeline; keep `DETECTOR_MODEL` fixed, use the
   recorded seed so both pipelines see the same order, measure depth and center
   offset before running `vision_eval`, and never use a detector under test to
   create ground truth.
4. Keep the captures, the raw CSV, the summary manifest, and the `*_scored.csv`;
   the raw files preserve what the robot saw and what each pipeline produced.
5. Run `llm_trials analyze` on the completed CSV and keep the `*_analysis.json`
   report beside the trial data.

For planning experiments, use the
[planning comparison guide](planning_comparison_guide.md).
