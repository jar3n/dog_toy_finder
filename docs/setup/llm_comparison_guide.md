# LLM comparison guide (vision and planning)

This guide walks through comparing vision-pipeline results and several planning
models. Vision comparison is **capture first**: save rectified left/right
images with operator-entered ground truth, then replay every saved capture
offline through the configured pipeline. Planning comparison still sends each
prompt to every model live with ROSA. Both trial kinds share one CSV schema and
one scoring tool (`dog_toy_finder_data.llm_trials`).

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

Each pipeline produces one answer per request/capture, so rows stay comparable
across runs.

Vision answers use a numeric `lateral_bearing_deg` as the sole lateral
measurement: degrees from the optical axis, positive to the robot's left (ROS
`+y`). No coarse side label is emitted or accepted. The same convention is used
by live control, mapping, and the offline evaluator.

| Tool | Purpose |
|---|---|
| `vision_capture` | Saves one fresh rectified stereo pair with operator-entered depth and bearing. |
| `vision_eval` | Replays every saved capture through the configured vision pipeline, then writes a scored CSV. |
| `vision_trial_collector` | Optional live-path collector: subscribes to `/vision/ask` and `/vision/result`; one CSV row per answer entry per request. |
| ROSA planning logging | `rosa_agent --compare --output-csv` writes one row per prompt and model. |
| `llm_trials` CLI | `template`, `score`, `aggregate`, and `analyze` for either CSV kind. |

The capture/eval commands write the automated columns and copy the ground truth
from each capture's `meta.json` into the `gt_*` columns. `score` fills the metric
columns when a CSV is annotated later.

## Where trial data lives

Generated data lives under the repository's `data/` folder, next to `hardware/`
and `software/`:

```text
data/
  vision/
    captures/<run_id>/<capture_id>/left.jpg
    captures/<run_id>/<capture_id>/right.jpg
    captures/<run_id>/<capture_id>/meta.json
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
    <live_run_id>/vision_trials.csv            # optional live-path collector
  planning/<run_id>/planning_trials.csv
```

Inside the ROS 2 container the repository is mounted at `/workspace`, so the
commands below use `/workspace/data/...`. On the host the same files appear
under `<repo>/data/...`. The `vision/` and `planning/` folders are tracked, but
their generated contents are ignored by git.

## 0. One-time setup

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

No vision Ollama server is needed. Ollama is only used by the ROSA planning
comparison in Part B; see
[`software/rosa_agent/README.md`](../../software/rosa_agent/README.md) for its
setup.

Confirm the data package is available:

```bash
ros2 run dog_toy_finder_data llm_trials --help
```

## Part A - Vision pipeline comparison

Vision comparison does not require the Pico or the motion stack. The only live
component is the rectified stereo pipeline: `vision_capture` waits for a fresh
left/right pair and saves it with the operator's measurements. No detector runs
at capture time. Later, `vision_eval` replays every saved pair offline with no
camera topics and no ROS graph.

### A1. Set the physical measurements and bearing convention

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
`cx`/`fx` set the bearing.

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
than `cx`, ROS `+y`), negative means right, and zero means centred. Numeric
`lateral_bearing_deg` is the only lateral measurement; no coarse side label is
emitted or accepted.

### A2. Capture a vision dataset (step by step)

Use two shells inside the ROS 2 container: one keeps the stereo pipeline
running, the other runs `vision_capture`. No host detector or depth service is
needed for capture; those are started later for `vision_eval`.

#### Step 1 - Load the same settings in both shells

```bash
cd /workspace
source /opt/ros/humble/setup.bash
source /workspace/software/ros2_ws/install/setup.bash
source /workspace/scripts/full_system_eval.conf
```

The config uses `:-` defaults, so values already exported win. For a different
run label or target, export them before sourcing the config:

```bash
export VISION_CAPTURE_RUN_ID=ball_set_01
export VISION_CAPTURE_TARGET="red ball"
source /workspace/scripts/full_system_eval.conf
```

Check where captures will go and which YAML supplies the intrinsics:

```bash
printf 'captures=%s\nconfig=%s\nrun=%s\ntarget=%s\n' \
    "$VISION_CAPTURE_DIR" "$VISION_DETECTOR_CONFIG" \
    "$VISION_CAPTURE_RUN_ID" "$VISION_CAPTURE_TARGET"
```

`VISION_DETECTOR_CONFIG` must be the same file whose `fx`, `cx`, and baseline
you checked in A1. `vision_capture` copies those values into each `meta.json`.
Keep `CAMERA_FX`, `CAMERA_FY`, `CAMERA_CX`, `CAMERA_CY`, and `STEREO_BASELINE`
in `full_system_eval.conf` equal to `camera.fx`, `camera.fy`, `camera.cx`,
`camera.cy`, and `camera.baseline_m` in that YAML. The launch uses the config
values; the capture metadata uses the YAML values.

#### Step 2 - Start a capture-only stereo pipeline (first shell)

```bash
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    left_url:="$LEFT_URL" \
    right_url:="$RIGHT_URL" \
    rate:="$CAMERA_RATE" \
    width:="$IMAGE_WIDTH" \
    height:="$IMAGE_HEIGHT" \
    fx:="$CAMERA_FX" \
    fy:="$CAMERA_FY" \
    cx:="$CAMERA_CX" \
    cy:="$CAMERA_CY" \
    baseline:="$STEREO_BASELINE" \
    config_path:="$VISION_DETECTOR_CONFIG" \
    stereo_proc:=false \
    vision:=false
```

`vision:=false` skips the live detector node, and `stereo_proc:=false` skips
disparity/point-cloud generation. Capture only needs the rectified colour
topics. Leave this launch running. Capturing while the full stack is running is
also fine; `vision_capture` never publishes `/vision/ask`.

#### Step 3 - Confirm both rectified topics are publishing (second shell)

```bash
ros2 topic hz /stereo/left/image_rect_color
ros2 topic hz /stereo/right/image_rect_color
```

Each should report roughly `$CAMERA_RATE` Hz. Press Ctrl+C after each check.
If either topic is silent, fix the camera URL or rectification before
capturing; `vision_capture` will otherwise time out.

#### Step 4 - Position the toy and measure ground truth

For each scene:

1. Put the toy where you want it and leave it still.
2. Measure `distance_m` with a tape measure from the camera baseline to the
   toy. This is operator-entered ground truth, never an estimate from the
   pipeline under test.
3. Measure `lateral_bearing_deg` from the optical axis using the convention in
   A1: `bearing = atan((cx - u) / fx)`. Positive means the toy is to the
   robot's left (image column less than `cx`, ROS `+y`); negative means the
   opposite side; zero means on the optical axis.
4. Write down a short, unique `--capture-id` for this scene.

#### Step 5 - Save one capture

```bash
ros2 run dog_toy_finder_vision vision_capture \
    --target "$VISION_CAPTURE_TARGET" \
    --depth-m 1.15 \
    --bearing-deg +12.0 \
    --size medium \
    --run-id "$VISION_CAPTURE_RUN_ID" \
    --capture-id ball_115m_b12 \
    --output-dir "$VISION_CAPTURE_DIR" \
    --config "$VISION_DETECTOR_CONFIG" \
    --timeout "$VISION_CAPTURE_TIMEOUT"
```

One invocation saves exactly one fresh matched stereo pair. Use a new
`--capture-id` for every scene; reusing an ID overwrites that capture
directory. If the ID is omitted, a timestamp is used. The command prints JSON
with `capture_dir` and the full `meta` when it succeeds.

If it times out, return to Step 3 and check the two rectified topics.

#### Step 6 - Verify the saved capture

```bash
CAP="$VISION_CAPTURE_DIR/$VISION_CAPTURE_RUN_ID/ball_115m_b12"
ls -l "$CAP"
cat "$CAP/meta.json"
```

You should see `left.jpg`, `right.jpg`, and `meta.json`. The metadata records
the target, camera intrinsics, image topics/timestamps, and the operator's
ground truth:

```json
{
  "target": "red ball",
  "ground_truth": {
    "target_visible": true,
    "lateral_bearing_deg": 12.0,
    "distance_m": 1.15,
    "size": "medium",
    "obstacle_ahead": "none",
    "notes": ""
  }
}
```

The bearing and distance in this file are the values `vision_eval` will score.

#### Step 7 - Capture the other scene cases

Move the toy or change the scene, then run Step 5 again with a new
`--capture-id`. Keep the same `--run-id` for the whole dataset. Common
variants:

```bash
# Interactively prompted for distance and bearing (requires a TTY)
ros2 run dog_toy_finder_vision vision_capture \
    --target "$VISION_CAPTURE_TARGET" \
    --run-id "$VISION_CAPTURE_RUN_ID" \
    --capture-id ball_prompted \
    --output-dir "$VISION_CAPTURE_DIR" \
    --config "$VISION_DETECTOR_CONFIG" \
    --timeout "$VISION_CAPTURE_TIMEOUT"

# Target absent: stores target_visible=false, distance=0, bearing=0
ros2 run dog_toy_finder_vision vision_capture \
    --target "$VISION_CAPTURE_TARGET" \
    --not-visible \
    --run-id "$VISION_CAPTURE_RUN_ID" \
    --capture-id ball_absent \
    --output-dir "$VISION_CAPTURE_DIR" \
    --config "$VISION_DETECTOR_CONFIG" \
    --timeout "$VISION_CAPTURE_TIMEOUT"

# Obstacle in the path plus free-form notes
ros2 run dog_toy_finder_vision vision_capture \
    --target "$VISION_CAPTURE_TARGET" \
    --depth-m 0.90 \
    --bearing-deg -8.0 \
    --obstacle chair \
    --notes "chair between cameras and toy" \
    --run-id "$VISION_CAPTURE_RUN_ID" \
    --capture-id ball_090m_bneg8_obstacle \
    --output-dir "$VISION_CAPTURE_DIR" \
    --config "$VISION_DETECTOR_CONFIG" \
    --timeout "$VISION_CAPTURE_TIMEOUT"
```

In non-interactive use, `--depth-m` and `--bearing-deg` are required unless
`--not-visible` is set. Use `--notes` for anything the numeric fields do not
capture.

#### Step 8 - Aim for a varied set

A useful comparison covers all of these cases:

| Scene | Example capture ID | Key options |
|---|---|---|
| On axis, medium range | `ball_100m_b0` | `--depth-m 1.00 --bearing-deg 0` |
| Positive bearing | `ball_115m_b12` | `--depth-m 1.15 --bearing-deg +12.0` |
| Negative bearing | `ball_090m_bneg8` | `--depth-m 0.90 --bearing-deg -8.0` |
| Near | `ball_040m_b5` | `--depth-m 0.40 --bearing-deg +5.0` |
| Far | `ball_200m_bneg3` | `--depth-m 2.00 --bearing-deg -3.0` |
| Target absent | `ball_absent` | `--not-visible` |
| Obstacle present | `ball_090m_bneg8_obstacle` | `--obstacle chair --notes "..."` |
| Different target | `bone_110m_b10` | `--target "dog bone"` |

#### Step 9 - Finish capture

When all scenes are saved, press Ctrl+C in the first shell to stop the stereo
launch. The captures can now be replayed offline; no camera topics, Pico, or
ROS graph are needed for `vision_eval`.

### A5. Replay every capture offline

Run **one pipeline per invocation**. `--pipeline` (or `$VISION_PIPELINE`, then
the YAML `pipeline` key when the flag is omitted) selects exactly one pipeline
for the whole run; the other is not touched. Keep the same host `DETECTOR_MODEL`
for both pipelines: `open_vocab` uses the detector service only, while `stereo-net`
also uses the depth service (one depth model at a time). To compare both, replay
the same captures twice with different output paths so the first run is not
overwritten. Captures are shuffled before replay so directory names do not
determine the processing order; `--limit` is applied after the shuffle.

```bash
# Optional: add --seed N to both commands; reuse a recorded seed for identical order.
ros2 run dog_toy_finder_vision vision_eval \
    --captures-dir "$VISION_CAPTURE_DIR" \
    --pipeline open_vocab \
    --output-csv /workspace/data/vision/open_vocab_trials.csv \
    --results-json /workspace/data/vision/open_vocab_results.jsonl \
    --summary-json /workspace/data/vision/open_vocab_summary.json \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE"

# start the host depth service, then replay the same captures with `stereo-net`
ros2 run dog_toy_finder_vision vision_eval \
    --captures-dir "$VISION_CAPTURE_DIR" \
    --pipeline stereo-net \
    --output-csv /workspace/data/vision/stereo-net_trials.csv \
    --results-json /workspace/data/vision/stereo-net_results.jsonl \
    --summary-json /workspace/data/vision/stereo-net_summary.json \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE"
```

Without the output overrides, each run writes to the same default paths:

```text
/workspace/data/vision/vision_eval_trials.csv
/workspace/data/vision/vision_eval_results.jsonl
/workspace/data/vision/vision_eval_summary.json
```

One scored row is written per capture (one answer per capture). For
`open_vocab` the `model` column is reported by the host detector service
(falling back to `open_vocab.name`), answers carry `method=open_vocab`, and
matched detections report `distance_source=stereo_triangulation`. For `stereo-net`
the `model` column is the dense depth model name from the host depth service,
answers carry `method=stereo-net` and `detector_model` for the one-eye detector, and
use `distance_source=dense_stereo_depth` plus `depth_sample_*` fields. Legacy
rows from the retired path use `method=ml`. Ground truth is copied from each
`meta.json`, so no spreadsheet annotation is needed. The summary manifest
(`vision_eval_summary.json`) records the pipeline that produced the rows, the
`shuffle_seed` used for capture order, and the resulting `capture_order` (paths
relative to `--captures-dir`). When `--seed` is omitted, a random seed is
generated and printed, then recorded in that manifest; pass the same seed to
another invocation to replay the exact order. Use the same seed for both
pipelines when comparing them on identical capture order.

Useful options:

```bash
# one run or one capture only
ros2 run dog_toy_finder_vision vision_eval \
    --capture /workspace/data/vision/captures/capture_run1/ball_115m_b12

# explicit pipeline and output paths; --limit applies after the shuffle
ros2 run dog_toy_finder_vision vision_eval --captures-dir "$VISION_CAPTURE_DIR" \
    --pipeline open_vocab --output-csv "$VISION_EVAL_CSV" --limit 10

# replay the exact order recorded in a previous summary
ros2 run dog_toy_finder_vision vision_eval --captures-dir "$VISION_CAPTURE_DIR" \
    --pipeline open_vocab --seed 1234 --output-csv "$VISION_EVAL_CSV"
```

### A6. Score and analyse the replay

`vision_eval` already writes scored columns and a per-name summary. For a
machine-readable summary or paired statistics:

```bash
ros2 run dog_toy_finder_data llm_trials aggregate "$VISION_EVAL_CSV" --json

ros2 run dog_toy_finder_data llm_trials analyze "$VISION_EVAL_CSV" \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE" \
    --confidence "$LLM_TRIAL_CONFIDENCE" \
    --output "$VISION_EVAL_ANALYSIS_JSON"
```

Vision rows are paired by `run_id` + `ask_id`; for offline captures, `ask_id` is
the capture ID, so each pipeline's `model` value is compared on the same saved
pair when both replay commands use the same seed.

### A7. Optional: live `/vision/ask` trials

The live path remains available for robot control and for collecting trials
without saving captures. It uses the same bearing field and answer schema:

```bash
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    pipeline:="$VISION_PIPELINE"
ros2 launch dog_toy_finder_data collect_llm_trials.launch.py \
    output_dir:="$VISION_TRIAL_DIR" run_id:="$VISION_TRIAL_RUN_ID" \
    distance_tolerance:="$LLM_TRIAL_DISTANCE_TOLERANCE" \
    bearing_tolerance:="$LLM_TRIAL_BEARING_TOLERANCE"

cat > gt_vision.json <<'EOF'
{"target_visible": true, "lateral_bearing_deg": 12.0,
 "size": "medium", "distance_m": [1.10, 1.15, 1.12],
 "obstacle_ahead": "none"}
EOF

python3 /workspace/scripts/vision_test.py "red ball" --gt-json gt_vision.json
```

Each ask appends one row per answer entry from the selected pipeline to
`$VISION_TRIAL_DIR/$VISION_TRIAL_RUN_ID/vision_trials.csv`; the `model` column is
the detector name for `open_vocab` or the dense depth model name for `stereo-net`.
`distance_m` may be a single number or repeated measurements; scoring uses the
mean. If ground truth is not supplied, fill these columns later:

```text
gt_target_visible, gt_lateral_bearing_deg, gt_size,
gt_distance_m, gt_obstacle_ahead, gt_notes
```

Capture-first is preferred for fair comparisons: the same saved images are
replayed by each selected pipeline with a recorded shuffle seed, and capture
and evaluation are fully decoupled.

## Part B - Planning model comparison

Planning comparison uses ROSA. The same prompt is sent to every model, and one
row per model is appended to the shared planning CSV.

### B1. Start the robot/vision stack the planner will use

ROSA's tools read odometry and call vision/motion. To score whether actions
actually worked, the normal stack should be running:

```bash
cd /workspace/docker
docker compose up -d micro_ros_agent
# then, in the ROS 2 container:
ros2 launch dog_toy_finder_bringup robot.launch.py
ros2 launch dog_toy_finder_vision stereo_vision.launch.py pipeline:="open_vocab"
```

If the Pico is intentionally off, you can still capture replies and tool calls,
but `gt_success` cannot honestly reflect physical execution. In that case score
only the text/tool-call checks (`action_mentioned`, `args_mentioned`).

### B2. Run the same prompt through several models

```bash
cd /workspace/software/rosa_agent

python3 -m rosa_agent \
    --compare "llama3.2,mistral,qwen2.5" \
    --prompt "Drive forward 1 meter." \
    --output-csv /workspace/data/planning/plan1/planning_trials.csv \
    --run-id plan1
```

Each model gets the same prompt. The row records the reply, wall-clock latency,
every tool call, and any error. Models run one at a time and are unloaded before
the next starts, so each planner gets all of the Ollama host's VRAM.

For an interactive session, the same CSV logging applies to every prompt:

```bash
python3 -m rosa_agent \
    --output-csv /workspace/data/planning/plan1/planning_trials.csv \
    --run-id plan1
```

### B3. Supply planning ground truth

Create `gt_planning.json` for a single prompt:

```json
{
  "expected_action": "drive_distance",
  "expected_args": "forward,1",
  "success": true,
  "notes": "robot moved forward about 1 m"
}
```

Then pass it to either compare or interactive mode:

```bash
python3 -m rosa_agent \
    --compare "llama3.2,mistral,qwen2.5" \
    --prompt "Drive forward 1 meter." \
    --output-csv /workspace/data/planning/plan1/planning_trials.csv \
    --run-id plan1 \
    --gt-json gt_planning.json
```

For several prompts or model-specific expectations, use the structured form:

```json
{
  "default": {
    "expected_action": "run_search",
    "expected_args": "red ball,one"
  },
  "models": {
    "mistral": {
      "expected_args": "1"
    }
  },
  "prompts": [
    {
      "prompt": "Turn left 90 degrees.",
      "ground_truth": {
        "expected_action": "turn",
        "expected_args": "left,90"
      }
    }
  ]
}
```

Matching order is `default`, then `models.<model>`, then a matching `prompts`
entry, with later values overriding earlier ones.

If no ground truth is supplied, leave the `gt_*` columns blank and fill them
later:

```text
gt_expected_action, gt_expected_args, gt_success, gt_notes
```

### B4. Add more prompts to the same run

`--output-csv` appends, so repeated compare invocations build one planning CSV:

```bash
python3 -m rosa_agent --compare "llama3.2,mistral,qwen2.5" \
    --prompt "Turn left 90 degrees." \
    --output-csv /workspace/data/planning/plan1/planning_trials.csv \
    --run-id plan1 \
    --gt-json gt_planning.json
```

Use the same model list and `run_id` for every prompt in the experiment.

### B5. Score the planning trials

```bash
ros2 run dog_toy_finder_data llm_trials score \
    /workspace/data/planning/plan1/planning_trials.csv
```

This writes `planning_trials_scored.csv` and prints per model:

- `mean_elapsed_s`
- `errors`
- `gt_rows`
- `action_mention_rate`
- `args_mention_rate`
- `mean_score`

`gt_success` overrides the text-based checks: `true` scores the row `1.0`,
`false` scores it `0.0`. Without `gt_success`, the score is the fraction of
matching `expected_action` / `expected_args` checks.

JSON summary:

```bash
ros2 run dog_toy_finder_data llm_trials aggregate \
    /workspace/data/planning/plan1/planning_trials.csv --json
```

## Part C - Statistical analysis (after collection)

`analyze` scores the CSV in memory, then reports descriptive statistics for
every model and paired comparisons between models on matched trials. Vision
rows are matched by `run_id` + `ask_id` (the capture ID for `vision_eval`);
planning rows are matched by `run_id` + `prompt`.

```bash
# Vision replay (capture-first)
source /workspace/scripts/full_system_eval.conf
ros2 run dog_toy_finder_data llm_trials analyze "$VISION_EVAL_CSV" \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE" \
    --confidence "$LLM_TRIAL_CONFIDENCE" \
    --output "$VISION_EVAL_ANALYSIS_JSON" \
    --csv-output /workspace/data/vision/vision_eval_analysis.csv

# Planning
ros2 run dog_toy_finder_data llm_trials analyze \
    /workspace/data/planning/plan1/planning_trials.csv \
    --output /workspace/data/planning/plan1/planning_analysis.json
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
ros2 run dog_toy_finder_data llm_trials analyze "$VISION_EVAL_CSV" \
    --confidence 0.99 --pair-field run_id,scene --json
```

Set `LLM_TRIAL_CONFIDENCE` in `scripts/full_system_eval.conf` for the confidence
level used in evaluation runs. Pairs with fewer than two observations get no
p-value; rows with different `ask_id`/`prompt` values are never compared.

## Part D - Collecting with a template instead

If another process produces the answers, create an empty CSV with the
correct columns and ground-truth fields. For vision, use the detector name
configured for the pipeline in the `model` column:

```bash
# Vision
ros2 run dog_toy_finder_data llm_trials template \
    --kind vision \
    --models "yolov8s-world" \
    --target "red ball" \
    --run-id vision1 \
    --output /workspace/data/vision/vision1/vision_trials.csv

# Planning
ros2 run dog_toy_finder_data llm_trials template \
    --kind planning \
    --models "llama3.2,mistral,qwen2.5" \
    --prompt "Drive forward 1 meter." \
    --run-id plan1 \
    --output /workspace/data/planning/plan1/planning_trials.csv
```

Fill in the answer and ground-truth columns (including
`gt_lateral_bearing_deg` for vision), then run `llm_trials score` exactly as
above.

## Part E - How to read the results

### Vision metrics

| Column | Meaning |
|---|---|
| `visible_correct` | Detector's visible/not-visible answer matches ground truth. |
| `lateral_bearing_deg` | Detector's numeric bearing in degrees, positive to the robot's left. |
| `bearing_error_deg` | Absolute bearing error in degrees against the operator's measurement. |
| `bearing_within_tolerance` | Error is within `--bearing-tolerance` (default `5°`). |
| `size_correct` | size matches. |
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

### Planning metrics

| Column | Meaning |
|---|---|
| `action_mentioned` | `gt_expected_action` appears in the reply or tool calls. |
| `args_mentioned` | Every comma-separated `gt_expected_args` token appears. |
| `score` | `gt_success` if recorded, otherwise the mean of the mention checks. |

Because `action_mentioned` and `args_mentioned` are text checks, they are cheap
signals, not a full plan-quality judgement. Use `gt_success` for the
authoritative outcome.

## Recommended experiment discipline

1. Keep the pipeline and its host detector/depth model selection identical
   across a comparison run.
2. Use one `run_id` per experiment and a new run ID when the setup changes.
3. For vision, save each scene once with `vision_capture` and replay those same
   captures with `vision_eval` once per pipeline; keep `DETECTOR_MODEL` fixed,
   use the recorded seed so both pipelines see the same order, measure depth and
   bearing before running `vision_eval`, and do not use a detector under test to
   create ground truth.
4. For planning, use the same prompt, same starting pose, and same vision state
   for every model.
5. Record the operator verdict in `gt_success` whenever a move actually
   executes.
6. Keep the captures, the raw CSV, the summary manifest, and the `*_scored.csv`;
   the raw files preserve what the robot saw and what each pipeline produced.
7. Run `llm_trials analyze` on the completed CSV and keep the `*_analysis.json`
   report beside the trial data.
