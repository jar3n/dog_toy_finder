# LLM comparison guide (vision and planning)

This guide walks through comparing several vision models and several planning
models. Vision comparison is **capture first**: save rectified left/right
images with operator-entered ground truth, then replay every saved capture
offline. Planning comparison still sends each
prompt to every model live with ROSA. Both trial kinds share one CSV schema and
one scoring tool (`dog_toy_finder_data.llm_trials`).

Vision answers use a numeric `lateral_bearing_deg` as the primary lateral
measurement: degrees from the optical axis, positive to the robot's left. The
legacy `left`/`center`/`right` label is derived from that number and kept only
for older consumers. The same convention is used by live control, mapping, and
the offline evaluator.

| Tool | Purpose |
|---|---|
| `vision_capture` | Saves one fresh rectified stereo pair with operator-entered depth and bearing. |
| `vision_eval` | Replays every saved capture through the configured Ollama models, then writes a scored CSV. |
| `vision_trial_collector` | Optional live-path collector: subscribes to `/vision/ask` and `/vision/result`; one CSV row per model per request. |
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
    vision_eval_trials.csv
    vision_eval_results.jsonl
    vision_eval_summary.json
    vision_eval_analysis.json
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

On the computer running Ollama, pull every model you intend to compare:

```bash
ollama pull gemma4
ollama pull minicpm-v4.5
ollama pull llama3.2
ollama pull mistral
ollama pull qwen2.5
ollama serve
```

Confirm the data package is available:

```bash
ros2 run dog_toy_finder_data llm_trials --help
```

## Part A - Vision model comparison

Vision comparison does not require the Pico or the motion stack. The only live
component is the rectified stereo pipeline: `vision_capture` waits for a fresh
left/right pair and saves it with the operator's measurements. No detector or
Ollama model runs at capture time. Later, `vision_eval` replays every saved pair
offline with no camera topics and no ROS graph.

### A1. Set the physical measurements and bearing convention

Edit
`/workspace/software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml`
(or point `VISION_DETECTOR_CONFIG` in `scripts/full_system_eval.conf` at your
own copy). At minimum, set the calibrated `fx`, `fy`, `cx`, `cy`, `baseline_m`,
and the measured `target.known_width_m`. These values are included in the model
prompt so the VLM can judge depth and bearing from the rectified pair.

`lateral_bearing_deg` is measured from the camera's optical axis in degrees:

```text
bearing = atan((cx - u) / fx)
```

where `u` is the target's horizontal pixel centre. Positive means the toy is to
the robot's **left** (image column less than `cx`, ROS `+y`), negative means
right, and zero means centred. `target.position_bearing_deg` (default `15°`) is
only a fallback for older results that contain a legacy `left`/`center`/`right`
label but no numeric bearing.

### A2. Start the rectified stereo pipeline

```bash
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    left_url:=http://esp32cam-left.local/capture \
    right_url:=http://esp32cam-right.local/capture \
    config_path:=/workspace/software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml \
    vision:=false
```

`vision:=false` leaves `/stereo/{left,right}/image_rect_color` publishing but
does not start the live detector node, so no VLM is loaded during capture. It is
also fine to capture while the full stack is running; `vision_capture` never
publishes a `/vision/ask`.

### A3. Capture a labelled stereo pair

```bash
cd /workspace
source scripts/full_system_eval.conf

ros2 run dog_toy_finder_vision vision_capture \
    --target "$VISION_CAPTURE_TARGET" \
    --depth-m 1.15 \
    --bearing-deg +12 \
    --size medium \
    --run-id "$VISION_CAPTURE_RUN_ID" \
    --capture-id ball_115m_left12 \
    --output-dir "$VISION_CAPTURE_DIR" \
    --timeout "$VISION_CAPTURE_TIMEOUT"
```

Measure `depth-m` with a tape measure from the cameras and `bearing-deg` from
the optical axis using the sign convention above. In an interactive terminal,
`--depth-m` and `--bearing-deg` may be omitted and are prompted for; without a
TTY they are required. For a frame where the target is absent, use `--not-visible`
instead (depth and bearing are stored as `0`).

Each invocation writes:

```text
$VISION_CAPTURE_DIR/<run_id>/<capture_id>/left.jpg
$VISION_CAPTURE_DIR/<run_id>/<capture_id>/right.jpg
$VISION_CAPTURE_DIR/<run_id>/<capture_id>/meta.json
```

`meta.json` records the target, the camera intrinsics, the image topics and
timestamps, and the operator's ground truth:

```json
{
  "target": "red ball",
  "ground_truth": {
    "target_visible": true,
    "position": "left",
    "lateral_bearing_deg": 12.0,
    "distance_m": 1.15,
    "size": "medium",
    "obstacle_ahead": "none",
    "notes": ""
  }
}
```

`position` is derived from `lateral_bearing_deg` for compatibility; the bearing
is the value operators enter and evaluators score.

### A4. Repeat across scene cases

Move the toy or change the scene and capture again with a new `--capture-id`.
Keep one `--run-id` for the experiment. A useful comparison covers:

- toy visible / not visible (`--not-visible`)
- left / centre / right (negative, zero, positive bearing)
- near / medium / far depth
- clear path / obstacle present (`--obstacle`)
- more than one toy kind (`--target`)

Use `--notes` for anything the numeric fields do not capture.

### A5. Replay every capture offline

```bash
ros2 run dog_toy_finder_vision vision_eval \
    --captures-dir "$VISION_CAPTURE_DIR" \
    --models "$VISION_EVAL_MODELS" \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE"
```

This writes:

```text
/workspace/data/vision/vision_eval_trials.csv
/workspace/data/vision/vision_eval_results.jsonl
/workspace/data/vision/vision_eval_summary.json
```

One row is written per capture and model, using the Ollama model name and
`method=ml`. Ground truth is copied from each `meta.json`, so no spreadsheet
annotation is needed. When more than one model is listed, each model is unloaded
after its answer (`--compare-models` forces this even for a single model), so
every model runs without another model holding VRAM.

Useful options:

```bash
# one run or one capture only
ros2 run dog_toy_finder_vision vision_eval \
    --capture /workspace/data/vision/captures/capture_run1/ball_115m_left12

# a subset of models with explicit output paths
ros2 run dog_toy_finder_vision vision_eval --captures-dir "$VISION_CAPTURE_DIR" \
    --models gemma4,minicpm-v4.5 \
    --output-csv "$VISION_EVAL_CSV" --limit 10
```

### A6. Score and analyse the replay

`vision_eval` already writes scored columns and a per-model summary. For a
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
the capture ID, so each model's answer on the same saved pair is compared.

### A7. Optional: live `/vision/ask` trials

The live path remains available for robot control and for collecting trials
without saving captures. It uses the same bearing field and answer schema:

```bash
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    models:="$VISION_MODELS"
ros2 launch dog_toy_finder_data collect_llm_trials.launch.py \
    output_dir:="$VISION_TRIAL_DIR" run_id:="$VISION_TRIAL_RUN_ID" \
    distance_tolerance:="$LLM_TRIAL_DISTANCE_TOLERANCE" \
    bearing_tolerance:="$LLM_TRIAL_BEARING_TOLERANCE"

cat > gt_vision.json <<'EOF'
{"target_visible": true, "position": "left", "lateral_bearing_deg": 12.0,
 "size": "medium", "distance_m": [1.10, 1.15, 1.12],
 "obstacle_ahead": "none"}
EOF

python3 /workspace/scripts/vision_test.py "red ball" --gt-json gt_vision.json
```

Each ask appends one row per model to
`$VISION_TRIAL_DIR/$VISION_TRIAL_RUN_ID/vision_trials.csv`. `distance_m` may be
a single number or repeated measurements; scoring uses the mean. If ground
truth is not supplied, fill these columns later:

```text
gt_target_visible, gt_position, gt_lateral_bearing_deg, gt_size,
gt_distance_m, gt_obstacle_ahead, gt_notes
```

Capture-first is preferred for fair model comparisons: the same saved image is
replayed for every model, and capture and evaluation are fully decoupled.

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
ros2 launch dog_toy_finder_vision stereo_vision.launch.py models:="gemma4"
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

If another process produces the model answers, create an empty CSV with the
correct columns and ground-truth fields:

```bash
# Vision
ros2 run dog_toy_finder_data llm_trials template \
    --kind vision \
    --models "gemma4,minicpm-v4.5" \
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
| `visible_correct` | Model's visible/not-visible answer matches ground truth. |
| `position_correct` | Derived left/centre/right label matches (compatibility view of the bearing). |
| `lateral_bearing_deg` | Model's numeric bearing in degrees, positive to the robot's left. |
| `bearing_error_deg` | Absolute bearing error in degrees against the operator's measurement. |
| `bearing_within_tolerance` | Error is within `--bearing-tolerance` (default `5°`). |
| `size_correct` | size matches. |
| `distance_error_m` | Absolute error in metres, against the mean when several ground-truth measurements are recorded. |
| `distance_within_tolerance` | Error is within `--tolerance`. |
| `obstacle_correct` | Obstacle description matches. |
| `score` | Mean of the checks that apply to that row. |

The per-model summary also reports `bearing_mae_deg` and
`bearing_within_rate`. Rows where the toy is not visible only score visibility
and obstacle checks. Blank ground-truth fields are ignored, so partial
annotation still produces useful results.

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

1. Keep the model list identical across a comparison run.
2. Use one `run_id` per experiment and a new run ID when the setup changes.
3. For vision, save each scene once with `vision_capture` and replay that same
   capture for every model; measure depth and bearing before running
   `vision_eval`, and do not use a detector under test to create ground truth.
4. For planning, use the same prompt, same starting pose, and same vision state
   for every model.
5. Record the operator verdict in `gt_success` whenever a move actually
   executes.
6. Keep the captures, the raw CSV, and the `*_scored.csv`; the raw files
   preserve what the robot saw and what each model actually said.
7. Run `llm_trials analyze` on the completed CSV and keep the `*_analysis.json`
   report beside the trial data.
