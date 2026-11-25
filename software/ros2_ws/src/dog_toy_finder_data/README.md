# Data collection

`dog_toy_finder_data` records a structured dataset during a test run for offline
analysis and vision-pipeline benchmarking.

## Record

```bash
ros2 launch dog_toy_finder_data collect.launch.py \
    output_dir:=/tmp/dog_toy_dataset run_id:=run1 image_every:=5
```

Writes `poses.csv`, `vision.jsonl`, `images/`, and `manifest.json` under
`<output_dir>/<run_id>/`.

## Summarize

```bash
ros2 run dog_toy_finder_data dataset_summary /tmp/dog_toy_dataset/run1
```

## LLM comparison trials

Vision and planning comparisons share one CSV schema and one scoring tool, so
vision-pipeline answers and a planner can be judged with the same fields. Vision
rows contain automated answers first (including a `method` column: `open_vocab`
for new vision rows, `ml` only for legacy rows), then `gt_*` ground-truth
fields, then metrics computed by `score`.

Trial data lives in the repository's `data/` folder, next to `hardware/` and
`software/`: `data/vision/captures/`, `data/vision/vision_eval_*` files, and
`data/planning/<run_id>/`. Inside the ROS 2 container the same paths are under
`/workspace/data`.

### Vision trials (capture-first)

`dog_toy_finder_vision` owns the capture and replay commands; this package owns
the shared CSV schema, scoring, and statistics. Save one fresh rectified
left/right pair per scene with operator-entered ground truth, then replay all
captures offline:

```bash
cd /workspace

ros2 run dog_toy_finder_vision vision_capture \
    --target "red ball" \
    --run-id "capture_run1" \
    --output-dir "/workspace/data/vision/captures" --config "/workspace/software/ros2_ws/src/dog_toy_finder_vision/config/vision_detectors.yaml"

ros2 run dog_toy_finder_vision vision_eval \
    --captures-dir "/workspace/data/vision/captures" --pipeline "open_vocab" \
    --tolerance "0.25" \
    --bearing-tolerance "5.0" \
    --output-csv "/workspace/data/vision/vision_eval_trials.csv" --results-json "/workspace/data/vision/vision_eval_results.jsonl" \
    --summary-json "/workspace/data/vision/vision_eval_summary.json"
```

The capture command is interactive: it prompts for
`center_offset_m along optical plane (+left)` and `distance_m along optical
axis (normal)`, computes `lateral_bearing_deg`, then suggests a `capture_id`
from the toy name and measurements (for example `red_ball_d1p15_op0p244`).
Press Enter to accept it or type a custom ID. `--capture-id` is not a
command-line flag; non-interactive callers get the same generated ID and must
pass `--depth-m` and `--center-offset-m` explicitly. `--bearing-deg` is still
accepted for older scripts.

Each capture stores `left.jpg`, `right.jpg`, and `meta.json` under
`/workspace/data/vision/captures/capture_run1/<capture_id>/`. The `gt_*` columns in
`/workspace/data/vision/vision_eval_trials.csv` are copied from `meta.json`, including
`gt_center_offset_m` and `gt_lateral_bearing_deg`; no spreadsheet annotation is
required. Both pipelines also emit `distance_m`, `center_offset_m`, and
`lateral_bearing_deg` on every live answer. Lateral offset and bearing are
positive to the robot's left (ROS `+y`); no coarse side label is emitted or
accepted.

`vision_eval` writes one scored row per capture per invocation, with the columns
already filled. The `model` column is the detector name for `open_vocab` or the
dense depth model name for `stereo-net`. Its summary manifest records the selected
pipeline, the `shuffle_seed`, and the resulting `capture_order`, so the same
captures can be replayed in the same order by both pipelines.

A live `/vision/ask` path remains available for robot control, but it does not
record comparison trials. Use `vision_capture` and `vision_eval` for measured,
repeatable comparisons.

### Planning trials

`rosa_agent` appends one row per prompt and model, including the reply, the
wall-clock time, and the tools the model called. Comparison models are loaded
and unloaded one at a time so each gets all the available VRAM:

```bash
cd /workspace/software/rosa_agent
python3 -m rosa_agent --compare "llama3.2,mistral,qwen2.5" \
    --prompt "Drive forward 1 meter." \
    --output-csv /workspace/data/planning/plan1/planning_trials.csv \
    --run-id plan1 --gt-json gt_planning.json
```

`--gt-json` is optional. A flat object applies to every model; use
`{"default": {...}, "models": {"mistral": {...}}, "prompts": [...]}` to vary
the expectation by model or prompt (see `software/rosa_agent/README.md`).

### Fill and score

Create an empty template when the automated data is collected another way:

```bash
ros2 run dog_toy_finder_data llm_trials template --kind vision \
    --models yolov8s-world --target "red ball" --run-id capture_run1 \
    --output /workspace/data/vision/manual_vision_trials.csv
ros2 run dog_toy_finder_data llm_trials template --kind planning \
    --models llama3.2,mistral --prompt "Drive forward 1 meter." \
    --run-id plan1 \
    --output /workspace/data/planning/plan1/planning_trials.csv
```

Score after entering the ground truth (offline `vision_eval` output is already
scored, but re-scoring is safe after manual annotation):

```bash
ros2 run dog_toy_finder_data llm_trials score "/workspace/data/vision/vision_eval_trials.csv"
ros2 run dog_toy_finder_data llm_trials score \
    /workspace/data/planning/plan1/planning_trials.csv --tolerance 0.3
ros2 run dog_toy_finder_data llm_trials aggregate \
    /workspace/data/planning/plan1/planning_trials.csv --json
```

`score` writes `<name>_scored.csv` and prints a per-model summary: latency,
errors, ground-truthed row count, accuracy for each field, distance/bearing
mean absolute error, and a mean score. `aggregate` prints the summary without
writing a file.

After collection and annotation, run statistical analysis:

```bash
ros2 run dog_toy_finder_data llm_trials analyze "/workspace/data/vision/vision_eval_trials.csv" \
    --tolerance "0.25" \
    --bearing-tolerance "5.0" \
    --output "/workspace/data/vision/vision_eval_analysis.json"
```

This writes `<name>_analysis.json` with per-model mean/median/std/confidence
intervals and paired model comparisons on matched `ask_id` (the capture ID for
`vision_eval`) or `prompt` (planning) values. Comparisons include a paired
t-test, Holm-adjusted p-value, effect size, and win/tie/loss counts. Use
`--confidence 0.99` to change the interval level or `--pair-field run_id,scene`
to override matching.

Vision metrics: `visible_correct`, `lateral_bearing_deg`, `bearing_error_deg`,
`bearing_within_tolerance`, `distance_error_m`, `distance_within_tolerance`,
`obstacle_correct`, `score`.
Planning metrics: `action_mentioned`, `args_mentioned`, and `score` (from
`gt_success` when recorded).
