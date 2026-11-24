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
source scripts/full_system_eval.conf

ros2 run dog_toy_finder_vision vision_capture \
    --target "$VISION_CAPTURE_TARGET" \
    --run-id "$VISION_CAPTURE_RUN_ID" --capture-id ball_115m_b12 \
    --output-dir "$VISION_CAPTURE_DIR" --config "$VISION_DETECTOR_CONFIG"

ros2 run dog_toy_finder_vision vision_eval \
    --captures-dir "$VISION_CAPTURE_DIR" --pipeline "$VISION_PIPELINE" \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE" \
    --output-csv "$VISION_EVAL_CSV" --results-json "$VISION_EVAL_JSONL" \
    --summary-json "$VISION_EVAL_SUMMARY_JSON"
```

The capture command is interactive: it prompts for `distance_m (metres)` and
`lateral_bearing_deg (+left)`; type the operator-measured values. For scripts,
pass `--depth-m` and `--bearing-deg` explicitly instead.

Each capture stores `left.jpg`, `right.jpg`, and `meta.json` under
`$VISION_CAPTURE_DIR/<run_id>/<capture_id>/`. The `gt_*` columns in
`$VISION_EVAL_CSV` are copied from `meta.json`, including
`gt_lateral_bearing_deg`; no spreadsheet annotation is required. Lateral
bearing is in degrees from the optical axis, positive to the robot's left
(ROS `+y`); it is the sole lateral measurement, and no coarse side label is
emitted or accepted.

`vision_eval` writes one scored row per capture per invocation, with the columns
already filled. The `model` column is the detector name for `open_vocab` or the
dense depth model name for `stereo-net`. Its summary manifest records the selected
pipeline, the `shuffle_seed`, and the resulting `capture_order`, so the same
captures can be replayed in the same order by both pipelines.

A live `/vision/ask` path is still available for robot control and optional
live trials:

```bash
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    pipeline:="$VISION_PIPELINE"
ros2 launch dog_toy_finder_data collect_llm_trials.launch.py \
    output_dir:=/workspace/data/vision run_id:=vision1 \
    bearing_tolerance:=$LLM_TRIAL_BEARING_TOLERANCE
python3 /workspace/scripts/vision_test.py "red ball" --gt-json gt.json
```

```json
{"target_visible": true, "lateral_bearing_deg": 12.0,
 "size": "medium", "distance_m": [0.7, 0.72], "obstacle_ahead": "none"}
```

`distance_m` also accepts a single number; a list stores repeated measurements
of the same target and scoring uses their mean. Without ground truth, fill the
`gt_*` columns in a spreadsheet after the run. Capture-first is preferred for
fair comparison because each selected pipeline replays the same saved images in
the same recorded order.

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
    --models yolov8s-world --target "red ball" --run-id vision1 \
    --output /workspace/data/vision/vision1/vision_trials.csv
ros2 run dog_toy_finder_data llm_trials template --kind planning \
    --models llama3.2,mistral --prompt "Drive forward 1 meter." \
    --run-id plan1 \
    --output /workspace/data/planning/plan1/planning_trials.csv
```

Score after entering the ground truth (offline `vision_eval` output is already
scored, but re-scoring is safe after manual annotation):

```bash
ros2 run dog_toy_finder_data llm_trials score "$VISION_EVAL_CSV"
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
ros2 run dog_toy_finder_data llm_trials analyze "$VISION_EVAL_CSV" \
    --tolerance "$VISION_EVAL_TOLERANCE" \
    --bearing-tolerance "$VISION_EVAL_BEARING_TOLERANCE" \
    --output "$VISION_EVAL_ANALYSIS_JSON"
```

This writes `<name>_analysis.json` with per-model mean/median/std/confidence
intervals and paired model comparisons on matched `ask_id` (the capture ID for
`vision_eval`) or `prompt` (planning) values. Comparisons include a paired
t-test, Holm-adjusted p-value, effect size, and win/tie/loss counts. Use
`--confidence 0.99` to change the interval level or `--pair-field run_id,scene`
to override matching.

Vision metrics: `visible_correct`, `lateral_bearing_deg`, `bearing_error_deg`,
`bearing_within_tolerance`, `size_correct`, `distance_error_m`,
`distance_within_tolerance`, `obstacle_correct`, `score`.
Planning metrics: `action_mentioned`, `args_mentioned`, and `score` (from
`gt_success` when recorded).
