# Data collection

`dog_toy_finder_data` records a structured dataset during a test run for offline
analysis and VLM benchmarking.

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
a vision model and a planner can be judged with the same fields. The collector
stores automated data first, then blank `gt_*` ground-truth fields, then the
metrics computed by `score`.

### Vision trials

Start the collector next to the vision node. Set `VISION_MODELS` to a
comma-separated list so every model answers the same stereo photo:

```bash
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    models:="gemma4,minicpm-v4.5"
ros2 launch dog_toy_finder_data collect_llm_trials.launch.py \
    output_dir:=/tmp/llm_trials run_id:=vision1
```

It writes `vision_trials.csv` (one row per ask and model) and `manifest.json`
under `<output_dir>/<run_id>/`. Supply ground truth in the ask JSON and the
`gt_*` columns are filled immediately:

```bash
python3 /workspace/scripts/vision_test.py "red ball" --gt-json gt.json
```

```json
{"target_visible": true, "position": "left", "size": "medium",
 "distance_m": 0.7, "obstacle_ahead": "none"}
```

Without ground truth, fill the `gt_*` columns in a spreadsheet after the run.

### Planning trials

`rosa_agent` appends one row per prompt and model, including the reply, the
wall-clock time, and the tools the model called:

```bash
cd /workspace/software/rosa_agent
python3 -m rosa_agent --compare "llama3.2,mistral,qwen2.5" \
    --prompt "Drive forward 1 meter." \
    --output-csv /tmp/llm_trials/planning1/planning_trials.csv \
    --run-id plan1 --gt-json gt_planning.json
```

`--gt-json` is optional. A flat object applies to every model; use
`{"default": {...}, "models": {"mistral": {...}}, "prompts": [...]}` to vary
the expectation by model or prompt (see `software/rosa_agent/README.md`).

### Fill and score

Create an empty template when the automated data is collected another way:

```bash
ros2 run dog_toy_finder_data llm_trials template --kind vision \
    --models gemma4,minicpm-v4.5 --output vision_trials.csv
ros2 run dog_toy_finder_data llm_trials template --kind planning \
    --models llama3.2,mistral --prompt "Drive forward 1 meter." \
    --output planning_trials.csv
```

Score after entering the ground truth:

```bash
ros2 run dog_toy_finder_data llm_trials score vision_trials.csv
ros2 run dog_toy_finder_data llm_trials score planning_trials.csv --tolerance 0.3
ros2 run dog_toy_finder_data llm_trials aggregate planning_trials.csv --json
```

`score` writes `<name>_scored.csv` and prints a per-model summary: latency,
errors, ground-truthed row count, accuracy for each field, distance mean
absolute error, and a mean score. `aggregate` prints the summary without
writing a file.

Vision metrics: `visible_correct`, `position_correct`, `size_correct`,
`distance_error_m`, `distance_within_tolerance`, `obstacle_correct`, `score`.
Planning metrics: `action_mentioned`, `args_mentioned`, and `score` (from
`gt_success` when recorded).
