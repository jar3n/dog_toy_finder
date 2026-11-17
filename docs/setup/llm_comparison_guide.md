# LLM comparison guide (vision and planning)

This guide walks through comparing several vision models and several planning
models with `dog_toy_finder_data`. Both kinds of trial share one CSV schema and
one scoring tool (`dog_toy_finder_data.llm_trials`), so the automated columns are
recorded in the same format and ground truth can be filled in as it becomes
available.

| Tool | Purpose |
|---|---|
| `vision_trial_collector` | Subscribes to `/vision/ask` and `/vision/result`; writes one CSV row per model per request. |
| ROSA planning logging | `rosa_agent --compare --output-csv` writes one row per prompt and model. |
| `llm_trials` CLI | `template`, `score`, and `aggregate` for either CSV kind. |

The collector fills the automated columns. The `gt_*` ground-truth columns are
either supplied up front, or filled later in a spreadsheet. `score` computes the
metric columns.

## Where trial data lives

Trials are written under the repository's `data/` folder, next to `hardware/`
and `software/`:

```text
data/
  vision/<run_id>/vision_trials.csv
  vision/<run_id>/manifest.json
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

Vision comparison does not require the Pico or the motion stack. It needs the
stereo vision node, the ESP32-CAM feeds, Ollama, and the trial collector.

### A1. Start the vision node with a model list

```bash
ros2 launch dog_toy_finder_vision stereo_vision.launch.py \
    left_url:=http://esp32cam-left.local/capture \
    right_url:=http://esp32cam-right.local/capture \
    models:="gemma4,minicpm-v4.5"
```

Every model in `models` answers the same stereo photo, and the node publishes
all answers together in one `/vision/result` message. The node loads one model
at a time and unloads it before starting the next, so each model gets all of the
Ollama host's VRAM rather than sharing it with the others.

### A2. Start the trial collector

In a second terminal, same container and sourced workspace:

```bash
ros2 launch dog_toy_finder_data collect_llm_trials.launch.py \
    output_dir:=/workspace/data/vision \
    run_id:=vision1 \
    distance_tolerance:=0.25
```

This writes:

```text
/workspace/data/vision/vision1/vision_trials.csv
/workspace/data/vision/vision1/manifest.json
```

The manifest records the run ID, model list, row count, and tolerance.

### A3. Collect trials with ground truth

Put the toy in a known position, then issue an ask with a ground-truth JSON
file. Create `gt_vision.json`:

```json
{
  "target_visible": true,
  "position": "left",
  "size": "medium",
  "distance_m": [0.7, 0.72, 0.69],
  "obstacle_ahead": "none",
  "notes": "toy about 0.7 m ahead, left of centre"
}
```

`distance_m` accepts a single number or a list of repeated measurements. The
list is stored as JSON in the `gt_distance_m` CSV column, and scoring uses the
mean as the ground-truth distance.

Ask the vision node:

```bash
python3 /workspace/scripts/vision_test.py "red ball" --gt-json gt_vision.json
```

Each invocation appends one row per model to `vision_trials.csv`, with the
`gt_*` columns already filled.

Repeat this for each scene you want to evaluate. Move the toy, update
`gt_vision.json`, and ask again. A useful comparison has several asks per model,
ideally covering:

- toy visible / not visible
- left / centre / right
- near / medium / far
- clear path / obstacle present
- more than one toy kind

The collector keeps appending to the same CSV for the same `run_id`.

### A4. Fill ground truth manually (optional)

If you did not pass `--gt-json`, open `vision_trials.csv` in a spreadsheet and
fill:

```text
gt_target_visible, gt_position, gt_size, gt_distance_m,
gt_obstacle_ahead, gt_notes
```

Leave a field blank when it does not apply. Blank ground truth means that row is
skipped for that metric. For `gt_distance_m`, enter a single number, a JSON list
such as `[0.7, 0.72]`, or comma-separated measurements such as `0.7, 0.72`.

### A5. Score the vision trials

```bash
ros2 run dog_toy_finder_data llm_trials score \
    /workspace/data/vision/vision1/vision_trials.csv \
    --tolerance 0.25
```

This writes `vision_trials_scored.csv` and prints, per model:

- `mean_latency_s`
- `errors`
- `gt_rows`
- `visible_accuracy`
- `position_accuracy`
- `size_accuracy`
- `distance_mae_m`
- `obstacle_accuracy`
- `mean_score`

`mean_latency_s` includes each model's cold-load time, because the node unloads
a model after its answer before starting the next one.

For machine-readable output use `aggregate`:

```bash
ros2 run dog_toy_finder_data llm_trials aggregate \
    /workspace/data/vision/vision1/vision_trials.csv --json
```

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

## Part C - Collecting with a template instead

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

Fill in the answer and ground-truth columns, then run `llm_trials score` exactly
as above.

## Part D - How to read the results

### Vision metrics

| Column | Meaning |
|---|---|
| `visible_correct` | Model's visible/not-visible answer matches ground truth. |
| `position_correct` | left/centre/right matches. |
| `size_correct` | size matches. |
| `distance_error_m` | Absolute error in metres, against the mean when several ground-truth measurements are recorded. |
| `distance_within_tolerance` | Error is within `--tolerance`. |
| `obstacle_correct` | Obstacle description matches. |
| `score` | Mean of the checks that apply to that row. |

Rows where the toy is not visible only score visibility and obstacle checks.
Blank ground-truth fields are ignored, so partial annotation still produces
useful results.

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
3. For vision, always ask every model about the same photo; use the same toy
   placement for the whole model list.
4. For planning, use the same prompt, same starting pose, and same vision state
   for every model.
5. Record the operator verdict in `gt_success` whenever a move actually
   executes.
6. Keep both the raw CSV and the `*_scored.csv`; the raw file preserves what the
   model actually said.
