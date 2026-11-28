# Planning model comparison guide

This guide covers live comparison of planning models through ROSA. The same
prompt is sent to every model, and one row per model is appended to the shared
planning CSV. Vision-pipeline comparison is separate; see the
[vision comparison guide](vision_comparison_guide.md).

Both trial kinds use `dog_toy_finder_data.llm_trials` for CSV schema, scoring,
aggregation, and statistical analysis, but their collection paths differ:
vision comparison is capture-first and offline, while planning comparison sends
each prompt to every model live.

## Where planning trial data lives

Generated data lives under the repository's `data/` folder:

```text
data/
  planning/<run_id>/planning_trials.csv
```

Inside the ROS 2 container the repository is mounted at `/workspace`, so the
commands below use `/workspace/data/...`. On the host the same files appear
under `<repo>/data/...`. The `planning/` folder is tracked, but its generated
contents are ignored by git.

## One-time setup

Inside the ROS 2 container:

```bash
cd /workspace/software/ros2_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Planning uses Ollama. Install and start it as described in
[`software/rosa_agent/README.md`](../../software/rosa_agent/README.md), and pull
the models you intend to compare. Models run one at a time and are unloaded
before the next starts, so each planner gets all of the Ollama host's VRAM.

The planner's tools call vision and motion. To score whether actions actually
worked, start the normal robot/vision stack as described below. The host
detector and depth services are configured in the
[vision comparison guide](vision_comparison_guide.md#one-time-setup); Ollama is
not needed by the vision pipelines.

Confirm the data package is available:

```bash
ros2 run dog_toy_finder_data llm_trials --help
```

## Start the robot/vision stack the planner will use

ROSA's tools read odometry and call vision/motion. To score whether actions
actually worked, the normal stack should be running:

```bash
cd /workspace/docker
docker compose up -d micro_ros_agent
# then, in the ROS 2 container:
ros2 launch dog_toy_finder_bringup robot.launch.py
ros2 launch dog_toy_finder_vision stereo_vision.launch.py pipeline:="triangulation"
```

If the Pico is intentionally off, you can still capture replies and tool calls,
but `gt_success` cannot honestly reflect physical execution. In that case score
only the text/tool-call checks (`action_mentioned`, `args_mentioned`).

## Run the same prompt through several models

```bash
cd /workspace/software/rosa_agent

python3 -m rosa_agent \
    --compare "llama3.2,mistral,qwen2.5" \
    --prompt "Drive forward 1 meter." \
    --output-csv /workspace/data/planning/plan1/planning_trials.csv \
    --run-id plan1
```

Each model gets the same prompt. The row records the reply, wall-clock latency,
every tool call, and any error.

For an interactive session, the same CSV logging applies to every prompt:

```bash
python3 -m rosa_agent \
    --output-csv /workspace/data/planning/plan1/planning_trials.csv \
    --run-id plan1
```

## Supply planning ground truth

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

## Add more prompts to the same run

`--output-csv` appends, so repeated compare invocations build one planning CSV:

```bash
python3 -m rosa_agent --compare "llama3.2,mistral,qwen2.5" \
    --prompt "Turn left 90 degrees." \
    --output-csv /workspace/data/planning/plan1/planning_trials.csv \
    --run-id plan1 \
    --gt-json gt_planning.json
```

Use the same model list and `run_id` for every prompt in the experiment.

## Score the planning trials

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

## Statistical analysis (after collection)

`analyze` scores the CSV in memory, then reports descriptive statistics for
every model and paired comparisons between models on matched trials. Planning
rows are matched by `run_id` + `prompt`.

```bash
ros2 run dog_toy_finder_data llm_trials analyze \
    /workspace/data/planning/plan1/planning_trials.csv \
    --output /workspace/data/planning/plan1/planning_analysis.json
```

The JSON report contains, for each model/metric: `n`, `mean`, `median`,
`std` (sample), `sem`, `ci_low`, `ci_high`, `min`, and `max`. Pairwise entries
report `mean_diff` (`model_a - model_b`), its confidence interval, a two-sided
paired t-test `p_value`, a Holm-adjusted `p_value_holm`, `effect_dz`, and
win/tie/loss counts. A `--csv-output` option writes one tidy table containing
both descriptive and comparison rows.

Useful options:

```bash
# 99% confidence intervals, and a different matching field
ros2 run dog_toy_finder_data llm_trials analyze \
    /workspace/data/planning/plan1/planning_trials.csv \
    --confidence 0.99 --pair-field run_id,scene --json
```

Set `LLM_TRIAL_CONFIDENCE` in `scripts/full_system_eval.conf` for the confidence
level used in evaluation runs. Pairs with fewer than two observations get no
p-value; rows with different `prompt` values are never compared.

## Collecting with a template instead

If another process produces the replies, create an empty CSV with the correct
columns and ground-truth fields:

```bash
ros2 run dog_toy_finder_data llm_trials template \
    --kind planning \
    --models "llama3.2,mistral,qwen2.5" \
    --prompt "Drive forward 1 meter." \
    --run-id plan1 \
    --output /workspace/data/planning/plan1/planning_trials.csv
```

Fill in the reply, tool-call, and ground-truth columns, then run
`llm_trials score` as described above.

## How to read the results

| Column | Meaning |
|---|---|
| `action_mentioned` | `gt_expected_action` appears in the reply or tool calls. |
| `args_mentioned` | Every comma-separated `gt_expected_args` token appears. |
| `score` | `gt_success` if recorded, otherwise the mean of the mention checks. |

Because `action_mentioned` and `args_mentioned` are text checks, they are cheap
signals, not a full plan-quality judgement. Use `gt_success` for the
authoritative outcome.

## Experiment discipline

1. Use one `run_id` per experiment and a new run ID when the setup changes.
2. Use the same prompt, same starting pose, and same vision state for every
   model.
3. Record the operator verdict in `gt_success` whenever a move actually
   executes. If the robot is not running, score only the text/tool-call checks.
4. Keep the raw CSV, the summary JSON, and the `*_scored.csv`; the raw files
   preserve what each model replied and which tools it called.
5. Run `llm_trials analyze` on the completed CSV and keep the `*_analysis.json`
   report beside the trial data.

For vision experiments, use the
[vision comparison guide](vision_comparison_guide.md).
