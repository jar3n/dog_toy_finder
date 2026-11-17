# Evaluation data

Generated vision and planning comparison trials live here. The directory is
next to `hardware/` and `software/` in the repository and is mounted at
`/workspace/data` inside the ROS 2 container.

```text
data/
  vision/
    <run_id>/
      vision_trials.csv
      vision_trials_scored.csv
      manifest.json
  planning/
    <run_id>/
      planning_trials.csv
      planning_trials_scored.csv
```

- `vision/` is written by `dog_toy_finder_data` `vision_trial_collector`
  (`collect_llm_trials.launch.py`), one row per model for every `/vision/ask`.
- `planning/` is written by `rosa_agent --compare --output-csv` (and the
  interactive session when `--output-csv` is set), one row per prompt and model.
- `*_scored.csv` is created by `ros2 run dog_toy_finder_data llm_trials score`
  next to the source CSV.
- `manifest.json` is collector metadata: run ID, models, row count, and the
  distance tolerance.

The two directories are tracked so the layout exists after a clone; their
contents are ignored by git. Use a new run ID for each experiment so a later
run cannot append to an earlier one.

See [`docs/setup/llm_comparison_guide.md`](../docs/setup/llm_comparison_guide.md)
for the full workflow.
