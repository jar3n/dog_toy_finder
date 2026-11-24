# Evaluation data

Generated vision-pipeline and planning comparison trials live here. The directory is
next to `hardware/` and `software/` in the repository and is mounted at
`/workspace/data` inside the ROS 2 container.

```text
data/
  vision/
    captures/
      <run_id>/
        <capture_id>/
          left.jpg
          right.jpg
          meta.json
    vision_eval_trials.csv
    vision_eval_trials_scored.csv
    vision_eval_results.jsonl
    vision_eval_summary.json
    vision_eval_analysis.json
    <live_run_id>/                         # optional live-path collector
      vision_trials.csv
      vision_trials_scored.csv
      manifest.json
  planning/
    <run_id>/
      planning_trials.csv
      planning_trials_scored.csv
```

- `vision/captures/` is written by `dog_toy_finder_vision vision_capture`: one
  fresh rectified stereo pair plus operator-entered ground truth per capture.
  `meta.json` records the target, intrinsics, image timestamps, and
  `ground_truth` (including `lateral_bearing_deg`).
- `vision_eval_*` files are written by `dog_toy_finder_vision vision_eval`,
  which replays every capture through one selected vision pipeline per
  invocation. `vision_eval_trials.csv` has one scored row per capture, with
  ground truth and scored columns already filled. The `model` column is the
  detector name for `open_vocab` or the dense depth model name for `stereo-net`.
  `vision_eval_summary.json` records the selected pipeline, `shuffle_seed`,
  `capture_order`, and the per-model summary. The trial CSV schema is unchanged:
  new vision rows use `method=open_vocab` or `method=stereo-net`, while `method=ml`
  only appears on legacy rows.
- `vision/<live_run_id>/` is the optional live `/vision/ask` collector
  (`collect_llm_trials.launch.py`), one row per answer entry per request.
- `planning/` is written by `rosa_agent --compare --output-csv` (and the
  interactive session when `--output-csv` is set), one row per prompt and model.
- `*_scored.csv` is created by `ros2 run dog_toy_finder_data llm_trials score`
  next to the source CSV.
- `manifest.json` is collector metadata: run ID, pipeline, model names, row
  count, and the distance/bearing tolerances.

The `vision/` and `planning/` directories are tracked so the layout exists
after a clone; generated contents are ignored by git. Use a new `run_id` for
each experiment so a later run cannot append to an earlier one.

See [`docs/setup/llm_comparison_guide.md`](../docs/setup/llm_comparison_guide.md)
for the full workflow.
