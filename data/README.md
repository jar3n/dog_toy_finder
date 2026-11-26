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
          scene.json
          shot_01/
            left.jpg
            right.jpg
            meta.json
          shot_02/
            ...
          shot_03/
            ...
    pilot_captures/
      <run_id>/
        <capture_id>/
          scene.json
          shot_01/
            ...
    vision_eval_trials.csv
    vision_eval_trials_scored.csv
    vision_eval_results.jsonl
    vision_eval_summary.json
    vision_eval_analysis.json
  planning/
    <run_id>/
      planning_trials.csv
      planning_trials_scored.csv
```

- `vision/captures/` is written by `dog_toy_finder_vision vision_capture`:
  several fresh rectified stereo shots per scene, with operator-entered ground
  truth shared by all shots. `scene.json` and each shot's `meta.json` record the
  target, intrinsics, image timestamps, and `ground_truth` (including the
  entered `center_offset_m` and computed `lateral_bearing_deg`). `--shots 1`
  keeps the older flat layout with one pair directly in `<capture_id>/`.
- `vision/pilot_captures/` is the pilot-only sibling of `vision/captures/`.
  Add `--pilot` to `vision_capture` and `vision_eval` to keep pilot scenes and
  their default `vision_eval_*` outputs out of the normal captures tree;
  explicit `--output-dir`, `--captures-dir`, and output paths still win.
- `vision_eval_*` files are written by `dog_toy_finder_vision vision_eval`,
  which replays every shot through one selected vision pipeline per invocation.
  `vision_eval_trials.csv` has one scored row per model per shot, with ground
  truth and scored columns already filled; multi-shot `ask_id` values end in
  `_shot_NN`. The `model` column is the detector name for `open_vocab` or the
  dense depth model name for `stereo-net`. `vision_eval_summary.json` records
  the selected pipeline, `shuffle_seed`, randomized `scene_order`, `shot_order`
  within each scene, `shots_per_scene`, and the per-model summary. The trial CSV
  schema is
  unchanged: new vision rows use `method=open_vocab` or `method=stereo-net`,
  while `method=ml` only appears on legacy rows.
- `planning/` is written by `rosa_agent --compare --output-csv` (and the
  interactive session when `--output-csv` is set), one row per prompt and model.
- `*_scored.csv` is created by `ros2 run dog_toy_finder_data llm_trials score`
  next to the source CSV.

The `vision/` and `planning/` directories are tracked so the layout exists
after a clone; generated contents are ignored by git. Use a new `run_id` for
each experiment so a later run cannot append to an earlier one.

See the [vision comparison guide](../docs/setup/vision_comparison_guide.md)
and [planning comparison guide](../docs/setup/planning_comparison_guide.md)
for the full workflows.
