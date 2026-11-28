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
    vision_eval_triangulation__yolo_world_trials.csv
    vision_eval_stereo-net__yolo_world__lite_any_stereo_trials.csv
    ...                                  # one trio per matrix run
    vision_eval_all_trials.csv
    vision_eval_all_trials_scored.csv
    vision_eval_triangulation__yolo_world_results.jsonl
    vision_eval_stereo-net__yolo_world__lite_any_stereo_results.jsonl
    vision_eval_triangulation__yolo_world_summary.json
    vision_eval_stereo-net__yolo_world__lite_any_stereo_summary.json
    vision_eval_all_summary.json
    vision_eval_overlays/
      <run_key>/
        <run_id>/<capture_id>/.../<object_id>_left.jpg
        <run_id>/<capture_id>/.../<object_id>_right.jpg
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
  which replays every shot through the full model matrix in one invocation:
  every detector for `triangulation` and every detector/depth pair for
  `stereo-net`. Each run gets its own `trials.csv`, `results.jsonl`, and
  `summary.json` with a model-qualified `run_key`;
  `vision_eval_all_trials.csv` combines every run's rows for paired analysis,
  and `vision_eval_all_summary.json` indexes the runs and records the host
  model unload status after each run. Each trial CSV
  has one scored row per model per object per shot, with ground truth and scored
  columns already filled. The primary toy and each `additional_objects` entry
  are evaluated with their own ground truth; the primary object keeps the shot
  ID (`_shot_NN` for multi-shot scenes) and additional objects append
  `__additional_NN`. The `model` column is the detector name for `triangulation` or
  the dense depth model name for `stereo-net`. Each per-run summary records
  its pipeline, the shared `shuffle_seed`, randomized `scene_order`, `shot_order`
  within each scene, `shots_per_scene`, `objects_per_scene`, and the per-model
  summary. The trial CSV schema is unchanged: new vision rows use
  `method=triangulation` or `method=stereo-net`. Older pilot rows may still say
  `method=open_vocab`; `method=ml` only appears on the retired ML path.
  `vision_eval_overlays/` holds one annotated JPEG per scored object and eye,
  keyed by run and shot path, so a model's reported detection can be checked
  against the saved capture; `--no-overlays` skips this tree.
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
