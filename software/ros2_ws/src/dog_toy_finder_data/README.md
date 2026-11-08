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