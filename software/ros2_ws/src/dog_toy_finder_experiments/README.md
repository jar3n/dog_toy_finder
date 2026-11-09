# Full-system experiments

`dog_toy_finder_experiments` drives a search mission, logs pose + vision data,
and reports performance metrics for evaluating the robot at finding a given toy.

## Run a search

Prerequisites: bringup (+ vision if using the VLM) running.

```bash
ros2 launch dog_toy_finder_experiments toy_search.launch.py \
    target:="red ball" side:=0.5 loops:=3 max_time:=120 \
    output_csv:=/tmp/run1.csv run_id:=run1
```

The node drives an expanding square, asking the vision node for the target after
every side, and stops early when the target is seen as medium/large.

## Evaluate

```bash
ros2 run dog_toy_finder_experiments evaluate_runs /tmp/run1.csv
ros2 run dog_toy_finder_experiments evaluate_runs /tmp/run*.csv
```

Metrics: success, time/steps to find, path length, vision call count and
average latency, target-visible rate, final pose.
