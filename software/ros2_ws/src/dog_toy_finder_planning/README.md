# Planning

`dog_toy_finder_planning` provides an autonomous (deterministic) search planner
used as a baseline against the LLM-driven (ROSA) planner.

## Autonomous search

```bash
ros2 launch dog_toy_finder_planning autonomous_search.launch.py \
    target:="red ball" side:=0.5 loops:=3 max_time:=120 \
    output_csv:=/tmp/auto1.csv run_id:=auto1
```

Strategy: expanding square with vision homing and obstacle avoidance, stopping
when the target is seen as medium/large.

## Path planning utilities

`path_planner.py` (pure Python, unit-tested):
- `expanding_square_waypoints(side, increment, loops)`
- `lawnmower_waypoints(width, height, spacing)`
- `a_star(grid, start, goal)`

## Compare with the LLM planner

Run both and evaluate:

```bash
ros2 launch dog_toy_finder_experiments toy_search.launch.py \
    target:="red ball" output_csv:=/tmp/llm1.csv run_id:=llm1
ros2 launch dog_toy_finder_planning autonomous_search.launch.py \
    target:="red ball" output_csv:=/tmp/auto1.csv run_id:=auto1
ros2 run dog_toy_finder_experiments evaluate_runs /tmp/llm1.csv /tmp/auto1.csv
```