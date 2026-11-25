# LLM comparison guides

The combined vision-and-planning guide has been split into two focused guides:

- [Vision pipeline comparison guide](vision_comparison_guide.md) - capture-first
  comparison of `open_vocab` and `stereo-net` using saved rectified pairs,
  operator-entered ground truth, and offline replay.
- [Planning model comparison guide](planning_comparison_guide.md) - live ROSA
  comparison of planning models using the same prompt, tool-call logging, and
  operator-recorded outcomes.

Both guides use [`dog_toy_finder_data.llm_trials`](../../software/ros2_ws/src/dog_toy_finder_data/README.md)
for CSV schema, scoring, aggregation, and statistical analysis. Pick the guide
that matches the experiment you are running.
