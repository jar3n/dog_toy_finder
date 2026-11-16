"""Launch the LLM vision-trial collector.

Records one row per vision model for every /vision/ask, so several VLMs can be
compared on the same stereo photo. Run it alongside the vision node:

    ros2 launch dog_toy_finder_data collect_llm_trials.launch.py \\
        output_dir:=/tmp/llm_trials run_id:=vision1

The CSV is written to ``<output_dir>/<run_id>/vision_trials.csv`` with the
automated columns filled and the ``gt_*`` ground-truth columns blank. Supply
ground truth in the ask JSON, or fill it in a spreadsheet and score with
``ros2 run dog_toy_finder_data llm_trials score``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

COLLECTOR = "/workspace/software/ros2_ws/src/dog_toy_finder_data/dog_toy_finder_data/vision_trial_collector.py"


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="/tmp/llm_trials",
                              description="trial root directory"),
        DeclareLaunchArgument("run_id", default_value="vision1",
                              description="label for this comparison run"),
        DeclareLaunchArgument("distance_tolerance", default_value="0.25",
                              description="distance tolerance in metres"),

        ExecuteProcess(output="screen", cmd=[
            "python3", COLLECTOR, "--ros-args",
            "-p", ["output_dir:=", arg("output_dir")],
            "-p", ["run_id:=", arg("run_id")],
            "-p", ["distance_tolerance:=", arg("distance_tolerance")]]),
    ])
