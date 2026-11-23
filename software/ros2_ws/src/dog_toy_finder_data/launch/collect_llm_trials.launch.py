"""Launch the LLM vision-trial collector.

Records one row per model for every /vision/ask, so several VLMs can be
compared on the same rectified stereo pair.
Run it alongside the vision node:

    ros2 launch dog_toy_finder_data collect_llm_trials.launch.py \\
        output_dir:=/workspace/data/vision run_id:=vision1

The CSV is written to ``<output_dir>/<run_id>/vision_trials.csv`` with the
automated columns filled and the ``gt_*`` ground-truth columns blank. Supply
ground truth in the ask JSON, or fill it in a spreadsheet and score with
``ros2 run dog_toy_finder_data llm_trials score``.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument(
            "output_dir",
            default_value=os.environ.get("VISION_TRIAL_DIR",
                                         "/workspace/data/vision"),
            description="trial root directory"),
        DeclareLaunchArgument("run_id", default_value="vision1",
                              description="label for this comparison run"),
        DeclareLaunchArgument("distance_tolerance", default_value="0.25",
                              description="distance tolerance in metres"),
        DeclareLaunchArgument("bearing_tolerance", default_value="5.0",
                              description="lateral bearing tolerance in degrees"),

        Node(
            package="dog_toy_finder_data",
            executable="vision_trial_collector",
            output="screen",
            parameters=[{
                "output_dir": arg("output_dir"),
                "run_id": arg("run_id"),
                "distance_tolerance": arg("distance_tolerance"),
                "bearing_tolerance": arg("bearing_tolerance"),
            }],
        ),
    ])
