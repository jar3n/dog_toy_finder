"""Launch the data collector node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

COLLECTOR = "/workspace/software/ros2_ws/src/dog_toy_finder_data/dog_toy_finder_data/data_collector.py"


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("output_dir", default_value="/tmp/dog_toy_dataset",
                              description="dataset root directory"),
        DeclareLaunchArgument("run_id", default_value="run1",
                              description="label for this run"),
        DeclareLaunchArgument("record_images", default_value="true",
                              description="save stereo frames"),
        DeclareLaunchArgument("image_every", default_value="5",
                              description="save every N-th frame"),
        DeclareLaunchArgument("record_disparity", default_value="false",
                              description="save disparity maps"),

        ExecuteProcess(output="screen", cmd=[
            "python3", COLLECTOR, "--ros-args",
            "-p", ["output_dir:=", arg("output_dir")],
            "-p", ["run_id:=", arg("run_id")],
            "-p", ["record_images:=", arg("record_images")],
            "-p", ["image_every:=", arg("image_every")],
            "-p", ["record_disparity:=", arg("record_disparity")]]),
    ])
