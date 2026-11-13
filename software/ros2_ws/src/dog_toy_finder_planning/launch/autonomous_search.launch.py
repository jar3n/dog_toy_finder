"""Launch the autonomous (non-LLM) search planner."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

NODE = "/workspace/software/ros2_ws/src/dog_toy_finder_planning/dog_toy_finder_planning/autonomous_search_node.py"


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("target", default_value="red ball"),
        DeclareLaunchArgument("side", default_value="0.5"),
        DeclareLaunchArgument("side_increment", default_value="0.5"),
        DeclareLaunchArgument("loops", default_value="3"),
        DeclareLaunchArgument("max_time", default_value="120.0"),
        DeclareLaunchArgument("output_csv", default_value="/tmp/autonomous_search.csv"),
        DeclareLaunchArgument("run_id", default_value="auto1"),
        DeclareLaunchArgument("found_distance", default_value="0.4"),
        DeclareLaunchArgument("find_all", default_value="false"),
        DeclareLaunchArgument("toy_side_offset", default_value="0.25"),
        DeclareLaunchArgument("toy_forward_distance", default_value="0.6"),
        DeclareLaunchArgument("toy_merge_radius", default_value="0.15"),
        DeclareLaunchArgument("escape_distance", default_value="0.3"),

        ExecuteProcess(output="screen", cmd=[
            "python3", NODE, "--ros-args",
            "-p", ["target:=", arg("target")],
            "-p", ["side:=", arg("side")],
            "-p", ["side_increment:=", arg("side_increment")],
            "-p", ["loops:=", arg("loops")],
            "-p", ["max_time:=", arg("max_time")],
            "-p", ["output_csv:=", arg("output_csv")],
            "-p", ["run_id:=", arg("run_id")],
            "-p", ["found_distance:=", arg("found_distance")],
            "-p", ["find_all:=", arg("find_all")],
            "-p", ["toy_side_offset:=", arg("toy_side_offset")],
            "-p", ["toy_forward_distance:=", arg("toy_forward_distance")],
            "-p", ["toy_merge_radius:=", arg("toy_merge_radius")],
            "-p", ["escape_distance:=", arg("escape_distance")]]),
    ])
