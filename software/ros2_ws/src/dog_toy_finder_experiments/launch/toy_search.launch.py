"""Launch the full-system toy search test node.

Assumes the robot bringup (and, for vision, the vision launch) are already
running. The test node drives a search pattern, queries /vision/ask, and
writes a CSV log for evaluation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

TEST_NODE = "/workspace/software/ros2_ws/src/dog_toy_finder_experiments/dog_toy_finder_experiments/toy_search_test.py"


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("target", default_value="red ball",
                              description="toy description passed to the vision node"),
        DeclareLaunchArgument("pattern", default_value="square",
                              description="search pattern: square or lawnmower"),
        DeclareLaunchArgument("side", default_value="0.5",
                              description="starting side length in metres"),
        DeclareLaunchArgument("side_increment", default_value="0.5",
                              description="side growth per loop in metres"),
        DeclareLaunchArgument("loops", default_value="3",
                              description="number of search loops"),
        DeclareLaunchArgument("max_time", default_value="120.0",
                              description="abort after this many seconds"),
        DeclareLaunchArgument("output_csv", default_value="/tmp/toy_search.csv",
                              description="CSV log path"),
        DeclareLaunchArgument("run_id", default_value="run1",
                              description="label stored in the log"),
        DeclareLaunchArgument("found_distance", default_value="0.4",
                              description="stop when the VLM estimates the toy within this (m)"),
        DeclareLaunchArgument("find_all", default_value="false",
                              description="demo: keep searching after each find"),
        DeclareLaunchArgument("toy_side_offset", default_value="0.25",
                              description="left/right offset in the found-toy estimate (m)"),
        DeclareLaunchArgument("toy_forward_distance", default_value="0.6",
                              description="fallback distance ahead in the estimate (m)"),
        DeclareLaunchArgument("toy_merge_radius", default_value="0.15",
                              description="toys closer than this are the same toy (m)"),
        DeclareLaunchArgument("escape_distance", default_value="0.3",
                              description="metres backed up after each find"),

        ExecuteProcess(output="screen", cmd=[
            "python3", TEST_NODE, "--ros-args",
            "-p", ["target:=", arg("target")],
            "-p", ["pattern:=", arg("pattern")],
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
