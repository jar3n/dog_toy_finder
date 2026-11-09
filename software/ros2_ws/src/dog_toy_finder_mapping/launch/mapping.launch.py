"""Launch the occupancy mapper."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

NODE = "/workspace/software/ros2_ws/src/dog_toy_finder_mapping/dog_toy_finder_mapping/occupancy_mapper.py"


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("resolution", default_value="0.05"),
        DeclareLaunchArgument("width", default_value="200"),
        DeclareLaunchArgument("height", default_value="200"),
        DeclareLaunchArgument("origin_x", default_value="-5.0"),
        DeclareLaunchArgument("origin_y", default_value="-5.0"),
        DeclareLaunchArgument("obstacle_range", default_value="0.25"),
        DeclareLaunchArgument("publish_period", default_value="1.0"),
        DeclareLaunchArgument("ir_topic", default_value="/ir_obstacle/detected"),
        DeclareLaunchArgument("use_vision", default_value="true"),
        DeclareLaunchArgument("vision_obstacle_range", default_value="0.8"),
        DeclareLaunchArgument("vision_timeout", default_value="5.0"),
        DeclareLaunchArgument("toy_detection_range", default_value="0.6"),
        DeclareLaunchArgument("toy_bearing_deg", default_value="30.0"),
        DeclareLaunchArgument("toy_dedupe_radius", default_value="0.15"),

        ExecuteProcess(output="screen", cmd=[
            "python3", NODE, "--ros-args",
            "-p", ["resolution:=", arg("resolution")],
            "-p", ["width:=", arg("width")],
            "-p", ["height:=", arg("height")],
            "-p", ["origin_x:=", arg("origin_x")],
            "-p", ["origin_y:=", arg("origin_y")],
            "-p", ["obstacle_range:=", arg("obstacle_range")],
            "-p", ["publish_period:=", arg("publish_period")],
            "-p", ["ir_topic:=", arg("ir_topic")],
            "-p", ["use_vision:=", arg("use_vision")],
            "-p", ["vision_obstacle_range:=", arg("vision_obstacle_range")],
            "-p", ["vision_timeout:=", arg("vision_timeout")],
            "-p", ["toy_detection_range:=", arg("toy_detection_range")],
            "-p", ["toy_bearing_deg:=", arg("toy_bearing_deg")],
            "-p", ["toy_dedupe_radius:=", arg("toy_dedupe_radius")]]),
    ])
