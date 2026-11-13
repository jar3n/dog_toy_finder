"""Standalone launch for the sensor heartbeat health monitor.

Usage inside the ROS 2 container:
    ros2 launch dog_toy_finder_diagnostics health.launch.py

The node is also started by dog_toy_finder_bringup robot.launch.py, so a
separate launch is only needed when the bringup stack is not running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Start the health monitor with optional timeout overrides."""
    timeout_args = [
        DeclareLaunchArgument(
            name,
            default_value=str(default),
            description=f"Seconds without a message before {name} fails",
        )
        for name, default in [
            ("imu_timeout", "0.5"),
            ("joint_states_timeout", "0.5"),
            ("ir_timeout", "1.0"),
            ("left_camera_timeout", "3.0"),
            ("right_camera_timeout", "3.0"),
            ("odometry_timeout", "2.0"),
        ]
    ]
    return LaunchDescription(
        timeout_args
        + [
            DeclareLaunchArgument(
                "check_period",
                default_value="0.5",
                description="Seconds between heartbeat checks",
            ),
            DeclareLaunchArgument(
                "startup_grace",
                default_value="10.0",
                description="Seconds to wait for a sensor's first message",
            ),
            Node(
                package="dog_toy_finder_diagnostics",
                executable="health_monitor",
                name="health_monitor",
                output="screen",
                parameters=[
                    {
                        "check_period": ParameterValue(
                            LaunchConfiguration("check_period"), value_type=float
                        ),
                        "startup_grace": ParameterValue(
                            LaunchConfiguration("startup_grace"), value_type=float
                        ),
                        "imu_timeout": ParameterValue(
                            LaunchConfiguration("imu_timeout"), value_type=float
                        ),
                        "joint_states_timeout": ParameterValue(
                            LaunchConfiguration("joint_states_timeout"),
                            value_type=float,
                        ),
                        "ir_timeout": ParameterValue(
                            LaunchConfiguration("ir_timeout"), value_type=float
                        ),
                        "left_camera_timeout": ParameterValue(
                            LaunchConfiguration("left_camera_timeout"),
                            value_type=float,
                        ),
                        "right_camera_timeout": ParameterValue(
                            LaunchConfiguration("right_camera_timeout"),
                            value_type=float,
                        ),
                        "odometry_timeout": ParameterValue(
                            LaunchConfiguration("odometry_timeout"),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
