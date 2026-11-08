"""Bring up the computer-side ROS 2 stack for the Dog Toy Finder.

Starts:
  * dog_toy_finder_odometry: /joint_states -> /wheel/odom
  * a static base_link -> imu_link transform for the BNO055 mounting
  * robot_localization EKF: /wheel/odom + /imu/data -> /odometry/filtered

The micro-ROS agent runs in its own container (see docker/docker-compose.yml),
so this launch file only starts the computer-side ROS 2 nodes.

The EKF publishes the odom -> base_link transform, so start only one instance
of this launch file per robot.

Usage inside the ROS 2 container:
    cd /workspace/software/ros2_ws
    colcon build --symlink-install
    source install/setup.bash
    ros2 launch dog_toy_finder_bringup robot.launch.py

Common overrides:
    ... wheel_radius:=0.0325 wheel_separation:=0.133
    ... publish_tf:=true          # only when running without the EKF
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Build the launch description for the real robot."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    publish_tf = LaunchConfiguration("publish_tf")

    config_dir = PathJoinSubstitution(
        [FindPackageShare("dog_toy_finder_bringup"), "config"]
    )
    robot_config = PathJoinSubstitution([config_dir, "robot.yaml"])
    ekf_config = PathJoinSubstitution([config_dir, "ekf.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the simulation clock instead of the wall clock",
            ),
            DeclareLaunchArgument(
                "wheel_radius",
                default_value="0.0325",
                description="Drive-wheel radius in metres",
            ),
            DeclareLaunchArgument(
                "wheel_separation",
                default_value="0.133",
                description="Distance between wheel contact patches in metres",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="false",
                description="Let the odometry node publish odom -> base_link",
            ),
            DeclareLaunchArgument(
                "imu_x", default_value="0.0", description="IMU x offset from base_link"
            ),
            DeclareLaunchArgument(
                "imu_y", default_value="0.0", description="IMU y offset from base_link"
            ),
            DeclareLaunchArgument(
                "imu_z",
                default_value="0.0085",
                description="IMU z offset from base_link in metres",
            ),
            DeclareLaunchArgument(
                "imu_roll",
                default_value="0.0",
                description="IMU roll mounting angle in radians",
            ),
            DeclareLaunchArgument(
                "imu_yaw",
                default_value="0.0",
                description="IMU yaw mounting angle in radians",
            ),
            Node(
                package="dog_toy_finder_odometry",
                executable="odometry_node",
                name="wheel_odometry",
                output="screen",
                parameters=[
                    robot_config,
                    {
                        "wheel_radius": ParameterValue(
                            wheel_radius, value_type=float
                        ),
                        "wheel_separation": ParameterValue(
                            wheel_separation, value_type=float
                        ),
                        "publish_tf": ParameterValue(publish_tf, value_type=bool),
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_mount",
                output="screen",
                arguments=[
                    "--x",
                    LaunchConfiguration("imu_x"),
                    "--y",
                    LaunchConfiguration("imu_y"),
                    "--z",
                    LaunchConfiguration("imu_z"),
                    "--roll",
                    LaunchConfiguration("imu_roll"),
                    "--pitch",
                    "0",
                    "--yaw",
                    LaunchConfiguration("imu_yaw"),
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "imu_link",
                ],
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    ekf_config,
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
        ]
    )
