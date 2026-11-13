"""Bring up the computer-side ROS 2 stack for the Dog Toy Finder.

Starts:
  * dog_toy_finder_odometry: /joint_states -> /wheel/odom
  * a static base_link -> imu_link transform for the BNO055 mounting
  * robot_localization EKF: /wheel/odom + /imu/data -> /odometry/filtered
  * dog_toy_finder_motion cmd_vel_mux: /cmd_vel_auto + /cmd_vel_manual ->
    /cmd_vel, with manual drive taking priority over autonomous motion
  * dog_toy_finder_diagnostics health monitor: logs an ERROR on /rosout when
    a Pico sensor, a camera, or the fused odometry stops publishing

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
    ... health_monitor:=false     # skip the sensor heartbeat monitor
    ... health_imu_timeout:=1.0 health_left_camera_timeout:=5.0
    ... cmd_vel_mux:=false        # only when an external mux owns /cmd_vel
    ... cmd_vel_manual_timeout:=0.5 cmd_vel_release_timeout:=0.5

All launch arguments (bringup, stereo, vision, mapping, web map, and the ROSA
environment) are collected in `scripts/full_system_eval.conf` for full-system
evaluation runs; `scripts/launch_all.sh` reads that file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    health_monitor = LaunchConfiguration("health_monitor")
    cmd_vel_mux = LaunchConfiguration("cmd_vel_mux")

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
                "health_monitor",
                default_value="true",
                description="Start the sensor heartbeat health monitor",
            ),
            DeclareLaunchArgument(
                "cmd_vel_mux",
                default_value="true",
                description="Start cmd_vel_mux (manual priority over autonomous)",
            ),
            DeclareLaunchArgument(
                "cmd_vel_manual_timeout",
                default_value="0.5",
                description="Seconds without a manual command before it expires",
            ),
            DeclareLaunchArgument(
                "cmd_vel_auto_timeout",
                default_value="0.5",
                description="Seconds without an autonomous command before it expires",
            ),
            DeclareLaunchArgument(
                "cmd_vel_release_timeout",
                default_value="0.5",
                description="Seconds of zero after manual control ends",
            ),
            DeclareLaunchArgument(
                "cmd_vel_publish_rate",
                default_value="20.0",
                description="Command publish rate to the Pico in Hz",
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
            # Health-monitor heartbeat timeouts, forwarded to the
            # dog_toy_finder_diagnostics node below.
            DeclareLaunchArgument(
                "health_check_period",
                default_value="0.5",
                description="Seconds between sensor heartbeat checks",
            ),
            DeclareLaunchArgument(
                "health_startup_grace",
                default_value="10.0",
                description="Seconds to wait for a sensor's first message",
            ),
            DeclareLaunchArgument(
                "health_imu_timeout",
                default_value="0.5",
                description="Seconds without /imu/data before failure",
            ),
            DeclareLaunchArgument(
                "health_joint_states_timeout",
                default_value="0.5",
                description="Seconds without /joint_states before failure",
            ),
            DeclareLaunchArgument(
                "health_ir_timeout",
                default_value="1.0",
                description="Seconds without /ir_obstacle/detected before failure",
            ),
            DeclareLaunchArgument(
                "health_left_camera_timeout",
                default_value="3.0",
                description="Seconds without left images before failure",
            ),
            DeclareLaunchArgument(
                "health_right_camera_timeout",
                default_value="3.0",
                description="Seconds without right images before failure",
            ),
            DeclareLaunchArgument(
                "health_odometry_timeout",
                default_value="2.0",
                description="Seconds without /odometry/filtered before failure",
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
            # The only publisher on the Pico's /cmd_vel topic. Manual commands
            # win over autonomous ones, so two controllers cannot fight.
            Node(
                package="dog_toy_finder_motion",
                executable="cmd_vel_mux",
                name="cmd_vel_mux",
                output="screen",
                condition=IfCondition(cmd_vel_mux),
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {
                        "manual_timeout": ParameterValue(
                            LaunchConfiguration("cmd_vel_manual_timeout"),
                            value_type=float,
                        ),
                        "auto_timeout": ParameterValue(
                            LaunchConfiguration("cmd_vel_auto_timeout"),
                            value_type=float,
                        ),
                        "release_timeout": ParameterValue(
                            LaunchConfiguration("cmd_vel_release_timeout"),
                            value_type=float,
                        ),
                        "publish_rate": ParameterValue(
                            LaunchConfiguration("cmd_vel_publish_rate"),
                            value_type=float,
                        ),
                    },
                ],
            ),
            Node(
                package="dog_toy_finder_diagnostics",
                executable="health_monitor",
                name="health_monitor",
                output="screen",
                condition=IfCondition(health_monitor),
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
                    {
                        "check_period": ParameterValue(
                            LaunchConfiguration("health_check_period"),
                            value_type=float,
                        ),
                        "startup_grace": ParameterValue(
                            LaunchConfiguration("health_startup_grace"),
                            value_type=float,
                        ),
                        "imu_timeout": ParameterValue(
                            LaunchConfiguration("health_imu_timeout"),
                            value_type=float,
                        ),
                        "joint_states_timeout": ParameterValue(
                            LaunchConfiguration("health_joint_states_timeout"),
                            value_type=float,
                        ),
                        "ir_timeout": ParameterValue(
                            LaunchConfiguration("health_ir_timeout"),
                            value_type=float,
                        ),
                        "left_camera_timeout": ParameterValue(
                            LaunchConfiguration("health_left_camera_timeout"),
                            value_type=float,
                        ),
                        "right_camera_timeout": ParameterValue(
                            LaunchConfiguration("health_right_camera_timeout"),
                            value_type=float,
                        ),
                        "odometry_timeout": ParameterValue(
                            LaunchConfiguration("health_odometry_timeout"),
                            value_type=float,
                        ),
                    },
                ],
            ),
        ]
    )
