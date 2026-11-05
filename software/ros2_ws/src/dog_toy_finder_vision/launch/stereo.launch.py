"""Stereo vision launch: two ESP32-CAM eyes -> disparity + point cloud.

Starts:
  * dog_toy_finder_vision stereo_camera_node (fetches both eyes over Wi-Fi)
  * stereo_image_proc (rectify + disparity + point cloud) in one container

The two ESP32-CAM modules are not hardware-synchronised, so approximate_sync
is on by default.

Run inside the ROS 2 container:
    ros2 launch dog_toy_finder_vision stereo.launch.py \\
        left_url:=http://esp32cam-left.local/capture \\
        right_url:=http://esp32cam-right.local/capture
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    left_url = LaunchConfiguration("left_url")
    right_url = LaunchConfiguration("right_url")
    rate = LaunchConfiguration("rate")

    stereo_camera = Node(
        package="dog_toy_finder_vision",
        executable="stereo_camera_node",
        name="stereo_camera_node",
        output="screen",
        parameters=[{
            "left_url": left_url,
            "right_url": right_url,
            "rate": rate,
            "width": LaunchConfiguration("width"),
            "height": LaunchConfiguration("height"),
            "fx": LaunchConfiguration("fx"),
            "fy": LaunchConfiguration("fy"),
            "cx": LaunchConfiguration("cx"),
            "cy": LaunchConfiguration("cy"),
            "baseline": LaunchConfiguration("baseline"),
        }],
    )

    stereo_proc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("stereo_image_proc"), "/launch/stereo_image_proc.launch.py"
        ]),
        launch_arguments={
            "left_namespace": "stereo/left",
            "right_namespace": "stereo/right",
            "approximate_sync": "True",
            "use_color": "True",
            "launch_image_proc": "True",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("left_url",
                             default_value="http://esp32cam-left.local/capture",
                             description="Left eye /capture URL"),
        DeclareLaunchArgument("right_url",
                             default_value="http://esp32cam-right.local/capture",
                             description="Right eye /capture URL"),
        DeclareLaunchArgument("rate", default_value="2.0",
                             description="Photos per second per eye"),
        DeclareLaunchArgument("width", default_value="640", description="Image width"),
        DeclareLaunchArgument("height", default_value="480", description="Image height"),
        DeclareLaunchArgument("fx", default_value="500.0", description="Focal length x (px)"),
        DeclareLaunchArgument("fy", default_value="500.0", description="Focal length y (px)"),
        DeclareLaunchArgument("cx", default_value="320.0", description="Principal point x (px)"),
        DeclareLaunchArgument("cy", default_value="240.0", description="Principal point y (px)"),
        DeclareLaunchArgument("baseline", default_value="0.08",
                             description="Distance between optical centres (m)"),
        stereo_camera,
        stereo_proc,
    ])
