"""Combined stereo-vision launch: cameras -> rectification -> detectors.

This single entry point replaces the old ``stereo.launch.py`` and
``vision.launch.py``. It starts, in order:

  1. ``stereo_camera_node`` -- downloads one JPEG per ESP32-CAM eye and
     publishes ``image_raw`` (rgb8), ``image_mono`` (mono8) and ``camera_info``
     under ``/stereo/{left,right}``. The eyes are ordinary colour cameras, so
     no debayer stage is needed.
  2. ``image_proc`` rectify nodes per eye -- ``image_mono`` -> ``image_rect``
     and ``image_raw`` -> ``image_rect_color``. ``queue_size`` is raised well
     above the default because the two ESP32-CAMs are not hardware
     synchronised and frames arrive in bursts.
  3. ``stereo_image_proc`` -- disparity and point cloud from the rectified
     pair, using approximate sync.
  4. ``vision_node`` -- answers ``/vision/ask`` with the selected vision
     pipeline over the same rectified colour pair.

The stereo stages and the vision node can be switched off independently
(``stereo``, ``stereo_proc``, ``vision``) so the same launch file serves
depth-only debugging and the full dog-toy-finder stack.

Run inside the ROS 2 container:
    ros2 launch dog_toy_finder_vision stereo_vision.launch.py
    ros2 launch dog_toy_finder_vision stereo_vision.launch.py \\
        pipeline:=open_vocab
    ros2 launch dog_toy_finder_vision stereo_vision.launch.py vision:=false

``pipeline`` falls back to ``$VISION_PIPELINE``, then the detector YAML's
``pipeline`` key, then ``open_vocab``. ``second`` is reserved for a pipeline
that has not been described yet. ``open_vocab`` sends rectified JPEGs to the
host-side detector service (``scripts/start_yolo_service.sh``); torch and
weights are not installed in this container.

The default capture size is 320x240. At 640x480 each colour frame is ~900 KB
and the host's DDS socket buffers drop a large fraction of them, which makes
image_proc log "do not appear to be synchronized" and stalls the point cloud;
320x240 keeps the frames small enough to arrive reliably. If you raise
width/height, scale fx, fy, cx and cy with them.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


DEFAULT_DETECTOR_CONFIG = os.path.join(
    get_package_share_directory("dog_toy_finder_vision"),
    "config", "vision_detectors.yaml")


def generate_launch_description():
    left_url = LaunchConfiguration("left_url")
    right_url = LaunchConfiguration("right_url")
    rate = LaunchConfiguration("rate")
    stereo_enabled = LaunchConfiguration("stereo")
    stereo_proc_enabled = LaunchConfiguration("stereo_proc")
    vision_enabled = LaunchConfiguration("vision")

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

    def side_nodes(namespace):
        return [
            Node(
                package="image_proc",
                executable="rectify_node",
                name="rectify_mono_node",
                namespace=namespace,
                output="screen",
                parameters=[{"queue_size": 50}],
                remappings=[("image", "image_mono"),
                            ("image_rect", "image_rect")],
            ),
            Node(
                package="image_proc",
                executable="rectify_node",
                name="rectify_color_node",
                namespace=namespace,
                output="screen",
                parameters=[{"queue_size": 50}],
                remappings=[("image", "image_raw"),
                            ("image_rect", "image_rect_color")],
            ),
        ]

    # stereo_image_proc provides disparity + point cloud from rectified pairs.
    # The stock launch file leaves its outputs as /disparity and /points2, so
    # they are remapped explicitly to the /stereo/... names the rest of the
    # stack (map, docs, tests) expects. Its own image_proc nodes are not used:
    # this file already rectifies the images, and the cameras are colour
    # (rgb8/mono8), so a debayer stage would drop every frame.
    disparity_node = ComposableNode(
        package="stereo_image_proc",
        plugin="stereo_image_proc::DisparityNode",
        name="disparity_node",
        parameters=[{
            "approximate_sync": LaunchConfiguration("approximate_sync"),
            "use_system_default_qos": False,
            "stereo_algorithm": LaunchConfiguration("stereo_algorithm"),
            "min_disparity": LaunchConfiguration("min_disparity"),
            "disparity_range": LaunchConfiguration("disparity_range"),
        }],
        remappings=[
            ("left/image_rect", "/stereo/left/image_rect"),
            ("left/camera_info", "/stereo/left/camera_info"),
            ("right/image_rect", "/stereo/right/image_rect"),
            ("right/camera_info", "/stereo/right/camera_info"),
            ("disparity", "/stereo/disparity"),
        ],
    )
    point_cloud_node = ComposableNode(
        package="stereo_image_proc",
        plugin="stereo_image_proc::PointCloudNode",
        name="point_cloud_node",
        parameters=[{
            "approximate_sync": LaunchConfiguration("approximate_sync"),
            "use_system_default_qos": False,
            "use_color": LaunchConfiguration("use_color"),
        }],
        remappings=[
            ("disparity", "/stereo/disparity"),
            ("left/camera_info", "/stereo/left/camera_info"),
            ("right/camera_info", "/stereo/right/camera_info"),
            ("left/image_rect_color", "/stereo/left/image_rect_color"),
            ("points2", "/stereo/points2"),
        ],
    )
    stereo_proc = ComposableNodeContainer(
        name="stereo_image_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        condition=IfCondition(stereo_proc_enabled),
        composable_node_descriptions=[disparity_node, point_cloud_node],
    )

    stereo_group = GroupAction(
        [
            stereo_camera,
            *side_nodes("/stereo/left"),
            *side_nodes("/stereo/right"),
            stereo_proc,
        ],
        condition=IfCondition(stereo_enabled),
    )

    vision_node = Node(
        package="dog_toy_finder_vision",
        executable="vision_node",
        name="vision_node",
        output="screen",
        condition=IfCondition(vision_enabled),
        parameters=[{
            "config_path": LaunchConfiguration("config_path"),
            "pipeline": LaunchConfiguration("pipeline"),
            "yolo_service_url": LaunchConfiguration("yolo_service_url"),
        }],
    )

    return LaunchDescription([
        # --- camera / stereo geometry ---
        DeclareLaunchArgument("left_url",
                              default_value="http://esp32cam-left.local/capture",
                              description="Left eye /capture URL"),
        DeclareLaunchArgument("right_url",
                              default_value="http://esp32cam-right.local/capture",
                              description="Right eye /capture URL"),
        DeclareLaunchArgument("rate", default_value="2.0",
                              description="Photos per second per eye"),
        DeclareLaunchArgument("width", default_value="320",
                              description="Image width (320 keeps rgb8 frames small enough for reliable delivery)"),
        DeclareLaunchArgument("height", default_value="240",
                              description="Image height"),
        DeclareLaunchArgument("fx", default_value="250.0", description="Focal length x (px)"),
        DeclareLaunchArgument("fy", default_value="250.0", description="Focal length y (px)"),
        DeclareLaunchArgument("cx", default_value="160.0", description="Principal point x (px)"),
        DeclareLaunchArgument("cy", default_value="120.0", description="Principal point y (px)"),
        DeclareLaunchArgument("baseline", default_value="0.08",
                              description="Distance between optical centres (m)"),

        # --- stage switches ---
        DeclareLaunchArgument("stereo", default_value="true",
                              description="Start the cameras, rectification and depth pipeline"),
        DeclareLaunchArgument("stereo_proc", default_value="true",
                              description="Start disparity + point cloud (needs stereo:=true)"),
        DeclareLaunchArgument("vision", default_value="true",
                              description="Start the vision_node detector server"),
        DeclareLaunchArgument("config_path",
                              default_value=DEFAULT_DETECTOR_CONFIG,
                              description="YAML with stereo geometry and pipeline settings"),
        DeclareLaunchArgument("pipeline", default_value="",
                              description="Vision pipeline: open_vocab|second "
                                          "(default: $VISION_PIPELINE / YAML / open_vocab)"),
        DeclareLaunchArgument("yolo_service_url", default_value="",
                              description="Host YOLO detector service URL "
                                          "(default: $YOLO_SERVICE_URL / YAML / http://localhost:5006)"),

        # --- stereo_image_proc tuning ---
        DeclareLaunchArgument("approximate_sync", default_value="true",
                              description="Approximate sync (eyes are not hardware-synced)"),
        DeclareLaunchArgument("use_color", default_value="true",
                              description="Build the point cloud with RGB data"),
        DeclareLaunchArgument("stereo_algorithm", default_value="0",
                              description="0 = block matching, 1 = semi-global block matching"),
        DeclareLaunchArgument("min_disparity", default_value="0",
                              description="Disparity to begin searching at (px)"),
        DeclareLaunchArgument("disparity_range", default_value="64",
                              description="Number of disparities to search (multiple of 16)"),

        stereo_group,
        vision_node,
    ])
