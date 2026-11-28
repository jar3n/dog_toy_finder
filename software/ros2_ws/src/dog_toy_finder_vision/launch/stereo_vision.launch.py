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
depth-only debugging and the full dog-toy-finder stack. Dataset collection uses
``vision:=false stereo_proc:=false``; ``vision_capture`` is a separate offline
tool and must not be run as part of the robot's live vision processing.

Run inside the ROS 2 container:
    ros2 launch dog_toy_finder_vision stereo_vision.launch.py
    ros2 launch dog_toy_finder_vision stereo_vision.launch.py \\
        pipeline:=triangulation
    ros2 launch dog_toy_finder_vision stereo_vision.launch.py vision:=false

``pipeline`` falls back to ``$VISION_PIPELINE``, then the detector YAML's
``pipeline`` key, then ``triangulation``. ``triangulation`` sends rectified
JPEGs to the host-side detector service (``scripts/start_yolo_service.sh``).
``stereo-net`` detects in the left image only and asks the host dense-stereo
service (``scripts/start_stereo_service.sh``) for depth. Torch, models, and
weights run
on the computer, never in this container.

Camera geometry is read from ``config_path`` (``vision_detectors.yaml`` by
default). The ``width``/``height``/``baseline`` and per-eye
``left_fx``/``left_fy``/``left_cx``/``left_cy`` /
``right_fx``/``right_fy``/``right_cx``/``right_cy`` launch arguments are
optional overrides; when left empty the YAML values are used. The default
capture size is 320x240 because at 640x480 each colour frame is ~900 KB and the
host's DDS socket buffers drop enough of them to stall the point cloud. If you
raise width/height in the YAML, scale both eyes' intrinsics with them.

``calibration_file`` may point at the JSON written by
``scripts/calibrate_stereo.py``. When set, ``stereo_camera_node`` publishes the
solved per-eye K/D/R/P matrices instead of assuming one shared K and identity
rotation; this is required when the two eyes have different principal points or
a non-horizontal baseline.
"""

import os

from ament_index_python.packages import get_package_share_directory
from dog_toy_finder_vision import vision_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


DEFAULT_DETECTOR_CONFIG = os.path.join(
    get_package_share_directory("dog_toy_finder_vision"),
    "config", "vision_detectors.yaml")


def _arg(context, name):
    """Return the resolved value of a declared launch argument."""
    return LaunchConfiguration(name).perform(context)


def _default_if_empty(value, fallback):
    """Use the YAML value unless the launch argument was given explicitly."""
    text = str(value).strip()
    return text if text else str(fallback)


def _bool_arg(context, name):
    """Resolve a launch argument as a real bool, not the string 'true'."""
    value = str(_arg(context, name)).strip().lower()
    if value in ("true", "1", "yes", "on"):
        return True
    if value in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"launch argument {name} must be a boolean, got {value!r}")


def _launch_setup(context):
    """Build the nodes after launch args are known, so YAML can fill defaults."""
    config = vision_config.load_config(_arg(context, "config_path") or None)
    camera = config["camera"]

    def camera_arg(name, value):
        return _default_if_empty(_arg(context, name), value)

    def eye_arg(side, field):
        name = "%s_%s" % (side, field)
        return camera_arg(
            name, vision_config.eye_intrinsics(config, side)[field])

    left_url = _arg(context, "left_url")
    right_url = _arg(context, "right_url")
    rate = _arg(context, "rate")
    width = camera_arg("width", camera["width"])
    height = camera_arg("height", camera["height"])
    intrinsics = {
        "%s_%s" % (side, field): eye_arg(side, field)
        for side in ("left", "right")
        for field in ("fx", "fy", "cx", "cy")
    }
    baseline = camera_arg("baseline", camera["baseline_m"])
    calibration_file = _arg(context, "calibration_file")

    stereo_camera = Node(
        package="dog_toy_finder_vision",
        executable="stereo_camera_node",
        name="stereo_camera_node",
        output="screen",
        parameters=[{
            "left_url": left_url,
            "right_url": right_url,
            "rate": float(rate),
            "width": int(width),
            "height": int(height),
            **{name: float(value) for name, value in intrinsics.items()},
            "baseline": float(baseline),
            "calibration_file": calibration_file,
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
            "approximate_sync": _bool_arg(context, "approximate_sync"),
            "use_system_default_qos": False,
            "stereo_algorithm": int(_arg(context, "stereo_algorithm")),
            "min_disparity": int(_arg(context, "min_disparity")),
            "disparity_range": int(_arg(context, "disparity_range")),
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
            "approximate_sync": _bool_arg(context, "approximate_sync"),
            "use_system_default_qos": False,
            "use_color": _bool_arg(context, "use_color"),
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
        condition=IfCondition(_arg(context, "stereo_proc")),
        composable_node_descriptions=[disparity_node, point_cloud_node],
    )

    stereo_group = GroupAction(
        [
            stereo_camera,
            *side_nodes("/stereo/left"),
            *side_nodes("/stereo/right"),
            stereo_proc,
        ],
        condition=IfCondition(_arg(context, "stereo")),
    )

    vision_node = Node(
        package="dog_toy_finder_vision",
        executable="vision_node",
        name="vision_node",
        output="screen",
        condition=IfCondition(_arg(context, "vision")),
        parameters=[{
            "config_path": _arg(context, "config_path"),
            "pipeline": _arg(context, "pipeline"),
            "yolo_service_url": _arg(context, "yolo_service_url"),
            "stereo_service_url": _arg(context, "stereo_service_url"),
        }],
    )

    return [stereo_group, vision_node]


def generate_launch_description():
    return LaunchDescription([
        # --- camera / stereo geometry (geometry defaults come from the YAML) ---
        DeclareLaunchArgument("left_url",
                              default_value="http://esp32cam-left.local/capture",
                              description="Left eye /capture URL"),
        DeclareLaunchArgument("right_url",
                              default_value="http://esp32cam-right.local/capture",
                              description="Right eye /capture URL"),
        DeclareLaunchArgument("rate", default_value="2.0",
                              description="Photos per second per eye"),
        DeclareLaunchArgument("width", default_value="",
                              description="Image width (default: camera.width in config_path)"),
        DeclareLaunchArgument("height", default_value="",
                              description="Image height (default: camera.height in config_path)"),
        DeclareLaunchArgument("left_fx", default_value="",
                              description="Left focal length x in px (default: camera.left.fx in config_path)"),
        DeclareLaunchArgument("left_fy", default_value="",
                              description="Left focal length y in px (default: camera.left.fy in config_path)"),
        DeclareLaunchArgument("left_cx", default_value="",
                              description="Left principal point x in px (default: camera.left.cx in config_path)"),
        DeclareLaunchArgument("left_cy", default_value="",
                              description="Left principal point y in px (default: camera.left.cy in config_path)"),
        DeclareLaunchArgument("right_fx", default_value="",
                              description="Right focal length x in px (default: camera.right.fx in config_path)"),
        DeclareLaunchArgument("right_fy", default_value="",
                              description="Right focal length y in px (default: camera.right.fy in config_path)"),
        DeclareLaunchArgument("right_cx", default_value="",
                              description="Right principal point x in px (default: camera.right.cx in config_path)"),
        DeclareLaunchArgument("right_cy", default_value="",
                              description="Right principal point y in px (default: camera.right.cy in config_path)"),
        DeclareLaunchArgument("baseline", default_value="",
                              description="Optical centre separation in m (default: camera.baseline_m in config_path)"),
        DeclareLaunchArgument("calibration_file", default_value="",
                              description="JSON from scripts/calibrate_stereo.py "
                                          "with per-eye K/D/R/P (optional)"),

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
                              description="Vision pipeline: triangulation|stereo-net "
                                          "(default: $VISION_PIPELINE / YAML / triangulation)"),
        DeclareLaunchArgument("yolo_service_url", default_value="",
                              description="Host open-vocabulary detector service URL "
                                          "(default: $YOLO_SERVICE_URL / YAML / http://localhost:5006)"),
        DeclareLaunchArgument("stereo_service_url", default_value="",
                              description="Host dense-stereo depth service URL "
                                          "(default: $STEREO_SERVICE_URL / YAML / http://localhost:5007)"),

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

        OpaqueFunction(function=_launch_setup),
    ])
