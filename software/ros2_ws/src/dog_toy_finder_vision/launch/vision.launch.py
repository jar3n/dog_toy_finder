"""Vision launch: stereo vision node (one or more VLMs).

Assumes the stereo pipeline is already running
(dog_toy_finder_vision stereo.launch.py), which provides
/stereo/{left,right}/image_raw.

Run inside the ROS 2 container:
    ros2 launch dog_toy_finder_vision vision.launch.py
    ros2 launch dog_toy_finder_vision vision.launch.py models:="gemma3,llava,moondream"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

VISION_NODE = "/workspace/software/ros2_ws/src/dog_toy_finder_vision/dog_toy_finder_vision/vision_node.py"


def _launch(context, *args, **kwargs):
    model = LaunchConfiguration("model").perform(context)
    models = LaunchConfiguration("models").perform(context).strip()
    ollama_url = LaunchConfiguration("ollama_url").perform(context)

    cmd = ["python3", VISION_NODE, "--ros-args",
           "-p", "model:=%s" % model,
           "-p", "ollama_url:=%s" % ollama_url]
    # Only pass models:= when non-empty; an empty value is rejected by rcl.
    if models:
        cmd += ["-p", "models:=%s" % models]

    return [ExecuteProcess(output="screen", cmd=cmd)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model", default_value="gemma3",
                              description="single VLM (used when `models` is empty)"),
        DeclareLaunchArgument("models", default_value="",
                              description="comma list of VLMs to compare, e.g. gemma3,llava"),
        DeclareLaunchArgument("ollama_url",
                              default_value="http://localhost:11434",
                              description="Ollama endpoint"),
        OpaqueFunction(function=_launch),
    ])
