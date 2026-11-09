"""Build the ROSA agent that drives the Dog Toy Finder.

The agent is intentionally thin: it wires an Ollama model to the tools in
``tools.py`` and the prompts in ``prompts.py``. ROS 2 details stay in
``robot.py`` and the ``dog_toy_finder_motion`` package.
"""

import os

from langchain_ollama import ChatOllama
from rosa import ROSA

from . import tools, vision_tools
from .prompts import build_prompts


def ollama_model_name() -> str:
    """Return the configured Ollama model name."""
    return os.environ.get("OLLAMA_MODEL", "llama3.2")


def ollama_url() -> str:
    """Return the Ollama base URL (localhost works with host networking)."""
    return os.environ.get("OLLAMA_URL", "http://localhost:11434")


def build_agent(model: str = None) -> ROSA:
    """Create the ROSA agent with the robot's motion tools."""
    llm = ChatOllama(
        model=(model if model else ollama_model_name()),
        temperature=0,
        num_ctx=16384,
        base_url=ollama_url(),
    )
    return ROSA(
        ros_version=2,
        llm=llm,
        tools=[
            tools.get_pose,
            tools.drive_distance,
            tools.turn,
            tools.go_to,
            tools.check_failsafe,
            tools.stop_robot,
            vision_tools.describe_scene,
            vision_tools.find_toy,
            vision_tools.is_path_clear,
        ],
        prompts=build_prompts(),
        verbose=True,
        streaming=False,
        accumulate_chat_history=False,
        max_iterations=10,
    )
