"""Build the ROSA agent that drives the Dog Toy Finder.

The agent is intentionally thin: it wires an Ollama model to the tools in
``tools.py`` and the prompts in ``prompts.py``. ROS 2 details stay in
``robot.py`` and the ``dog_toy_finder_motion`` package.
"""

import asyncio
import json
import os
import urllib.request

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


def unload_model(model: str) -> bool:
    """Ask Ollama to evict a model so the next comparison gets all the VRAM.

    Returns True when the unload request completed. Ollama keeps models
    resident by default, so a comparison that switches models can otherwise
    have the next model load beside the previous one and run with less VRAM.
    """
    body = json.dumps({"model": model, "keep_alive": 0}).encode()
    request = urllib.request.Request(
        ollama_url().rstrip("/") + "/api/generate", data=body,
        headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=15) as reply:
            reply.read()
    except Exception:
        return False
    return True


def build_agent(model: str = None, streaming: bool = False) -> ROSA:
    """Create the ROSA agent with the robot's motion tools.

    ``streaming`` must be true when the caller wants the tool-call events that
    ``invoke_with_events`` captures for the planning comparison CSV.
    """
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
            tools.run_search,
            tools.check_failsafe,
            tools.stop_robot,
            vision_tools.describe_scene,
            vision_tools.find_toy,
            vision_tools.is_path_clear,
        ],
        prompts=build_prompts(),
        verbose=True,
        streaming=streaming,
        accumulate_chat_history=False,
        max_iterations=10,
    )


def _tool_value(value):
    """Make a tool input/output JSON-friendly without losing its information."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, dict):
        return {str(key): _tool_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_tool_value(item) for item in value]
    text = getattr(value, "content", None)
    return text if isinstance(text, str) else json.dumps(value, default=str)


def invoke_with_events(agent, prompt):
    """Run one prompt while collecting the reply and every tool call.

    ROSA's ``invoke`` returns only the final text. The streaming API also
    emits ``tool_start``/``tool_end`` events, which are what the planning trial
    CSV needs to score whether a model chose the expected action. Returns
    ``(reply, tool_calls, error)``.
    """

    async def collect():
        reply = ""
        calls = []
        error = ""
        async for event in agent.astream(prompt):
            kind = event.get("type")
            if kind == "token":
                reply += event.get("content", "")
            elif kind == "tool_start":
                calls.append({
                    "name": event.get("name", ""),
                    "input": _tool_value(event.get("input")),
                    "output": "",
                })
            elif kind == "tool_end":
                output = _tool_value(event.get("output"))
                name = event.get("name", "")
                for call in reversed(calls):
                    if call["name"] == name and not call["output"]:
                        call["output"] = output
                        break
                else:
                    calls.append({"name": name, "input": "", "output": output})
            elif kind == "final":
                reply = event.get("content") or reply
            elif kind == "error":
                error = event.get("content", "ROSA error")
        return reply.strip(), calls, error

    return asyncio.run(collect())
