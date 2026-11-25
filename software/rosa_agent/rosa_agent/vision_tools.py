"""ROSA tools that use the stereo vision node.

These tools ask the vision node (/vision/ask -> /vision/result) and report what
the vision pipelines saw. They work whether one model or several are configured, and
summarise the first model's answer for the planner.
"""

import os

from langchain_core.tools import tool

from .vision import get_vision

MAX_TARGET_LEN = 80


def _bearing_text(entry):
    """Format a numeric lateral bearing, or 'unknown' when it is missing."""
    try:
        return f"{float(entry.get('lateral_bearing_deg')):+.1f}"
    except (TypeError, ValueError):
        return "unknown"


def _distance_text(entry):
    """Format a numeric distance in metres, or 'unknown' when it is missing."""
    try:
        return f"{float(entry.get('distance_m')):.2f}"
    except (TypeError, ValueError):
        return "unknown"


def _summary(result):
    if result is None:
        return "No vision result (is the vision node running?)."
    if result.get("error"):
        return f"Vision error: {result['error']}"
    entries = result.get("results") or []
    if not entries:
        return "Vision returned no answers."
    first = entries[0]
    lines = []
    if len(entries) > 1:
        lines.append("Compared %d detectors/models:" % len(entries))
    for e in entries:
        model = e.get("model", "?")
        if e.get("method"):
            model = "%s/%s" % (e["method"], model)
        if e.get("error"):
            lines.append(f"  {model}: ERROR {e['error']}")
            continue
        desc = e.get("description", "")
        visible = e.get("target_visible", False)
        bearing = _bearing_text(e)
        obstacle = e.get("obstacle_ahead", "none")
        distance = _distance_text(e)
        lines.append(
            f"  {model}: {desc} | target_visible={visible} "
            f"lateral_bearing_deg={bearing} "
            f"obstacle_ahead={obstacle}, ~{distance} m "
            f"({e.get('seconds', 0):.1f}s)")
    if len(entries) == 1:
        e = entries[0]
        if e.get("error"):
            return f"Vision error: {e['error']}"
        path = "clear" if e.get("path_clear") else f"blocked by {e.get('obstacle_ahead')}"
        return (
            f"{e.get('description', '')} Target visible: {e.get('target_visible')}. "
            f"Lateral bearing: {_bearing_text(e)} deg. "
            f"Estimated distance: {_distance_text(e)} m. "
            f"Path ahead: {path}."
        )
    # Multi-model: also give a compact consensus line.
    visible_votes = sum(1 for e in entries if e.get("target_visible"))
    lines.append(f"Consensus: target visible in {visible_votes}/{len(entries)} models.")
    return "\n".join(lines)


@tool
def describe_scene() -> str:
    """Look at the stereo camera and describe what is in front of the robot."""
    result = get_vision().ask("")
    return _summary(result)


@tool
def find_toy(target: str) -> str:
    """Look for a toy and report whether it is visible and where it is.

    target: the toy to look for, e.g. 'red ball' or 'blue bone'.
    """
    target = str(target).strip()[:MAX_TARGET_LEN]
    if not target:
        return "No target given. The robot did not look."
    result = get_vision().ask(target)
    return _summary(result)


@tool
def is_path_clear() -> str:
    """Check whether the path straight ahead is clear of obstacles."""
    result = get_vision().ask("")
    if result is None or result.get("error"):
        return result and result.get("error") or "No vision result."
    obstacle = None
    entries = result.get("results") or []
    if entries:
        obstacle = entries[0].get("obstacle_ahead", "none")
    obstacle = obstacle or "none"
    if obstacle.lower() in ("", "none", "nothing", "no", "n/a"):
        return "Path ahead is clear."
    return f"Path ahead is blocked by {obstacle}."


def close_vision():
    from .vision import close_vision as _close
    _close()
