"""Pure (non-ROS) vision answer normalisation shared by both pipelines.

The live ``vision_node`` and the offline ``vision_eval`` replay use this module
so a saved capture is interpreted exactly the same way as a live photo.

Lateral direction is reported only as the numeric ``lateral_bearing_deg``:
degrees from the optical axis, positive to the robot's left.
"""

import math


def as_bool(value, default=False):
    """Accept booleans and the usual string spellings."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("true", "1", "yes", "y", "on")
    return bool(value) if value is not None else default


def _as_float(value):
    """Return a finite float, or None for a missing/non-numeric value."""
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def tidy(answer, config=None):
    """Normalise an answer while preserving pipeline-specific extra fields.

    The numeric bearing is clamped to +/-90 degrees and rounded; a missing or
    non-numeric bearing defaults to zero. ``center_offset_m`` is preserved when
    supplied, otherwise derived from optical-axis distance and bearing.
    """
    result = dict(answer)
    result.pop("position", None)
    result.pop("size", None)
    obstacle = str(result.get("obstacle_ahead") or "").strip().strip(".")
    if obstacle.lower() in ("", "none", "nothing", "no", "n/a"):
        obstacle = "none"
    result["obstacle_ahead"] = obstacle
    result["path_clear"] = obstacle == "none"
    result["target_visible"] = as_bool(result.get("target_visible"), False)

    distance = _as_float(result.get("distance_m"))
    result["distance_m"] = max(0.0, distance) if distance is not None else 0.0

    bearing = _as_float(result.get("lateral_bearing_deg"))
    if bearing is None:
        bearing = 0.0
    bearing = max(-90.0, min(90.0, bearing))
    result["lateral_bearing_deg"] = round(bearing, 2)

    if not result["target_visible"]:
        result["distance_m"] = 0.0
        result["lateral_bearing_deg"] = 0.0

    offset = _as_float(result.get("center_offset_m"))
    if offset is None:
        offset = (result["distance_m"] *
                  math.tan(math.radians(result["lateral_bearing_deg"])))
    if not result["target_visible"]:
        offset = 0.0
    result["center_offset_m"] = round(offset, 3)
    return result
