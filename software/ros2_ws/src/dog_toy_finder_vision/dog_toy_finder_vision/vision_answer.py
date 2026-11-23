"""Pure (non-ROS) vision answer normalisation shared by both pipelines.

The live ``vision_node`` and the offline ``vision_eval`` replay use this module
so a saved capture is interpreted exactly the same way as a live photo.

The numeric ``lateral_bearing_deg`` is the primary lateral measurement: degrees
from the optical axis, positive to the robot's left. ``position`` is kept as a
derived ``left|center|right|none`` label for older consumers.
"""

import math

from . import vision_config


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

    A missing bearing is recovered from the legacy label, and the legacy label
    is always re-derived from the bearing so mapping and homing see one
    consistent lateral measurement.
    """
    result = dict(answer)
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
        bearing = vision_config.bearing_from_position(
            result.get("position"), config)
    bearing = max(-90.0, min(90.0, bearing))
    result["lateral_bearing_deg"] = round(bearing, 2)

    if not result["target_visible"]:
        result["position"] = "none"
        result["size"] = "none"
        result["distance_m"] = 0.0
        result["lateral_bearing_deg"] = 0.0
    else:
        result["position"] = vision_config.position_from_bearing(
            bearing, config)
    return result
