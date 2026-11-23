"""Pure (non-ROS) vision answer schema, prompts, and normalisation.

Both the live ``vision_node`` and the offline ``vision_eval`` replay use this
module so a saved capture is interpreted exactly the same way as a live photo.

The numeric ``lateral_bearing_deg`` is the primary lateral measurement: degrees
from the optical axis, positive to the robot's left. ``position`` is kept as a
derived ``left|center|right|none`` label for older consumers.
"""

import math

from . import vision_config


ANSWER_SCHEMA = {
    "type": "object",
    "properties": {
        "description": {"type": "string"},
        "target_visible": {"type": "boolean"},
        "position": {"type": "string", "enum": ["left", "center", "right", "none"]},
        "lateral_bearing_deg": {"type": "number", "minimum": -90, "maximum": 90},
        "size": {"type": "string", "enum": ["small", "medium", "large", "none"]},
        "distance_m": {"type": "number", "minimum": 0},
        "obstacle_ahead": {"type": "string"},
    },
    "required": ["description", "target_visible", "position",
                 "lateral_bearing_deg", "size", "distance_m", "obstacle_ahead"],
}

COMMON = (
    "You are the forward-facing stereo camera of a small robot car that drives on the "
    "floor. You are given TWO RECTIFIED images: the first is the LEFT eye, the second "
    "is the RIGHT eye, captured together. Rectification means matching rows have the "
    "same y coordinate, so horizontal image shift is disparity. Use the pair, the "
    "camera measurements below, and the toy's real size to judge distance in metres, "
    "not just appearance. Answer in this order. "
    "description: {description} "
)

TARGET_DESC = ("one or two sentences naming every object you see, its color, and "
               "whether it is on the left, in the center, or on the right of the image.")
SCENE_DESC = ("two or three sentences naming every object you see, its color, and "
              "whether it is on the left, in the center, or on the right of the image.")

TARGET_PROMPT = COMMON.format(description=TARGET_DESC) + (
    "target_visible: true only if the target ({target}) appears in your description. "
    "position: which third of the image the target's center is in: left, center, or right "
    "(none if not visible). "
    "lateral_bearing_deg: the angle of the target's center from the camera's optical "
    "axis in DEGREES, computed as atan((cx - u) / fx) where u is the target's horizontal "
    "pixel center. Positive means left of the optical axis (image column less than cx), "
    "negative means right; use 0 when the target is centred and 0 if not visible. "
    "size: how wide the target looks compared with the image width: small (less than one "
    "tenth), medium, or large (more than one quarter); none if not visible. "
    "distance_m: your best estimate of the target's distance from the camera in metres; "
    "0 if not visible or unknown. "
    "obstacle_ahead: the name of the object, other than the target, that stands in the "
    "center third of the image where the robot would drive into it if it drove straight "
    "ahead; write none if there is no such object. Walls do not count."
)

SCENE_PROMPT = COMMON.format(description=SCENE_DESC) + (
    "There is no target: set target_visible false, position none, "
    "lateral_bearing_deg 0, size none, distance_m 0. "
    "obstacle_ahead: the name of the object that stands in the center third of the image "
    "where the robot would drive into it if it drove straight ahead; write none if "
    "there is no such object. Walls do not count."
)


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


def prompt_for(target, config=None):
    """Build the VLM prompt for a target (or a scene description)."""
    base = TARGET_PROMPT.format(target=target) if target else SCENE_PROMPT
    summary = vision_config.physical_summary(config or {}) if config else ""
    return base + (" " + summary if summary else "")


def tidy(answer, config=None):
    """Normalise an answer while preserving detector-specific extra fields.

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
