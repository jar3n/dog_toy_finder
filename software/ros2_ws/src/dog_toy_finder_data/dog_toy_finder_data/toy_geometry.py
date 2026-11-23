"""Pure geometry helpers for acting on a numeric lateral bearing.

The vision stack reports ``lateral_bearing_deg`` (degrees from the optical
axis, positive to the robot's left). Mapping, homing, and the experiment
harness use these helpers so those three consumers stay consistent. The legacy
left/center/right label is only a fallback for older results.
"""

import math

# Approximate bearing represented by the legacy labels when no numeric bearing
# is present. Matches the detector fallback in vision_detectors.yaml.
LEGACY_BEARING_DEG = 15.0


def bearing_deg(entry, default=0.0):
    """Read a lateral bearing from a vision result entry."""
    if isinstance(entry, (int, float)) and not isinstance(entry, bool):
        return float(entry)
    if not isinstance(entry, dict):
        return float(default)
    for key in ("lateral_bearing_deg", "bearing_deg", "lateral_bearing"):
        try:
            value = float(entry.get(key))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value):
            return max(-90.0, min(90.0, value))
    position = str(entry.get("position", "center")).strip().lower()
    return float({
        "left": LEGACY_BEARING_DEG,
        "center": 0.0,
        "right": -LEGACY_BEARING_DEG,
    }.get(position, default))


def place(x, y, yaw, distance_m, lateral_bearing_deg):
    """Place a toy in the odom frame from forward distance and bearing.

    ROS convention: +x forward, +y left, yaw counter-clockwise. Positive
    bearing puts the toy to the robot's left at the robot's current pose.
    """
    distance = float(distance_m)
    bearing = math.radians(float(lateral_bearing_deg))
    ahead = distance * math.cos(bearing)
    side = distance * math.sin(bearing)
    return {
        "x": x + ahead * math.cos(yaw) - side * math.sin(yaw),
        "y": y + ahead * math.sin(yaw) + side * math.cos(yaw),
        "ahead_m": ahead,
        "side_m": side,
    }


def home_turn_rad(entry_or_bearing, max_bearing_deg=45.0):
    """Turn angle that points the robot at a visible toy.

    Uses the actual bearing rather than a fixed ±15° from a label. The turn is
    clamped so a badly estimated bearing cannot spin the robot in place.
    """
    value = bearing_deg(entry_or_bearing)
    limit = abs(float(max_bearing_deg))
    return math.radians(max(-limit, min(limit, value)))
