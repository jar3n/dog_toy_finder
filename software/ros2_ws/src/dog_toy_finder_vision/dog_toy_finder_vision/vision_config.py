"""Load and validate the shared rectified-stereo detector configuration.

The live node, capture tool, and offline evaluator read the same YAML file, so
the physical measurements in the VLM prompt match the calibrated intrinsics,
baseline, valid depth range, and the toy's real-world size.

The path is normally supplied by ``scripts/full_system_eval.conf`` as
``VISION_DETECTOR_CONFIG`` and forwarded through ``stereo_vision.launch.py``.
"""

import copy
import math
import os

try:
    import yaml
except ImportError as error:  # pragma: no cover - dependency is installed in Docker
    raise ImportError(
        "vision_config needs PyYAML; install python3-yaml") from error


_PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG_PATH = os.path.join(
    _PACKAGE_DIR, "config", "vision_detectors.yaml")

# Minimal defaults so a partial user config still loads. The committed YAML is
# the authoritative, fully commented example.
DEFAULTS = {
    "camera": {
        "width": 320,
        "height": 240,
        "fx": 250.0,
        "fy": 250.0,
        "cx": 160.0,
        "cy": 120.0,
        "baseline_m": 0.08,
        "depth_min_m": 0.25,
        "depth_max_m": 3.0,
        "left_rectified_topic": "/stereo/left/image_rect_color",
        "right_rectified_topic": "/stereo/right/image_rect_color",
    },
    "target": {
        "known_width_m": 0.075,
        "known_height_m": 0.075,
        "position_split_left": 0.333,
        "position_split_right": 0.667,
        "position_bearing_deg": 15.0,
    },
    "ml": {
        "enabled": True,
        "include_physical_summary": True,
    },
}


def _deep_update(base, updates):
    """Recursively merge ``updates`` into a copy of ``base``."""
    result = copy.deepcopy(base)
    for key, value in (updates or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_update(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def resolve_config_path(path=None):
    """Return an existing config path, trying the installed share directory."""
    candidates = []
    if path:
        candidates.append(os.path.expanduser(str(path)))
    candidates.append(DEFAULT_CONFIG_PATH)
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory("dog_toy_finder_vision"),
            "config", "vision_detectors.yaml"))
    except Exception:  # ament index is unavailable outside a sourced ROS install
        pass
    for candidate in candidates:
        if candidate and os.path.isfile(candidate):
            return candidate
    raise FileNotFoundError(
        "Vision detector config not found; tried: %s" % ", ".join(candidates))


def default_capture_dir():
    """Return the capture root used by both the capture and replay tools.

    ``VISION_CAPTURE_DIR`` wins; otherwise an older ``VISION_TRIAL_DIR`` maps to
    its ``captures`` child. Outside those overrides the repository's
    ``data/vision/captures`` directory is used when the checkout is visible.
    """
    override = os.environ.get("VISION_CAPTURE_DIR")
    if override:
        return os.path.abspath(os.path.expanduser(override))
    trial_dir = os.environ.get("VISION_TRIAL_DIR")
    if trial_dir:
        return os.path.join(os.path.abspath(os.path.expanduser(trial_dir)),
                            "captures")
    # Source checkout: <repo>/software/ros2_ws/src/<pkg>/<pkg>/this_file.py
    source_root = os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", ".."))
    candidate = os.path.join(source_root, "data", "vision", "captures")
    if os.path.isdir(os.path.dirname(candidate)):
        return candidate
    return os.path.join(os.getcwd(), "data", "vision", "captures")


def load_config(path=None):
    """Load the detector YAML and fill missing sections with defaults."""
    config_path = resolve_config_path(path)
    with open(config_path) as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, dict):
        raise ValueError("Vision detector config must be a YAML object")
    config = _deep_update(DEFAULTS, loaded)
    config["config_path"] = config_path
    validate_config(config)
    return config


def validate_config(config):
    """Raise ValueError when a required physical measurement is unusable."""
    camera = config.get("camera", {})
    for field in ("width", "height"):
        if int(camera.get(field, 0)) <= 0:
            raise ValueError("camera.%s must be positive" % field)
    for field in ("fx", "fy", "baseline_m"):
        if float(camera.get(field, 0.0)) <= 0.0:
            raise ValueError("camera.%s must be positive" % field)
    minimum = float(camera.get("depth_min_m", 0.0))
    maximum = float(camera.get("depth_max_m", 0.0))
    if minimum <= 0.0 or maximum <= minimum:
        raise ValueError("camera.depth_min_m/depth_max_m are inconsistent")
    target = config.get("target", {})
    if float(target.get("known_width_m", 0.0)) <= 0.0:
        raise ValueError("target.known_width_m must be measured and positive")
    return config


def get(config, *keys, default=None):
    """Read a nested config value without raising on a missing section."""
    value = config
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def physical_summary(config):
    """One-line measurement summary appended to the ML prompt."""
    if not get(config, "ml", "include_physical_summary", default=True):
        return ""
    camera = config.get("camera", {})
    target = config.get("target", {})
    return (
        "Camera measurements: rectified %dx%d images, fx=%.1f px, "
        "fy=%.1f px, baseline=%.3f m, valid depth %.2f-%.2f m. "
        "The requested toy is about %.3f m wide and %.3f m tall. "
        "Report lateral_bearing_deg from the optical axis in degrees, "
        "positive when the toy is to the robot's left (image x smaller "
        "than cx=%.1f px), negative when it is to the right."
        % (int(camera.get("width", 0)), int(camera.get("height", 0)),
           float(camera.get("fx", 0.0)), float(camera.get("fy", 0.0)),
           float(camera.get("baseline_m", 0.0)),
           float(camera.get("depth_min_m", 0.0)),
           float(camera.get("depth_max_m", 0.0)),
           float(target.get("known_width_m", 0.0)),
           float(target.get("known_height_m", 0.0)),
           float(camera.get("cx", 0.0)))
    )


# --- Lateral bearing -------------------------------------------------------
#
# Bearing is measured from the optical axis, in degrees, positive to the
# robot's left. In a rectified image a point at column u has
#
#     bearing = atan((cx - u) / fx)
#
# which is positive for u < cx (the left half of the image) and matches the
# ROS convention that +y is left. The old left/center/right label is retained
# only as a compatibility view of this number.


def lateral_bearing_deg(u_px, config, image_width=None):
    """Return the numeric lateral bearing of image column ``u_px`` in degrees."""
    camera = config.get("camera", {}) if isinstance(config, dict) else {}
    width = float(image_width or camera.get("width", 320))
    cx = float(camera.get("cx", width / 2.0))
    fx = float(camera.get("fx", 250.0))
    if fx <= 0.0:
        return 0.0
    bearing = math.degrees(math.atan((cx - float(u_px)) / fx))
    return round(max(-90.0, min(90.0, bearing)), 2)


def position_bearing_deg(config=None):
    """Fallback bearing magnitude for the legacy left/right labels."""
    magnitude = float(get(config or {}, "target", "position_bearing_deg",
                          default=15.0))
    return abs(magnitude)


def bearing_from_position(position, config=None):
    """Convert a legacy left/center/right label to an approximate bearing."""
    label = str(position or "none").strip().lower()
    magnitude = position_bearing_deg(config)
    return {"left": magnitude, "center": 0.0, "right": -magnitude,
            "none": 0.0}.get(label, 0.0)


def position_from_bearing(bearing_deg, config=None, image_width=None):
    """Derive the compatibility left/center/right label from a bearing."""
    try:
        bearing = float(bearing_deg)
    except (TypeError, ValueError):
        bearing = 0.0
    if not isinstance(config, dict) or not config.get("camera"):
        edge = position_bearing_deg(config)
        if bearing > edge:
            return "left"
        if bearing < -edge:
            return "right"
        return "center"
    camera = config["camera"]
    width = float(image_width or camera.get("width", 320))
    left_edge = lateral_bearing_deg(
        float(get(config, "target", "position_split_left",
                  default=0.333)) * width, config, width)
    right_edge = lateral_bearing_deg(
        float(get(config, "target", "position_split_right",
                  default=0.667)) * width, config, width)
    if bearing >= left_edge:
        return "left"
    if bearing <= right_edge:
        return "right"
    return "center"
