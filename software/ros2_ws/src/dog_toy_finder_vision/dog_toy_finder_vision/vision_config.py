"""Load and validate the shared rectified-stereo detector configuration.

The live node, capture tool, and offline evaluator read the same YAML file.
It selects the vision pipeline and holds the calibrated intrinsics, baseline,
valid depth range, and open-vocabulary detector settings.

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
    "pipeline": "open_vocab",
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
    },
    "open_vocab": {
        "name": "yolov8s-world",
        "service_url": "http://localhost:5006",
        "timeout_s": 60,
        "confidence_threshold": 0.25,
        "max_detections": 10,
        "min_disparity": 3,
        "max_disparity": 96,
        "max_vertical_delta_px": 20,
    },
    "second": {
        "name": "lite_any_stereo",
        "service_url": "http://localhost:5007",
        "timeout_s": 180,
        "window_radius_px": 3,
        "max_disparity": 192,
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


# --- Lateral bearing -------------------------------------------------------
#
# Bearing is measured from the optical axis, in degrees, positive to the
# robot's left. In a rectified image a point at column u has
#
#     bearing = atan((cx - u) / fx)
#
# which is positive for u < cx (the left half of the image) and matches the
# ROS convention that +y is left.


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
