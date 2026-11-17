"""Load and validate the shared rectified-stereo detector configuration.

Both detector paths read the same YAML file. Keeping the physical measurements
in one place means the classical depth calculation and the ML prompt agree on
intrinsics, baseline, valid range, and the toy's real-world size.

The path is normally supplied by ``scripts/full_system_eval.conf`` as
``VISION_DETECTOR_CONFIG`` and forwarded through ``stereo_vision.launch.py``.
"""

import copy
import os
import re

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
        "default_color": "red",
        "known_width_m": 0.075,
        "known_height_m": 0.075,
        "apparent_width_small_fraction": 0.08,
        "apparent_width_large_fraction": 0.25,
        "position_split_left": 0.333,
        "position_split_right": 0.667,
    },
    "color_ranges": {},
    "classical": {
        "enabled": True,
        "min_confidence": 0.30,
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


def target_color_names(config, target):
    """Return configured colour names found in the target text.

    ``"red ball"`` maps to ``["red"]``; a target with no recognised colour uses
    ``target.default_color`` so the classical path still has a deterministic
    segmentation rule.
    """
    text = " " + re.sub(r"[^a-z0-9]+", " ", str(target or "").lower()) + " "
    ranges = config.get("color_ranges", {}) or {}
    names = [name for name in ranges if re.search(
        r"\b%s\b" % re.escape(str(name).lower()), text)]
    if not names:
        fallback = str(get(config, "target", "default_color", default="red"))
        if fallback.lower() in ranges:
            names = [fallback]
    return names


def color_ranges(config, target):
    """Return all (lower, upper) HSV ranges for the target's colours."""
    ranges = config.get("color_ranges", {}) or {}
    result = []
    for name in target_color_names(config, target):
        for entry in ranges.get(name, []) or []:
            lower = entry.get("lower")
            upper = entry.get("upper")
            if lower is not None and upper is not None:
                result.append((list(lower), list(upper)))
    return result


def physical_summary(config):
    """One-line measurement summary appended to the ML prompt."""
    if not get(config, "ml", "include_physical_summary", default=True):
        return ""
    camera = config.get("camera", {})
    target = config.get("target", {})
    return (
        "Camera measurements: rectified %dx%d images, fx=%.1f px, "
        "fy=%.1f px, baseline=%.3f m, valid depth %.2f-%.2f m. "
        "The requested toy is about %.3f m wide and %.3f m tall."
        % (int(camera.get("width", 0)), int(camera.get("height", 0)),
           float(camera.get("fx", 0.0)), float(camera.get("fy", 0.0)),
           float(camera.get("baseline_m", 0.0)),
           float(camera.get("depth_min_m", 0.0)),
           float(camera.get("depth_max_m", 0.0)),
           float(target.get("known_width_m", 0.0)),
           float(target.get("known_height_m", 0.0)))
    )
