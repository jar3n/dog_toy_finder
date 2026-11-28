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
    "pipeline": "triangulation",
    "camera": {
        "width": 320,
        "height": 240,
        "baseline_m": 0.08,
        # Rectified intrinsics: the 3x3 part of stereoRectify's P1/P2. Detection
        # and triangulation use the left eye; the right eye is stored so the
        # camera node can publish its own K instead of forcing a shared one.
        "left": {
            "fx": 250.0,
            "fy": 250.0,
            "cx": 160.0,
            "cy": 120.0,
        },
        "right": {
            "fx": 250.0,
            "fy": 250.0,
            "cx": 160.0,
            "cy": 120.0,
        },
        "depth_min_m": 0.25,
        "depth_max_m": 3.0,
        "left_rectified_topic": "/stereo/left/image_rect_color",
        "right_rectified_topic": "/stereo/right/image_rect_color",
    },
    "open_vocab": {
        "detector": "yolo_world",
        "name": "yolov8s-world",
        "service_url": "http://localhost:5006",
        "timeout_s": 60,
        "confidence_threshold": 0.25,
        "max_detections": 10,
        "min_disparity": 3,
        "max_disparity": 96,
        "max_vertical_delta_px": 20,
    },
    "stereo-net": {
        "model": "lite_any_stereo",
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


_LEGACY_EYE_FIELDS = ("fx", "fy", "cx", "cy")


def _normalize_camera_block(config):
    """Move old shared camera.fx/fy/cx/cy values into both eye blocks.

    Older YAML files and hand-built configs used one shared intrinsics set.
    They still load, but the canonical schema is camera.left and camera.right;
    removing the shared keys prevents a consumer from silently using them.
    """
    camera = config.get("camera")
    if not isinstance(camera, dict):
        return config
    legacy = {key: camera.pop(key)
              for key in _LEGACY_EYE_FIELDS if key in camera}
    for side in ("left", "right"):
        eye = camera.get(side)
        if not isinstance(eye, dict):
            eye = {}
            camera[side] = eye
        for key, value in legacy.items():
            eye.setdefault(key, value)
    return config


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


def default_capture_dir(pilot=False):
    """Return the capture root used by both the capture and replay tools.

    ``VISION_CAPTURE_DIR`` wins for normal captures. ``pilot=True`` selects the
    sibling ``data/vision/pilot_captures`` directory instead and ignores that
    override, unless the caller supplies an explicit path on the command line.
    """
    if not pilot:
        override = os.environ.get("VISION_CAPTURE_DIR")
        if override:
            return os.path.abspath(os.path.expanduser(override))
    name = "pilot_captures" if pilot else "captures"
    # Source checkout: <repo>/software/ros2_ws/src/<pkg>/<pkg>/this_file.py
    source_root = os.path.abspath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "..", "..", "..", ".."))
    candidate = os.path.join(source_root, "data", "vision", name)
    if os.path.isdir(os.path.dirname(candidate)):
        return candidate
    return os.path.join(os.getcwd(), "data", "vision", name)


def load_config(path=None):
    """Load the detector YAML and fill missing sections with defaults."""
    config_path = resolve_config_path(path)
    with open(config_path) as handle:
        loaded = yaml.safe_load(handle) or {}
    if not isinstance(loaded, dict):
        raise ValueError("Vision detector config must be a YAML object")
    # Normalize before merging defaults so explicit per-eye values win over a
    # legacy shared value instead of being masked by the default eye blocks.
    _normalize_camera_block(loaded)
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
    for side in ("left", "right"):
        eye = camera.get(side) or {}
        for field in ("fx", "fy"):
            if float(eye.get(field, 0.0)) <= 0.0:
                raise ValueError("camera.%s.%s must be positive" % (side, field))
    if float(camera.get("baseline_m", 0.0)) <= 0.0:
        raise ValueError("camera.baseline_m must be positive")
    minimum = float(camera.get("depth_min_m", 0.0))
    maximum = float(camera.get("depth_max_m", 0.0))
    if minimum <= 0.0 or maximum <= minimum:
        raise ValueError("camera.depth_min_m/depth_max_m are inconsistent")
    return config


def eye_intrinsics(config, eye="left", image_width=None, image_height=None):
    """Return ``fx``/``fy``/``cx``/``cy`` for one rectified eye.

    ``eye`` may be ``"left"`` or ``"right"``. A hand-built config that still
    carries shared camera.fx/fy/cx/cy is accepted as a legacy fallback, but the
    canonical YAML keeps the two eyes separate under ``camera.left`` and
    ``camera.right``.
    """
    camera = config.get("camera", {}) if isinstance(config, dict) else {}
    side = "right" if str(eye).strip().lower().startswith("r") else "left"
    values = camera.get(side) if isinstance(camera.get(side), dict) else {}
    result = {}
    for field in ("fx", "fy", "cx", "cy"):
        if field in values:
            result[field] = float(values[field])
        elif field in camera:
            result[field] = float(camera[field])
        elif field == "cx" and image_width is not None:
            result[field] = float(image_width) / 2.0
        elif field == "cy" and image_height is not None:
            result[field] = float(image_height) / 2.0
        else:
            result[field] = float(DEFAULTS["camera"][side][field])
    return result


def stereo_disparity_offset_px(config):
    """Return ``left.cx - right.cx`` for rectified disparity measurements.

    ``stereoRectify`` keeps fx/fy equal but does not force the two principal
    points to the same column. The disparity reported by a matcher or a dense
    stereo network is therefore ``(u_left - u_right)``, which includes this
    constant offset. Depth must use ``disparity - offset``; when the calibration
    happens to give the same cx for both eyes the offset is zero and the classic
    ``fx * baseline / disparity`` formula is unchanged.
    """
    left = eye_intrinsics(config, "left")
    right = eye_intrinsics(config, "right")
    return float(left["cx"]) - float(right["cx"])


def get(config, *keys, default=None):
    """Read a nested config value without raising on a missing section."""
    value = config
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


# --- Host service model selection ------------------------------------------

DETECTOR_BACKENDS = ("yolo_world", "yoloe_26", "grounding_dino")
DETECTOR_NAME_ALIASES = {
    "yolo_world": "yolo_world",
    "yolov8s_world": "yolo_world",
    "yolov8s-world": "yolo_world",
    "yoloe_26": "yoloe_26",
    "yoloe-26": "yoloe_26",
    "yoloe_26s_seg": "yoloe_26",
    "yoloe-26s-seg": "yoloe_26",
    "grounding_dino": "grounding_dino",
    "grounding-dino": "grounding_dino",
    "grounding_dino_tiny": "grounding_dino",
    "grounding-dino-tiny": "grounding_dino",
}
DETECTOR_DISPLAY_NAMES = {
    "yolo_world": "yolov8s-world",
    "yoloe_26": "yoloe-26s-seg",
    "grounding_dino": "grounding-dino-tiny",
}
DEPTH_BACKENDS = ("lite_any_stereo", "fast_foundation_stereo", "rt_igevpp")
DEPTH_NAME_ALIASES = {
    "lite_any_stereo": "lite_any_stereo",
    "las2_s": "lite_any_stereo",
    "fast_foundation_stereo": "fast_foundation_stereo",
    "model_best_bp2_serialize": "fast_foundation_stereo",
    "rt_igevpp": "rt_igevpp",
    "sceneflow": "rt_igevpp",
}
DEPTH_DISPLAY_NAMES = {
    "lite_any_stereo": "lite_any_stereo",
    "fast_foundation_stereo": "fast_foundation_stereo",
    "rt_igevpp": "rt_igevpp",
}


def _normalise_name(value):
    return str(value or "").strip().lower().replace("-", "_")


def canonical_detector_backend(value):
    """Resolve a detector backend id or known weights name to a backend id."""
    name = _normalise_name(value)
    if name in DETECTOR_BACKENDS:
        return name
    if name in DETECTOR_NAME_ALIASES:
        return DETECTOR_NAME_ALIASES[name]
    raise ValueError(
        "Unknown detector %r; choose one of: %s"
        % (value, ", ".join(DETECTOR_BACKENDS)))


def canonical_depth_backend(value):
    """Resolve a dense-stereo backend id or known weights name."""
    name = _normalise_name(value)
    if name in DEPTH_BACKENDS:
        return name
    if name in DEPTH_NAME_ALIASES:
        return DEPTH_NAME_ALIASES[name]
    raise ValueError(
        "Unknown dense stereo model %r; choose one of: %s"
        % (value, ", ".join(DEPTH_BACKENDS)))


def detector_backend(config):
    """Return the host detector backend selected by ``open_vocab`` config.

    ``open_vocab.name`` historically held the reported weights name, so accept
    both backend ids and the known weight stems as aliases.
    """
    configured = (get(config, "open_vocab", "detector")
                  or get(config, "open_vocab", "name")
                  or "yolov8s-world")
    return canonical_detector_backend(configured)


def detector_display_name(config):
    """Return the model name to record in results for the detector service."""
    return str(get(config, "open_vocab", "name")
               or detector_backend(config))


def depth_backend(config):
    """Return the dense-stereo backend selected by ``stereo-net`` config."""
    configured = (get(config, "stereo-net", "model")
                  or get(config, "stereo-net", "name")
                  or "lite_any_stereo")
    return canonical_depth_backend(configured)


def with_model_selection(config, detector=None, depth=None):
    """Return a copy of ``config`` pinned to specific detector/depth backends.

    This is used by the offline evaluator's full model matrix. Setting ``name``
    alongside the backend keeps logs and fallback labels accurate; actual
    weights are selected by the host service from the backend id.
    """
    result = copy.deepcopy(config)
    if detector is not None:
        backend = canonical_detector_backend(detector)
        result.setdefault("open_vocab", {})["detector"] = backend
        result["open_vocab"]["name"] = DETECTOR_DISPLAY_NAMES[backend]
    if depth is not None:
        backend = canonical_depth_backend(depth)
        result.setdefault("stereo-net", {})["model"] = backend
        result["stereo-net"]["name"] = DEPTH_DISPLAY_NAMES[backend]
    return result


# --- Lateral bearing -------------------------------------------------------
#
# Bearing is measured from the optical axis, in degrees, positive to the
# robot's left. In a rectified image a point at column u has
#
#     bearing = atan((cx - u) / fx)
#
# which is positive for u < cx (the left half of the image) and matches the
# ROS convention that +y is left. Detections are made in the rectified left
# image, so the left eye's intrinsics are the default even in stereo-net.


def lateral_bearing_deg(u_px, config, image_width=None, eye="left"):
    """Return the numeric lateral bearing of image column ``u_px`` in degrees."""
    camera = config.get("camera", {}) if isinstance(config, dict) else {}
    width = float(image_width or camera.get("width", 320))
    height = float(camera.get("height", 240))
    intrinsics = eye_intrinsics(config, eye, width, height)
    fx = intrinsics["fx"]
    cx = intrinsics["cx"]
    if fx <= 0.0:
        return 0.0
    bearing = math.degrees(math.atan((cx - float(u_px)) / fx))
    return round(max(-90.0, min(90.0, bearing)), 2)


def bearing_deg_from_center_offset(center_offset_m, depth_m):
    """Return bearing from a signed lateral offset and optical-axis depth.

    ``center_offset_m`` is measured in the optical plane from the point where
    the optical axis meets it; positive means robot-left. ``depth_m`` is the
    distance along the normal to that plane (the pipeline's ``distance_m``).
    This is the same bearing convention as ``lateral_bearing_deg``:
    ``atan(offset / depth)`` in degrees, clamped to +/-90.
    """
    offset = float(center_offset_m)
    depth = max(0.0, float(depth_m))
    bearing = math.degrees(math.atan2(offset, depth))
    return round(max(-90.0, min(90.0, bearing)), 2)


def center_offset_m(u_px, depth_m, config, image_width=None, eye="left"):
    """Return signed lateral distance from the optical axis, in metres.

    Positive means robot-left, matching ``lateral_bearing_deg``. The result is
    ``depth_m * (cx - u_px) / fx``: the lateral distance in the optical plane
    at the target's measured optical-axis depth. The left eye is the default
    because both pipelines detect in the rectified left image.
    """
    camera = config.get("camera", {}) if isinstance(config, dict) else {}
    width = float(image_width or camera.get("width", 320))
    height = float(camera.get("height", 240))
    intrinsics = eye_intrinsics(config, eye, width, height)
    fx = intrinsics["fx"]
    cx = intrinsics["cx"]
    if fx <= 0.0:
        return 0.0
    offset = float(depth_m) * (cx - float(u_px)) / fx
    return round(offset, 3)
