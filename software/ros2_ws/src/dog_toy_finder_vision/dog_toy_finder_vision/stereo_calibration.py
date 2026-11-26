"""Load the full stereo calibration saved by scripts/calibrate_stereo.py.

The JSON keeps both eyes' intrinsics and the solved rectification. The camera
node publishes those per-eye K/D/R/P matrices so image_proc can rectify with
the real relative rotation instead of assuming one shared K and identity R.
"""

import json
import os

KIND = "dog_toy_finder_stereo_calibration"


def _matrix(value, name, rows=3, cols=3):
    """Validate and copy a nested JSON matrix."""
    if (not isinstance(value, list) or len(value) != rows
            or any(not isinstance(row, list) or len(row) != cols
                   for row in value)):
        raise ValueError(f"{name} must be a {rows}x{cols} list")
    return [[float(cell) for cell in row] for row in value]


def _vector(value, name, size=None):
    """Validate and copy a flat JSON vector."""
    if not isinstance(value, list):
        raise ValueError(f"{name} must be a list")
    if size is not None and len(value) != size:
        raise ValueError(f"{name} must have {size} entries")
    return [float(item) for item in value]


def _eye(data, side):
    """Extract one eye's CameraInfo matrices from a calibration dict."""
    if not isinstance(data, dict):
        raise ValueError(f"{side} calibration must be an object")
    return {
        "k": _matrix(data.get("k"), f"{side}.k"),
        "d": _vector(data.get("d"), f"{side}.d"),
        "r": _matrix(data.get("r"), f"{side}.r"),
        "p": _matrix(data.get("p"), f"{side}.p", rows=3, cols=4),
    }


def load_stereo_calibration(path):
    """Return a full calibration dict, or None when no path was supplied.

    An explicitly named but missing or malformed file is an error: silently
    falling back to one shared K would recreate the disparity bias this format
    exists to prevent.
    """
    if not path:
        return None
    if not os.path.isfile(path):
        raise FileNotFoundError(f"stereo calibration file not found: {path}")
    with open(path) as handle:
        data = json.load(handle)
    if not isinstance(data, dict) or data.get("kind") != KIND:
        raise ValueError(f"not a dog_toy_finder stereo calibration: {path}")
    return {
        "width": int(data["width"]),
        "height": int(data["height"]),
        "baseline_m": float(data["baseline_m"]),
        "rotation": _matrix(data["rotation"], "rotation"),
        "translation": _vector(data["translation"], "translation", size=3),
        "left": _eye(data["left"], "left"),
        "right": _eye(data["right"], "right"),
    }
