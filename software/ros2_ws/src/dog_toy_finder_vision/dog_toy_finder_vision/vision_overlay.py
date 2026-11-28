"""Annotated detection images for offline vision evaluation.

``vision_eval`` writes one JPEG per scored object (and per eye with detector
output) next to its other artifacts. The image is the saved rectified shot with
the pipeline's selected detection drawn on it, so an operator can confirm
whether the model actually saw the requested target in that scene.

``triangulation`` selects one matched detection pair, so both eyes are drawn.
``stereo-net`` detects only in the left eye, so only that eye is drawn. The
left-eye overlay also marks the operator-measured bearing when available, which
makes a false localisation easy to distinguish from a missed detection. For an
absent target (including a ``none`` scene), a clean image is labelled as the
correct no-detection result instead of a miss; a hallucinated box is flagged as
a false positive.

This module only uses Pillow, already required by the ROS-side vision package.
It never contacts a host model service or modifies the saved capture images.
"""

import io
import math
import os
import re

from PIL import Image, ImageDraw, ImageFont

from . import vision_config, vision_pipelines

BOX_COLOR = (0, 255, 0)
CENTER_COLOR = (0, 220, 255)
TRUTH_COLOR = (255, 235, 0)
MISS_COLOR = (255, 90, 90)
OK_COLOR = (110, 255, 130)
TEXT_COLOR = (255, 255, 255)
BANNER_COLOR = (22, 22, 22)

_UNSAFE = re.compile(r"[^A-Za-z0-9_.-]+")


def _safe_component(value):
    """Return a filesystem-safe single path component."""
    text = _UNSAFE.sub("_", str(value or "")).strip("._")
    return text or "unknown"


def overlay_image_path(overlays_dir, run_key, shot_rel, object_id, eye):
    """Return the annotated JPEG path for one object and eye."""
    parts = [_safe_component(run_key)]
    for part in str(shot_rel or "").replace("\\", "/").split("/"):
        if part and part not in (".", ".."):
            parts.append(_safe_component(part))
    name = "%s_%s.jpg" % (_safe_component(object_id), _safe_component(eye))
    return os.path.join(overlays_dir, *parts, name)


def _as_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _expected_absent(ground_truth):
    """Return whether this object was annotated as absent from the scene."""
    if not isinstance(ground_truth, dict):
        return False
    value = ground_truth.get("target_visible")
    if value is None:
        return False
    if isinstance(value, bool):
        return not value
    return str(value).strip().lower() in (
        "false", "0", "no", "n", "not visible", "absent", "missing", "none")


def _bbox(entry, eye):
    """Return the selected detection box for one eye, clamped to sanity."""
    key = "bbox_right_px" if eye == "right" else "bbox_px"
    raw = entry.get(key) if isinstance(entry, dict) else None
    if isinstance(raw, dict):
        raw = [raw.get("x1"), raw.get("y1"), raw.get("x2"), raw.get("y2")]
    if not isinstance(raw, (list, tuple)) or len(raw) < 4:
        return None
    values = [_as_float(value) for value in raw[:4]]
    if any(value is None for value in values):
        return None
    x1, y1, x2, y2 = values
    if x2 < x1:
        x1, x2 = x2, x1
    if y2 < y1:
        y1, y2 = y2, y1
    return x1, y1, x2, y2


def _center(entry, eye, box):
    """Return the model-reported centre, falling back to the box centre."""
    if eye == "left" and isinstance(entry, dict):
        raw = entry.get("mask_centroid_px")
        if isinstance(raw, dict):
            raw = [raw.get("u"), raw.get("v")]
        if isinstance(raw, (list, tuple)) and len(raw) >= 2:
            u, v = _as_float(raw[0]), _as_float(raw[1])
            if u is not None and v is not None:
                return u, v
    if box is None:
        return None
    return (box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0


def _eyes(entry, pipeline):
    """Return the eyes that received detector output for this pipeline."""
    if pipeline == vision_pipelines.STEREO_NET:
        return ("left",)
    if isinstance(entry, dict) and entry.get("method") == "stereo-net":
        return ("left",)
    return ("left", "right")


def _ground_truth_u(ground_truth, config, width):
    """Convert the operator bearing back to an expected left-eye column."""
    if not isinstance(ground_truth, dict) or _expected_absent(ground_truth):
        return None
    bearing = _as_float(ground_truth.get("lateral_bearing_deg"))
    if bearing is None:
        return None
    left = (config or {}).get("camera", {}).get("left", {}) or {}
    fx = _as_float(left.get("fx")) or 250.0
    cx = _as_float(left.get("cx"))
    if cx is None:
        cx = width / 2.0
    u = cx - fx * math.tan(math.radians(bearing))
    return u if 0 <= u <= width else None


def _label(draw, position, text, fill):
    """Draw small text on an opaque background so it stays readable."""
    font = ImageFont.load_default()
    bounds = draw.textbbox((0, 0), text, font=font)
    width = bounds[2] - bounds[0]
    height = bounds[3] - bounds[1]
    x, y = position
    draw.rectangle((x - 2, y - 2, x + width + 2, y + height + 2),
                   fill=BANNER_COLOR)
    draw.text((x, y), text, fill=fill, font=font)


def _dashed_vertical(draw, x, top, bottom, color):
    """Draw a dashed vertical guide from ``top`` to ``bottom``."""
    y = top
    while y < bottom:
        draw.line((x, y, x, min(y + 6, bottom)), fill=color, width=2)
        y += 12


def annotate_image(jpeg, target, entry, eye, config, ground_truth=None):
    """Return a PIL image with the selected detection drawn on it."""
    image = Image.open(io.BytesIO(jpeg)).convert("RGB")
    draw = ImageDraw.Draw(image)
    width, height = image.size
    visible = bool(entry.get("target_visible")) if isinstance(entry, dict) else False
    expected_absent = _expected_absent(ground_truth)

    header = "target: %s" % (target or "")
    if visible:
        details = []
        confidence = _as_float(entry.get("confidence"))
        if confidence is not None:
            details.append("conf %.2f" % confidence)
        distance = _as_float(entry.get("distance_m"))
        if distance is not None:
            details.append("depth %.2f m" % distance)
        bearing = _as_float(entry.get("lateral_bearing_deg"))
        if bearing is not None:
            details.append("bearing %.1f deg" % bearing)
        if details:
            header += "  |  " + ", ".join(details)
        if expected_absent:
            header += "  |  UNEXPECTED DETECTION"
    elif expected_absent:
        header += "  |  CORRECT: NO DETECTION"
    else:
        header += "  |  NOT SEEN"
    draw.rectangle((0, 0, width, 18), fill=BANNER_COLOR)
    draw.text((4, 4), header[:max(1, width // 6)], fill=TEXT_COLOR)
    draw.text((4, height - 13), "%s eye" % eye, fill=TEXT_COLOR)

    if not visible:
        if expected_absent:
            _label(draw, (4, 24), "NO TARGET EXPECTED", OK_COLOR)
        else:
            _label(draw, (4, 24), "NO DETECTION FOR THIS TARGET", MISS_COLOR)
        return image

    if expected_absent:
        _label(draw, (4, 24), "FALSE POSITIVE: TARGET ABSENT", MISS_COLOR)

    box = _bbox(entry, eye)
    center = _center(entry, eye, box)
    if eye == "left":
        truth_u = _ground_truth_u(ground_truth, config, width)
        if truth_u is not None:
            _dashed_vertical(draw, truth_u, 19, height - 14, TRUTH_COLOR)
            _label(draw, (4, 24), "yellow dashed = operator bearing",
                   TRUTH_COLOR)
    if box is not None:
        draw.rectangle(box, outline=BOX_COLOR, width=2)
        _label(draw, (box[0], max(19, box[1] - 13)),
               "model box", BOX_COLOR)
    if center is not None:
        x, y = center
        draw.line((x - 7, y, x + 7, y), fill=CENTER_COLOR, width=2)
        draw.line((x, y - 7, x, y + 7), fill=CENTER_COLOR, width=2)
    if box is None and center is None:
        _label(draw, (4, 24), "visible but no box/centre in answer",
               MISS_COLOR)
    return image


def write_object_overlays(shot_dir, object_result, config, overlays_dir,
                          run_key, root, artifacts=None):
    """Write annotated images for one scored object and return their paths.

    The returned mapping is ``{eye: absolute JPEG path}``; ``--pilot`` and
    explicit output paths share this layout. ``artifacts``, when supplied, is an
    ``_ArtifactTracker`` used to restore/remove the images on interrupt.
    """
    entries = object_result.get("results") or []
    entry = next((item for item in entries
                  if isinstance(item, dict) and item.get("target_visible")),
                 None)
    if entry is None and entries:
        entry = entries[0] if isinstance(entries[0], dict) else {}
    entry = entry or {}
    pipeline = entry.get("method") or object_result.get("pipeline") or ""
    shot_rel = os.path.relpath(shot_dir, root) if root else os.path.basename(
        shot_dir)
    object_id = (object_result.get("object_id")
                 or object_result.get("id") or "object")
    target = object_result.get("target") or object_result.get("label") or ""
    ground_truth = object_result.get("ground_truth")
    written = {}
    for eye in _eyes(entry, pipeline):
        source = os.path.join(shot_dir, "%s.jpg" % eye)
        if not os.path.isfile(source):
            continue
        path = overlay_image_path(overlays_dir, run_key, shot_rel, object_id,
                                  eye)
        directory = os.path.dirname(path)
        if artifacts is not None:
            artifacts.track_dir(directory)
            artifacts.track(path)
        os.makedirs(directory, exist_ok=True)
        with open(source, "rb") as handle:
            jpeg = handle.read()
        image = annotate_image(jpeg, target, entry, eye, config, ground_truth)
        image.save(path, quality=92)
        written[eye] = path
    return written
