"""Open-vocabulary stereo detection and triangulation.

The YOLO-world inference runs in a separate HTTP service on the computer (see
``software/host_vision/yolo_service.py``). This container-side module decodes
only enough JPEG metadata to know the image width, asks the service to detect
the requested text target independently in the rectified left and right eyes,
then pairs detections by label, row, and horizontal disparity. Depth is
``distance = fx * baseline / disparity``; lateral bearing comes from the target
centre in the left rectified image using the shared convention in
``vision_config``.

Tests inject a fake ``client`` with the same signature as
``yolo_client.detect``, so no model weights or torch install are needed.
"""

import io
import time

from PIL import Image

from . import vision_answer, vision_config, yolo_client


def jpeg_size(data):
    """Return ``(width, height)`` for a JPEG byte string without decoding pixels."""
    with Image.open(io.BytesIO(data)) as image:
        return image.size


def model_name(config, remote_name=None):
    """Prefer the host service's model name, then config, then a safe default."""
    if remote_name:
        return str(remote_name)
    configured = vision_config.get(config, "open_vocab", "name")
    if configured:
        return str(configured)
    return "yolov8s-world"


def target_classes(target):
    """Split a text target into open-vocabulary class prompts."""
    text = str(target or "").strip()
    if not text:
        return []
    parts = [part.strip() for part in text.replace(" or ", ",").split(",")]
    return [part for part in parts if part] or [text]


def _match(left, right, config):
    """Choose the best left/right detection pair and recover its depth."""
    camera = config.get("camera", {}) or {}
    fx = float(camera.get("fx", 0.0))
    baseline = float(camera.get("baseline_m", 0.0))
    min_disparity = float(vision_config.get(
        config, "open_vocab", "min_disparity", default=3))
    max_disparity = float(vision_config.get(
        config, "open_vocab", "max_disparity", default=96))
    max_row_delta = float(vision_config.get(
        config, "open_vocab", "max_vertical_delta_px", default=20))
    depth_min = float(camera.get("depth_min_m", 0.0))
    depth_max = float(camera.get("depth_max_m", 0.0))

    pairs = []
    for left_detection in left:
        for right_detection in right:
            if left_detection["label"].lower() != right_detection["label"].lower():
                continue
            if abs(left_detection["v"] - right_detection["v"]) > max_row_delta:
                continue
            disparity = left_detection["u"] - right_detection["u"]
            if disparity < min_disparity or disparity > max_disparity:
                continue
            if fx <= 0.0 or baseline <= 0.0 or disparity <= 0.0:
                continue
            distance = fx * baseline / disparity
            if distance < depth_min or distance > depth_max:
                continue
            confidence = (left_detection["confidence"]
                          + right_detection["confidence"]) / 2.0
            pairs.append({
                "label": left_detection["label"],
                "confidence": confidence,
                "u_left": left_detection["u"],
                "v_left": left_detection["v"],
                "bbox_px": left_detection["bbox_px"],
                "bbox_right_px": right_detection["bbox_px"],
                "width_px": left_detection["width_px"],
                "disparity_px": disparity,
                "distance_m": distance,
            })
    if not pairs:
        return None
    return max(pairs, key=lambda pair: (pair["confidence"],
                                        -abs(pair["v_left"] - 120.0)))


def _size(width_px, image_width):
    """Map a box width to the legacy small/medium/large label."""
    fraction = float(width_px) / max(1, int(image_width))
    if fraction < 0.10:
        return "small"
    if fraction > 0.25:
        return "large"
    return "medium"


def _entry(answer, name, confidence, seconds, distance_source):
    answer.update({
        "model": name,
        "method": "open_vocab",
        "distance_source": distance_source,
        "confidence": round(float(confidence), 3),
        "seconds": round(seconds, 3),
    })
    return answer


def detect(left_jpeg, right_jpeg, target, config, client=None):
    """Detect the text target in both eyes and triangulate its 3D location."""
    start = time.time()
    classes = target_classes(target)

    if not classes:
        answer = vision_answer.tidy({
            "description": "No text target supplied; nothing to detect.",
            "target_visible": False,
            "position": "none",
            "lateral_bearing_deg": 0.0,
            "size": "none",
            "distance_m": 0.0,
            "obstacle_ahead": "none",
        }, config)
        return _entry(answer, model_name(config), 0.0,
                      time.time() - start, "none")

    remote = yolo_client.detect if client is None else client
    left, right, remote_name = remote(left_jpeg, right_jpeg, classes, config)
    name = model_name(config, remote_name)
    pair = _match(left, right, config)
    image_width = jpeg_size(left_jpeg)[0]

    if pair is None:
        answer = vision_answer.tidy({
            "description": "No matching %s detection in the rectified stereo pair."
                           % classes[0],
            "target_visible": False,
            "position": "none",
            "lateral_bearing_deg": 0.0,
            "size": "none",
            "distance_m": 0.0,
            "obstacle_ahead": "none",
        }, config)
        return _entry(answer, name, 0.0, time.time() - start, "none")

    bearing = vision_config.lateral_bearing_deg(
        pair["u_left"], config, image_width)
    answer = vision_answer.tidy({
        "description": (
            "Detected '%s' in both rectified images at u=%.1f, v=%.1f px; "
            "disparity %.1f px."
            % (pair["label"], pair["u_left"], pair["v_left"],
               pair["disparity_px"])),
        "target_visible": True,
        "position": "center",
        "lateral_bearing_deg": bearing,
        "size": _size(pair["width_px"], image_width),
        "distance_m": round(pair["distance_m"], 3),
        "obstacle_ahead": "none",
    }, config)
    answer.update({
        "bbox_px": pair["bbox_px"],
        "bbox_right_px": pair["bbox_right_px"],
        "disparity_px": round(pair["disparity_px"], 2),
    })
    return _entry(answer, name, pair["confidence"], time.time() - start,
                  "stereo_triangulation")
