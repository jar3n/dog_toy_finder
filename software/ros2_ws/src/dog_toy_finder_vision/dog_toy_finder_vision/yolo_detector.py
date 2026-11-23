"""YOLO-world loading and detection helpers for the host-side service.

This module is imported only by ``software/host_vision/yolo_service.py`` running
on the computer. Keeping it separate from ``open_vocab_stereo`` means the ROS
container does not import torch or ultralytics: it just sends JPEGs to the host
over HTTP and receives plain-Python detection dictionaries.

The ultralytics import is lazy so a ``--help`` or a health check never pays the
torch import cost. Tests can inject a model with the small ``set_classes`` /
``__call__`` interface used below.
"""

import io
import os

import numpy as np
from PIL import Image

from . import vision_config

_MODEL_CACHE = {}


def decode_jpeg(data):
    """Return an RGB uint8 array for one rectified JPEG byte string."""
    with Image.open(io.BytesIO(data)) as image:
        return np.asarray(image.convert("RGB"))


def load_model(config, model=None):
    """Return an open-vocabulary model, importing ultralytics on first use."""
    if model is not None:
        return model
    path = str(vision_config.get(config, "open_vocab", "model_path",
                                 default="yolov8s-world.pt"))
    device = str(vision_config.get(config, "open_vocab", "device",
                                   default="") or "")
    key = (path, device)
    if key not in _MODEL_CACHE:
        try:
            from ultralytics import YOLO
        except ImportError as error:  # pragma: no cover - host venv dependency
            raise ImportError(
                "The host YOLO service needs ultralytics. Install the host "
                "vision dependencies with scripts/start_yolo_service.sh") from error
        loaded = YOLO(path)
        if device:
            loaded.to(device)
        _MODEL_CACHE[key] = loaded
    return _MODEL_CACHE[key]


def model_name(config, model=None):
    """Return the display name recorded in results and trial CSVs."""
    injected = getattr(model, "name", None) if model is not None else None
    if injected:
        return str(injected)
    configured = vision_config.get(config, "open_vocab", "name")
    if configured:
        return str(configured)
    path = str(vision_config.get(config, "open_vocab", "model_path",
                                 default="yolov8s-world.pt"))
    return os.path.splitext(os.path.basename(path))[0]


def detections_for_image(model, rgb, classes, config):
    """Run the model on one image and return plain-Python detections."""
    if not classes:
        return []
    confidence = float(vision_config.get(
        config, "open_vocab", "confidence_threshold", default=0.25))
    max_detections = int(vision_config.get(
        config, "open_vocab", "max_detections", default=10))
    device = str(vision_config.get(config, "open_vocab", "device",
                                   default="") or "") or None
    if hasattr(model, "set_classes"):
        model.set_classes(classes)
    results = model(rgb, conf=confidence, max_det=max_detections,
                    device=device, verbose=False)
    result = results[0]
    boxes = getattr(result, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return []
    names = getattr(result, "names", None) or getattr(model, "names", {}) or {}
    detections = []
    for xyxy, score, class_id in zip(boxes.xyxy, boxes.conf, boxes.cls):
        x1, y1, x2, y2 = [float(value) for value in xyxy]
        index = int(class_id)
        label = str(names.get(index, classes[min(index, len(classes) - 1)]))
        detections.append({
            "label": label,
            "confidence": float(score),
            "bbox_px": [x1, y1, x2, y2],
            "u": (x1 + x2) / 2.0,
            "v": (y1 + y2) / 2.0,
            "width_px": x2 - x1,
            "height_px": y2 - y1,
        })
    return detections
