"""Open-vocabulary detector backends for the host-side detection service.

Runs only on the computer (see ``yolo_service.py``). The ROS container never
imports this module: it sends rectified JPEGs over HTTP and gets plain-Python
detection dictionaries back. Three backends share the same interface:

``yolo_world``
    Ultralytics YOLO-World (``yolov8s-world.pt``) with ``set_classes``.
``yoloe_26``
    Ultralytics YOLOE-26 text-prompted segmentation (``yoloe-26s-seg.pt``),
    also using ``set_classes``. Requires ultralytics >= 8.4 and a one-time
    CLIP text-encoder download on the first prompt.
``grounding_dino``
    Hugging Face ``IDEA-Research/grounding-dino-tiny`` via transformers
    ``AutoModelForZeroShotObjectDetection``; text queries are lowercased and
    dot-terminated as required by that model card.

Heavy imports are lazy so ``--help``, health checks, and tests do not load torch.
Tests can inject fakes with ``load_detector(config, model=fake)``.
"""

import io

import numpy as np
from PIL import Image

YOLO_WORLD = "yolo_world"
YOLOE_26 = "yoloe_26"
GROUNDING_DINO = "grounding_dino"
DETECTORS = (YOLO_WORLD, YOLOE_26, GROUNDING_DINO)

# Small defaults chosen for a CPU-equipped computer. Every one is overridable
# by the service CLI or its environment variable (see yolo_service.py).
DEFAULT_WEIGHTS = {
    YOLO_WORLD: "yolov8s-world.pt",
    YOLOE_26: "yoloe-26s-seg.pt",
    GROUNDING_DINO: "IDEA-Research/grounding-dino-tiny",
}

_CACHE = {}


def detector_name(config):
    """Return the selected backend name from a config dict or a name string."""
    raw = config.get("detector") if isinstance(config, dict) else config
    name = str(raw or YOLO_WORLD).strip().lower()
    if name not in DETECTORS:
        raise ValueError("Unknown detector %r; choose one of: %s"
                         % (name, ", ".join(DETECTORS)))
    return name


def decode_jpeg(data):
    """Return an RGB uint8 array for one rectified JPEG byte string."""
    with Image.open(io.BytesIO(data)) as image:
        return np.asarray(image.convert("RGB"))


def load_detector(config, model=None):
    """Return ``(backend_name, loaded_model)``, importing libraries on first use."""
    backend = detector_name(config)
    if model is not None:
        return backend, model

    path = str(config.get("model_path") or DEFAULT_WEIGHTS[backend])
    device = str(config.get("device") or "")
    key = (backend, path, device)
    if key not in _CACHE:
        if backend in (YOLO_WORLD, YOLOE_26):
            try:
                from ultralytics import YOLO, YOLOE
            except ImportError as error:  # pragma: no cover - host venv dependency
                raise ImportError(
                    "The host detector service needs ultralytics >= 8.4 for "
                    "YOLO-World/YOLOE-26. Install it with "
                    "scripts/start_yolo_service.sh") from error
            loaded = YOLOE(path) if backend == YOLOE_26 else YOLO(path)
            if device:
                loaded.to(device)
        else:
            try:
                import torch
                from transformers import (AutoModelForZeroShotObjectDetection,
                                          AutoProcessor)
            except ImportError as error:  # pragma: no cover - host venv dependency
                raise ImportError(
                    "The host detector service needs transformers and torch "
                    "for Grounding DINO. Install them with "
                    "scripts/start_yolo_service.sh") from error
            processor = AutoProcessor.from_pretrained(path)
            network = AutoModelForZeroShotObjectDetection.from_pretrained(path)
            network.to(device or "cpu")
            loaded = (processor, network)
        _CACHE[key] = loaded
    return backend, _CACHE[key]


def _detection(xyxy, score, label):
    x1, y1, x2, y2 = [float(value) for value in xyxy]
    return {
        "label": str(label),
        "confidence": float(score),
        "bbox_px": [x1, y1, x2, y2],
        "u": (x1 + x2) / 2.0,
        "v": (y1 + y2) / 2.0,
        "width_px": x2 - x1,
        "height_px": y2 - y1,
    }


def _yolo_detections(model, rgb, classes, config):
    confidence = float(config.get("confidence_threshold", 0.25))
    max_detections = int(config.get("max_detections", 10))
    device = str(config.get("device") or "") or None
    if hasattr(model, "set_classes"):
        model.set_classes(list(classes))
    results = model(rgb, conf=confidence, max_det=max_detections,
                    device=device, verbose=False)
    boxes = getattr(results[0], "boxes", None)
    if boxes is None or len(boxes) == 0:
        return []
    names = getattr(results[0], "names", None) or getattr(model, "names", {}) or {}
    detections = []
    for xyxy, score, class_id in zip(boxes.xyxy, boxes.conf, boxes.cls):
        index = int(class_id)
        label = names.get(index, classes[min(index, len(classes) - 1)])
        detections.append(_detection(xyxy, score, label))
    return detections


def _dino_query(classes):
    """Grounding DINO expects lowercase phrases ending in a dot."""
    phrases = [str(name).strip().lower() for name in classes if str(name).strip()]
    return " ".join(phrase.rstrip(".") + "." for phrase in phrases)


def _dino_detections(loaded, rgb, classes, config):
    import torch

    processor, network = loaded
    device = str(config.get("device") or "") or "cpu"
    confidence = float(config.get("confidence_threshold", 0.25))
    text_threshold = float(config.get("text_threshold", 0.25))
    inputs = processor(images=Image.fromarray(rgb), text=_dino_query(classes),
                       return_tensors="pt").to(device)
    with torch.no_grad():
        outputs = network(**inputs)
    results = processor.post_process_grounded_object_detection(
        outputs, inputs.input_ids, box_threshold=confidence,
        text_threshold=text_threshold,
        target_sizes=[(int(rgb.shape[0]), int(rgb.shape[1]))])[0]
    detections = []
    for xyxy, score, label in zip(results.get("boxes", []),
                                  results.get("scores", []),
                                  results.get("labels", [])):
        # Map the model's lowercase label back to the operator's original text
        # so left/right matching uses exactly the requested class prompt.
        text = str(label)
        for name in classes:
            if text.strip().lower() == str(name).strip().lower():
                text = str(name)
                break
        detections.append(_detection(xyxy, score, text))
    return detections


def detections_for_image(loaded, rgb, classes, config):
    """Detect ``classes`` in one RGB image; return plain-Python dictionaries."""
    if not classes:
        return []
    backend, model = loaded
    if backend == GROUNDING_DINO:
        return _dino_detections(model, rgb, classes, config)
    return _yolo_detections(model, rgb, classes, config)
