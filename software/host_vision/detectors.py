"""Open-vocabulary detector backends for the host-side detection service.

Runs only on the computer (see ``yolo_service.py``). The ROS container never
imports this module: it sends rectified JPEGs over HTTP and gets plain-Python
detection dictionaries back. Three backends share the same interface:

``yolo_world``
    Ultralytics YOLO-World (``yolov8s-world.pt``) with ``set_classes``.
``yoloe_26``
    Ultralytics YOLOE-26 text-prompted segmentation (``yoloe-26s-seg.pt``),
    also using ``set_classes``. Requires ultralytics >= 8.4 and a one-time
    CLIP text-encoder download on the first prompt. Its instance masks are used
    directly: the reported ``u``/``v`` is the mean of the mask's pixels.
``grounding_dino``
    Hugging Face ``IDEA-Research/grounding-dino-tiny`` via transformers
    ``AutoModelForZeroShotObjectDetection``; text queries are lowercased and
    dot-terminated as required by that model card.

YOLO-World and Grounding DINO are box-only models, so their reported centre is
the box centre. Every detection records ``center_source`` (``mask_centroid`` or
``box_center``) so the stereo stage can report which model output was used.
Only the small centroid/box/area values cross HTTP; full masks stay on the host.

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

MASK_CENTROID = "mask_centroid"
BOX_CENTER = "box_center"

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


def _as_array(data):
    """Return a numpy view of a torch tensor, numpy array, or nested list."""
    if hasattr(data, "detach"):
        data = data.detach()
    if hasattr(data, "cpu"):
        data = data.cpu()
    if hasattr(data, "numpy"):
        data = data.numpy()
    return np.asarray(data)


def _polygon_centroid(points):
    """Area centroid ``(u, v)`` of an instance-mask polygon in pixel coords."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3:
        return None
    x, y = points[:, 0], points[:, 1]
    cross = x * np.roll(y, -1) - np.roll(x, -1) * y
    area = 0.5 * cross.sum()
    if abs(area) < 1e-9:
        return None
    return (float(((x + np.roll(x, -1)) * cross).sum() / (6.0 * area)),
            float(((y + np.roll(y, -1)) * cross).sum() / (6.0 * area)))


def _mask_geometry(masks, index, image_shape):
    """Return centroid, mask-derived box, and area, or ``None`` if unavailable.

    Ultralytics ``Masks.data`` can be at network resolution while ``xy``
    polygons are in original-image pixels, so binary-mask coordinates are
    scaled to ``image_shape`` before they are compared with box detections.
    """
    if masks is None:
        return None
    height, width = int(image_shape[0]), int(image_shape[1])
    data = getattr(masks, "data", None)
    if data is not None:
        try:
            array = _as_array(data[index])
        except (IndexError, TypeError, ValueError):
            array = None
        if array is not None and array.ndim == 2:
            mask_h, mask_w = array.shape
            if mask_h > 0 and mask_w > 0:
                ys, xs = np.nonzero(array > 0)
                if len(xs):
                    sx, sy = width / mask_w, height / mask_h
                    x1, x2 = float(xs.min()) * sx, (float(xs.max()) + 1) * sx
                    y1, y2 = float(ys.min()) * sy, (float(ys.max()) + 1) * sy
                    return {
                        "u": float(xs.mean()) * sx,
                        "v": float(ys.mean()) * sy,
                        "bbox_px": [x1, y1, x2, y2],
                        "width_px": x2 - x1,
                        "height_px": y2 - y1,
                        "mask_area_px": int(round(len(xs) * sx * sy)),
                        # The second pipeline asks the dense-stereo service for
                        # the median over these pixels. They stay inside this
                        # host-to-host request and are not part of the answer.
                        "depth_sample_pixels": [
                            [round(float(x) * sx, 2), round(float(y) * sy, 2)]
                            for y, x in zip(ys, xs)],
                    }
    polygons = getattr(masks, "xy", None)
    if polygons is not None and index < len(polygons):
        center = _polygon_centroid(polygons[index])
        if center is not None:
            points = np.asarray(polygons[index], dtype=float)
            x1, y1 = points.min(axis=0)
            x2, y2 = points.max(axis=0)
            cross = (points[:, 0] * np.roll(points[:, 1], -1)
                     - np.roll(points[:, 0], -1) * points[:, 1])
            return {
                "u": center[0],
                "v": center[1],
                "bbox_px": [float(x1), float(y1), float(x2), float(y2)],
                "width_px": float(x2 - x1),
                "height_px": float(y2 - y1),
                "mask_area_px": int(round(abs(cross.sum()) / 2.0)),
                "depth_sample_pixels": [
                    [round(float(x), 2), round(float(y), 2)] for x, y in points],
            }
    return None


def _detection(xyxy, score, label, masks=None, index=None, image_shape=None):
    """Build a detection dict, preferring a real mask centroid when present."""
    x1, y1, x2, y2 = [float(value) for value in xyxy]
    entry = {
        "label": str(label),
        "confidence": float(score),
        "bbox_px": [x1, y1, x2, y2],
        "u": (x1 + x2) / 2.0,
        "v": (y1 + y2) / 2.0,
        "width_px": x2 - x1,
        "height_px": y2 - y1,
        "center_source": BOX_CENTER,
        "mask_centroid_px": None,
        "mask_area_px": 0,
        "depth_sample_pixels": [],
    }
    if masks is not None and index is not None and image_shape is not None:
        geometry = _mask_geometry(masks, index, image_shape)
        if geometry:
            entry.update(geometry)
            entry["center_source"] = MASK_CENTROID
            entry["mask_centroid_px"] = [round(entry["u"], 2),
                                         round(entry["v"], 2)]
    return entry


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
    # YOLOE-26-seg supplies instance masks; YOLO-World detection weights do not.
    masks = getattr(results[0], "masks", None)
    detections = []
    for index, (xyxy, score, class_id) in enumerate(
            zip(boxes.xyxy, boxes.conf, boxes.cls)):
        label = names.get(int(class_id),
                          classes[min(int(class_id), len(classes) - 1)])
        detections.append(_detection(xyxy, score, label, masks=masks,
                                     index=index, image_shape=rgb.shape))
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
