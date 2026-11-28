"""Open-vocabulary detection in one eye plus dense-stereo depth.

This is the ``stereo-net`` comparison pipeline. The text target is found by the
same host open-vocabulary detector as ``triangulation``, but only in the rectified
left image. A dense stereo model on the computer computes a disparity map from
the full pair; its median over the detected object's mask pixels (YOLOE-26) or
over a small window at the box centre (YOLO-World / Grounding DINO) gives the
distance. Depth is ``fx * baseline / disparity`` after subtracting the
``left.cx - right.cx`` principal-point offset from the network's raw
``u_left - u_right`` output; bearing uses the same ``atan((cx - u) / fx)``
convention as ``triangulation``.

Only detection numbers and the depth samples cross HTTP. Weights, torch, and
the dense stereo models stay on the computer. Tests inject small callables with
the same signatures as ``yolo_client.detect_one`` and ``depth_client.sample``.
"""

import time

from . import depth_client, open_vocab_stereo, vision_answer, vision_config, \
    yolo_client


def model_name(config):
    """Fallback dense model name when the host service does not report one."""
    configured = vision_config.get(config, "stereo-net", "name")
    return str(configured or "lite_any_stereo")


def detector_name(config, remote_name=None):
    """Report the detector used on the single left image."""
    if remote_name:
        return str(remote_name)
    return str(vision_config.get(config, "open_vocab", "name") or "yolov8s-world")


def _entry(answer, name, detector, confidence, seconds, distance_source):
    answer.update({
        "model": name,
        "detector_model": detector,
        "method": "stereo-net",
        "distance_source": distance_source,
        "confidence": round(float(confidence), 3),
        "seconds": round(seconds, 3),
    })
    defaults = {
        "center_source": "none",
        "mask_centroid_px": None,
        "mask_area_px": 0,
        "bbox_px": None,
        "depth_sample_source": "none",
        "depth_sample_count": 0,
        "depth_valid_count": 0,
        "depth_disparity_px": None,
        "depth_raw_disparity_px": None,
        "depth_disparity_offset_px": None,
    }
    for key, value in defaults.items():
        answer.setdefault(key, value)
    return answer


def _not_visible(description, config, name, detector, confidence, seconds,
                 distance_source="none"):
    answer = vision_answer.tidy({
        "description": description,
        "target_visible": False,
        "lateral_bearing_deg": 0.0,
        "distance_m": 0.0,
        "obstacle_ahead": "none",
    }, config)
    return _entry(answer, name, detector, confidence, seconds, distance_source)


def _best_detection(detections):
    """Choose one left-image detection: the highest-confidence match."""
    if not isinstance(detections, list):
        return None
    valid = [entry for entry in detections
             if isinstance(entry, dict) and "u" in entry and "v" in entry]
    if not valid:
        return None
    return max(valid, key=lambda entry: float(entry.get("confidence", 0.0)))


def _sample_spec(detection, config):
    """Use all mask pixels when present, else a small box-centre window."""
    pixels = detection.get("depth_sample_pixels") or []
    if detection.get("center_source") == "mask_centroid" and pixels:
        return {"sample_source": "mask_pixels", "points": pixels}
    radius = int(vision_config.get(
        config, "stereo-net", "window_radius_px", default=3))
    return {
        "sample_source": "box_window",
        "window": {"u": detection["u"], "v": detection["v"], "radius": radius},
    }


def detect(left_jpeg, right_jpeg, target, config, client=None,
           depth_client_override=None):
    """Find the target in the left eye and read its depth from a dense map."""
    start = time.time()
    classes = open_vocab_stereo.target_classes(target)
    name = model_name(config)
    if not classes:
        return _not_visible(
            "No text target supplied; nothing to detect.", config, name,
            detector_name(config), 0.0, time.time() - start)

    detector = yolo_client.detect_one if client is None else client
    depth_service = (depth_client_override if depth_client_override is not None
                     else depth_client.sample)
    detections, detector_model = detector(left_jpeg, classes, config)
    detector_model = detector_name(config, detector_model)

    detection = _best_detection(detections)
    if detection is None:
        return _not_visible(
            "No %s detection in the left rectified image." % classes[0],
            config, name, detector_model, 0.0, time.time() - start)

    spec = _sample_spec(detection, config)
    sampled = depth_service(left_jpeg, right_jpeg, spec, config)
    if not isinstance(sampled, dict):
        sampled = {}
    depth_m = sampled.get("depth_m")
    source = sampled.get("sample_source") or spec["sample_source"]
    confidence = float(detection.get("confidence", 0.0))

    details = {
        "bbox_px": detection.get("bbox_px"),
        "center_source": detection.get("center_source") or "box_center",
        "mask_centroid_px": detection.get("mask_centroid_px"),
        "mask_area_px": detection.get("mask_area_px", 0),
        "depth_sample_source": source,
        "depth_sample_count": sampled.get("sample_count", 0),
        "depth_valid_count": sampled.get("valid_count", 0),
        "depth_disparity_px": sampled.get("disparity_px"),
        "depth_raw_disparity_px": sampled.get("raw_disparity_px"),
        "depth_disparity_offset_px": sampled.get("disparity_offset_px"),
    }
    if depth_m is None:
        return _not_visible(
            "Detected '%s' but the dense depth map had no valid disparity at "
            "the sampled pixels." % classes[0], config,
            sampled.get("model") or name, detector_model, confidence,
            time.time() - start, "dense_stereo_depth")

    camera = config.get("camera", {}) if isinstance(config, dict) else {}
    minimum = float(camera.get("depth_min_m", 0.0))
    maximum = float(camera.get("depth_max_m", 0.0))
    if (minimum and float(depth_m) < minimum) or (maximum and float(depth_m) > maximum):
        return _not_visible(
            "Detected '%s' but dense depth %.3f m is outside the calibrated "
            "range." % (classes[0], float(depth_m)), config,
            sampled.get("model") or name, detector_model, confidence,
            time.time() - start, "dense_stereo_depth")

    image_width = open_vocab_stereo.jpeg_size(left_jpeg)[0]
    bearing = vision_config.lateral_bearing_deg(
        detection["u"], config, image_width)
    distance_m = round(float(depth_m), 3)
    center_offset = vision_config.center_offset_m(
        detection["u"], distance_m, config, image_width)
    answer = vision_answer.tidy({
        "description": (
            "Detected '%s' in the left rectified image at u=%.1f, v=%.1f px; "
            "dense %s depth %.3f m from %d %s sample(s)."
            % (detection.get("label", classes[0]), detection["u"],
               detection["v"], sampled.get("model") or name, float(depth_m),
               details["depth_valid_count"], source)),
        "target_visible": True,
        "lateral_bearing_deg": bearing,
        "center_offset_m": center_offset,
        "distance_m": distance_m,
        "obstacle_ahead": "none",
    }, config)
    answer.update(details)
    return _entry(answer, sampled.get("model") or name, detector_model,
                  confidence, time.time() - start, "dense_stereo_depth")
