"""HTTP client for the host-side open-vocabulary detector service.

The ROS container has no torch/ultralytics installation. It sends rectified
JPEGs and text class prompts to the service running on the computer (host
networking makes ``localhost`` reachable from the container), and receives plain
Python detection dictionaries back. ``detect`` sends both eyes for triangulation;
``detect_one`` sends only the left eye for the dense-stereo ``stereo-net`` pipeline.
``unload`` releases the host model between pipeline configurations.

``YOLO_SERVICE_URL`` overrides the ``open_vocab.service_url`` config key when
set in the container environment.
"""

import base64
import json
import os
import urllib.error
import urllib.request

from . import vision_config

DEFAULT_SERVICE_URL = "http://localhost:5006"


def service_url(config):
    """Return the detector service base URL from env, config, or default."""
    return (os.environ.get("YOLO_SERVICE_URL")
            or vision_config.get(config, "open_vocab", "service_url",
                                 default=DEFAULT_SERVICE_URL)
            or DEFAULT_SERVICE_URL)


def configure(config):
    """Tell the already-running host service which detector backend to load.

    The service process starts without a model selection; the ROS side owns that
    choice via ``open_vocab.detector`` (or the legacy ``open_vocab.name`` alias).
    """
    url = service_url(config).rstrip("/") + "/configure"
    timeout = float(vision_config.get(
        config, "open_vocab", "timeout_s", default=60))
    payload = {
        "detector": vision_config.detector_backend(config),
        "name": vision_config.detector_display_name(config),
        "confidence_threshold": float(vision_config.get(
            config, "open_vocab", "confidence_threshold", default=0.25)),
        "max_detections": int(vision_config.get(
            config, "open_vocab", "max_detections", default=10)),
        "text_threshold": float(vision_config.get(
            config, "open_vocab", "text_threshold", default=0.25)),
    }
    device = vision_config.get(config, "open_vocab", "device", default="")
    if device:
        payload["device"] = device
    request = urllib.request.Request(
        url, data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", "replace")[:500]
        raise RuntimeError(
            "YOLO detector service at %s returned HTTP %s on /configure: %s"
            % (url, error.code, detail)) from error
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise RuntimeError(
            "Cannot reach the YOLO detector service at %s to configure it. "
            "Start it with ./scripts/start_yolo_service.sh (%s)"
            % (url, error)) from error
    if not isinstance(body, dict) or not body.get("ok"):
        raise RuntimeError("YOLO detector service at %s returned %r on "
                           "/configure" % (url, body))
    return body


def _post_detect(jpegs, classes, config, include_depth_samples):
    """Send one or two JPEGs to the host service and return its response body."""
    url = service_url(config).rstrip("/") + "/detect"
    timeout = float(vision_config.get(
        config, "open_vocab", "timeout_s", default=60))
    payload = {
        "images": [base64.b64encode(jpeg).decode("ascii") for jpeg in jpegs],
        "classes": list(classes),
        "confidence_threshold": float(vision_config.get(
            config, "open_vocab", "confidence_threshold", default=0.25)),
        "max_detections": int(vision_config.get(
            config, "open_vocab", "max_detections", default=10)),
        "include_depth_samples": bool(include_depth_samples),
    }
    request = urllib.request.Request(
        url, data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", "replace")[:500]
        raise RuntimeError(
            "YOLO detector service at %s returned HTTP %s: %s"
            % (url, error.code, detail)) from error
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise RuntimeError(
            "Cannot reach the YOLO detector service at %s. Start it on the "
            "computer with ./scripts/start_yolo_service.sh (%s)"
            % (url, error)) from error


def detect(left_jpeg, right_jpeg, classes, config, include_depth_samples=False):
    """Return ``(left_detections, right_detections, model_name)`` from the host."""
    body = _post_detect((left_jpeg, right_jpeg), classes, config,
                        include_depth_samples)
    url = service_url(config)
    detections = body.get("detections")
    if (not isinstance(detections, list) or len(detections) != 2
            or any(not isinstance(eye, list) for eye in detections)):
        raise RuntimeError(
            "YOLO detector service at %s returned an invalid detections list"
            % url)
    return detections[0], detections[1], body.get("model")


def detect_one(jpeg, classes, config, include_depth_samples=True):
    """Return ``(detections, model_name)`` for a single (left) image."""
    body = _post_detect((jpeg,), classes, config, include_depth_samples)
    url = service_url(config)
    detections = body.get("detections")
    if (not isinstance(detections, list) or len(detections) != 1
            or not isinstance(detections[0], list)):
        raise RuntimeError(
            "YOLO detector service at %s returned an invalid single-image "
            "detections list" % url)
    return detections[0], body.get("model")


def unload(config):
    """Ask the host service to release its loaded detector model."""
    url = service_url(config).rstrip("/") + "/unload"
    timeout = float(vision_config.get(
        config, "open_vocab", "timeout_s", default=60))
    request = urllib.request.Request(
        url, data=b"{}", headers={"Content-Type": "application/json"},
        method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", "replace")[:500]
        raise RuntimeError(
            "YOLO detector service at %s returned HTTP %s on /unload: %s"
            % (url, error.code, detail)) from error
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise RuntimeError(
            "Cannot reach the YOLO detector service at %s to unload it (%s)"
            % (url, error)) from error
    if not isinstance(body, dict) or not body.get("ok"):
        raise RuntimeError("YOLO detector service at %s returned %r on /unload"
                           % (url, body))
    return body
