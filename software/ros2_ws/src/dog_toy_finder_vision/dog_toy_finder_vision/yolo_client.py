"""HTTP client for the host-side YOLO-world detector service.

The ROS container has no torch/ultralytics installation. It sends the two
rectified JPEGs and the text class prompts to the service running on the
computer (host networking makes ``localhost`` reachable from the container),
and receives one list of plain-Python detections per eye.

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


def detect(left_jpeg, right_jpeg, classes, config):
    """Return ``(left_detections, right_detections, model_name)`` from the host."""
    url = service_url(config).rstrip("/") + "/detect"
    timeout = float(vision_config.get(
        config, "open_vocab", "timeout_s", default=60))
    payload = {
        "images": [base64.b64encode(jpeg).decode("ascii")
                   for jpeg in (left_jpeg, right_jpeg)],
        "classes": list(classes),
        "confidence_threshold": float(vision_config.get(
            config, "open_vocab", "confidence_threshold", default=0.25)),
        "max_detections": int(vision_config.get(
            config, "open_vocab", "max_detections", default=10)),
    }
    request = urllib.request.Request(
        url, data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
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

    detections = body.get("detections")
    if (not isinstance(detections, list) or len(detections) != 2
            or any(not isinstance(eye, list) for eye in detections)):
        raise RuntimeError(
            "YOLO detector service at %s returned an invalid detections list"
            % url)
    return detections[0], detections[1], body.get("model")
