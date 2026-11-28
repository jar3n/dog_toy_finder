"""HTTP client for the host-side dense-stereo depth service.

The ROS container has no torch or dense-stereo checkpoints. For the ``stereo-net``
pipeline it sends the two rectified JPEGs plus the pixels to sample (all mask
pixels, or a small window around a box centre) to the service on the computer
and receives the median disparity and derived depth. ``unload`` releases the
host depth model between pipeline configurations.

``STEREO_SERVICE_URL`` overrides the ``stereo-net.service_url`` config key when set
in the container environment.
"""

import base64
import json
import os
import urllib.error
import urllib.request

from . import vision_config

DEFAULT_SERVICE_URL = "http://localhost:5007"


def service_url(config):
    """Return the depth service base URL from env, config, or default."""
    return (os.environ.get("STEREO_SERVICE_URL")
            or vision_config.get(config, "stereo-net", "service_url",
                                 default=DEFAULT_SERVICE_URL)
            or DEFAULT_SERVICE_URL)


def configure(config):
    """Tell the already-running host service which depth backend to load.

    The service process starts without a model selection; ``stereo-net.name``
    (or ``stereo-net.model``) in the ROS config decides which model to load.
    """
    url = service_url(config).rstrip("/") + "/configure"
    timeout = float(vision_config.get(
        config, "stereo-net", "timeout_s", default=180))
    backend = vision_config.depth_backend(config)
    payload = {
        "model": backend,
        "name": str(vision_config.get(config, "stereo-net", "name")
                    or backend),
        "max_disp": int(vision_config.get(
            config, "stereo-net", "max_disparity", default=192)),
    }
    device = vision_config.get(config, "stereo-net", "device", default="")
    if device:
        payload["device"] = device
    iterations = vision_config.get(config, "stereo-net", "iterations",
                                   default=None)
    if iterations is not None:
        payload["iters"] = int(iterations)
    request = urllib.request.Request(
        url, data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"}, method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", "replace")[:500]
        raise RuntimeError(
            "Stereo depth service at %s returned HTTP %s on /configure: %s"
            % (url, error.code, detail)) from error
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise RuntimeError(
            "Cannot reach the stereo depth service at %s to configure it. "
            "Start it with ./scripts/start_stereo_service.sh (%s)"
            % (url, error)) from error
    if not isinstance(body, dict) or not body.get("ok"):
        raise RuntimeError("Stereo depth service at %s returned %r on /configure"
                           % (url, body))
    return body


def sample(left_jpeg, right_jpeg, sample_spec, config):
    """Return the host's depth sampling result for one rectified pair."""
    url = service_url(config).rstrip("/") + "/depth"
    timeout = float(vision_config.get(
        config, "stereo-net", "timeout_s", default=180))
    camera = config.get("camera", {}) if isinstance(config, dict) else {}
    payload = {
        "images": [base64.b64encode(jpeg).decode("ascii")
                   for jpeg in (left_jpeg, right_jpeg)],
        "fx": vision_config.eye_intrinsics(config, "left")["fx"],
        "baseline_m": float(camera.get("baseline_m", 0.08)),
        "disparity_offset_px": vision_config.stereo_disparity_offset_px(config),
        "depth_min_m": float(camera.get("depth_min_m", 0.0)),
        "depth_max_m": float(camera.get("depth_max_m", 0.0)),
        "max_disp": int(vision_config.get(
            config, "stereo-net", "max_disparity", default=192)),
        "points": sample_spec.get("points"),
        "window": sample_spec.get("window"),
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
            "Stereo depth service at %s returned HTTP %s: %s"
            % (url, error.code, detail)) from error
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise RuntimeError(
            "Cannot reach the stereo depth service at %s. Start it on the "
            "computer with ./scripts/start_stereo_service.sh (%s)"
            % (url, error)) from error

    if not isinstance(body, dict) or "error" in body:
        raise RuntimeError("Stereo depth service at %s returned %r"
                           % (url, body))
    return body


def unload(config):
    """Ask the host service to release its loaded dense-stereo model."""
    url = service_url(config).rstrip("/") + "/unload"
    timeout = float(vision_config.get(
        config, "stereo-net", "timeout_s", default=180))
    request = urllib.request.Request(
        url, data=b"{}", headers={"Content-Type": "application/json"},
        method="POST")
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            body = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as error:
        detail = error.read().decode("utf-8", "replace")[:500]
        raise RuntimeError(
            "Stereo depth service at %s returned HTTP %s on /unload: %s"
            % (url, error.code, detail)) from error
    except (urllib.error.URLError, TimeoutError, OSError) as error:
        raise RuntimeError(
            "Cannot reach the stereo depth service at %s to unload it (%s)"
            % (url, error)) from error
    if not isinstance(body, dict) or not body.get("ok"):
        raise RuntimeError("Stereo depth service at %s returned %r on /unload"
                           % (url, body))
    return body
