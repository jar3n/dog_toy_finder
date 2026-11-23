"""HTTP client for the host-side dense-stereo depth service.

The ROS container has no torch or dense-stereo checkpoints. For the ``second``
pipeline it sends the two rectified JPEGs plus the pixels to sample (all mask
pixels, or a small window around a box centre) to the service on the computer
and receives the median disparity and derived depth.

``STEREO_SERVICE_URL`` overrides the ``second.service_url`` config key when set
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
            or vision_config.get(config, "second", "service_url",
                                 default=DEFAULT_SERVICE_URL)
            or DEFAULT_SERVICE_URL)


def sample(left_jpeg, right_jpeg, sample_spec, config):
    """Return the host's depth sampling result for one rectified pair."""
    url = service_url(config).rstrip("/") + "/depth"
    timeout = float(vision_config.get(
        config, "second", "timeout_s", default=180))
    camera = config.get("camera", {}) if isinstance(config, dict) else {}
    payload = {
        "images": [base64.b64encode(jpeg).decode("ascii")
                   for jpeg in (left_jpeg, right_jpeg)],
        "fx": float(camera.get("fx", 250.0)),
        "baseline_m": float(camera.get("baseline_m", 0.08)),
        "depth_min_m": float(camera.get("depth_min_m", 0.0)),
        "depth_max_m": float(camera.get("depth_max_m", 0.0)),
        "max_disp": int(vision_config.get(
            config, "second", "max_disparity", default=192)),
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
