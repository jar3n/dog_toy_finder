#!/usr/bin/env python3
"""Host-side YOLO-world detector service for the open_vocab vision pipeline.

Runs on the computer (not in the ROS 2 container) so torch/ultralytics and the
YOLO-world weights stay out of the container image. The container sends two
rectified JPEGs plus the text class prompts to ``POST /detect``; this service
loads the weights once, runs the detector on each eye, and returns plain-Python
detection dictionaries. Stereo matching and triangulation stay in the container.

Usage (see scripts/start_yolo_service.sh for venv/weight setup)::

    python3 software/host_vision/yolo_service.py \
        --model software/host_vision/models/yolov8s-world.pt --port 5006

Endpoints::

    GET  /health  -> {"ok": true, "model": ..., "weights": ...}
    POST /detect   -> {"model": ..., "detections": [[left...], [right...]]}
"""

import argparse
import base64
import json
import os
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "software" / "ros2_ws" / "src"))

from dog_toy_finder_vision import yolo_detector  # noqa: E402

DEFAULT_MODEL = str(
    REPO_ROOT / "software" / "host_vision" / "models" / "yolov8s-world.pt")
DEFAULT_PORT = 5006

_CONFIG = {}
_MODEL = None
_MODEL_NAME = ""
_WEIGHTS = ""


def build_config(args):
    """Return the detector settings used by the shared yolo_detector helpers."""
    return {
        "open_vocab": {
            "name": args.name,
            "model_path": args.model,
            "device": args.device,
            "confidence_threshold": args.confidence,
            "max_detections": args.max_detections,
        }
    }


def _apply_request(config, body):
    """Allow a request to override only the detector thresholds."""
    overrides = dict(config["open_vocab"])
    for key in ("confidence_threshold", "max_detections"):
        if key in body:
            overrides[key] = body[key]
    return {"open_vocab": overrides}


def detect(body):
    """Run both eyes and return ``(model_name, detections)``."""
    global _MODEL
    images = body.get("images")
    if not isinstance(images, list) or len(images) != 2:
        raise ValueError("'images' must be a list of exactly two base64 JPEGs")

    classes = body.get("classes")
    if not isinstance(classes, list) or not classes:
        target = str(body.get("target") or "").strip()
        classes = [part.strip() for part in target.replace(" or ", ",").split(",")]
        classes = [part for part in classes if part]
    if not classes:
        raise ValueError("'classes' or 'target' must name the text target")

    config = _apply_request(_CONFIG, body)
    if _MODEL is None:
        _MODEL = yolo_detector.load_model(config)
    detections = []
    for encoded in images:
        rgb = yolo_detector.decode_jpeg(base64.b64decode(encoded))
        detections.append(
            yolo_detector.detections_for_image(_MODEL, rgb, classes, config))
    return _MODEL_NAME, detections


class Handler(BaseHTTPRequestHandler):
    """Minimal JSON API; stdlib only so the host needs no web framework."""

    server_version = "DtfYoloService/1.0"

    def log_message(self, fmt, *args):
        sys.stderr.write("[yolo] %s - %s\n" % (self.address_string(), fmt % args))

    def _json(self, status, payload):
        data = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.split("?")[0] != "/health":
            self._json(404, {"error": "not found"})
            return
        self._json(200, {"ok": True, "model": _MODEL_NAME, "weights": _WEIGHTS})

    def do_POST(self):
        if self.path.split("?")[0] != "/detect":
            self._json(404, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length).decode("utf-8"))
            start = time.time()
            name, detections = detect(body)
            self._json(200, {
                "model": name,
                "detections": detections,
                "seconds": round(time.time() - start, 3),
            })
        except ValueError as error:
            self._json(400, {"error": str(error)})
        except Exception as error:  # keep the service alive across bad frames
            sys.stderr.write("[yolo] detection failed: %s\n" % error)
            self._json(500, {"error": "detection failed: %s" % error})


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1",
                        help="bind address (127.0.0.1 is enough with host networking)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help="TCP port (default: %d)" % DEFAULT_PORT)
    parser.add_argument("--model", default=os.environ.get("YOLO_MODEL_PATH", DEFAULT_MODEL),
                        help="YOLO-world weights path on the computer")
    parser.add_argument("--name", default=None,
                        help="model name recorded in results (default: weights filename)")
    parser.add_argument("--device", default=os.environ.get("YOLO_DEVICE", ""),
                        help="optional torch device, e.g. cpu or cuda:0")
    parser.add_argument("--confidence", type=float, default=0.25,
                        help="default detection confidence threshold")
    parser.add_argument("--max-detections", type=int, default=10,
                        help="default maximum detections per image")
    return parser.parse_args(argv)


def main(argv=None):
    global _CONFIG, _MODEL_NAME, _WEIGHTS
    args = parse_args(argv)
    if args.name is None:
        args.name = Path(args.model).stem or "yolov8s-world"
    _CONFIG = build_config(args)
    _MODEL_NAME = args.name
    _WEIGHTS = args.model

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print("YOLO service on http://%s:%d -> %s (model %s)"
          % (args.host, args.port, args.model, args.name), flush=True)
    print("Load happens on the first /detect request; GET /health for status.",
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
