#!/usr/bin/env python3
"""Host-side open-vocabulary detector service for the open_vocab pipeline.

Runs on the computer (not in the ROS 2 container) so torch, ultralytics,
transformers and the model weights stay out of the container image. The
container sends one or two rectified JPEGs plus text class prompts to
``POST /detect``; this service runs the selected detector on each image and
returns plain-Python detection dictionaries. ``open_vocab`` sends both eyes for
stereo matching in the container; ``stereo-net`` sends only the left eye and asks
for mask-pixel samples for the host depth service.

One detector runs at a time, selected on the host with ``DETECTOR_MODEL`` or
``--detector``: ``yolo_world``, ``yoloe_26``, or ``grounding_dino``.

Usage (see scripts/start_yolo_service.sh for venv/weight setup)::

    DETECTOR_MODEL=yoloe_26 ./scripts/start_yolo_service.sh

Endpoints::

    GET  /health  -> {"ok": true, "detector": ..., "model": ..., "weights": ...}
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

HOST_VISION = Path(__file__).resolve().parent
REPO_ROOT = HOST_VISION.parents[1]
sys.path.insert(0, str(REPO_ROOT / "software" / "host_vision"))

import detectors  # noqa: E402

MODELS_DIR = HOST_VISION / "models"
DEFAULT_PORT = 5006
WEIGHTS_ENV = {
    detectors.YOLO_WORLD: "YOLO_MODEL_PATH",
    detectors.YOLOE_26: "YOLOE_MODEL_PATH",
    detectors.GROUNDING_DINO: "GROUNDING_DINO_MODEL",
}

_CONFIG = {}
_MODEL = None
_MODEL_NAME = ""
_WEIGHTS = ""
_DETECTOR = detectors.YOLO_WORLD


def default_weights(detector):
    """Return the local weights path (or HF model id) for one backend."""
    detector = detectors.detector_name(detector)
    configured = os.environ.get(WEIGHTS_ENV[detector])
    if configured:
        return configured
    filename = detectors.DEFAULT_WEIGHTS[detector]
    if detector == detectors.GROUNDING_DINO:
        return filename  # Hugging Face model id, cached under HF_HOME
    return str(MODELS_DIR / filename)


def build_config(args):
    """Return the detector settings used by the shared host detector helpers."""
    return {
        "detector": args.detector,
        "model_path": args.model,
        "device": args.device,
        "confidence_threshold": args.confidence,
        "text_threshold": args.text_threshold,
        "max_detections": args.max_detections,
    }


def _apply_request(config, body):
    """Allow a request to override only the detector thresholds."""
    overrides = dict(config)
    for key in ("confidence_threshold", "max_detections", "text_threshold"):
        if key in body:
            overrides[key] = body[key]
    return overrides


def detect(body):
    """Run both eyes and return ``(model_name, detections)``."""
    global _MODEL
    images = body.get("images")
    if not isinstance(images, list) or len(images) not in (1, 2):
        raise ValueError(
            "'images' must be a list of one or two base64 JPEGs")

    classes = body.get("classes")
    if not isinstance(classes, list) or not classes:
        target = str(body.get("target") or "").strip()
        classes = [part.strip() for part in target.replace(" or ", ",").split(",")]
        classes = [part for part in classes if part]
    if not classes:
        raise ValueError("'classes' or 'target' must name the text target")

    config = _apply_request(_CONFIG, body)
    if _MODEL is None:
        _MODEL = detectors.load_detector(config)
    detections = []
    for encoded in images:
        rgb = detectors.decode_jpeg(base64.b64decode(encoded))
        detections.append(
            detectors.detections_for_image(_MODEL, rgb, classes, config))

    # The stereo-net pipeline needs the mask pixels for dense-depth sampling. Keep
    # them off the normal open_vocab response so those answers stay small.
    if not body.get("include_depth_samples"):
        for eye in detections:
            for detection in eye:
                detection.pop("depth_sample_pixels", None)
    return _MODEL_NAME, detections


class Handler(BaseHTTPRequestHandler):
    """Minimal JSON API; stdlib only so the host needs no web framework."""

    server_version = "DtfDetectorService/1.1"

    def log_message(self, fmt, *args):
        sys.stderr.write("[detector] %s - %s\n" % (self.address_string(), fmt % args))

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
        self._json(200, {
            "ok": True,
            "detector": _DETECTOR,
            "model": _MODEL_NAME,
            "weights": _WEIGHTS,
        })

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
            sys.stderr.write("[detector] detection failed: %s\n" % error)
            self._json(500, {"error": "detection failed: %s" % error})


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1",
                        help="bind address (127.0.0.1 is enough with host networking)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help="TCP port (default: %d)" % DEFAULT_PORT)
    parser.add_argument("--detector", default=os.environ.get(
        "DETECTOR_MODEL", detectors.YOLO_WORLD), choices=detectors.DETECTORS,
        help="open-vocabulary backend (default: $DETECTOR_MODEL / yolo_world)")
    parser.add_argument("--model", default=None,
                        help="weights path or HF model id (default: per-detector "
                             "YOLO_MODEL_PATH / YOLOE_MODEL_PATH / "
                             "GROUNDING_DINO_MODEL, then built-in small variant)")
    parser.add_argument("--name", default=None,
                        help="model name recorded in results (default: weights filename)")
    parser.add_argument("--device", default=os.environ.get("YOLO_DEVICE", ""),
                        help="optional torch device, e.g. cpu or cuda:0")
    parser.add_argument("--confidence", type=float, default=0.25,
                        help="default box confidence threshold")
    parser.add_argument("--text-threshold", type=float,
                        default=float(os.environ.get(
                            "GROUNDING_DINO_TEXT_THRESHOLD", "0.25")),
                        help="Grounding DINO text threshold (default: 0.25)")
    parser.add_argument("--max-detections", type=int, default=10,
                        help="default maximum detections per image")
    args = parser.parse_args(argv)
    if args.model is None:
        args.model = default_weights(args.detector)
    if args.name is None:
        args.name = Path(args.model.replace("\\", "/")).stem or args.detector
    return args


def main(argv=None):
    global _CONFIG, _MODEL, _MODEL_NAME, _WEIGHTS, _DETECTOR
    args = parse_args(argv)
    _CONFIG = build_config(args)
    _MODEL = None
    _MODEL_NAME = args.name
    _WEIGHTS = args.model
    _DETECTOR = args.detector

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print("Detector service on http://%s:%d -> %s / %s (model %s)"
          % (args.host, args.port, args.detector, args.model, args.name),
          flush=True)
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
