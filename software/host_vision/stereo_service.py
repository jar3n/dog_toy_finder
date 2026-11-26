#!/usr/bin/env python3
"""Host-side dense-stereo depth service for the ``stereo-net`` pipeline.

Runs on the computer (not in the ROS 2 container) so torch, the upstream model
repositories, and their weights stay out of the container image. The container
sends two rectified JPEGs plus one sampling request; this service computes a
dense disparity map with the selected model and returns the median disparity and
derived depth at the requested pixels.

One model runs at a time, selected on the host with ``DEPTH_MODEL`` or
``--model``: ``lite_any_stereo``, ``fast_foundation_stereo``, or ``rt_igevpp``.

Usage (see scripts/start_stereo_service.sh for venv/weight setup)::

    DEPTH_MODEL=lite_any_stereo ./scripts/start_stereo_service.sh

Endpoints::

    GET  /health  -> {"ok": true, "model": ..., "weights": ...}
    POST /depth   -> {"model": ..., "sample_source": ..., "depth_m": ...}
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
sys.path.insert(0, str(HOST_VISION))

import dense_stereo  # noqa: E402
import detectors  # noqa: E402

DEFAULT_PORT = 5007

_CONFIG = {}
_MODEL = None
_BACKEND = dense_stereo.DEFAULT_MODEL
_MODEL_NAME = ""
_WEIGHTS = ""


def default_weights(name):
    """Return the local weights path for one dense-stereo backend."""
    return dense_stereo.default_weights(name)


def build_config(args):
    """Return the dense-stereo settings shared with the adapter helpers."""
    return {
        "model": args.model,
        "model_path": args.model_path,
        "name": args.name,
        "device": args.device,
        "iters": args.iters,
        "max_disp": args.max_disp,
        "camera": {},
    }


def _request_config(body):
    """Apply the container's calibration to a copy of the host config."""
    config = dict(_CONFIG)
    camera = dict(config.get("camera") or {})
    for key in ("fx", "baseline_m", "disparity_offset_px",
                "depth_min_m", "depth_max_m"):
        if key in body:
            camera[key] = body[key]
    config["camera"] = camera
    for key in ("max_disp", "iters"):
        if key in body:
            config[key] = body[key]
    return config


def depth(body):
    """Compute a dense disparity map and sample it at the requested pixels."""
    global _BACKEND, _MODEL
    images = body.get("images")
    if not isinstance(images, list) or len(images) != 2:
        raise ValueError("'images' must be a list of two base64 JPEGs")
    points, window = body.get("points"), body.get("window")
    if not points and not window:
        raise ValueError("either 'points' or 'window' must name depth samples")

    config = _request_config(body)
    if _MODEL is None:
        _BACKEND, _MODEL = dense_stereo.load_model(config)
    left = detectors.decode_jpeg(base64.b64decode(images[0]))
    right = detectors.decode_jpeg(base64.b64decode(images[1]))
    disparity = dense_stereo.disparity_map(
        _BACKEND, _MODEL, left, right, config)
    sampled = dense_stereo.sample_disparity(
        disparity, points=points, window=window, config=config)
    return _MODEL_NAME, sampled


class Handler(BaseHTTPRequestHandler):
    """Minimal JSON API; stdlib only so the host needs no web framework."""

    server_version = "DtfStereoService/1.0"

    def log_message(self, fmt, *args):
        sys.stderr.write("[stereo] %s - %s\n" % (self.address_string(), fmt % args))

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
            "model": _BACKEND,
            "name": _MODEL_NAME,
            "weights": _WEIGHTS,
        })

    def do_POST(self):
        if self.path.split("?")[0] != "/depth":
            self._json(404, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length).decode("utf-8"))
            start = time.time()
            name, sampled = depth(body)
            sampled["model"] = name
            sampled["seconds"] = round(time.time() - start, 3)
            self._json(200, sampled)
        except ValueError as error:
            self._json(400, {"error": str(error)})
        except Exception as error:  # keep the service alive across bad frames
            sys.stderr.write("[stereo] depth failed: %s\n" % error)
            self._json(500, {"error": "depth failed: %s" % error})


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1",
                        help="bind address (127.0.0.1 is enough with host networking)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT,
                        help="TCP port (default: %d)" % DEFAULT_PORT)
    parser.add_argument("--model", default=os.environ.get(
        "DEPTH_MODEL", dense_stereo.DEFAULT_MODEL),
        choices=dense_stereo.MODELS,
        help="dense stereo backend (default: $DEPTH_MODEL / lite_any_stereo)")
    parser.add_argument("--model-path", default=None,
                        help="checkpoint path (default: per-model local path or "
                             "its *_MODEL_PATH environment variable)")
    parser.add_argument("--name", default=None,
                        help="model name recorded in results (default: weights filename)")
    parser.add_argument("--device", default=os.environ.get("STEREO_DEVICE", ""),
                        help="torch device, e.g. cpu or cuda:0 (default: auto)")
    parser.add_argument("--iters", type=int, default=None,
                        help="update iterations (default: 8 Fast, 6 RT, 1 Lite)")
    parser.add_argument("--max-disp", type=int, default=192,
                        help="maximum disparity searched (default: 192)")
    args = parser.parse_args(argv)
    if args.model_path is None:
        args.model_path = default_weights(args.model)
    if args.iters is None:
        args.iters = dense_stereo.DEFAULT_ITERS[args.model]
    if args.name is None:
        args.name = Path(args.model_path.replace("\\", "/")).stem or args.model
    return args


def main(argv=None):
    global _CONFIG, _MODEL, _MODEL_NAME, _WEIGHTS, _BACKEND
    args = parse_args(argv)
    _CONFIG = build_config(args)
    _MODEL = None
    _MODEL_NAME = args.name
    _WEIGHTS = args.model_path
    _BACKEND = args.model

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print("Stereo depth service on http://%s:%d -> %s / %s (model %s)"
          % (args.host, args.port, args.model, args.model_path, args.name),
          flush=True)
    print("Load happens on the first /depth request; GET /health for status.",
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
