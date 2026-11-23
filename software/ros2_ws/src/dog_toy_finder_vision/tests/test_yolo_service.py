"""Tests for the host-side YOLO HTTP service's detection core.

No ultralytics weights are needed: a fake detector returns a box around the red
disc in each synthetic image through the same interface as the real model.
"""

import base64
import importlib.util
import io
import os
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

import numpy as np
from PIL import Image

SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, SRC_ROOT)
from dog_toy_finder_vision import vision_config  # noqa: E402
from dog_toy_finder_vision import yolo_client  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[5]
SERVICE_PATH = REPO_ROOT / "software" / "host_vision" / "yolo_service.py"

spec = importlib.util.spec_from_file_location("dtf_yolo_service", SERVICE_PATH)
yolo_service = importlib.util.module_from_spec(spec)
spec.loader.exec_module(yolo_service)


class _Boxes:
    def __init__(self, xyxy):
        self.xyxy = np.asarray([xyxy], dtype=float)
        self.conf = np.asarray([0.95], dtype=float)
        self.cls = np.asarray([0], dtype=float)

    def __len__(self):
        return len(self.xyxy)


class _Result:
    def __init__(self, xyxy=None):
        self.boxes = _Boxes(xyxy) if xyxy is not None else None
        self.names = {}


class RedDotModel:
    def __init__(self, name="fake-world"):
        self.name = name
        self.classes = []

    def set_classes(self, classes):
        self.classes = list(classes)

    def __call__(self, rgb, conf=None, max_det=None, device=None, verbose=None):
        mask = ((rgb[:, :, 0] > 200) & (rgb[:, :, 1] < 80)
                & (rgb[:, :, 2] < 80))
        ys, xs = np.nonzero(mask)
        if len(xs) == 0:
            return [_Result()]
        xyxy = [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1]
        return [_Result(xyxy)]


def _jpeg(center_x):
    image = np.full((240, 320, 3), 210, dtype=np.uint8)
    yy, xx = np.ogrid[:240, :320]
    image[(xx - center_x) ** 2 + (yy - 120) ** 2 <= 25] = [255, 0, 0]
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="JPEG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


class HostServiceTests(unittest.TestCase):
    def setUp(self):
        yolo_service._CONFIG = {
            "open_vocab": {"name": "fake-world", "model_path": "fake.pt",
                           "device": "", "confidence_threshold": 0.25,
                           "max_detections": 10}}
        yolo_service._MODEL_NAME = "fake-world"
        yolo_service._MODEL = None

    def test_build_config_uses_cli_values(self):
        args = argparse_namespace()
        config = yolo_service.build_config(args)
        self.assertEqual(config["open_vocab"]["model_path"], "/tmp/weights.pt")
        self.assertEqual(config["open_vocab"]["name"], "custom")
        self.assertEqual(config["open_vocab"]["max_detections"], 7)

    def test_detect_returns_one_detection_list_per_eye(self):
        with mock.patch.object(yolo_service.yolo_detector, "load_model",
                               return_value=RedDotModel()):
            name, detections = yolo_service.detect({
                "images": [_jpeg(90), _jpeg(70)],
                "classes": ["red ball"],
            })
        self.assertEqual(name, "fake-world")
        self.assertEqual(len(detections), 2)
        self.assertAlmostEqual(detections[0][0]["u"], 90.0, delta=1.0)
        self.assertAlmostEqual(detections[1][0]["u"], 70.0, delta=1.0)
        self.assertEqual(detections[0][0]["label"], "red ball")

    def test_request_threshold_overrides_are_applied(self):
        model = RedDotModel()
        seen = {}

        def capture_config(config):
            seen.update(config["open_vocab"])
            return model

        with mock.patch.object(yolo_service.yolo_detector, "load_model",
                               side_effect=capture_config):
            yolo_service.detect({"images": [_jpeg(90), _jpeg(70)],
                                 "target": "red ball",
                                 "confidence_threshold": 0.7,
                                 "max_detections": 3})
        self.assertEqual(seen["confidence_threshold"], 0.7)
        self.assertEqual(seen["max_detections"], 3)

    def test_http_round_trip_with_container_client(self):
        """The real HTTP path works without a model install or ROS graph."""
        with mock.patch.object(yolo_service.yolo_detector, "load_model",
                               return_value=RedDotModel()):
            server = ThreadingHTTPServer(("127.0.0.1", 0), yolo_service.Handler)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            self.addCleanup(server.server_close)
            self.addCleanup(server.shutdown)
            url = "http://127.0.0.1:%d" % server.server_port
            config = vision_config.load_config()
            with mock.patch.dict(os.environ, {"YOLO_SERVICE_URL": url}):
                left, right, name = yolo_client.detect(
                    base64.b64decode(_jpeg(90)),
                    base64.b64decode(_jpeg(70)),
                    ["red ball"], config)
        self.assertEqual(name, "fake-world")
        self.assertAlmostEqual(left[0]["u"], 90.0, delta=1.0)
        self.assertAlmostEqual(right[0]["u"], 70.0, delta=1.0)

    def test_invalid_requests_are_rejected(self):
        with self.assertRaises(ValueError):
            yolo_service.detect({"images": ["only-one"]})
        with self.assertRaises(ValueError):
            yolo_service.detect({"images": [_jpeg(90), _jpeg(70)]})


def argparse_namespace():
    class Args:
        name = "custom"
        model = "/tmp/weights.pt"
        device = "cpu"
        confidence = 0.4
        max_detections = 7
    return Args()


if __name__ == "__main__":
    unittest.main()
