"""Tests for the host-side open-vocabulary HTTP service.

No model weights are needed: a fake detector finds the red disc in each
synthetic image. The tests cover backend selection on the host, the unchanged
HTTP contract used by the container, and the Grounding DINO call conversion.
"""

import base64
import contextlib
import importlib.util
import io
import json
import os
import sys
import threading
import types
import unittest
import urllib.request
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
HOST_VISION = REPO_ROOT / "software" / "host_vision"
sys.path.insert(0, str(HOST_VISION))
import detectors  # noqa: E402

spec = importlib.util.spec_from_file_location(
    "dtf_detector_service", HOST_VISION / "yolo_service.py")
service = importlib.util.module_from_spec(spec)
spec.loader.exec_module(service)


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


class _FakeBatch(dict):
    input_ids = ["input-ids"]

    def to(self, device):
        return self


class _FakeDinoProcessor:
    def __call__(self, images, text, return_tensors):
        return _FakeBatch()

    def post_process_grounded_object_detection(self, outputs, input_ids,
                                               box_threshold, text_threshold,
                                               target_sizes):
        return [{"boxes": [[80.0, 110.0, 100.0, 130.0]],
                 "scores": [0.9], "labels": ["red ball"]}]


class _FakeDinoNetwork:
    def __call__(self, **inputs):
        return "outputs"


def _jpeg(center_x):
    image = np.full((240, 320, 3), 210, dtype=np.uint8)
    yy, xx = np.ogrid[:240, :320]
    image[(xx - center_x) ** 2 + (yy - 120) ** 2 <= 25] = [255, 0, 0]
    buffer = io.BytesIO()
    Image.fromarray(image).save(buffer, format="JPEG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def _args(detector="yolo_world", model="/tmp/weights.pt"):
    class Args:
        pass
    args = Args()
    args.detector = detector
    args.model = model
    args.name = "custom"
    args.device = "cpu"
    args.confidence = 0.4
    args.text_threshold = 0.3
    args.max_detections = 7
    return args


class ServiceConfigTests(unittest.TestCase):
    def test_build_config_includes_backend_and_thresholds(self):
        config = service.build_config(_args(detector="grounding_dino"))
        self.assertEqual(config["detector"], "grounding_dino")
        self.assertEqual(config["model_path"], "/tmp/weights.pt")
        self.assertEqual(config["confidence_threshold"], 0.4)
        self.assertEqual(config["text_threshold"], 0.3)
        self.assertEqual(config["max_detections"], 7)

    def test_default_weights_follow_selected_detector(self):
        self.assertTrue(service.default_weights("yolo_world").endswith(
            "models/yolov8s-world.pt"))
        self.assertTrue(service.default_weights("yoloe_26").endswith(
            "models/yoloe-26s-seg.pt"))
        self.assertEqual(service.default_weights("grounding_dino"),
                         "IDEA-Research/grounding-dino-tiny")

    def test_per_detector_env_overrides_weights(self):
        with mock.patch.dict(os.environ, {"YOLOE_MODEL_PATH": "/tmp/custom.pt"}):
            self.assertEqual(service.default_weights("yoloe_26"), "/tmp/custom.pt")

    def test_parse_args_selects_detector_and_derives_name(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            args = service.parse_args([])
        self.assertEqual(args.detector, "yolo_world")
        self.assertTrue(args.model.endswith("yolov8s-world.pt"))
        self.assertEqual(args.name, "yolov8s-world")

        args = service.parse_args(["--detector", "grounding_dino"])
        self.assertEqual(args.model, "IDEA-Research/grounding-dino-tiny")
        self.assertEqual(args.name, "grounding-dino-tiny")

        with mock.patch.dict(os.environ, {
                "DETECTOR_MODEL": "yoloe_26",
                "YOLOE_MODEL_PATH": "/opt/yoloe-26s-seg.pt"}):
            args = service.parse_args([])
        self.assertEqual(args.detector, "yoloe_26")
        self.assertEqual(args.model, "/opt/yoloe-26s-seg.pt")
        self.assertEqual(args.name, "yoloe-26s-seg")


class HostServiceTests(unittest.TestCase):
    def setUp(self):
        service._CONFIG = {
            "detector": "yolo_world",
            "model_path": "fake.pt",
            "device": "",
            "confidence_threshold": 0.25,
            "text_threshold": 0.25,
            "max_detections": 10,
        }
        service._MODEL_NAME = "fake-world"
        service._WEIGHTS = "fake.pt"
        service._DETECTOR = "yolo_world"
        service._MODEL = None

    def test_detect_returns_one_detection_list_per_eye(self):
        with mock.patch.object(service.detectors, "load_detector",
                               return_value=("yolo_world", RedDotModel())):
            name, detections = service.detect({
                "images": [_jpeg(90), _jpeg(70)],
                "classes": ["red ball"],
            })
        self.assertEqual(name, "fake-world")
        self.assertEqual(len(detections), 2)
        self.assertAlmostEqual(detections[0][0]["u"], 90.0, delta=1.0)
        self.assertAlmostEqual(detections[1][0]["u"], 70.0, delta=1.0)
        self.assertEqual(detections[0][0]["label"], "red ball")
        self.assertEqual(detections[0][0]["center_source"], "box_center")
        self.assertIn("mask_centroid_px", detections[0][0])

    def test_request_threshold_overrides_are_applied(self):
        seen = {}

        def capture_config(config):
            seen.update(config)
            return "yolo_world", RedDotModel()

        with mock.patch.object(service.detectors, "load_detector",
                               side_effect=capture_config):
            service.detect({"images": [_jpeg(90), _jpeg(70)],
                            "target": "red ball",
                            "confidence_threshold": 0.7,
                            "max_detections": 3})
        self.assertEqual(seen["confidence_threshold"], 0.7)
        self.assertEqual(seen["max_detections"], 3)

    def test_grounding_dino_backend_is_used_when_selected(self):
        service._CONFIG["detector"] = "grounding_dino"
        seen = {}

        def capture_config(config):
            seen.update(config)
            return "grounding_dino", (_FakeDinoProcessor(), _FakeDinoNetwork())

        fake_torch = types.SimpleNamespace(no_grad=contextlib.nullcontext)
        with mock.patch.dict(sys.modules, {"torch": fake_torch}), \
                mock.patch.object(service.detectors, "load_detector",
                                  side_effect=capture_config):
            service.detect({"images": [_jpeg(90), _jpeg(70)],
                            "classes": ["red ball"]})
        self.assertEqual(seen["detector"], "grounding_dino")
        self.assertIn("text_threshold", seen)

    def test_http_round_trip_with_container_client(self):
        """The real HTTP path works without a model install or ROS graph."""
        with mock.patch.object(service.detectors, "load_detector",
                               return_value=("yolo_world", RedDotModel())):
            server = ThreadingHTTPServer(("127.0.0.1", 0), service.Handler)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            self.addCleanup(server.server_close)
            self.addCleanup(server.shutdown)
            url = "http://127.0.0.1:%d" % server.server_port
            with urllib.request.urlopen(url + "/health") as response:
                health = json.load(response)
            self.assertEqual(health["detector"], "yolo_world")
            config = vision_config.load_config()
            with mock.patch.dict(os.environ, {"YOLO_SERVICE_URL": url}):
                left, right, name = yolo_client.detect(
                    base64.b64decode(_jpeg(90)),
                    base64.b64decode(_jpeg(70)),
                    ["red ball"], config)
        self.assertEqual(name, "fake-world")
        self.assertAlmostEqual(left[0]["u"], 90.0, delta=1.0)
        self.assertAlmostEqual(right[0]["u"], 70.0, delta=1.0)

    def test_single_image_request_supports_second_pipeline(self):
        """The dense pipeline asks for mask pixels on the left image only."""
        with mock.patch.object(service.detectors, "load_detector",
                               return_value=("yolo_world", RedDotModel())):
            server = ThreadingHTTPServer(("127.0.0.1", 0), service.Handler)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            self.addCleanup(server.server_close)
            self.addCleanup(server.shutdown)
            url = "http://127.0.0.1:%d" % server.server_port
            config = vision_config.load_config()
            with mock.patch.dict(os.environ, {"YOLO_SERVICE_URL": url}):
                detections, name = yolo_client.detect_one(
                    base64.b64decode(_jpeg(90)), ["red ball"], config)
        self.assertEqual(name, "fake-world")
        self.assertEqual(len(detections), 1)
        self.assertAlmostEqual(detections[0]["u"], 90.0, delta=1.0)
        self.assertIn("depth_sample_pixels", detections[0])

    def test_invalid_requests_are_rejected(self):
        with self.assertRaises(ValueError):
            service.detect({"images": []})
        with self.assertRaises(ValueError):
            service.detect({"images": [_jpeg(90), _jpeg(70)]})


if __name__ == "__main__":
    unittest.main()
