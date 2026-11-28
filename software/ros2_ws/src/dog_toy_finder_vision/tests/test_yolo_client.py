"""Tests for the container-side HTTP client to the host detector service."""

import json
import os
import sys
import unittest
from unittest import mock
from urllib.error import URLError

SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, SRC_ROOT)

from dog_toy_finder_vision import vision_config  # noqa: E402
from dog_toy_finder_vision import yolo_client  # noqa: E402


class _Response:
    def __init__(self, payload):
        self._data = json.dumps(payload).encode("utf-8")

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return False

    def read(self):
        return self._data


class ServiceUrlTests(unittest.TestCase):
    def test_env_overrides_config(self):
        config = {"open_vocab": {"service_url": "http://from-yaml:1"}}
        with mock.patch.dict(os.environ, {"YOLO_SERVICE_URL": "http://from-env:2"}):
            self.assertEqual(yolo_client.service_url(config),
                             "http://from-env:2")

    def test_config_overrides_default(self):
        config = {"open_vocab": {"service_url": "http://from-yaml:1"}}
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(yolo_client.service_url(config),
                             "http://from-yaml:1")

    def test_default_when_unset(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(yolo_client.service_url({}),
                             "http://localhost:5006")


class ConfigureTests(unittest.TestCase):
    def setUp(self):
        self.config = vision_config.load_config()
        self.config["open_vocab"]["service_url"] = "http://host:5006"

    def test_posts_selected_backend_and_thresholds(self):
        payload = {"ok": True, "configured": True, "detector": "yolo_world"}
        with mock.patch.object(yolo_client.urllib.request, "urlopen",
                               return_value=_Response(payload)) as urlopen:
            status = yolo_client.configure(self.config)
        request = urlopen.call_args.args[0]
        self.assertEqual(request.full_url, "http://host:5006/configure")
        self.assertEqual(request.method, "POST")
        body = json.loads(request.data.decode("utf-8"))
        self.assertEqual(body["detector"], "yolo_world")
        self.assertEqual(body["name"], "yolov8s-world")
        self.assertIn("confidence_threshold", body)
        self.assertIn("max_detections", body)
        self.assertTrue(status["ok"])

    def test_weights_name_alias_selects_backend(self):
        self.config["open_vocab"]["detector"] = "yoloe-26s-seg"
        payload = {"ok": True, "configured": True}
        with mock.patch.object(yolo_client.urllib.request, "urlopen",
                               return_value=_Response(payload)) as urlopen:
            yolo_client.configure(self.config)
        body = json.loads(urlopen.call_args.args[0].data.decode("utf-8"))
        self.assertEqual(body["detector"], "yoloe_26")

    def test_unreachable_service_has_actionable_error(self):
        with mock.patch.object(yolo_client.urllib.request, "urlopen",
                               side_effect=URLError("connection refused")):
            with self.assertRaises(RuntimeError) as context:
                yolo_client.configure(self.config)
        message = str(context.exception)
        self.assertIn("http://host:5006/configure", message)
        self.assertIn("start_yolo_service.sh", message)


class DetectTests(unittest.TestCase):
    def setUp(self):
        self.config = vision_config.load_config()
        self.config["open_vocab"]["service_url"] = "http://host:5006"

    def test_posts_both_images_and_returns_service_answer(self):
        left_detections = [{"label": "red ball", "confidence": 0.9,
                            "u": 90.0, "v": 120.0, "width_px": 12,
                            "height_px": 12, "bbox_px": [84, 114, 96, 126]}]
        payload = {"model": "fake-world",
                   "detections": [left_detections, []]}
        with mock.patch.object(yolo_client.urllib.request, "urlopen",
                               return_value=_Response(payload)) as urlopen:
            left, right, name = yolo_client.detect(
                b"left-jpeg", b"right-jpeg", ["red ball"], self.config)

        self.assertEqual(left, left_detections)
        self.assertEqual(right, [])
        self.assertEqual(name, "fake-world")
        request = urlopen.call_args.args[0]
        self.assertEqual(request.full_url, "http://host:5006/detect")
        body = json.loads(request.data.decode("utf-8"))
        self.assertEqual(body["classes"], ["red ball"])
        self.assertEqual(len(body["images"]), 2)
        self.assertEqual(body["images"][0], "bGVmdC1qcGVn")
        self.assertIn("confidence_threshold", body)
        self.assertIn("max_detections", body)

    def test_unload_posts_to_host_service(self):
        with mock.patch.object(yolo_client.urllib.request, "urlopen",
                               return_value=_Response(
                                   {"ok": True, "unloaded": True})) as urlopen:
            status = yolo_client.unload(self.config)
        request = urlopen.call_args.args[0]
        self.assertEqual(request.full_url, "http://host:5006/unload")
        self.assertEqual(request.method, "POST")
        self.assertTrue(status["unloaded"])

    def test_unreachable_service_has_actionable_error(self):
        with mock.patch.object(yolo_client.urllib.request, "urlopen",
                               side_effect=URLError("connection refused")):
            with self.assertRaises(RuntimeError) as context:
                yolo_client.detect(b"l", b"r", ["red ball"], self.config)
        message = str(context.exception)
        self.assertIn("http://host:5006/detect", message)
        self.assertIn("start_yolo_service.sh", message)

    def test_invalid_response_is_rejected(self):
        with mock.patch.object(yolo_client.urllib.request, "urlopen",
                               return_value=_Response({"detections": []})):
            with self.assertRaises(RuntimeError):
                yolo_client.detect(b"l", b"r", ["red ball"], self.config)


if __name__ == "__main__":
    unittest.main()
