"""Tests for the host-side dense-stereo HTTP service and container client.

No model weights are needed: the dense map is a constant and the backend call
is replaced with a fake. The tests cover model selection, the HTTP contract, and
the calibration values passed from the container.
"""

import base64
import importlib.util
import io
import json
import os
import sys
import threading
import unittest
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path
from unittest import mock

import numpy as np
from PIL import Image

SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, SRC_ROOT)
from dog_toy_finder_vision import depth_client  # noqa: E402
from dog_toy_finder_vision import vision_config  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[5]
HOST_VISION = REPO_ROOT / "software" / "host_vision"
sys.path.insert(0, str(HOST_VISION))
import dense_stereo  # noqa: E402

spec = importlib.util.spec_from_file_location(
    "dtf_stereo_service", HOST_VISION / "stereo_service.py")
service = importlib.util.module_from_spec(spec)
spec.loader.exec_module(service)


def _jpeg_bytes():
    buffer = io.BytesIO()
    Image.new("RGB", (320, 240), (200, 200, 200)).save(buffer, format="JPEG")
    return buffer.getvalue()


def _jpeg():
    return base64.b64encode(_jpeg_bytes()).decode("ascii")


class ServiceConfigTests(unittest.TestCase):
    def test_parse_args_selects_model_and_derives_name(self):
        with mock.patch.dict(os.environ, {"DEPTH_MODEL": "rt_igevpp"},
                             clear=True):
            args = service.parse_args([])
        self.assertEqual(args.model, dense_stereo.RT_IGEVPP)
        self.assertTrue(args.model_path.endswith("rt_igevpp/sceneflow.pth"))
        self.assertEqual(args.name, "sceneflow")
        self.assertEqual(args.iters, 6)

        args = service.parse_args([
            "--model", "fast_foundation_stereo", "--device", "cpu",
            "--model-path", "/opt/fast.pth"])
        self.assertEqual(args.name, "fast")
        self.assertEqual(args.device, "cpu")
        self.assertEqual(args.iters, 8)

    def test_build_config_keeps_model_and_device(self):
        args = service.parse_args(["--model", "lite_any_stereo",
                                   "--device", "cuda:0"])
        config = service.build_config(args)
        self.assertEqual(config["model"], dense_stereo.LITE_ANY_STEREO)
        self.assertEqual(config["device"], "cuda:0")
        self.assertEqual(config["max_disp"], 192)

    def test_request_calibration_overrides_are_applied(self):
        service._CONFIG = {"model": "lite_any_stereo", "camera": {"fx": 100.0}}
        config = service._request_config({
            "fx": 250.0, "baseline_m": 0.08, "disparity_offset_px": 29.04,
            "max_disp": 64})
        self.assertEqual(config["camera"]["fx"], 250.0)
        self.assertEqual(config["camera"]["baseline_m"], 0.08)
        self.assertEqual(config["camera"]["disparity_offset_px"], 29.04)
        self.assertEqual(config["max_disp"], 64)
        self.assertEqual(service._CONFIG["camera"]["fx"], 100.0)


class HostServiceTests(unittest.TestCase):
    def setUp(self):
        service._CONFIG = service.build_config(service.parse_args([]))
        service._MODEL = None
        service._MODEL_NAME = "fake-stereo"
        service._WEIGHTS = "fake.pth"
        service._BACKEND = dense_stereo.LITE_ANY_STEREO

    def _patch_backend(self):
        return (
            mock.patch.object(service.dense_stereo, "load_model",
                              return_value=(dense_stereo.LITE_ANY_STEREO,
                                            object())),
            mock.patch.object(service.dense_stereo, "disparity_map",
                              return_value=np.full((240, 320), 20.0,
                                                   dtype=np.float32)),
            mock.patch.object(service.dense_stereo, "sample_disparity",
                              return_value={"sample_source": "box_window",
                                            "sample_count": 9,
                                            "valid_count": 9,
                                            "disparity_px": 20.0,
                                            "depth_m": 1.0}),
        )

    def test_depth_sends_both_eyes_and_sampling_request(self):
        load, disparity, sample = self._patch_backend()
        with load, disparity as disparity_mock, sample as sample_mock:
            name, result = service.depth({
                "images": [_jpeg(), _jpeg()],
                "window": {"u": 100, "v": 120, "radius": 3},
                "fx": 250.0, "baseline_m": 0.08,
            })
        self.assertEqual(name, "fake-stereo")
        self.assertEqual(result["depth_m"], 1.0)
        left, right = disparity_mock.call_args.args[2:4]
        self.assertEqual(left.shape, (240, 320, 3))
        self.assertEqual(right.shape, (240, 320, 3))
        self.assertEqual(sample_mock.call_args.kwargs["config"]["camera"]["fx"],
                         250.0)

    def test_depth_reuses_loaded_backend_on_later_requests(self):
        load, disparity, sample = self._patch_backend()
        body = {
            "images": [_jpeg(), _jpeg()],
            "window": {"u": 100, "v": 120, "radius": 3},
        }
        with load as load_mock, disparity as disparity_mock, sample:
            service.depth(body)
            self.assertEqual(load_mock.call_count, 1)
            self.assertEqual(service._BACKEND, dense_stereo.LITE_ANY_STEREO)
            service.depth(body)
        self.assertEqual(load_mock.call_count, 1,
                         "the model must load only once per service process")
        self.assertEqual(disparity_mock.call_count, 2)

    def test_http_round_trip_with_container_client(self):
        load, disparity, sample = self._patch_backend()
        with load, disparity, sample as sample_mock:
            server = ThreadingHTTPServer(("127.0.0.1", 0), service.Handler)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            self.addCleanup(server.server_close)
            self.addCleanup(server.shutdown)
            url = "http://127.0.0.1:%d" % server.server_port
            with urllib.request.urlopen(url + "/health") as response:
                health = json.load(response)
            self.assertEqual(health["model"], dense_stereo.LITE_ANY_STEREO)

            config = vision_config.load_config()
            with mock.patch.dict(os.environ, {"STEREO_SERVICE_URL": url}):
                result = depth_client.sample(
                    _jpeg_bytes(), _jpeg_bytes(),
                    {"sample_source": "mask_pixels",
                     "points": [[100, 120], [101, 120]]}, config)
        self.assertEqual(result["model"], "fake-stereo")
        self.assertEqual(result["depth_m"], 1.0)
        self.assertEqual(sample_mock.call_args.kwargs["points"],
                         [[100, 120], [101, 120]])
        self.assertEqual(
            sample_mock.call_args.kwargs["config"]["camera"]["fx"],
            vision_config.eye_intrinsics(config, "left")["fx"])
        self.assertEqual(
            sample_mock.call_args.kwargs["config"]["camera"][
                "disparity_offset_px"],
            vision_config.stereo_disparity_offset_px(config))

    def test_invalid_requests_are_rejected(self):
        with self.assertRaises(ValueError):
            service.depth({"images": [_jpeg()]})
        with self.assertRaises(ValueError):
            service.depth({"images": [_jpeg(), _jpeg()]})


if __name__ == "__main__":
    unittest.main()
