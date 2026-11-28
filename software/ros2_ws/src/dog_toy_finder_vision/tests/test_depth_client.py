"""Tests for the container-side HTTP client to the host depth service."""

import json
import os
import sys
import unittest
from unittest import mock
from urllib.error import URLError

SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, SRC_ROOT)

from dog_toy_finder_vision import depth_client  # noqa: E402
from dog_toy_finder_vision import vision_config  # noqa: E402


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
    def test_env_overrides_config_and_default(self):
        config = {"stereo-net": {"service_url": "http://from-yaml:1"}}
        with mock.patch.dict(os.environ, {"STEREO_SERVICE_URL": "http://env:2"}):
            self.assertEqual(depth_client.service_url(config), "http://env:2")

    def test_default_when_unset(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(depth_client.service_url({}),
                             "http://localhost:5007")


class ConfigureTests(unittest.TestCase):
    def setUp(self):
        self.config = vision_config.load_config()
        self.config["stereo-net"]["service_url"] = "http://host:5007"

    def test_posts_selected_backend_and_sampling_settings(self):
        payload = {"ok": True, "configured": True, "model": "lite_any_stereo"}
        with mock.patch.object(depth_client.urllib.request, "urlopen",
                               return_value=_Response(payload)) as urlopen:
            status = depth_client.configure(self.config)
        request = urlopen.call_args.args[0]
        self.assertEqual(request.full_url, "http://host:5007/configure")
        self.assertEqual(request.method, "POST")
        body = json.loads(request.data.decode("utf-8"))
        self.assertEqual(body["model"], "lite_any_stereo")
        self.assertEqual(body["name"], "lite_any_stereo")
        self.assertEqual(body["max_disp"], 192)
        self.assertTrue(status["ok"])

    def test_unreachable_service_has_actionable_error(self):
        with mock.patch.object(depth_client.urllib.request, "urlopen",
                               side_effect=URLError("connection refused")):
            with self.assertRaises(RuntimeError) as context:
                depth_client.configure(self.config)
        message = str(context.exception)
        self.assertIn("http://host:5007/configure", message)
        self.assertIn("start_stereo_service.sh", message)


class SampleTests(unittest.TestCase):
    def setUp(self):
        self.config = vision_config.load_config()
        self.config["stereo-net"]["service_url"] = "http://host:5007"

    def test_unload_posts_to_host_service(self):
        with mock.patch.object(depth_client.urllib.request, "urlopen",
                               return_value=_Response(
                                   {"ok": True, "unloaded": True})) as urlopen:
            status = depth_client.unload(self.config)
        request = urlopen.call_args.args[0]
        self.assertEqual(request.full_url, "http://host:5007/unload")
        self.assertEqual(request.method, "POST")
        self.assertTrue(status["unloaded"])

    def test_invalid_response_is_rejected(self):
        with mock.patch.object(depth_client.urllib.request, "urlopen",
                               return_value=_Response({"error": "bad frame"})):
            with self.assertRaises(RuntimeError):
                depth_client.sample(b"left", b"right",
                                    {"sample_source": "mask_pixels",
                                     "points": [[1, 2]]}, self.config)


if __name__ == "__main__":
    unittest.main()
