"""Tests for host dense-stereo model selection and depth sampling.

No torch, upstream repositories, or checkpoints are needed: model loading is
tested with an injected fake and sampling is tested on a synthetic disparity
map. The real backends are only imported inside load_model/disparity_map.
"""

import importlib.util
import os
import sys
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[5]
HOST_VISION = REPO_ROOT / "software" / "host_vision"
sys.path.insert(0, str(HOST_VISION))
import dense_stereo  # noqa: E402


class ModelSelectionTests(unittest.TestCase):
    def test_names_are_normalised_and_validated(self):
        self.assertEqual(dense_stereo.model_name("fast-foundation-stereo"),
                         dense_stereo.FAST_FOUNDATION_STEREO)
        self.assertEqual(dense_stereo.model_name({"name": "rt_igevpp"}),
                         dense_stereo.RT_IGEVPP)
        with self.assertRaises(ValueError):
            dense_stereo.model_name("magic")

    def test_environment_overrides_default_model(self):
        with mock.patch.dict(os.environ, {"DEPTH_MODEL": "rt_igevpp"},
                             clear=True):
            self.assertEqual(dense_stereo.model_name({}),
                             dense_stereo.RT_IGEVPP)
            self.assertEqual(dense_stereo.default_weights(
                dense_stereo.LITE_ANY_STEREO),
                str(dense_stereo.MODELS_DIR / "LAS2_S.pth"))

    def test_weights_environment_variable_is_used(self):
        with mock.patch.dict(os.environ, {
                "RT_IGEVPP_MODEL_PATH": "/opt/rt.pth"}, clear=True):
            self.assertEqual(dense_stereo.default_weights(
                dense_stereo.RT_IGEVPP), "/opt/rt.pth")

    def test_inference_iterations_default_per_model(self):
        self.assertEqual(dense_stereo.inference_iters(
            dense_stereo.LITE_ANY_STEREO), 1)
        self.assertEqual(dense_stereo.inference_iters(
            dense_stereo.FAST_FOUNDATION_STEREO), 8)
        self.assertEqual(dense_stereo.inference_iters(
            dense_stereo.RT_IGEVPP), 6)
        self.assertEqual(dense_stereo.inference_iters(
            dense_stereo.RT_IGEVPP, {"iters": 9}), 9)

    def test_injected_model_is_returned_without_torch(self):
        fake = object()
        config = {"model": dense_stereo.RT_IGEVPP, "device": "cpu"}
        backend, model = dense_stereo.load_model(config, model=fake)
        self.assertEqual(backend, dense_stereo.RT_IGEVPP)
        self.assertIs(model, fake)
        self.assertEqual(dense_stereo.resolve_device(config), "cpu")

    def test_timm_compat_adds_identity_activation(self):
        class FakeFeature:
            pass

        class FakeIdentity:
            pass

        class FakeNn:
            Identity = FakeIdentity

        class FakeTorch:
            nn = FakeNn()

        class FakeTimm:
            def __init__(self):
                self.create_model = lambda *args, **kwargs: FakeFeature()

        timm = FakeTimm()
        create_model, original = dense_stereo._timm_create_model_with_act1(
            FakeTorch, timm)
        feature = create_model()
        self.assertIsInstance(feature.act1, FakeIdentity)
        self.assertIs(timm.create_model, original)


class DepthSamplingTests(unittest.TestCase):
    def setUp(self):
        self.config = {
            "camera": {"fx": 250.0, "baseline_m": 0.08},
            "max_disp": 192,
        }

    def test_mask_pixels_use_median_and_ignore_invalid_disparity(self):
        disparity = np.full((4, 4), 10.0, dtype=np.float32)
        disparity[0, 0] = np.inf
        disparity[1, 1] = 0.0
        result = dense_stereo.sample_disparity(
            disparity, points=[[0, 0], [1, 1], [2, 2], [3, 3]],
            config=self.config)
        self.assertEqual(result["sample_source"], "mask_pixels")
        self.assertEqual(result["sample_count"], 4)
        self.assertEqual(result["valid_count"], 2)
        self.assertEqual(result["disparity_px"], 10.0)
        self.assertEqual(result["depth_m"], 2.0)

    def test_box_window_uses_all_pixels_around_centre(self):
        disparity = np.array([[2, 4, 6], [8, 10, 12], [14, 16, 18]],
                             dtype=np.float32)
        result = dense_stereo.sample_disparity(
            disparity, window={"u": 1, "v": 1, "radius": 1},
            config=self.config)
        self.assertEqual(result["sample_source"], "box_window")
        self.assertEqual(result["sample_count"], 9)
        self.assertEqual(result["disparity_px"], 10.0)
        self.assertAlmostEqual(result["depth_m"], 2.0, places=3)

    def test_no_valid_samples_returns_no_depth(self):
        result = dense_stereo.sample_disparity(
            np.zeros((3, 3), dtype=np.float32),
            points=[[1, 1]], config=self.config)
        self.assertIsNone(result["depth_m"])
        self.assertIsNone(result["disparity_px"])
        self.assertEqual(result["valid_count"], 0)


if __name__ == "__main__":
    unittest.main()
