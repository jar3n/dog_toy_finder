"""Tests for pipeline selection and both selectable vision pipelines."""

import io
import os
import sys
import unittest

from PIL import Image

SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, SRC_ROOT)

from dog_toy_finder_vision import vision_config  # noqa: E402
from dog_toy_finder_vision import vision_pipelines  # noqa: E402


class PipelineNameTests(unittest.TestCase):
    def test_explicit_value_wins_over_environment_and_config(self):
        name = vision_pipelines.pipeline_name(
            "second", {"pipeline": "open_vocab"},
            {"VISION_PIPELINE": "open_vocab"})
        self.assertEqual(name, "second")

    def test_environment_wins_over_config(self):
        name = vision_pipelines.pipeline_name(
            None, {"pipeline": "second"}, {"VISION_PIPELINE": "open_vocab"})
        self.assertEqual(name, "open_vocab")

    def test_config_wins_over_default(self):
        name = vision_pipelines.pipeline_name(
            None, {"pipeline": "second"}, {})
        self.assertEqual(name, "second")

    def test_default_is_open_vocab(self):
        self.assertEqual(vision_pipelines.pipeline_name(None, {}, {}),
                         "open_vocab")

    def test_names_are_normalised(self):
        self.assertEqual(vision_pipelines.pipeline_name(" Open_Vocab ", {}, {}),
                         "open_vocab")

    def test_unknown_pipeline_is_rejected(self):
        with self.assertRaises(ValueError):
            vision_pipelines.pipeline_name("magic", {}, {})


def _empty_client(left_jpeg, right_jpeg, classes, config):
    """Minimal stand-in for the host detector: no detections in either eye."""
    return [], [], "fake-world"


def _one_detection_client(jpeg, classes, config):
    """Stand-in for detect_one: one box detection in the left image."""
    return [{
        "label": classes[0] if classes else "red ball",
        "confidence": 0.9, "u": 10.0, "v": 12.0, "width_px": 8.0,
        "height_px": 8.0, "bbox_px": [6.0, 8.0, 14.0, 16.0],
        "center_source": "box_center", "mask_centroid_px": None,
        "mask_area_px": 0, "depth_sample_pixels": [],
    }], "fake-world"


def _depth_client(left_jpeg, right_jpeg, spec, config):
    """Stand-in for the host dense-stereo depth sampler."""
    return {
        "model": "fake-stereo", "depth_m": 1.0, "disparity_px": 20.0,
        "sample_source": spec["sample_source"], "sample_count": 9,
        "valid_count": 9,
    }


def _jpeg(color=(255, 255, 255)):
    buffer = io.BytesIO()
    Image.new("RGB", (32, 24), color).save(buffer, format="JPEG")
    return buffer.getvalue()


class RunPipelineTests(unittest.TestCase):
    def setUp(self):
        self.config = vision_config.load_config()

    def test_open_vocab_returns_one_entry(self):
        entries = vision_pipelines.run_pipeline(
            "open_vocab", "red ball", _jpeg(), _jpeg(), self.config,
            client=_empty_client)
        self.assertEqual(len(entries), 1)
        entry = entries[0]
        self.assertEqual(entry["method"], "open_vocab")
        self.assertEqual(entry["distance_source"], "none")
        self.assertFalse(entry["target_visible"])
        self.assertEqual(entry["model"], "fake-world")

    def test_second_detects_once_and_samples_dense_depth(self):
        entries = vision_pipelines.run_pipeline(
            "second", "red ball", _jpeg(), _jpeg(), self.config,
            client=_one_detection_client, depth_client=_depth_client)
        self.assertEqual(len(entries), 1)
        entry = entries[0]
        self.assertEqual(entry["method"], "second")
        self.assertEqual(entry["distance_source"], "dense_stereo_depth")
        self.assertEqual(entry["model"], "fake-stereo")
        self.assertEqual(entry["detector_model"], "fake-world")
        self.assertEqual(entry["depth_sample_source"], "box_window")
        self.assertEqual(entry["depth_sample_count"], 9)
        self.assertTrue(entry["target_visible"])
        self.assertAlmostEqual(entry["distance_m"], 1.0)
        self.assertEqual(entry["center_source"], "box_center")

    def test_second_reports_no_detection_without_calling_depth(self):
        calls = []

        def fail_depth(*args):
            calls.append(args)
            raise AssertionError("depth service must not be called")

        entries = vision_pipelines.run_pipeline(
            "second", "red ball", _jpeg(), _jpeg(), self.config,
            client=lambda jpeg, classes, config: ([], "fake-world"),
            depth_client=fail_depth)
        self.assertFalse(entries[0]["target_visible"])
        self.assertEqual(entries[0]["distance_source"], "none")
        self.assertEqual(calls, [])

    def test_second_uses_mask_pixels_when_detector_supplies_them(self):
        seen = {}

        def mask_client(jpeg, classes, config):
            return [{
                "label": classes[0], "confidence": 0.8, "u": 10.0, "v": 12.0,
                "width_px": 8.0, "height_px": 8.0,
                "bbox_px": [6.0, 8.0, 14.0, 16.0],
                "center_source": "mask_centroid",
                "mask_centroid_px": [10.0, 12.0], "mask_area_px": 4,
                "depth_sample_pixels": [[9.0, 11.0], [10.0, 12.0]],
            }], "fake-yoloe"

        def capture_depth(left, right, spec, config):
            seen.update(spec)
            return _depth_client(left, right, spec, config)

        vision_pipelines.run_pipeline(
            "second", "red ball", _jpeg(), _jpeg(), self.config,
            client=mask_client, depth_client=capture_depth)
        self.assertEqual(seen["sample_source"], "mask_pixels")
        self.assertEqual(seen["points"], [[9.0, 11.0], [10.0, 12.0]])

    def test_unknown_pipeline_is_rejected_before_dispatch(self):
        with self.assertRaises(ValueError):
            vision_pipelines.run_pipeline(
                "magic", "red ball", _jpeg(), _jpeg(), self.config)


if __name__ == "__main__":
    unittest.main()
