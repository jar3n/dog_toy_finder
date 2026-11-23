"""Tests for pipeline selection and the reserved second-pipeline placeholder."""

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

    def test_second_pipeline_is_a_clear_placeholder(self):
        with self.assertRaises(NotImplementedError) as context:
            vision_pipelines.run_pipeline(
                "second", "red ball", _jpeg(), _jpeg(), self.config)
        message = str(context.exception)
        self.assertIn("second", message)
        self.assertIn("open_vocab", message)

    def test_unknown_pipeline_is_rejected_before_dispatch(self):
        with self.assertRaises(ValueError):
            vision_pipelines.run_pipeline(
                "magic", "red ball", _jpeg(), _jpeg(), self.config)


if __name__ == "__main__":
    unittest.main()
