"""Tests for the shared lateral-bearing and answer-normalisation logic."""

import os
import sys
import unittest

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from dog_toy_finder_vision import vision_answer  # noqa: E402
from dog_toy_finder_vision import vision_config  # noqa: E402


class BearingTests(unittest.TestCase):
    def test_bearing_sign_and_zero(self):
        config = vision_config.load_config()
        self.assertGreater(
            vision_config.lateral_bearing_deg(90, config),
            0.0, "left of cx must be positive")
        self.assertLess(
            vision_config.lateral_bearing_deg(230, config),
            0.0, "right of cx must be negative")
        self.assertAlmostEqual(
            vision_config.lateral_bearing_deg(config["camera"]["cx"], config),
            0.0, places=2)

    def test_bearing_magnitude_grows_away_from_optical_axis(self):
        config = vision_config.load_config()
        cx = config["camera"]["cx"]
        self.assertLess(
            vision_config.lateral_bearing_deg(cx - 40, config),
            vision_config.lateral_bearing_deg(cx - 80, config))
        self.assertGreater(
            vision_config.lateral_bearing_deg(cx + 40, config),
            vision_config.lateral_bearing_deg(cx + 80, config))


class AnswerTests(unittest.TestCase):
    def setUp(self):
        self.config = vision_config.load_config()

    def test_tidy_preserves_pipeline_specific_fields(self):
        answer = vision_answer.tidy({
            "target_visible": True,
            "lateral_bearing_deg": 4.0,
            "distance_m": 1.0,
            "method": "open_vocab",
            "distance_source": "stereo_triangulation",
            "disparity_px": 20.0,
            "bbox_px": [10, 20, 30, 40],
        }, self.config)
        self.assertEqual(answer["method"], "open_vocab")
        self.assertEqual(answer["distance_source"], "stereo_triangulation")
        self.assertEqual(answer["disparity_px"], 20.0)
        self.assertEqual(answer["bbox_px"], [10, 20, 30, 40])

    def test_tidy_drops_legacy_categorical_size(self):
        answer = vision_answer.tidy({
            "target_visible": True,
            "lateral_bearing_deg": 4.0,
            "distance_m": 1.0,
            "size": "medium",
        }, self.config)
        self.assertNotIn("size", answer)

    def test_tidy_keeps_numeric_bearing(self):
        answer = vision_answer.tidy({
            "description": "toy",
            "target_visible": True,
            "lateral_bearing_deg": 28.0,
            "distance_m": 1.2,
            "obstacle_ahead": "",
        }, self.config)
        self.assertEqual(answer["lateral_bearing_deg"], 28.0)
        self.assertTrue(answer["path_clear"])
        self.assertNotIn("size", answer)

    def test_tidy_defaults_missing_bearing_to_zero(self):
        answer = vision_answer.tidy({
            "target_visible": True,
            "distance_m": 0.8,
        }, self.config)
        self.assertEqual(answer["lateral_bearing_deg"], 0.0)

    def test_tidy_clears_bearing_when_not_visible(self):
        answer = vision_answer.tidy({
            "target_visible": False,
            "lateral_bearing_deg": 25.0,
            "distance_m": 1.0,
        }, self.config)
        self.assertEqual(answer["lateral_bearing_deg"], 0.0)
        self.assertEqual(answer["distance_m"], 0.0)
        self.assertEqual(answer["center_offset_m"], 0.0)

    def test_tidy_derives_center_offset_from_distance_and_bearing(self):
        answer = vision_answer.tidy({
            "target_visible": True,
            "lateral_bearing_deg": 28.0,
            "distance_m": 1.2,
        }, self.config)
        self.assertAlmostEqual(answer["center_offset_m"], 0.638, places=2)

    def test_tidy_preserves_explicit_center_offset(self):
        answer = vision_answer.tidy({
            "target_visible": True,
            "lateral_bearing_deg": 12.0,
            "center_offset_m": 0.244,
            "distance_m": 1.15,
        }, self.config)
        self.assertEqual(answer["center_offset_m"], 0.244)


if __name__ == "__main__":
    unittest.main()
