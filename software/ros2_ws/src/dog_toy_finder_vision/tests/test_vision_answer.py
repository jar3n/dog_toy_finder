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

    def test_position_derived_from_bearing(self):
        config = vision_config.load_config()
        self.assertEqual(vision_config.position_from_bearing(30, config), "left")
        self.assertEqual(vision_config.position_from_bearing(-30, config),
                         "right")
        self.assertEqual(vision_config.position_from_bearing(0, config),
                         "center")

    def test_legacy_label_recovers_a_bearing(self):
        self.assertEqual(vision_config.bearing_from_position("left"), 15.0)
        self.assertEqual(vision_config.bearing_from_position("right"), -15.0)
        self.assertEqual(vision_config.bearing_from_position("none"), 0.0)


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

    def test_tidy_derives_position_from_bearing(self):
        answer = vision_answer.tidy({
            "description": "toy",
            "target_visible": True,
            "position": "center",  # stale label; bearing wins
            "lateral_bearing_deg": 28.0,
            "size": "small",
            "distance_m": 1.2,
            "obstacle_ahead": "",
        }, self.config)
        self.assertEqual(answer["position"], "left")
        self.assertEqual(answer["lateral_bearing_deg"], 28.0)
        self.assertTrue(answer["path_clear"])

    def test_tidy_recovers_bearing_from_legacy_label(self):
        answer = vision_answer.tidy({
            "target_visible": True,
            "position": "right",
            "distance_m": 0.8,
        }, self.config)
        self.assertEqual(answer["lateral_bearing_deg"], -15.0)

    def test_tidy_clears_bearing_when_not_visible(self):
        answer = vision_answer.tidy({
            "target_visible": False,
            "position": "left",
            "lateral_bearing_deg": 25.0,
            "distance_m": 1.0,
        }, self.config)
        self.assertEqual(answer["position"], "none")
        self.assertEqual(answer["lateral_bearing_deg"], 0.0)
        self.assertEqual(answer["distance_m"], 0.0)


if __name__ == "__main__":
    unittest.main()
