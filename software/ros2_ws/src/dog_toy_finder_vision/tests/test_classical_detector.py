"""Tests for the deterministic rectified-stereo toy detector."""

import os
import sys
import unittest

import numpy as np

sys.path.insert(
    0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from dog_toy_finder_vision import classical_detector as detector  # noqa: E402
from dog_toy_finder_vision import vision_config  # noqa: E402


class ConfigTests(unittest.TestCase):
    def test_default_config_loads_with_physical_measurements(self):
        config = vision_config.load_config()
        self.assertGreater(float(config["camera"]["fx"]), 0.0)
        self.assertGreater(float(config["camera"]["baseline_m"]), 0.0)
        self.assertGreater(float(config["target"]["known_width_m"]), 0.0)
        self.assertIn("/image_rect_color",
                      config["camera"]["left_rectified_topic"])
        self.assertIn("/image_rect_color",
                      config["camera"]["right_rectified_topic"])

    def test_target_colour_and_physical_summary(self):
        config = vision_config.load_config()
        self.assertEqual(vision_config.target_color_names(config, "red ball"),
                         ["red"])
        self.assertEqual(vision_config.target_color_names(config, "blue bone"),
                         ["blue"])
        # Unknown colours use the configured deterministic fallback.
        self.assertEqual(vision_config.target_color_names(config, "toy"),
                         [config["target"]["default_color"]])
        summary = vision_config.physical_summary(config)
        self.assertIn("baseline", summary)
        self.assertIn("0.075", summary)

    def test_invalid_measurement_rejected(self):
        config = vision_config.load_config()
        config["camera"]["baseline_m"] = 0.0
        with self.assertRaises(ValueError):
            vision_config.validate_config(config)


class DetectorTests(unittest.TestCase):
    @staticmethod
    def _synthetic_pair(config, center_x=160, disparity=20):
        height = int(config["camera"]["height"])
        width = int(config["camera"]["width"])
        left = np.full((height, width, 3), 210, dtype=np.uint8)
        right = np.full((height, width, 3), 210, dtype=np.uint8)
        yy, xx = np.ogrid[:height, :width]
        radius = 10
        left_circle = (xx - center_x) ** 2 + (yy - 120) ** 2 <= radius ** 2
        right_circle = ((xx - (center_x - disparity)) ** 2
                        + (yy - 120) ** 2 <= radius ** 2)
        left[left_circle] = [255, 0, 0]
        right[right_circle] = [255, 0, 0]
        return left, right

    def test_hsv_mask_finds_red_and_not_blue(self):
        config = vision_config.load_config()
        red = np.zeros((10, 10, 3), dtype=np.uint8)
        blue = np.zeros((10, 10, 3), dtype=np.uint8)
        red[5, 5] = [255, 0, 0]
        blue[5, 5] = [0, 0, 255]
        red_mask = detector.color_mask(
            red, vision_config.color_ranges(config, "red ball"))
        blue_mask = detector.color_mask(
            blue, vision_config.color_ranges(config, "red ball"))
        self.assertTrue(red_mask[5, 5])
        self.assertFalse(blue_mask[5, 5])

    def test_detects_toy_and_measures_distance_from_disparity(self):
        config = vision_config.load_config()
        left, right = self._synthetic_pair(config, center_x=90, disparity=20)
        result = detector.detect(left, right, "red ball", config)
        self.assertTrue(result["target_visible"])
        self.assertEqual(result["method"], "classical")
        self.assertEqual(result["position"], "left")
        self.assertGreater(result["lateral_bearing_deg"], 0.0)
        self.assertLess(result["lateral_bearing_deg"], 30.0)
        self.assertIn(result["size"], ("small", "medium", "large"))
        self.assertAlmostEqual(result["distance_m"], 1.0, delta=0.25)
        self.assertEqual(result["distance_source"], "fused_stereo_size")
        self.assertGreater(result["confidence"], 0.3)
        self.assertEqual(len(result["bbox_px"]), 4)
        self.assertEqual(result["obstacle_ahead"], "none")
        self.assertTrue(result["path_clear"])

    def test_right_side_toy_has_negative_bearing(self):
        config = vision_config.load_config()
        left, right = self._synthetic_pair(config, center_x=230, disparity=20)
        result = detector.detect(left, right, "red ball", config)
        self.assertTrue(result["target_visible"])
        self.assertEqual(result["position"], "right")
        self.assertLess(result["lateral_bearing_deg"], 0.0)

    def test_no_toy_returns_not_visible(self):
        config = vision_config.load_config()
        blank = np.full((config["camera"]["height"],
                         config["camera"]["width"], 3), 210, dtype=np.uint8)
        result = detector.detect(blank, blank, "red ball", config)
        self.assertFalse(result["target_visible"])
        self.assertEqual(result["position"], "none")
        self.assertEqual(result["distance_m"], 0.0)
        self.assertEqual(result["method"], "classical")

    def test_mismatched_images_raise(self):
        config = vision_config.load_config()
        left = np.zeros((10, 10, 3), dtype=np.uint8)
        right = np.zeros((10, 12, 3), dtype=np.uint8)
        with self.assertRaises(ValueError):
            detector.detect(left, right, "red ball", config)


if __name__ == "__main__":
    unittest.main()
