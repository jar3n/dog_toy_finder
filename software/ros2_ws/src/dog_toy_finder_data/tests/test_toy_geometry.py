"""Tests for bearing-based toy placement and homing helpers."""

import math
import os
import sys
import unittest

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "dog_toy_finder_data"))

import toy_geometry  # noqa: E402


class ToyGeometryTests(unittest.TestCase):
    def test_positive_bearing_places_toy_left_at_zero_yaw(self):
        placement = toy_geometry.place(0.0, 0.0, 0.0, 2.0, 30.0)
        self.assertAlmostEqual(placement["x"], 2.0 * math.cos(math.radians(30)),
                               places=4)
        self.assertAlmostEqual(placement["y"], 1.0, places=4)
        self.assertAlmostEqual(placement["side_m"], 1.0, places=4)

    def test_negative_bearing_places_toy_right(self):
        placement = toy_geometry.place(0.0, 0.0, 0.0, 2.0, -30.0)
        self.assertAlmostEqual(placement["y"], -1.0, places=4)

    def test_bearing_falls_back_to_legacy_label(self):
        self.assertAlmostEqual(
            toy_geometry.bearing_deg({"position": "left"}),
            toy_geometry.LEGACY_BEARING_DEG)
        self.assertAlmostEqual(
            toy_geometry.bearing_deg({"lateral_bearing_deg": -7.5}), -7.5)

    def test_home_turn_uses_bearing_and_is_clamped(self):
        self.assertAlmostEqual(
            math.degrees(toy_geometry.home_turn_rad(
                {"lateral_bearing_deg": 8.0})), 8.0, places=4)
        self.assertAlmostEqual(
            math.degrees(toy_geometry.home_turn_rad(
                {"lateral_bearing_deg": 80.0}, max_bearing_deg=45.0)), 45.0)

    def test_yaw_rotation_moves_left_offset_with_robot(self):
        placement = toy_geometry.place(0.0, 0.0, math.pi / 2.0, 1.0, 0.0)
        self.assertAlmostEqual(placement["x"], 0.0, places=4)
        self.assertAlmostEqual(placement["y"], 1.0, places=4)


if __name__ == "__main__":
    unittest.main()
