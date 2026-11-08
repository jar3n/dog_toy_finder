"""Pure-Python tests for the shared 2D transforms.

Run from software/rosa_agent:
    python3 -m unittest discover -s tests
"""

import math
import sys
import unittest
from pathlib import Path

MOTION_SRC = (
    Path(__file__).resolve().parents[2]
    / "ros2_ws"
    / "src"
    / "dog_toy_finder_motion"
)
sys.path.insert(0, str(MOTION_SRC))

from dog_toy_finder_motion.transforms import (  # noqa: E402
    distance_and_bearing,
    robot_to_world,
    world_to_robot,
    wrap_angle,
)


class TransformTests(unittest.TestCase):
    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(math.pi + 0.1), -math.pi + 0.1)

    def test_robot_to_world_identity(self):
        self.assertEqual(robot_to_world((0.0, 0.0, 0.0), 1.0, 2.0), (1.0, 2.0))

    def test_robot_to_world_quarter_turn(self):
        x, y = robot_to_world((0.0, 0.0, math.pi / 2.0), 1.0, 0.0)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 1.0)

    def test_world_to_robot_inverts_robot_to_world(self):
        pose = (1.5, -0.5, math.radians(30.0))
        wx, wy = robot_to_world(pose, 0.4, -0.2)
        rx, ry = world_to_robot(pose, wx, wy)
        self.assertAlmostEqual(rx, 0.4)
        self.assertAlmostEqual(ry, -0.2)

    def test_distance_and_bearing(self):
        distance, bearing = distance_and_bearing((0.0, 0.0, 0.0), 1.0, 1.0)
        self.assertAlmostEqual(distance, math.sqrt(2.0))
        self.assertAlmostEqual(bearing, math.pi / 4.0)

    def test_bearing_uses_robot_heading(self):
        distance, bearing = distance_and_bearing(
            (0.0, 0.0, math.pi / 2.0), 0.0, 1.0
        )
        self.assertAlmostEqual(distance, 1.0)
        self.assertAlmostEqual(bearing, 0.0)


if __name__ == "__main__":
    unittest.main()
