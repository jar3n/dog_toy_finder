import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "dog_toy_finder_planning"))
from path_planner import a_star, expanding_square_waypoints, lawnmower_waypoints


class ExpandingSquareTests(unittest.TestCase):
    def test_first_loop_four_corners(self):
        pts = expanding_square_waypoints(side=1.0, increment=1.0, loops=1)
        self.assertEqual(len(pts), 4)
        self.assertEqual(pts[0], (1.0, 0.0))
        self.assertEqual(pts[1], (1.0, 1.0))
        self.assertEqual(pts[2], (0.0, 1.0))
        self.assertEqual(pts[3], (0.0, 0.0))

    def test_two_loops_grows(self):
        pts = expanding_square_waypoints(side=1.0, increment=1.0, loops=2)
        self.assertEqual(len(pts), 8)
        # second loop side = 2.0
        self.assertEqual(pts[4], (2.0, 0.0))


class LawnmowerTests(unittest.TestCase):
    def test_basic(self):
        pts = lawnmower_waypoints(width=2.0, height=1.0, spacing=1.0)
        self.assertEqual(pts[0], (0.0, 0.0))
        self.assertEqual(pts[1], (2.0, 0.0))
        self.assertEqual(pts[2], (2.0, 1.0))
        self.assertEqual(pts[3], (0.0, 1.0))


class AStarTests(unittest.TestCase):
    def test_straight_path(self):
        grid = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        path = a_star(grid, (0, 0), (0, 2))
        self.assertEqual(path, [(0, 0), (0, 1), (0, 2)])

    def test_around_obstacle(self):
        grid = [[0, 1, 0], [0, 1, 0], [0, 0, 0]]
        path = a_star(grid, (0, 0), (0, 2))
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (0, 2))
        self.assertNotIn((0, 1), path)

    def test_no_path(self):
        grid = [[0, 1, 0], [0, 1, 0], [0, 1, 0]]
        path = a_star(grid, (0, 0), (0, 2))
        self.assertIsNone(path)

    def test_start_is_goal(self):
        grid = [[0, 0], [0, 0]]
        path = a_star(grid, (0, 0), (0, 0))
        self.assertEqual(path, [(0, 0)])

    def test_invalid_bounds(self):
        grid = [[0, 0], [0, 0]]
        self.assertIsNone(a_star(grid, (5, 5), (0, 0)))
        self.assertIsNone(a_star(grid, (0, 0), (5, 5)))


if __name__ == "__main__":
    unittest.main()