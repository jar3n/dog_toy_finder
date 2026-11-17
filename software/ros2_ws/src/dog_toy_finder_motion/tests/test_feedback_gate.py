"""Tests for the EKF-only motion feedback invariant.

    cd software/ros2_ws/src/dog_toy_finder_motion
    PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests

The pure gate tests run without ROS. The Mover tests need rclpy (run them in
the ROS 2 container) and check that only the fused odometry topic is wired into
the node and that non-zero commands are refused until it is fresh.
"""

import os
import sys
import unittest
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = TESTS_DIR.parent / "dog_toy_finder_motion"
SRC_DIR = TESTS_DIR.parents[1]
sys.path.insert(0, str(PACKAGE_DIR))
sys.path.insert(0, str(SRC_DIR))

from feedback_gate import (  # noqa: E402
    FEEDBACK_TOPIC,
    FeedbackGate,
    validate_feedback_topic,
)

try:
    import rclpy  # noqa: E402
    from nav_msgs.msg import Odometry  # noqa: E402

    from dog_toy_finder_motion.mover import Mover  # noqa: E402

    HAS_ROS = True
except ImportError:  # pragma: no cover - host without ROS 2 Python packages
    HAS_ROS = False


class FeedbackGateTests(unittest.TestCase):
    """Topic and freshness policy shared by Mover and the command mux."""

    def test_only_ekf_topic_is_accepted(self):
        validate_feedback_topic("/odometry/filtered")
        self.assertEqual(FEEDBACK_TOPIC, "/odometry/filtered")

    def test_raw_wheel_odometry_is_rejected(self):
        with self.assertRaises(ValueError):
            validate_feedback_topic("/wheel/odom")

    def test_gate_starts_stale(self):
        self.assertFalse(FeedbackGate().fresh(now=100.0))

    def test_gate_is_fresh_after_update(self):
        gate = FeedbackGate(timeout=1.0)
        gate.update(now=10.0)
        self.assertTrue(gate.fresh(now=10.9))

    def test_gate_expires(self):
        gate = FeedbackGate(timeout=1.0)
        gate.update(now=10.0)
        self.assertFalse(gate.fresh(now=11.01))


@unittest.skipUnless(HAS_ROS, "rclpy is not available")
class MoverEKFOnlyTests(unittest.TestCase):
    """Runtime wiring and command gating for Mover."""

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def make_mover(self, **kwargs):
        kwargs.setdefault("node_name", "ekf_test_%d" % os.getpid())
        return Mover(**kwargs)

    def test_mover_subscribes_only_to_ekf_odometry(self):
        node = self.make_mover()
        try:
            topics = {sub.topic for sub in node.subscriptions}
            self.assertIn(FEEDBACK_TOPIC, topics)
            self.assertNotIn("/wheel/odom", topics)
        finally:
            node.destroy_node()

    def test_mover_rejects_raw_feedback_topic(self):
        with self.assertRaises(ValueError):
            self.make_mover(feedback_topic="/wheel/odom")

    def test_nonzero_command_requires_fresh_ekf(self):
        node = self.make_mover()
        try:
            with self.assertRaises(RuntimeError):
                node.send(0.1, 0.0)
            # Stops are always allowed so the robot can be brought to rest.
            node.send(0.0, 0.0)

            node._on_feedback(Odometry())
            self.assertTrue(node.feedback_fresh())
            node.send(0.1, 0.0)
        finally:
            node.destroy_node()


if __name__ == "__main__":
    unittest.main()
