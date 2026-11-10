"""Tests for the cmd_vel mux source selection.

The ROS node is a thin wrapper around CommandSelector, so these cover the
safety-relevant rules: manual priority, manual stop blocking auto, the
hands-off gap after manual control, and source expiry.

    cd software/ros2_ws/src/dog_toy_finder_motion
    python3 -m unittest discover -s tests
"""

import os
import sys
import unittest

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "dog_toy_finder_motion"))

from cmd_vel_mux import (
    CommandSelector,
    SOURCE_AUTO,
    SOURCE_MANUAL,
    SOURCE_NONE,
    SOURCE_RELEASED,
)


class CommandSelectorTests(unittest.TestCase):
    """Source priority and timeout behaviour of the velocity mux."""

    def make(self, release_timeout=0.5):
        return CommandSelector(
            manual_timeout=0.5,
            auto_timeout=0.5,
            release_timeout=release_timeout,
        )

    def test_starts_with_no_command(self):
        self.assertEqual(self.make().select(0.0), (SOURCE_NONE, 0.0, 0.0))

    def test_auto_commands_are_forwarded(self):
        selector = self.make()
        selector.update_auto(0.1, -0.2, now=1.0)
        self.assertEqual(selector.select(1.0), (SOURCE_AUTO, 0.1, -0.2))
        self.assertEqual(selector.select(1.5), (SOURCE_AUTO, 0.1, -0.2))

    def test_auto_expires_to_zero(self):
        selector = self.make()
        selector.update_auto(0.1, 0.0, now=1.0)
        self.assertEqual(selector.select(1.51), (SOURCE_NONE, 0.0, 0.0))

    def test_manual_overrides_auto(self):
        selector = self.make()
        selector.update_auto(0.1, 0.0, now=1.0)
        selector.update_manual(-0.05, 0.4, now=1.0)
        self.assertEqual(
            selector.select(1.0), (SOURCE_MANUAL, -0.05, 0.4))

    def test_manual_stop_blocks_auto(self):
        selector = self.make()
        selector.update_auto(0.1, 0.0, now=1.0)
        selector.update_manual(0.0, 0.0, now=1.0)
        self.assertEqual(selector.select(1.0), (SOURCE_MANUAL, 0.0, 0.0))

    def test_manual_expiry_holds_zero_then_resumes_auto(self):
        selector = self.make()
        selector.update_manual(0.1, 0.0, now=1.0)
        self.assertEqual(selector.select(1.0), (SOURCE_MANUAL, 0.1, 0.0))

        # Auto keeps publishing while manual is in control, but must not win.
        selector.update_auto(-0.1, 0.0, now=1.4)
        self.assertEqual(selector.select(1.4), (SOURCE_MANUAL, 0.1, 0.0))

        # Manual expires at 1.5; the release window runs to 2.0.
        selector.update_auto(-0.1, 0.0, now=1.6)
        self.assertEqual(selector.select(1.6), (SOURCE_RELEASED, 0.0, 0.0))
        selector.update_auto(-0.1, 0.0, now=1.9)
        self.assertEqual(selector.select(1.9), (SOURCE_RELEASED, 0.0, 0.0))

        # After the release gap autonomous commands are allowed again.
        selector.update_auto(-0.1, 0.0, now=2.1)
        self.assertEqual(selector.select(2.1), (SOURCE_AUTO, -0.1, 0.0))

    def test_manual_resumes_during_release(self):
        selector = self.make()
        selector.update_manual(0.1, 0.0, now=1.0)
        selector.select(1.0)
        self.assertEqual(selector.select(1.6), (SOURCE_RELEASED, 0.0, 0.0))

        selector.update_manual(-0.2, 0.0, now=1.8)
        self.assertEqual(selector.select(1.8), (SOURCE_MANUAL, -0.2, 0.0))

    def test_release_can_be_disabled(self):
        selector = self.make(release_timeout=0.0)
        selector.update_auto(0.1, 0.0, now=1.0)
        selector.update_manual(0.2, 0.0, now=1.0)
        selector.select(1.0)
        selector.update_auto(0.1, 0.0, now=1.6)
        self.assertEqual(selector.select(1.6), (SOURCE_AUTO, 0.1, 0.0))

    def test_manual_expiry_without_auto_returns_none(self):
        selector = self.make()
        selector.update_manual(0.1, 0.0, now=1.0)
        selector.select(1.0)
        self.assertEqual(selector.select(1.6), (SOURCE_RELEASED, 0.0, 0.0))
        self.assertEqual(selector.select(2.1), (SOURCE_NONE, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
