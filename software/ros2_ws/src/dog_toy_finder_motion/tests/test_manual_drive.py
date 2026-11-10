"""Tests for the keyboard manual drive helpers.

The node itself needs a TTY and a running ROS graph, so these cover the pure
parts: terminal decoding and the exclusive drive-or-turn key mapping.

    cd software/ros2_ws/src/dog_toy_finder_motion
    python3 -m unittest discover -s tests
"""

import os
import sys
import unittest

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "dog_toy_finder_motion"))

from manual_drive import command_for_key, decode_keys


class DecodeKeysTests(unittest.TestCase):
    """Terminal bytes to logical key names."""

    def test_wasd_characters(self):
        self.assertEqual(decode_keys("wasd"), (["w", "a", "s", "d"], ""))

    def test_arrow_sequences(self):
        data = "\x1b[A\x1b[B\x1b[C\x1b[D"
        self.assertEqual(
            decode_keys(data), (["up", "down", "right", "left"], ""))

    def test_application_cursor_arrows(self):
        data = "\x1bOA\x1bOB\x1bOC\x1bOD"
        self.assertEqual(
            decode_keys(data), (["up", "down", "right", "left"], ""))

    def test_split_escape_sequence(self):
        keys, pending = decode_keys("\x1b[")
        self.assertEqual(keys, [])
        self.assertEqual(pending, "\x1b[")

        keys, pending = decode_keys("A", pending)
        self.assertEqual(keys, ["up"])
        self.assertEqual(pending, "")

    def test_lone_escape_then_key(self):
        # Escape followed by a normal character is not an arrow sequence.
        self.assertEqual(decode_keys("\x1bw"), (["w"], ""))

    def test_unknown_escape_is_dropped(self):
        self.assertEqual(decode_keys("\x1b[Hw"), (["w"], ""))

    def test_mixed_keys(self):
        keys, pending = decode_keys("w\x1b[Da")
        self.assertEqual(keys, ["w", "left", "a"])
        self.assertEqual(pending, "")


class CommandForKeyTests(unittest.TestCase):
    """Key mapping and the drive-or-turn exclusivity guarantee."""

    linear = 0.12
    angular = 0.5

    def command(self, key):
        return command_for_key(key, self.linear, self.angular)

    def test_forward_keys(self):
        for key in ("w", "W", "up"):
            self.assertEqual(self.command(key), (self.linear, 0.0))

    def test_backward_keys(self):
        for key in ("s", "S", "down"):
            self.assertEqual(self.command(key), (-self.linear, 0.0))

    def test_left_keys(self):
        for key in ("a", "A", "left"):
            self.assertEqual(self.command(key), (0.0, self.angular))

    def test_right_keys(self):
        for key in ("d", "D", "right"):
            self.assertEqual(self.command(key), (0.0, -self.angular))

    def test_space_stops(self):
        self.assertEqual(self.command(" "), (0.0, 0.0))

    def test_unbound_keys_ignored(self):
        for key in ("q", "Q", "x", "1", "\n"):
            self.assertIsNone(self.command(key))

    def test_never_drives_and_turns_together(self):
        for key in ("w", "s", "a", "d", "up", "down", "left", "right"):
            linear, angular = self.command(key)
            self.assertFalse(
                linear != 0.0 and angular != 0.0,
                "key %r produced both a drive and a turn" % key)


if __name__ == "__main__":
    unittest.main()
