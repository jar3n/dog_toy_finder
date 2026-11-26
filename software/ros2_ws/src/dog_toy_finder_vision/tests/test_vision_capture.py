"""Tests for capture labeling in ``vision_capture``."""

import os
import re
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, SRC_ROOT)

from dog_toy_finder_vision import vision_capture  # noqa: E402


class CaptureIdTests(unittest.TestCase):
    def test_capture_id_is_not_a_command_line_flag(self):
        parser = vision_capture._build_parser()
        options = [
            option for action in parser._actions
            for option in action.option_strings
        ]
        self.assertNotIn("--capture-id", options)
        self.assertIn("--run-id", options)
        self.assertIn("--shots", options)
        self.assertIn("--pilot", options)
        self.assertFalse(parser.get_default("pilot"))
        self.assertEqual(parser.get_default("shots"), 3)

    def test_shot_directory_names_are_zero_padded(self):
        self.assertEqual(vision_capture.shot_dir_name(1), "shot_01")
        self.assertEqual(vision_capture.shot_dir_name(12), "shot_12")

    def test_reusing_a_scene_id_removes_stale_numbered_shots(self):
        with tempfile.TemporaryDirectory() as directory:
            stale = os.path.join(directory, "shot_03")
            os.makedirs(stale)
            keep = os.path.join(directory, "notes")
            os.makedirs(keep)
            vision_capture._clear_previous_shots(directory)
            self.assertFalse(os.path.exists(stale))
            self.assertTrue(os.path.isdir(keep))

    def test_generated_id_uses_toy_name_and_measurements(self):
        self.assertEqual(
            vision_capture.capture_id_from_ground_truth(
                "red ball", 1.15, 0.244),
            "red_ball_d1p15_op0p244")
        self.assertEqual(
            vision_capture.capture_id_from_ground_truth(
                "red ball", 0.90, -0.126),
            "red_ball_d0p9_om0p126")
        self.assertEqual(
            vision_capture.capture_id_from_ground_truth(
                "dog bone", 0.0, 0.0, target_visible=False),
            "dog_bone_absent")

    def test_interactive_prompt_requires_a_nonempty_id_without_default(self):
        answers = iter(["", "   ", "ball_115m_b12"])
        with patch("builtins.input", lambda prompt: next(answers)):
            self.assertEqual(
                vision_capture.capture_id_for_run(True), "ball_115m_b12")

    def test_interactive_empty_answer_accepts_generated_default(self):
        with patch("builtins.input", return_value="   ") as prompt:
            value = vision_capture.capture_id_for_run(
                True, "red_ball_d1p15_op0p244")
        self.assertEqual(value, "red_ball_d1p15_op0p244")
        self.assertIn("[red_ball_d1p15_op0p244]", prompt.call_args.args[0])

    def test_noninteractive_uses_the_generated_default(self):
        self.assertEqual(
            vision_capture.capture_id_for_run(
                False, "red_ball_d1p15_op0p244"),
            "red_ball_d1p15_op0p244")

    def test_noninteractive_timestamp_fallback_without_default(self):
        value = vision_capture.capture_id_for_run(False)
        self.assertRegex(value, r"^\d{8}_\d{6}_\d{3}$")

    def test_id_sanitising_replaces_spaces(self):
        self.assertEqual(
            re.sub(r"[^A-Za-z0-9_.-]+", "_", "red ball 1"),
            "red_ball_1")


class FreshShotTests(unittest.TestCase):
    @staticmethod
    def _image(stamp_seconds=1.0):
        stamp = SimpleNamespace(sec=int(stamp_seconds),
                                nanosec=int((stamp_seconds % 1.0) * 1e9))
        return SimpleNamespace(header=SimpleNamespace(stamp=stamp))

    def _node(self):
        node = object.__new__(vision_capture.StereoCapture)
        node._started = 0.0
        node._pair_tolerance = 0.5
        node._left = None
        node._right = None
        return node

    def test_fresh_pair_returns_receipt_time_for_shot_ordering(self):
        node = self._node()
        node._left = (1.0, self._image(1.0))
        node._right = (1.1, self._image(1.0))
        pair = node.fresh_pair()
        self.assertIsNotNone(pair)
        self.assertEqual(len(pair), 4)
        self.assertAlmostEqual(pair[3], 1.1)

    def test_fresh_pair_rejects_frames_from_a_previous_shot(self):
        node = self._node()
        node._left = (1.0, self._image(1.0))
        node._right = (1.1, self._image(1.0))
        first = node.fresh_pair()
        self.assertIsNone(node.fresh_pair(after=first[3]))

        node._left = (1.2, self._image(1.2))
        self.assertIsNone(node.fresh_pair(after=first[3]),
                          "a stale right frame must not be reused")
        node._right = (1.3, self._image(1.2))
        second = node.fresh_pair(after=first[3])
        self.assertIsNotNone(second)
        self.assertAlmostEqual(second[3], 1.3)


if __name__ == "__main__":
    unittest.main()
