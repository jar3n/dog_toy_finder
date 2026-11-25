"""Tests for capture labeling in ``vision_capture``."""

import os
import re
import sys
import unittest
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


if __name__ == "__main__":
    unittest.main()
