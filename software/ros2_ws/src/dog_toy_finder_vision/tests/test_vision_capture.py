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
        self.assertIn("--occlusion-percent", options)
        self.assertIn("--visible-side", options)
        self.assertFalse(parser.get_default("pilot"))
        self.assertEqual(parser.get_default("shots"), 3)
        self.assertEqual(parser.get_default("occlusion_percent"), 0.0)
        self.assertEqual(parser.get_default("visible_side"), "left")

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

    def test_generated_id_includes_occlusion_when_present(self):
        self.assertEqual(
            vision_capture.capture_id_from_ground_truth(
                "red ball", 1.15, 0.244,
                occlusion_percent=40.0, visible_side="right"),
            "red_ball_d1p15_op0p244_occ40_right")
        self.assertEqual(
            vision_capture.capture_id_from_ground_truth(
                "red ball", 1.15, 0.244,
                occlusion_percent=12.5, visible_side="left"),
            "red_ball_d1p15_op0p244_occ12p5_left")
        self.assertEqual(
            vision_capture.capture_id_from_ground_truth(
                "red ball", 1.15, 0.244,
                occlusion_percent=0.0, visible_side="right"),
            "red_ball_d1p15_op0p244")
        self.assertEqual(
            vision_capture.capture_id_from_ground_truth(
                "dog bone", 0.0, 0.0, target_visible=False,
                occlusion_percent=25.0, visible_side="left"),
            "dog_bone_absent_occ25_left")

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


class InteractiveSessionTests(unittest.TestCase):
    def test_exit_commands_are_recognised_case_insensitively(self):
        for value in ("q", "QUIT", " done ", "Exit"):
            self.assertTrue(vision_capture._is_exit_command(value))
        self.assertFalse(vision_capture._is_exit_command("red_ball_d1_op0"))

    def test_exit_at_measurement_prompt_finishes_session(self):
        with patch("builtins.input", return_value="q"):
            with self.assertRaises(vision_capture.CaptureFinished):
                vision_capture._prompt_float("distance_m")

    def test_exit_at_capture_id_prompt_finishes_session(self):
        with patch("builtins.input", return_value="quit"):
            with self.assertRaises(vision_capture.CaptureFinished):
                vision_capture.capture_id_for_run(
                    True, "red_ball_d1p15_op0p244")

    def test_capture_ready_prompt_waits_before_shots(self):
        with patch("builtins.input", return_value="") as prompted:
            vision_capture._prompt_capture_ready()
        self.assertIn("Press Enter to capture stereo pair",
                      prompted.call_args.args[0])

    def test_exit_at_capture_ready_prompt_finishes_session(self):
        with patch("builtins.input", return_value="q"):
            with self.assertRaises(vision_capture.CaptureFinished):
                vision_capture._prompt_capture_ready()

    def test_notes_prompt_offers_default_and_accepts_text(self):
        with patch("builtins.input", return_value="") as prompted:
            self.assertEqual(vision_capture._prompt_notes("cli note"),
                             "cli note")
        self.assertIn("Add in brief notes on the scene",
                      prompted.call_args.args[0])
        with patch("builtins.input", return_value="scene note"):
            self.assertEqual(vision_capture._prompt_notes("cli note"),
                             "scene note")

    def test_exit_at_notes_prompt_finishes_session(self):
        with patch("builtins.input", return_value="q"):
            with self.assertRaises(vision_capture.CaptureFinished):
                vision_capture._prompt_notes()

    def test_occlusion_percent_prompt_validates_range(self):
        answers = iter(["150", "40"])
        with patch("builtins.input",
                   side_effect=lambda prompt: next(answers)) as prompted:
            self.assertEqual(
                vision_capture._prompt_occlusion_percent(0.0), 40.0)
        self.assertIn("occlusion_percent (0-100)",
                      prompted.call_args_list[-1].args[0])

    def test_exit_at_occlusion_percent_prompt_finishes_session(self):
        with patch("builtins.input", return_value="q"):
            with self.assertRaises(vision_capture.CaptureFinished):
                vision_capture._prompt_occlusion_percent()

    def test_visible_side_prompt_validates_and_normalizes(self):
        answers = iter(["middle", "RIGHT"])
        with patch("builtins.input",
                   side_effect=lambda prompt: next(answers)) as prompted:
            self.assertEqual(vision_capture._prompt_visible_side("left"),
                             "right")
        self.assertIn("visible_side (left/right)",
                      prompted.call_args_list[-1].args[0])

    def test_exit_at_visible_side_prompt_finishes_session(self):
        with patch("builtins.input", return_value="done"):
            with self.assertRaises(vision_capture.CaptureFinished):
                vision_capture._prompt_visible_side()

    def test_occlusion_prompt_offers_cli_defaults(self):
        args = SimpleNamespace(occlusion_percent=25.0, visible_side="right")
        with patch("builtins.input", return_value="") as prompted:
            self.assertEqual(vision_capture._prompt_occlusion(args),
                             (25.0, "right"))
        prompts = [call.args[0] for call in prompted.call_args_list]
        self.assertIn("25.00", prompts[0])
        self.assertIn("[right]", prompts[1])

    def test_zero_occlusion_skips_visible_side_prompt(self):
        args = SimpleNamespace(occlusion_percent=0.0, visible_side="left")
        with patch("builtins.input", return_value="") as prompted:
            self.assertEqual(vision_capture._prompt_occlusion(args),
                             (0.0, None))
        self.assertEqual(prompted.call_count, 1)
        self.assertIn("occlusion_percent (0-100)", prompted.call_args.args[0])

    def test_interactive_notes_and_occlusion_are_saved_for_scene(self):
        args = SimpleNamespace(notes="cli note", obstacle="none", shots=1,
                               target="red ball", depth_m=None,
                               center_offset_m=None, bearing_deg=None,
                               not_visible=False, occlusion_percent=0.0,
                               visible_side="left")
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(vision_capture, "_scene_ground_truth",
                              return_value=("red ball", True, 1.0, 0.0, 0.0)), \
                 patch.object(vision_capture, "_prompt_notes",
                              return_value="scene note"), \
                 patch.object(vision_capture, "_prompt_occlusion",
                              return_value=(40.0, "right")), \
                 patch.object(vision_capture, "_prompt_capture_ready"), \
                 patch.object(vision_capture, "capture_id_for_run",
                              return_value="scene_a"), \
                 patch.object(vision_capture.vision_config, "eye_intrinsics",
                              return_value={}):
                scene = vision_capture._prepare_scene(
                    args, {}, tmp, "run", "/l", "/r", True)
        ground_truth = scene["common_meta"]["ground_truth"]
        self.assertEqual(ground_truth["notes"], "scene note")
        self.assertEqual(ground_truth["occlusion_percent"], 40.0)
        self.assertEqual(ground_truth["visible_side"], "right")

    def test_scripted_occlusion_flags_are_validated_and_saved(self):
        args = SimpleNamespace(notes="cli note", obstacle="none", shots=1,
                               target="red ball", depth_m=1.0,
                               center_offset_m=0.0, bearing_deg=None,
                               not_visible=False, occlusion_percent=35.0,
                               visible_side="right")
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(vision_capture.vision_config, "eye_intrinsics",
                              return_value={}):
                scene = vision_capture._prepare_scene(
                    args, {}, tmp, "run", "/l", "/r", False)
        ground_truth = scene["common_meta"]["ground_truth"]
        self.assertEqual(ground_truth["occlusion_percent"], 35.0)
        self.assertEqual(ground_truth["visible_side"], "right")
        self.assertEqual(scene["capture_id"], "red_ball_d1_op0_occ35_right")

        args.occlusion_percent = 0.0
        with tempfile.TemporaryDirectory() as tmp:
            with patch.object(vision_capture.vision_config, "eye_intrinsics",
                              return_value={}):
                scene = vision_capture._prepare_scene(
                    args, {}, tmp, "run", "/l", "/r", False)
        ground_truth = scene["common_meta"]["ground_truth"]
        self.assertEqual(ground_truth["occlusion_percent"], 0.0)
        self.assertNotIn("visible_side", ground_truth)
        self.assertEqual(scene["capture_id"], "red_ball_d1_op0")

        args.occlusion_percent = 101.0
        with self.assertRaises(SystemExit):
            vision_capture._prepare_scene(
                args, {}, tmp, "run", "/l", "/r", False)

    def test_exit_at_toy_group_prompt_finishes_session(self):
        args = SimpleNamespace(target="red ball")
        with patch("builtins.input", return_value="q"):
            with self.assertRaises(vision_capture.CaptureFinished):
                vision_capture._prompt_toy_group(args)

    def test_toy_group_prompts_for_toy_and_scene_count(self):
        args = SimpleNamespace(target="red ball")
        answers = iter(["white bottle", "3"])
        with patch("builtins.input", lambda prompt: next(answers)):
            toy, count = vision_capture._prompt_toy_group(args)
        self.assertEqual(toy, "white bottle")
        self.assertEqual(count, 3)

    def test_scene_count_reprompts_until_a_positive_integer(self):
        answers = iter(["0", "two", "2"])
        with patch("builtins.input", lambda prompt: next(answers)):
            self.assertEqual(
                vision_capture._prompt_int("scenes with this toy"), 2)

    def test_scene_ground_truth_uses_the_group_toy_and_assumes_visible(self):
        args = SimpleNamespace(target="red ball", depth_m=None,
                               center_offset_m=None, bearing_deg=None,
                               not_visible=False)
        answers = iter(["1.10", "0.194"])
        with patch("builtins.input", lambda prompt: next(answers)):
            result = vision_capture._scene_ground_truth(
                args, True, target="dog bone", target_visible=True)
        target, visible, depth, bearing, center_offset = result
        self.assertEqual(target, "dog bone")
        self.assertTrue(visible)
        self.assertAlmostEqual(depth, 1.10)
        self.assertAlmostEqual(center_offset, 0.194)
        self.assertAlmostEqual(bearing, 10.0, delta=0.05)

    def test_center_offset_prompt_accepts_decimal_float(self):
        args = SimpleNamespace(target="red ball", depth_m=None,
                               center_offset_m=None, bearing_deg=None,
                               not_visible=False)
        answers = iter(["1.0", "0.2"])
        with patch("builtins.input", lambda prompt: next(answers)):
            result = vision_capture._scene_ground_truth(
                args, True, target="red ball", target_visible=True)
        self.assertAlmostEqual(result[4], 0.2)

    def test_scene_recap_reports_ground_truth_for_scene(self):
        scene = {
            "capture_id": "red_ball_d1p15_op0p244_occ40_right",
            "common_meta": {
                "target": "red ball",
                "ground_truth": {
                    "target_visible": True,
                    "distance_m": 1.15,
                    "center_offset_m": 0.244,
                    "lateral_bearing_deg": 11.98,
                    "obstacle_ahead": "none",
                    "occlusion_percent": 40.0,
                    "visible_side": "right",
                    "notes": "white bottle on the mat",
                },
            },
        }
        lines = vision_capture._scene_recap(scene).splitlines()
        self.assertEqual(
            lines[0], "Recap red_ball_d1p15_op0p244_occ40_right (red ball):")
        for expected in (
                "\ttarget_visible: true",
                "\tdistance_m: 1.150",
                "\tcenter_offset_m: 0.244",
                "\tlateral_bearing_deg: +11.98",
                "\tobstacle_ahead: none",
                "\tocclusion_percent: 40.0",
                "\tvisible_side: right",
                "\tnotes: white bottle on the mat"):
            self.assertIn(expected, lines)

        scene["common_meta"]["ground_truth"]["center_offset_m"] = 0.2
        self.assertIn("\tcenter_offset_m: 0.2",
                      vision_capture._scene_recap(scene).splitlines())

        scene["common_meta"]["ground_truth"] = {
            "target_visible": False,
            "distance_m": 1.15,
            "lateral_bearing_deg": 0.0,
            "obstacle_ahead": "none",
            "occlusion_percent": 0.0,
            "notes": "empty scene",
        }
        lines = vision_capture._scene_recap(scene).splitlines()
        self.assertIn("\ttarget_visible: false", lines)
        self.assertIn("\tocclusion_percent: 0.0", lines)
        self.assertFalse(any(line.startswith("\tcenter_offset_m")
                             for line in lines))
        self.assertFalse(any(line.startswith("\tvisible_side")
                             for line in lines))

    def test_capture_scene_does_not_print_each_saved_shot(self):
        args = SimpleNamespace(shots=2, timeout=1.0)
        scene = {
            "capture_id": "scene_a",
            "directory": "/tmp/vision_capture_no_shot_print",
            "common_meta": {"image_topics": {"left": "/l", "right": "/r"}},
        }
        pair = (object(), object(), (1.0, 1.1), 2.0)
        with patch.object(vision_capture, "_wait_for_pair", return_value=pair), \
             patch.object(vision_capture, "write_capture"), \
             patch("builtins.print") as printed:
            saved, after = vision_capture._capture_scene(object(), args, scene)
        self.assertEqual(len(saved), 2)
        self.assertEqual(after, 2.0)
        printed.assert_not_called()

    def test_main_loops_over_toy_groups_until_exit(self):
        def make_scene(capture_id, center_offset):
            return {
                "capture_id": capture_id,
                "directory": "/tmp/" + capture_id,
                "common_meta": {
                    "target": "white bottle",
                    "image_topics": {"left": "/l", "right": "/r"},
                    "ground_truth": {
                        "target_visible": True,
                        "distance_m": 1.0,
                        "center_offset_m": center_offset,
                        "lateral_bearing_deg": 0.0,
                        "obstacle_ahead": "none",
                        "occlusion_percent": 0.0,
                        "visible_side": "left",
                        "notes": "note " + capture_id,
                    },
                },
            }

        scene_a = make_scene("scene_a", 0.1)
        scene_b = make_scene("scene_b", -0.2)
        with patch("sys.stdin.isatty", return_value=True), \
                patch.object(vision_capture.vision_config, "load_config",
                             return_value={}), \
                patch.object(vision_capture.vision_config, "get",
                             return_value="/stereo/topic"), \
                patch.object(vision_capture.rclpy, "init"), \
                patch.object(vision_capture.rclpy, "try_shutdown"), \
                patch.object(vision_capture, "StereoCapture"), \
                patch.object(vision_capture, "_prompt_toy_group",
                             side_effect=[("white bottle", 2),
                                          vision_capture.CaptureFinished()]) \
                as toy_prompt, \
                patch.object(vision_capture, "_prepare_scene",
                             side_effect=[scene_a, scene_b]) as prepare, \
                patch.object(vision_capture, "_capture_scene",
                             side_effect=[(["a"], 1.0), (["b"], 2.0)]) \
                as capture, \
                patch("builtins.print") as printed:
            code = vision_capture.main([])
        self.assertEqual(code, 0)
        self.assertEqual(toy_prompt.call_count, 2)
        self.assertEqual(prepare.call_count, 2)
        self.assertEqual(capture.call_count, 2)
        prepare_kwargs = [call.kwargs for call in prepare.call_args_list]
        self.assertEqual([item["target"] for item in prepare_kwargs],
                         ["white bottle", "white bottle"])
        self.assertTrue(all(item["target_visible"] for item in prepare_kwargs))
        printed_lines = [call.args[0] for call in printed.call_args_list
                         if call.args]
        self.assertFalse(any(
            str(line).lstrip().startswith("{") or "scene_count" in str(line)
            for line in printed_lines))
        self.assertIn("Scene 1/2 for white bottle", printed_lines)
        self.assertIn("--- Next scene ---", printed_lines)
        self.assertIn("Scene 2/2 for white bottle", printed_lines)
        self.assertEqual(
            [line for line in printed_lines if str(line).startswith("Recap ")],
            [vision_capture._scene_recap(scene_a),
             vision_capture._scene_recap(scene_b)])
        scene_positions = [
            index for index, call in enumerate(printed.call_args_list)
            if call.args and str(call.args[0]).startswith("Scene ")
        ]
        self.assertEqual(len(scene_positions), 2)
        between_scenes = printed.call_args_list[
            scene_positions[0] + 1:scene_positions[1]]
        self.assertEqual(sum(1 for call in between_scenes if not call.args), 3)
        self.assertIn("--- Next scene ---",
                      [call.args[0] for call in between_scenes if call.args])


if __name__ == "__main__":
    unittest.main()
