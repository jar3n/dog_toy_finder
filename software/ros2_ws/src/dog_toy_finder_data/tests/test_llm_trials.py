"""Tests for the shared LLM comparison schema, metrics, and CSV helpers.

Run from the package directory:

    cd software/ros2_ws/src/dog_toy_finder_data
    python3 -m unittest discover -s tests
"""

import csv
import os
import sys
import tempfile
import unittest

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "dog_toy_finder_data"))

import llm_trials


class GroundTruthTests(unittest.TestCase):
    def test_short_keys_map_to_gt_columns(self):
        mapped = llm_trials.merged_ground_truth({
            "target_visible": True,
            "position": "left",
            "distance": 0.6,
            "notes": "on the rug",
        })
        self.assertEqual(mapped["gt_target_visible"], True)
        self.assertEqual(mapped["gt_position"], "left")
        self.assertEqual(mapped["gt_distance_m"], 0.6)
        self.assertEqual(mapped["gt_notes"], "on the rug")

    def test_unknown_keys_are_ignored(self):
        self.assertEqual(llm_trials.merged_ground_truth({"bogus": 1}), {})

    def test_non_dict_ground_truth_is_ignored(self):
        self.assertEqual(llm_trials.merged_ground_truth(["not", "a", "dict"]), {})


class VisionMetricsTests(unittest.TestCase):
    def test_full_match_scores_one(self):
        row = {
            "target_visible": "True",
            "position": "left",
            "size": "medium",
            "distance_m": "0.72",
            "obstacle_ahead": "none",
            "gt_target_visible": "True",
            "gt_position": "left",
            "gt_size": "medium",
            "gt_distance_m": "0.7",
            "gt_obstacle_ahead": "none",
        }
        metrics = llm_trials.vision_metrics(row, distance_tolerance=0.1)
        self.assertTrue(metrics["visible_correct"])
        self.assertTrue(metrics["position_correct"])
        self.assertTrue(metrics["size_correct"])
        self.assertAlmostEqual(metrics["distance_error_m"], 0.02)
        self.assertTrue(metrics["distance_within_tolerance"])
        self.assertTrue(metrics["obstacle_correct"])
        self.assertEqual(metrics["score"], 1.0)

    def test_distance_outside_tolerance_fails_check(self):
        row = {
            "target_visible": "True",
            "position": "left",
            "size": "medium",
            "distance_m": "1.2",
            "obstacle_ahead": "none",
            "gt_target_visible": "True",
            "gt_position": "left",
            "gt_size": "medium",
            "gt_distance_m": "0.7",
            "gt_obstacle_ahead": "none",
        }
        metrics = llm_trials.vision_metrics(row, distance_tolerance=0.25)
        self.assertFalse(metrics["distance_within_tolerance"])
        self.assertAlmostEqual(metrics["distance_error_m"], 0.5)
        self.assertAlmostEqual(metrics["score"], 0.8)

    def test_not_visible_only_scores_visibility_and_obstacle(self):
        row = {
            "target_visible": "False",
            "position": "none",
            "size": "none",
            "distance_m": "0",
            "obstacle_ahead": "chair",
            "gt_target_visible": "False",
            "gt_position": "none",
            "gt_obstacle_ahead": "chair",
        }
        metrics = llm_trials.vision_metrics(row)
        self.assertTrue(metrics["visible_correct"])
        self.assertTrue(metrics["obstacle_correct"])
        self.assertEqual(metrics["score"], 1.0)
        self.assertEqual(metrics["position_correct"], "")

    def test_blank_obstacle_ground_truth_is_not_scored(self):
        row = {
            "target_visible": "True",
            "position": "left",
            "size": "medium",
            "distance_m": "0.7",
            "obstacle_ahead": "chair",
            "gt_target_visible": "True",
            "gt_position": "left",
            "gt_size": "medium",
            "gt_distance_m": "0.7",
        }
        metrics = llm_trials.vision_metrics(row)
        self.assertEqual(metrics["obstacle_correct"], "")
        self.assertEqual(metrics["score"], 1.0)

    def test_missing_ground_truth_leaves_metrics_blank(self):
        metrics = llm_trials.vision_metrics({"target_visible": "True"})
        for field in llm_trials.VISION_METRIC_FIELDS:
            self.assertEqual(metrics[field], "")


class PlanningMetricsTests(unittest.TestCase):
    def test_expected_action_and_args_are_found(self):
        row = {
            "reply": "I will drive forward 1 metre.",
            "tool_calls": '[{"name": "drive_distance", "input": {"direction": "forward", "meters": 1}}]',
            "gt_expected_action": "drive_distance",
            "gt_expected_args": "forward,1",
        }
        metrics = llm_trials.planning_metrics(row)
        self.assertTrue(metrics["action_mentioned"])
        self.assertTrue(metrics["args_mentioned"])
        self.assertEqual(metrics["score"], 1.0)

    def test_missing_argument_lowers_score(self):
        row = {
            "reply": "I will call drive_distance to go forward.",
            "tool_calls": "",
            "gt_expected_action": "drive_distance",
            "gt_expected_args": "forward,2",
        }
        metrics = llm_trials.planning_metrics(row)
        self.assertTrue(metrics["action_mentioned"])
        self.assertFalse(metrics["args_mentioned"])
        self.assertEqual(metrics["score"], 0.5)

    def test_recorded_success_overrides_text_checks(self):
        row = {
            "reply": "I cannot do that.",
            "tool_calls": "",
            "gt_expected_action": "drive_distance",
            "gt_success": "True",
        }
        self.assertEqual(llm_trials.planning_metrics(row)["score"], 1.0)


class RowAndCsvTests(unittest.TestCase):
    def test_vision_row_uses_ask_ground_truth(self):
        result = {
            "id": "42",
            "target": "red ball",
            "image": "vision/results/images/a.jpg",
            "models": ["gemma4"],
            "results": [{
                "model": "gemma4",
                "description": "a red ball on the left",
                "target_visible": True,
                "position": "left",
                "size": "medium",
                "distance_m": 0.68,
                "obstacle_ahead": "none",
                "path_clear": True,
                "seconds": 1.5,
            }],
        }
        row = llm_trials.vision_row(
            result, result["results"][0], "gemma4", "vision1",
            ground_truth={"target_visible": True, "position": "left",
                          "size": "medium", "distance_m": 0.7,
                          "obstacle_ahead": "none"})
        self.assertEqual(row["trial_id"], "vision1-42-gemma4")
        self.assertEqual(row["latency_s"], 1.5)
        self.assertEqual(row["gt_distance_m"], 0.7)
        self.assertEqual(row["visible_correct"], True)
        self.assertEqual(row["score"], 1.0)

    def test_append_then_read_roundtrip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "nested", "vision_trials.csv")
            row = {"kind": "vision", "run_id": "run1", "model": "gemma4",
                   "target": "red ball", "score": 1.0}
            llm_trials.append_csv(path, row, llm_trials.VISION_FIELDS)
            llm_trials.append_csv(path, dict(row, model="minicpm"),
                                  llm_trials.VISION_FIELDS)
            rows, fields = llm_trials.read_csv(path)
            self.assertEqual(len(rows), 2)
            self.assertEqual(fields, llm_trials.VISION_FIELDS)
            self.assertEqual(rows[0]["model"], "gemma4")
            self.assertEqual(rows[1]["model"], "minicpm")

    def test_template_writes_one_row_per_model(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "planning.csv")
            llm_trials.write_template(path, llm_trials.KIND_PLANNING,
                                      models=["llama3.2", "mistral"],
                                      run_id="plan1", prompt="Drive 1 m")
            with open(path, newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([row["model"] for row in rows],
                             ["llama3.2", "mistral"])
            self.assertTrue(all(row["gt_expected_action"] == "" for row in rows))


class AggregateTests(unittest.TestCase):
    def test_per_model_summary(self):
        rows = [
            {"kind": "vision", "model": "a", "latency_s": "1.0",
             "target_visible": "True", "gt_target_visible": "True",
             "position": "left", "gt_position": "left",
             "size": "medium", "gt_size": "medium",
             "distance_m": "0.7", "gt_distance_m": "0.7",
             "obstacle_ahead": "none", "gt_obstacle_ahead": "none"},
            {"kind": "vision", "model": "a", "latency_s": "2.0",
             "target_visible": "False", "gt_target_visible": "True",
             "position": "none", "gt_position": "left",
             "size": "none", "gt_size": "medium",
             "distance_m": "0", "gt_distance_m": "0.7",
             "obstacle_ahead": "none", "gt_obstacle_ahead": "none"},
            {"kind": "vision", "model": "b", "latency_s": "3.0",
             "target_visible": "True", "gt_target_visible": "True",
             "position": "left", "gt_position": "left",
             "size": "medium", "gt_size": "medium",
             "distance_m": "0.8", "gt_distance_m": "0.7",
             "obstacle_ahead": "none", "gt_obstacle_ahead": "none"},
        ]
        summaries = {item["model"]: item
                     for item in llm_trials.aggregate(rows)}
        self.assertEqual(summaries["a"]["trials"], 2)
        self.assertEqual(summaries["a"]["visible_accuracy"], 0.5)
        self.assertEqual(summaries["a"]["mean_latency_s"], 1.5)
        self.assertEqual(summaries["b"]["visible_accuracy"], 1.0)
        self.assertAlmostEqual(summaries["b"]["distance_mae_m"], 0.1)


if __name__ == "__main__":
    unittest.main()
