"""Tests for the shared LLM comparison schema, metrics, and CSV helpers.

Run from the package directory:

    cd software/ros2_ws/src/dog_toy_finder_data
    python3 -m unittest discover -s tests
"""

import csv
import io
import json
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout

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

    def test_distance_list_maps_to_json_csv_value(self):
        mapped = llm_trials.merged_ground_truth({
            "distance_m": [0.7, 0.72, 0.68],
        })
        self.assertEqual(json.loads(mapped["gt_distance_m"]),
                         [0.7, 0.72, 0.68])

    def test_distances_alias_maps_to_gt_column(self):
        mapped = llm_trials.merged_ground_truth({
            "distances_m": [0.8, 0.9],
        })
        self.assertEqual(json.loads(mapped["gt_distance_m"]), [0.8, 0.9])

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

    def test_multiple_distance_measurements_use_mean(self):
        row = {
            "target_visible": "True",
            "distance_m": "0.72",
            "gt_target_visible": "True",
            "gt_distance_m": "[0.7, 0.8]",
        }
        metrics = llm_trials.vision_metrics(row, distance_tolerance=0.1)
        self.assertAlmostEqual(metrics["distance_error_m"], 0.03)
        self.assertTrue(metrics["distance_within_tolerance"])

    def test_comma_separated_distance_ground_truth_is_supported(self):
        row = {
            "target_visible": "True",
            "distance_m": "0.72",
            "gt_target_visible": "True",
            "gt_distance_m": "0.7, 0.8",
        }
        metrics = llm_trials.vision_metrics(row, distance_tolerance=0.1)
        self.assertAlmostEqual(metrics["distance_error_m"], 0.03)

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
        self.assertEqual(row["method"], "ml")

    def test_vision_row_records_classical_detector_method(self):
        result = {"id": "43", "target": "red ball", "image": ""}
        entry = {
            "model": "classical", "method": "classical",
            "target_visible": False, "position": "none", "size": "none",
            "distance_m": 0.0, "obstacle_ahead": "none", "seconds": 0.02,
        }
        row = llm_trials.vision_row(result, entry, "classical", "vision1")
        self.assertEqual(row["model"], "classical")
        self.assertEqual(row["method"], "classical")
        self.assertIn("method", llm_trials.VISION_FIELDS)

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

    def test_analyze_cli_writes_json_and_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "vision_trials.csv")
            rows = []
            for ask_id in ("1", "2"):
                for model, latency in (("gemma4", 1.0), ("minicpm", 2.0)):
                    rows.append({
                        "kind": "vision", "run_id": "vision1",
                        "ask_id": ask_id, "model": model,
                        "latency_s": latency, "target_visible": "True",
                        "gt_target_visible": "True",
                    })
            llm_trials.write_csv(path, rows, llm_trials.VISION_FIELDS)
            output = os.path.join(directory, "analysis.json")
            csv_output = os.path.join(directory, "analysis.csv")
            with redirect_stdout(io.StringIO()):
                self.assertEqual(llm_trials.main([
                    "analyze", path, "-o", output,
                    "--csv-output", csv_output]), 0)
            with open(output) as handle:
                analysis = json.load(handle)
            self.assertEqual(analysis["kind"], "vision")
            self.assertTrue(os.path.exists(csv_output))

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


class BearingScoringTests(unittest.TestCase):
    def test_bearing_aliases_map_to_gt_column(self):
        mapped = llm_trials.merged_ground_truth({
            "lateral_bearing_deg": 12.5,
            "bearing_deg": -4.0,
        })
        # Both aliases feed the same column; the later key wins.
        self.assertEqual(mapped["gt_lateral_bearing_deg"], -4.0)

    def test_vision_metrics_score_bearing_tolerance(self):
        row = {
            "gt_target_visible": True,
            "target_visible": True,
            "position": "center",
            "size": "small",
            "distance_m": 1.0,
            "gt_distance_m": 1.0,
            "lateral_bearing_deg": 12.0,
            "gt_lateral_bearing_deg": 8.0,
            "obstacle_ahead": "none",
            "gt_obstacle_ahead": "none",
        }
        metrics = llm_trials.vision_metrics(row, bearing_tolerance=5.0)
        self.assertAlmostEqual(metrics["bearing_error_deg"], 4.0)
        self.assertTrue(metrics["bearing_within_tolerance"])
        strict = llm_trials.vision_metrics(row, bearing_tolerance=2.0)
        self.assertFalse(strict["bearing_within_tolerance"])
        self.assertLess(strict["score"], metrics["score"])

    def test_vision_row_copies_bearing_and_ground_truth(self):
        row = llm_trials.vision_row(
            {"id": "cap1", "target": "red ball", "image": "run1/cap1/left.jpg"},
            {"model": "classical", "method": "classical",
             "target_visible": True, "lateral_bearing_deg": -9.5,
             "position": "right", "distance_m": 1.1},
            "classical", "run1",
            ground_truth={"target_visible": True,
                          "lateral_bearing_deg": -10.0, "distance_m": 1.0},
            bearing_tolerance=3.0)
        self.assertEqual(row["lateral_bearing_deg"], -9.5)
        self.assertEqual(row["gt_lateral_bearing_deg"], -10.0)
        self.assertTrue(row["bearing_within_tolerance"])
        self.assertAlmostEqual(row["bearing_error_deg"], 0.5)

    def test_aggregate_reports_bearing_summary(self):
        rows = [{
            "kind": "vision", "model": "m", "gt_target_visible": True,
            "target_visible": True, "lateral_bearing_deg": 11.0,
            "gt_lateral_bearing_deg": 10.0,
            "distance_m": 1.0, "gt_distance_m": 1.0,
        }]
        summary = llm_trials.aggregate(rows, bearing_tolerance=2.0)[0]
        self.assertAlmostEqual(summary["bearing_mae_deg"], 1.0)
        self.assertEqual(summary["bearing_within_rate"], 1.0)


if __name__ == "__main__":
    unittest.main()
