"""Tests for post-collection statistical analysis of LLM trials."""

import os
import sys
import tempfile
import unittest

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "dog_toy_finder_data"))

import llm_trials  # noqa: E402
import trial_stats  # noqa: E402


class DistributionTests(unittest.TestCase):
    def test_descriptive_statistics(self):
        summary = trial_stats.describe([1, 2, 3, 4, 5], confidence=0.95)
        self.assertEqual(summary["n"], 5)
        self.assertAlmostEqual(summary["mean"], 3.0)
        self.assertAlmostEqual(summary["median"], 3.0)
        self.assertAlmostEqual(summary["std"], 2.5 ** 0.5)
        self.assertAlmostEqual(summary["sem"], (2.5 ** 0.5) / 5 ** 0.5)
        self.assertAlmostEqual(summary["ci_low"], 1.037, places=2)
        self.assertAlmostEqual(summary["ci_high"], 4.963, places=2)

    def test_t_critical_values(self):
        self.assertAlmostEqual(trial_stats.t_critical(1), 12.706, places=2)
        self.assertAlmostEqual(trial_stats.t_critical(4), 2.776, places=2)

    def test_two_sided_p_values(self):
        self.assertEqual(trial_stats.t_two_sided_p(0.0, 10), 1.0)
        self.assertLess(trial_stats.t_two_sided_p(3.0, 10), 0.02)
        self.assertGreater(trial_stats.t_two_sided_p(1.0, 10), 0.3)

    def test_holm_adjustment(self):
        comparisons = [{"p_value": value} for value in (0.01, 0.04, 0.03)]
        trial_stats.adjust_holm(comparisons)
        self.assertAlmostEqual(comparisons[0]["p_value_holm"], 0.03)
        self.assertAlmostEqual(comparisons[1]["p_value_holm"], 0.06)
        self.assertAlmostEqual(comparisons[2]["p_value_holm"], 0.06)


class AnalyzeTests(unittest.TestCase):
    def _vision_rows(self):
        rows = []
        for ask_id, latency_a, latency_b in (("1", 1.0, 2.0),
                                             ("2", 2.0, 4.0)):
            for model, latency in (("gemma4", latency_a),
                                   ("minicpm", latency_b)):
                rows.append({
                    "kind": "vision",
                    "run_id": "vision1",
                    "ask_id": ask_id,
                    "model": model,
                    "latency_s": latency,
                    "target_visible": "True",
                    "gt_target_visible": "True",
                })
        return rows

    def test_analyze_produces_descriptives_and_paired_comparisons(self):
        analysis = trial_stats.analyze(self._vision_rows())
        self.assertEqual(analysis["kind"], "vision")
        self.assertEqual(analysis["models"], ["gemma4", "minicpm"])
        self.assertEqual(analysis["pair_field"], "run_id,ask_id")

        latency_descriptives = {
            row["model"]: row for row in analysis["descriptives"]
            if row["metric"] == "latency_s"}
        self.assertAlmostEqual(latency_descriptives["gemma4"]["mean"], 1.5)
        self.assertAlmostEqual(latency_descriptives["minicpm"]["mean"], 3.0)

        comparison = next(row for row in analysis["comparisons"]
                          if row["metric"] == "latency_s")
        self.assertAlmostEqual(comparison["mean_diff"], -1.5)
        self.assertEqual(comparison["n"], 2)
        self.assertIsNotNone(comparison["p_value"])
        self.assertIsNotNone(comparison["p_value_holm"])

    def test_planning_rows_pair_on_prompt(self):
        rows = []
        for prompt, elapsed_a, elapsed_b in (("find red ball", 1.0, 2.0),):
            for model, elapsed in (("llama3.2", elapsed_a),
                                   ("mistral", elapsed_b)):
                rows.append({
                    "kind": "planning",
                    "run_id": "plan1",
                    "prompt": prompt,
                    "model": model,
                    "elapsed_s": elapsed,
                })
        analysis = trial_stats.analyze(rows)
        self.assertEqual(analysis["pair_field"], "run_id,prompt")
        elapsed = next(row for row in analysis["comparisons"]
                       if row["metric"] == "elapsed_s")
        self.assertAlmostEqual(elapsed["mean_diff"], -1.0)

    def test_custom_pair_field(self):
        rows = self._vision_rows()
        for row in rows:
            row["scene"] = row["ask_id"]
        analysis = trial_stats.analyze(rows, pair_field="run_id,scene")
        self.assertEqual(analysis["pair_field"], "run_id,scene")
        self.assertTrue(analysis["comparisons"])

    def test_write_analysis_csv(self):
        analysis = trial_stats.analyze(self._vision_rows())
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "analysis.csv")
            trial_stats.write_analysis_csv(path, analysis)
            rows, fields = llm_trials.read_csv(path)
            self.assertIn("scope", fields)
            self.assertEqual(rows[0]["scope"], "descriptive")
            self.assertTrue(any(row["scope"] == "comparison" for row in rows))


if __name__ == "__main__":
    unittest.main()
