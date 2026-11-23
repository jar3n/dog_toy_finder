"""Tests for offline replay of saved rectified stereo captures."""

import json
import os
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image

SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, SRC_ROOT)

from dog_toy_finder_vision import vision_config  # noqa: E402
from dog_toy_finder_vision import vision_eval  # noqa: E402
try:  # installed ROS workspace vs. plain source checkout
    from dog_toy_finder_data import llm_trials  # noqa: E402
except ImportError:  # pragma: no cover - host-side fallback
    import llm_trials  # noqa: E402


class OfflineEvalTests(unittest.TestCase):
    def setUp(self):
        self.config = vision_config.load_config()
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.capture_dir = os.path.join(
            self.temp.name, "captures", "run1", "cap1")
        os.makedirs(self.capture_dir)

    def _write_synthetic_pair(self, center_x=90, disparity=20, visible=True):
        config = self.config
        width = int(config["camera"]["width"])
        height = int(config["camera"]["height"])
        left = np.full((height, width, 3), 210, dtype=np.uint8)
        right = np.full((height, width, 3), 210, dtype=np.uint8)
        yy, xx = np.ogrid[:height, :width]
        radius = 10
        if visible:
            left[(xx - center_x) ** 2 + (yy - 120) ** 2 <= radius ** 2] = \
                [255, 0, 0]
            right[((xx - (center_x - disparity)) ** 2
                   + (yy - 120) ** 2) <= radius ** 2] = [255, 0, 0]
        for eye, image in (("left", left), ("right", right)):
            Image.fromarray(image).save(
                os.path.join(self.capture_dir, "%s.jpg" % eye))
        truth = {
            "target_visible": visible,
            "position": "left" if visible else "none",
            "lateral_bearing_deg": vision_config.lateral_bearing_deg(
                center_x, config) if visible else 0.0,
            "distance_m": 1.0 if visible else 0.0,
            "size": "small" if visible else "none",
            "obstacle_ahead": "none",
        }
        meta = {
            "kind": "vision_capture",
            "capture_id": "cap1",
            "run_id": "run1",
            "target": "red ball",
            "ground_truth": truth,
        }
        with open(os.path.join(self.capture_dir, "meta.json"), "w") as handle:
            json.dump(meta, handle)

    def test_iter_captures_finds_complete_directories(self):
        self._write_synthetic_pair()
        incomplete = os.path.join(self.temp.name, "captures", "run1", "bad")
        os.makedirs(incomplete)
        found = list(vision_eval.iter_captures(
            os.path.join(self.temp.name, "captures")))
        self.assertEqual(found, [self.capture_dir])

    def test_replay_classical_answer_uses_ground_truth(self):
        self._write_synthetic_pair()
        result, meta = vision_eval.evaluate_capture(
            self.capture_dir, self.config, [], True, "", False, 1.0,
            include_ml=False)
        self.assertEqual(result["id"], "cap1")
        self.assertEqual(len(result["results"]), 1)
        entry = result["results"][0]
        self.assertTrue(entry["target_visible"])
        self.assertEqual(entry["position"], "left")
        self.assertGreater(entry["lateral_bearing_deg"], 0.0)

        row = llm_trials.vision_row(
            result, entry, "classical", "run1",
            ground_truth=meta["ground_truth"], bearing_tolerance=5.0)
        self.assertEqual(row["gt_lateral_bearing_deg"],
                         meta["ground_truth"]["lateral_bearing_deg"])
        self.assertTrue(row["bearing_within_tolerance"])
        self.assertAlmostEqual(row["bearing_error_deg"], 0.0)


if __name__ == "__main__":
    unittest.main()
