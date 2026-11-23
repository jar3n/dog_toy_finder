"""Tests for offline replay of saved rectified stereo captures."""

import contextlib
import io
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

import numpy as np
from PIL import Image

SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, SRC_ROOT)

from dog_toy_finder_vision import depth_client  # noqa: E402
from dog_toy_finder_vision import vision_config  # noqa: E402
from dog_toy_finder_vision import vision_eval  # noqa: E402
from dog_toy_finder_vision import yolo_client  # noqa: E402
try:  # installed ROS workspace vs. plain source checkout
    from dog_toy_finder_data import llm_trials  # noqa: E402
except ImportError:  # pragma: no cover - host-side fallback
    import llm_trials  # noqa: E402


def _detection(u, v, label):
    width = 12
    return {"label": label, "confidence": 0.95, "u": float(u), "v": float(v),
            "width_px": width, "height_px": width,
            "bbox_px": [u - width / 2, v - width / 2,
                        u + width / 2, v + width / 2]}


class FakeClient:
    """Stand-in for the host detector service using the saved red disc."""

    def __init__(self):
        self.calls = []

    def __call__(self, left_jpeg, right_jpeg, classes, config):
        self.calls.append((left_jpeg, right_jpeg, list(classes)))
        found = []
        for jpeg in (left_jpeg, right_jpeg):
            image = np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB"))
            mask = ((image[:, :, 0] > 200) & (image[:, :, 1] < 80)
                    & (image[:, :, 2] < 80))
            ys, xs = np.nonzero(mask)
            if len(xs) == 0:
                found.append([])
            else:
                found.append([_detection(xs.mean(), ys.mean(), classes[0])])
        return found[0], found[1], "fake-world"


class OfflineEvalTests(unittest.TestCase):
    def setUp(self):
        self.config = vision_config.load_config()
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = os.path.join(self.temp.name, "captures")
        self.capture_dir = os.path.join(self.root, "run1", "cap1")
        os.makedirs(self.capture_dir)

    def _write_synthetic_pair(self, center_x=90, disparity=20, visible=True):
        config = self.config
        width = int(config["camera"]["width"])
        height = int(config["camera"]["height"])
        left = np.full((height, width, 3), 210, dtype=np.uint8)
        right = np.full((height, width, 3), 210, dtype=np.uint8)
        yy, xx = np.ogrid[:height, :width]
        radius = 6
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
        incomplete = os.path.join(self.root, "run1", "bad")
        os.makedirs(incomplete)
        found = list(vision_eval.iter_captures(self.root))
        self.assertEqual(found, [self.capture_dir])

    def test_replay_pipeline_answer_uses_ground_truth(self):
        self._write_synthetic_pair()
        client = FakeClient()

        result, meta = vision_eval.evaluate_capture(
            self.capture_dir, self.config, "open_vocab", client=client)

        self.assertEqual(result["id"], "cap1")
        self.assertEqual(result["pipeline"], "open_vocab")
        self.assertEqual(result["models"], ["fake-world"])
        self.assertEqual(len(client.calls), 1, "one request carries both eyes")
        entry = result["results"][0]
        self.assertTrue(entry["target_visible"])
        self.assertEqual(entry["method"], "open_vocab")
        self.assertEqual(entry["distance_source"], "stereo_triangulation")
        self.assertEqual(entry["position"], "left")
        self.assertAlmostEqual(entry["distance_m"], 1.0, places=2)

        row = llm_trials.vision_row(
            result, entry, "fake-world", "run1",
            ground_truth=meta["ground_truth"], bearing_tolerance=5.0)
        self.assertEqual(row["method"], "open_vocab")
        self.assertEqual(row["gt_lateral_bearing_deg"],
                         meta["ground_truth"]["lateral_bearing_deg"])
        self.assertTrue(row["bearing_within_tolerance"])
        self.assertAlmostEqual(row["bearing_error_deg"], 0.0, delta=0.5)

    def test_second_pipeline_replays_with_dense_depth(self):
        self._write_synthetic_pair()

        def detect_one(jpeg, classes, config):
            image = np.asarray(Image.open(io.BytesIO(jpeg)).convert("RGB"))
            ys, xs = np.nonzero((image[:, :, 0] > 200) & (image[:, :, 1] < 80))
            if not len(xs):
                return [], "fake-world"
            return [{
                "label": classes[0], "confidence": 0.9,
                "u": float(xs.mean()), "v": float(ys.mean()),
                "width_px": 12.0, "height_px": 12.0,
                "bbox_px": [float(xs.min()), float(ys.min()),
                            float(xs.max()) + 1, float(ys.max()) + 1],
                "center_source": "box_center", "mask_area_px": 0,
                "depth_sample_pixels": [],
            }], "fake-world"

        def sample_depth(left, right, spec, config):
            return {
                "model": "fake-stereo", "depth_m": 1.0,
                "disparity_px": 20.0, "sample_source": spec["sample_source"],
                "sample_count": 49, "valid_count": 49,
            }

        result, meta = vision_eval.evaluate_capture(
            self.capture_dir, self.config, "second", client=detect_one,
            depth_client=sample_depth)

        self.assertEqual(result["pipeline"], "second")
        self.assertEqual(result["models"], ["fake-stereo"])
        entry = result["results"][0]
        self.assertEqual(entry["method"], "second")
        self.assertEqual(entry["distance_source"], "dense_stereo_depth")
        self.assertTrue(entry["target_visible"])
        self.assertAlmostEqual(entry["distance_m"], 1.0, places=2)

        row = llm_trials.vision_row(
            result, entry, entry["model"], "run1",
            ground_truth=meta["ground_truth"], bearing_tolerance=5.0)
        self.assertEqual(row["method"], "second")
        self.assertTrue(row["distance_within_tolerance"])
        self.assertTrue(row["bearing_within_tolerance"])

    def test_main_writes_csv_and_pipeline_manifest(self):
        self._write_synthetic_pair()
        original = vision_eval.evaluate_capture

        def evaluate_with_fake_client(path, config, pipeline=None, client=None):
            return original(path, config, pipeline, FakeClient())

        vision_eval.evaluate_capture = evaluate_with_fake_client
        self.addCleanup(setattr, vision_eval, "evaluate_capture", original)

        csv_path = os.path.join(self.temp.name, "trials.csv")
        results_path = os.path.join(self.temp.name, "results.jsonl")
        summary_path = os.path.join(self.temp.name, "summary.json")
        with contextlib.redirect_stdout(io.StringIO()):
            code = vision_eval.main([
                "--captures-dir", self.root, "--pipeline", "open_vocab",
                "--output-csv", csv_path, "--results-json", results_path,
                "--summary-json", summary_path])
        self.assertEqual(code, 0)
        with open(summary_path) as handle:
            summary = json.load(handle)
        self.assertEqual(summary["pipeline"], "open_vocab")
        self.assertEqual(summary["rows"], 1)
        self.assertEqual(summary["models"], ["fake-world"])
        rows, fields = llm_trials.read_csv(csv_path)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["method"], "open_vocab")
        self.assertIn("method", fields)
        self.assertTrue(os.path.isfile(results_path))


    def test_main_selects_each_pipeline_one_at_a_time(self):
        """`--pipeline` chooses exactly one pipeline per evaluation run."""
        self._write_synthetic_pair()

        def detect_both(left_jpeg, right_jpeg, classes, config):
            left, right, _ = FakeClient()(left_jpeg, right_jpeg, classes, config)
            return left, right, "fake-world"

        def detect_one(jpeg, classes, config):
            detections, _, _ = FakeClient()(jpeg, jpeg, classes, config)
            for entry in detections:
                entry["center_source"] = "box_center"
                entry["mask_area_px"] = 0
                entry["depth_sample_pixels"] = []
            return detections, "fake-world"

        def sample_depth(left_jpeg, right_jpeg, spec, config):
            return {
                "model": "fake-stereo", "depth_m": 1.0, "disparity_px": 20.0,
                "sample_source": spec["sample_source"], "sample_count": 49,
                "valid_count": 49,
            }

        expected = {
            "open_vocab": ("open_vocab", "stereo_triangulation"),
            "second": ("second", "dense_stereo_depth"),
        }
        for pipeline, (method, distance_source) in expected.items():
            with self.subTest(pipeline=pipeline):
                csv_path = os.path.join(self.temp.name, pipeline + ".csv")
                jsonl_path = os.path.join(self.temp.name, pipeline + ".jsonl")
                summary_path = os.path.join(self.temp.name,
                                            pipeline + "_summary.json")
                with mock.patch.object(yolo_client, "detect", detect_both), \
                     mock.patch.object(yolo_client, "detect_one", detect_one), \
                     mock.patch.object(depth_client, "sample", sample_depth), \
                     contextlib.redirect_stdout(io.StringIO()):
                    code = vision_eval.main([
                        "--captures-dir", self.root, "--pipeline", pipeline,
                        "--output-csv", csv_path, "--results-json", jsonl_path,
                        "--summary-json", summary_path])
                self.assertEqual(code, 0)
                with open(summary_path) as handle:
                    summary = json.load(handle)
                self.assertEqual(summary["pipeline"], pipeline)
                self.assertEqual(summary["rows"], 1)
                with open(jsonl_path) as handle:
                    results = [json.loads(line) for line in handle]
                self.assertEqual(len(results), 1)
                entry = results[0]["results"][0]
                self.assertEqual(entry["method"], method)
                self.assertEqual(entry["distance_source"], distance_source)
                rows, fields = llm_trials.read_csv(csv_path)
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["method"], method)
                self.assertIn("method", fields)

    def test_main_honours_environment_and_yaml_pipeline_selection(self):
        """With no `--pipeline`, $VISION_PIPELINE then YAML choose one pipeline."""
        self._write_synthetic_pair()

        def detect_both(left_jpeg, right_jpeg, classes, config):
            left, right, _ = FakeClient()(left_jpeg, right_jpeg, classes, config)
            return left, right, "fake-world"

        def detect_one(jpeg, classes, config):
            detections, _, _ = FakeClient()(jpeg, jpeg, classes, config)
            for entry in detections:
                entry["center_source"] = "box_center"
                entry["mask_area_px"] = 0
                entry["depth_sample_pixels"] = []
            return detections, "fake-world"

        def sample_depth(left_jpeg, right_jpeg, spec, config):
            return {
                "model": "fake-stereo", "depth_m": 1.0, "disparity_px": 20.0,
                "sample_source": spec["sample_source"], "sample_count": 49,
                "valid_count": 49,
            }

        config_path = os.path.join(self.temp.name, "second.yaml")
        with open(config_path, "w") as handle:
            handle.write("pipeline: second\n")
        cases = (
            ("env", None, {"VISION_PIPELINE": "second"}, "second"),
            ("yaml", config_path, {
                "VISION_PIPELINE": ""}, "second"),
        )
        for label, config_arg, environ, expected in cases:
            with self.subTest(source=label):
                summary_path = os.path.join(self.temp.name,
                                            label + "_summary.json")
                argv = ["--captures-dir", self.root,
                        "--output-csv", os.path.join(self.temp.name,
                                                     label + ".csv"),
                        "--results-json", os.path.join(self.temp.name,
                                                       label + ".jsonl"),
                        "--summary-json", summary_path]
                if config_arg:
                    argv += ["--config", config_arg]
                with mock.patch.dict(os.environ, environ, clear=True), \
                     mock.patch.object(yolo_client, "detect", detect_both), \
                     mock.patch.object(yolo_client, "detect_one", detect_one), \
                     mock.patch.object(depth_client, "sample", sample_depth), \
                     contextlib.redirect_stdout(io.StringIO()):
                    code = vision_eval.main(argv)
                self.assertEqual(code, 0)
                with open(summary_path) as handle:
                    self.assertEqual(json.load(handle)["pipeline"], expected)


if __name__ == "__main__":
    unittest.main()
