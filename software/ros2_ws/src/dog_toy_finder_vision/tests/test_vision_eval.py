"""Tests for offline replay of saved rectified stereo captures."""

import contextlib
import io
import json
import os
import random
import shutil
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
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = vision_config.load_config()
        # Offline replay mechanics are tested with coincident rectified
        # principal points. The principal-point disparity correction itself is
        # covered in test_vision_config, test_open_vocab_stereo, and
        # test_dense_stereo; keeping it out of the synthetic JPEGs avoids
        # sub-pixel compression bias in the expected distance.
        self.config["camera"]["right"]["cx"] = \
            self.config["camera"]["left"]["cx"]
        self.config_path = os.path.join(self.temp.name, "test_config.json")
        with open(self.config_path, "w") as handle:
            json.dump(self.config, handle)
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
            # ``disparity`` is the rectified value; the saved right image must
            # include the principal-point offset that the pipeline subtracts.
            raw_disparity = disparity + vision_config.stereo_disparity_offset_px(
                config)
            left[(xx - center_x) ** 2 + (yy - 120) ** 2 <= radius ** 2] = \
                [255, 0, 0]
            right[((xx - (center_x - raw_disparity)) ** 2
                   + (yy - 120) ** 2) <= radius ** 2] = [255, 0, 0]
        for eye, image in (("left", left), ("right", right)):
            Image.fromarray(image).save(
                os.path.join(self.capture_dir, "%s.jpg" % eye))
        fx = vision_config.eye_intrinsics(config, "left")["fx"]
        baseline_m = float(config["camera"]["baseline_m"])
        expected_distance = fx * baseline_m / disparity
        truth = {
            "target_visible": visible,
            "lateral_bearing_deg": vision_config.lateral_bearing_deg(
                center_x, config) if visible else 0.0,
            "center_offset_m": vision_config.center_offset_m(
                center_x, expected_distance, config, width) if visible else 0.0,
            "distance_m": expected_distance if visible else 0.0,
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

    def _add_capture(self, capture_id, run_id="run1"):
        """Copy the synthetic pair into another complete capture directory."""
        destination = os.path.join(self.root, run_id, capture_id)
        os.makedirs(destination, exist_ok=True)
        for name in ("left.jpg", "right.jpg", "meta.json"):
            shutil.copyfile(os.path.join(self.capture_dir, name),
                            os.path.join(destination, name))
        meta_path = os.path.join(destination, "meta.json")
        with open(meta_path) as handle:
            meta = json.load(handle)
        meta["capture_id"] = capture_id
        meta["run_id"] = run_id
        with open(meta_path, "w") as handle:
            json.dump(meta, handle)
        return destination

    def _write_multi_shot_scene(self, capture_id="scene1", shot_count=3,
                                run_id="run1"):
        """Copy the synthetic pair into a numbered multi-shot scene."""
        scene_dir = os.path.join(self.root, run_id, capture_id)
        os.makedirs(scene_dir, exist_ok=True)
        with open(os.path.join(self.capture_dir, "meta.json")) as handle:
            base_meta = json.load(handle)
        scene_meta = dict(base_meta, capture_id=capture_id, scene_id=capture_id,
                          run_id=run_id, shot_count=shot_count)
        with open(os.path.join(scene_dir, "scene.json"), "w") as handle:
            json.dump(scene_meta, handle)
        shot_dirs = []
        for index in range(1, shot_count + 1):
            shot_dir = os.path.join(scene_dir, "shot_%02d" % index)
            os.makedirs(shot_dir, exist_ok=True)
            for name in ("left.jpg", "right.jpg"):
                shutil.copyfile(os.path.join(self.capture_dir, name),
                                os.path.join(shot_dir, name))
            meta = dict(scene_meta, shot_index=index)
            with open(os.path.join(shot_dir, "meta.json"), "w") as handle:
                json.dump(meta, handle)
            shot_dirs.append(shot_dir)
        return scene_dir, shot_dirs

    def test_iter_captures_finds_all_shots_in_a_scene(self):
        self._write_synthetic_pair()
        scene_dir, shot_dirs = self._write_multi_shot_scene()
        found = list(vision_eval.iter_captures(self.root))
        self.assertEqual(found, sorted([self.capture_dir] + shot_dirs))
        self.assertEqual(list(vision_eval.iter_captures(
            self.root, capture=scene_dir)), shot_dirs)
        self.assertEqual(vision_eval._capture_root(scene_dir), self.root)
        self.assertEqual(vision_eval._capture_root(shot_dirs[0]), self.root)

    def test_iter_captures_filters_nested_shots_by_recorded_run_id(self):
        self._write_synthetic_pair()
        _, run1_shots = self._write_multi_shot_scene("scene_run1", 2, "run1")
        _, run2_shots = self._write_multi_shot_scene("scene_run2", 2, "run2")
        found = list(vision_eval.iter_captures(self.root, run_id="run1"))
        self.assertIn(self.capture_dir, found)
        for path in run1_shots:
            self.assertIn(path, found)
        for path in run2_shots:
            self.assertNotIn(path, found)

    def test_evaluate_capture_gives_each_shot_a_unique_pairing_id(self):
        self._write_synthetic_pair()
        _, shot_dirs = self._write_multi_shot_scene()
        result, meta = vision_eval.evaluate_capture(
            shot_dirs[1], self.config, "open_vocab", client=FakeClient())
        self.assertEqual(result["id"], "scene1_shot_02")
        self.assertEqual(result["scene_id"], "scene1")
        self.assertEqual(result["shot_index"], 2)
        self.assertEqual(result["shot_count"], 3)
        self.assertEqual(result["image"],
                         os.path.join("run1", "scene1", "shot_02", "left.jpg"))
        self.assertEqual(result["capture_dir"], shot_dirs[1])
        self.assertEqual(meta["shot_index"], 2)

    def test_main_writes_one_row_per_shot_and_records_scene_counts(self):
        self._write_synthetic_pair()
        _, shot_dirs = self._write_multi_shot_scene()
        shutil.rmtree(self.capture_dir)
        original = vision_eval.evaluate_capture

        def evaluate_with_fake_client(path, config, pipeline=None, client=None,
                                      depth_client=None):
            return original(path, config, pipeline, FakeClient())

        csv_path = os.path.join(self.temp.name, "multi_shot.csv")
        results_path = os.path.join(self.temp.name, "multi_shot.jsonl")
        summary_path = os.path.join(self.temp.name, "multi_shot_summary.json")
        with mock.patch.object(vision_eval, "evaluate_capture",
                               evaluate_with_fake_client), \
             contextlib.redirect_stdout(io.StringIO()):
            code = vision_eval.main([
                "--config", self.config_path, "--captures-dir", self.root, "--pipeline", "open_vocab",
                "--seed", "42", "--output-csv", csv_path,
                "--results-json", results_path, "--summary-json", summary_path])
        self.assertEqual(code, 0)

        rows, _ = llm_trials.read_csv(csv_path)
        self.assertEqual(len(rows), 3)
        self.assertEqual({row["ask_id"] for row in rows}, {
            "scene1_shot_01", "scene1_shot_02", "scene1_shot_03"})
        self.assertEqual(len({row["trial_id"] for row in rows}), 3)
        for row in rows:
            self.assertTrue(row["visible_correct"])
            self.assertAlmostEqual(float(row["distance_error_m"]), 0.0, delta=0.01)

        with open(results_path) as handle:
            results = [json.loads(line) for line in handle]
        self.assertEqual(len(results), 3)
        self.assertEqual(len({result["id"] for result in results}), 3)

        with open(summary_path) as handle:
            summary = json.load(handle)
        self.assertEqual(summary["shots"], 3)
        self.assertEqual(summary["captures"], 3)
        self.assertEqual(summary["scenes"], 1)
        self.assertEqual(summary["scene_order"], ["scene1"])
        self.assertEqual(summary["shots_per_scene"], {"scene1": 3})
        self.assertEqual(summary["shuffle_seed"], 42)
        self.assertEqual(sorted(summary["shot_order"]), sorted(
            os.path.relpath(path, self.root) for path in shot_dirs))

    def test_main_pilot_mode_keeps_default_outputs_in_pilot_root(self):
        self._write_synthetic_pair()
        self._write_multi_shot_scene()
        shutil.rmtree(self.capture_dir)
        original = vision_eval.evaluate_capture

        def evaluate_with_fake_client(path, config, pipeline=None, client=None,
                                      depth_client=None):
            return original(path, config, pipeline, FakeClient())

        with mock.patch.object(vision_config, "default_capture_dir",
                               return_value=self.root) as default_dir, \
             mock.patch.object(vision_eval, "evaluate_capture",
                               evaluate_with_fake_client), \
             contextlib.redirect_stdout(io.StringIO()):
            code = vision_eval.main([
                "--config", self.config_path, "--pilot", "--pipeline",
                "open_vocab", "--seed", "42"])
        self.assertEqual(code, 0)
        default_dir.assert_called_once_with(pilot=True)
        for name in ("vision_eval_trials.csv", "vision_eval_results.jsonl",
                     "vision_eval_summary.json"):
            self.assertTrue(os.path.isfile(os.path.join(self.root, name)), name)
        with open(os.path.join(self.root, "vision_eval_summary.json")) as handle:
            summary = json.load(handle)
        self.assertEqual(summary["captures_dir"], self.root)

    def test_main_records_randomized_scene_and_shot_order(self):
        self._write_synthetic_pair()
        self._write_multi_shot_scene("sceneA", 3)
        self._write_multi_shot_scene("sceneB", 3)
        shutil.rmtree(self.capture_dir)
        original = vision_eval.evaluate_capture

        def evaluate_with_fake_client(path, config, pipeline=None, client=None,
                                      depth_client=None):
            return original(path, config, pipeline, FakeClient())

        summary_path = os.path.join(self.temp.name, "shuffled_scenes.json")
        with mock.patch.object(vision_eval, "evaluate_capture",
                               evaluate_with_fake_client), \
             contextlib.redirect_stdout(io.StringIO()):
            code = vision_eval.main([
                "--config", self.config_path, "--captures-dir", self.root, "--pipeline", "open_vocab",
                "--seed", "1",
                "--output-csv", os.path.join(self.temp.name,
                                             "shuffled_scenes.csv"),
                "--results-json", os.path.join(self.temp.name,
                                               "shuffled_scenes.jsonl"),
                "--summary-json", summary_path])
        self.assertEqual(code, 0)

        with open(summary_path) as handle:
            summary = json.load(handle)
        expected_order = list(vision_eval.iter_captures(
            self.root, rng=random.Random(1)))
        self.assertEqual(summary["shot_order"], [
            os.path.relpath(path, self.root) for path in expected_order])
        self.assertEqual(summary["scene_order"], ["sceneB", "sceneA"])
        self.assertEqual(summary["shots_per_scene"],
                         {"sceneA": 3, "sceneB": 3})

    def test_iter_captures_finds_complete_directories(self):
        self._write_synthetic_pair()
        incomplete = os.path.join(self.root, "run1", "bad")
        os.makedirs(incomplete)
        found = list(vision_eval.iter_captures(self.root))
        self.assertEqual(found, [self.capture_dir])

    def test_iter_captures_shuffles_with_seeded_rng(self):
        self._write_synthetic_pair()
        expected = [self.capture_dir] + [
            self._add_capture("cap%d" % index) for index in range(2, 6)]
        for seed in (7, 42):
            with self.subTest(seed=seed):
                found = list(vision_eval.iter_captures(
                    self.root, rng=random.Random(seed)))
                self.assertCountEqual(found, expected)
                self.assertNotEqual(found, expected,
                                    "seeded RNG must change processing order")

    def test_iter_captures_shuffles_scenes_then_shots_within_each_scene(self):
        self._write_synthetic_pair()
        _, first = self._write_multi_shot_scene("sceneA", 3)
        _, second = self._write_multi_shot_scene("sceneB", 3)
        shutil.rmtree(self.capture_dir)

        found = list(vision_eval.iter_captures(
            self.root, rng=random.Random(1)))
        repeated = list(vision_eval.iter_captures(
            self.root, rng=random.Random(1)))
        self.assertEqual(found, repeated)
        self.assertCountEqual(found, first + second)

        seen_scenes = []
        for path in found:
            scene = os.path.basename(os.path.dirname(path))
            if scene not in seen_scenes:
                seen_scenes.append(scene)
        self.assertEqual(seen_scenes, ["sceneB", "sceneA"])

        grouped = []
        for scene in seen_scenes:
            ordered = [path for path in found
                       if os.path.basename(os.path.dirname(path)) == scene]
            self.assertEqual(len(ordered), 3)
            self.assertNotEqual(ordered, sorted(ordered),
                                "shots inside a scene must be shuffled")
            grouped.extend(ordered)
        self.assertEqual(found, grouped, "each scene must stay contiguous")

    def test_capture_order_repeatable_by_seed_and_changes_with_seed(self):
        self._write_synthetic_pair()
        for index in range(2, 6):
            self._add_capture("cap%d" % index)
        first = list(vision_eval.iter_captures(
            self.root, rng=random.Random(1234)))
        repeated = list(vision_eval.iter_captures(
            self.root, rng=random.Random(1234)))
        changed = list(vision_eval.iter_captures(
            self.root, rng=random.Random(1235)))
        self.assertEqual(first, repeated)
        self.assertNotEqual(first, changed)

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
        self.assertAlmostEqual(
            entry["lateral_bearing_deg"],
            meta["ground_truth"]["lateral_bearing_deg"], delta=0.5)
        self.assertAlmostEqual(
            entry["distance_m"],
            meta["ground_truth"]["distance_m"], places=2)
        self.assertAlmostEqual(
            entry["center_offset_m"],
            meta["ground_truth"]["center_offset_m"], places=2)

        row = llm_trials.vision_row(
            result, entry, "fake-world", "run1",
            ground_truth=meta["ground_truth"], bearing_tolerance=5.0)
        self.assertEqual(row["method"], "open_vocab")
        self.assertNotIn("size", row)
        self.assertNotIn("gt_size", row)
        self.assertEqual(row["gt_lateral_bearing_deg"],
                         meta["ground_truth"]["lateral_bearing_deg"])
        self.assertAlmostEqual(row["center_offset_m"],
                               entry["center_offset_m"], places=2)
        self.assertAlmostEqual(row["gt_center_offset_m"],
                               meta["ground_truth"]["center_offset_m"], places=2)
        self.assertTrue(row["bearing_within_tolerance"])
        self.assertAlmostEqual(row["bearing_error_deg"], 0.0, delta=0.5)

    def test_stereo_net_pipeline_replays_with_dense_depth(self):
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

        expected_distance = (
            vision_config.eye_intrinsics(self.config, "left")["fx"]
            * float(self.config["camera"]["baseline_m"]) / 20.0)

        def sample_depth(left, right, spec, config):
            return {
                "model": "fake-stereo", "depth_m": expected_distance,
                "disparity_px": 20.0, "sample_source": spec["sample_source"],
                "sample_count": 49, "valid_count": 49,
            }

        result, meta = vision_eval.evaluate_capture(
            self.capture_dir, self.config, "stereo-net", client=detect_one,
            depth_client=sample_depth)

        self.assertEqual(result["pipeline"], "stereo-net")
        self.assertEqual(result["models"], ["fake-stereo"])
        entry = result["results"][0]
        self.assertEqual(entry["method"], "stereo-net")
        self.assertEqual(entry["distance_source"], "dense_stereo_depth")
        self.assertTrue(entry["target_visible"])
        self.assertAlmostEqual(entry["distance_m"], expected_distance, places=2)
        self.assertIn("center_offset_m", entry)

        row = llm_trials.vision_row(
            result, entry, entry["model"], "run1",
            ground_truth=meta["ground_truth"], bearing_tolerance=5.0)
        self.assertEqual(row["method"], "stereo-net")
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
                "--config", self.config_path, "--captures-dir", self.root, "--pipeline", "open_vocab",
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

    def test_main_records_seed_and_shuffled_capture_order(self):
        self._write_synthetic_pair()
        for index in range(2, 6):
            self._add_capture("cap%d" % index)
        seen = []
        original = vision_eval.evaluate_capture

        def evaluate_with_fake_client(path, config, pipeline=None, client=None,
                                      depth_client=None):
            seen.append(path)
            return original(path, config, pipeline, FakeClient())

        output = io.StringIO()
        csv_path = os.path.join(self.temp.name, "shuffled.csv")
        results_path = os.path.join(self.temp.name, "shuffled.jsonl")
        summary_path = os.path.join(self.temp.name, "shuffled_summary.json")
        with mock.patch.object(vision_eval, "evaluate_capture",
                               evaluate_with_fake_client), \
             contextlib.redirect_stdout(output):
            code = vision_eval.main([
                "--config", self.config_path, "--captures-dir", self.root, "--pipeline", "open_vocab",
                "--seed", "42", "--limit", "2",
                "--output-csv", csv_path, "--results-json", results_path,
                "--summary-json", summary_path])
        self.assertEqual(code, 0)
        self.assertIn("seed 42", output.getvalue())
        with open(summary_path) as handle:
            summary = json.load(handle)
        self.assertEqual(summary["shuffle_seed"], 42)
        self.assertEqual(summary["captures"], 2)
        self.assertEqual(len(summary["capture_order"]), 2)
        self.assertEqual(
            [os.path.join(self.root, relative)
             for relative in summary["capture_order"]], seen)
        expected = list(vision_eval.iter_captures(
            self.root, rng=random.Random(42)))[:2]
        self.assertEqual(seen, expected)
        self.assertNotEqual(summary["capture_order"], sorted(
            os.path.relpath(path, self.root) for path in expected))

    def test_main_generates_and_records_seed_when_omitted(self):
        self._write_synthetic_pair()

        def detect_both(left_jpeg, right_jpeg, classes, config):
            return FakeClient()(left_jpeg, right_jpeg, classes, config)

        summary_path = os.path.join(self.temp.name, "generated_summary.json")
        with mock.patch.object(vision_eval.secrets, "randbelow",
                               return_value=777), \
             mock.patch.object(yolo_client, "detect", detect_both), \
             contextlib.redirect_stdout(io.StringIO()):
            code = vision_eval.main([
                "--config", self.config_path, "--captures-dir", self.root, "--pipeline", "open_vocab",
                "--output-csv", os.path.join(self.temp.name, "generated.csv"),
                "--results-json", os.path.join(self.temp.name,
                                               "generated.jsonl"),
                "--summary-json", summary_path])
        self.assertEqual(code, 0)
        with open(summary_path) as handle:
            summary = json.load(handle)
        self.assertEqual(summary["shuffle_seed"], 777)
        self.assertEqual(summary["capture_order"], [os.path.join(
            "run1", "cap1")])

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
            "stereo-net": ("stereo-net", "dense_stereo_depth"),
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
                        "--config", self.config_path, "--captures-dir", self.root, "--pipeline", pipeline,
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

        config_path = os.path.join(self.temp.name, "stereo-net.yaml")
        with open(config_path, "w") as handle:
            handle.write("pipeline: stereo-net\n")
        cases = (
            ("env", None, {"VISION_PIPELINE": "stereo-net"}, "stereo-net"),
            ("yaml", config_path, {
                "VISION_PIPELINE": ""}, "stereo-net"),
        )
        for label, config_arg, environ, expected in cases:
            with self.subTest(source=label):
                summary_path = os.path.join(self.temp.name,
                                            label + "_summary.json")
                argv = ["--config", self.config_path,
                        "--captures-dir", self.root,
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
