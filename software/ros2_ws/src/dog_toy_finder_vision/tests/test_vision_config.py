import os
import tempfile
import unittest
from unittest import mock

import yaml

from dog_toy_finder_vision import vision_config


class TestVisionConfig(unittest.TestCase):
    def _load(self, data):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "vision.yaml")
            with open(path, "w") as handle:
                yaml.safe_dump(data, handle)
            return vision_config.load_config(path)

    def test_defaults_have_no_target_section(self):
        self.assertNotIn("target", vision_config.DEFAULTS)

    def test_config_loads_without_target_measurements(self):
        config = self._load({
            "camera": {
                "width": 320,
                "height": 240,
                "baseline_m": 0.0431,
                "left": {"fx": 299.72, "fy": 300.92,
                         "cx": 175.61, "cy": 127.57},
                "right": {"fx": 298.50, "fy": 299.70,
                          "cx": 150.11, "cy": 126.40},
                "depth_min_m": 0.25,
                "depth_max_m": 3.0,
            },
        })
        self.assertNotIn("target", config)
        self.assertEqual(
            vision_config.eye_intrinsics(config, "left")["fx"], 299.72)
        self.assertEqual(
            vision_config.eye_intrinsics(config, "right")["cx"], 150.11)

    def test_legacy_shared_intrinsics_are_copied_to_both_eyes(self):
        config = self._load({
            "camera": {"fx": 250.0, "fy": 251.0,
                       "cx": 160.0, "cy": 120.0},
        })
        for side in ("left", "right"):
            self.assertEqual(
                vision_config.eye_intrinsics(config, side),
                {"fx": 250.0, "fy": 251.0, "cx": 160.0, "cy": 120.0})
        for field in ("fx", "fy", "cx", "cy"):
            self.assertNotIn(field, config["camera"])

    def test_bearing_from_center_offset_is_signed(self):
        bearing = vision_config.bearing_deg_from_center_offset(0.24, 1.0)
        self.assertAlmostEqual(bearing, 13.5, places=1)
        self.assertAlmostEqual(
            vision_config.bearing_deg_from_center_offset(-0.24, 1.0),
            -13.5, places=1)
        self.assertEqual(
            vision_config.bearing_deg_from_center_offset(0.0, 1.0), 0.0)

    def test_bearing_from_center_offset_is_capped_at_zero_depth(self):
        self.assertEqual(
            vision_config.bearing_deg_from_center_offset(0.1, 0.0), 90.0)
        self.assertEqual(
            vision_config.bearing_deg_from_center_offset(-0.1, 0.0), -90.0)

    def test_center_offset_m_is_signed_and_scales_with_depth(self):
        config = self._load({
            "camera": {"left": {"fx": 250.0, "cx": 160.0},
                       "right": {"fx": 260.0, "cx": 140.0}},
        })
        self.assertAlmostEqual(
            vision_config.center_offset_m(90.0, 1.0, config, 320), 0.28)
        self.assertAlmostEqual(
            vision_config.center_offset_m(230.0, 2.0, config, 320), -0.56)
        self.assertAlmostEqual(
            vision_config.center_offset_m(90.0, 1.0, config, 320, "right"),
            0.192, places=3)

    def test_center_offset_m_matches_bearing_geometry(self):
        config = self._load({
            "camera": {"left": {"fx": 299.72, "cx": 175.61}},
        })
        depth = 1.15
        u_px = 120.0
        bearing = vision_config.lateral_bearing_deg(u_px, config, 320)
        offset = vision_config.center_offset_m(u_px, depth, config, 320)
        self.assertAlmostEqual(
            vision_config.bearing_deg_from_center_offset(offset, depth),
            bearing, delta=0.05)

    def test_stereo_disparity_offset_uses_both_principal_points(self):
        config = self._load({
            "camera": {"left": {"cx": 165.98}, "right": {"cx": 136.94}},
        })
        self.assertAlmostEqual(
            vision_config.stereo_disparity_offset_px(config), 29.04, places=2)

    def test_stereo_disparity_offset_is_zero_for_shared_principal_point(self):
        config = self._load({
            "camera": {"left": {"cx": 160.0}, "right": {"cx": 160.0}},
        })
        self.assertEqual(vision_config.stereo_disparity_offset_px(config), 0.0)

    def test_detector_backend_accepts_backend_ids_and_weights_aliases(self):
        self.assertEqual(vision_config.detector_backend({}), "yolo_world")
        self.assertEqual(
            vision_config.detector_backend(
                {"open_vocab": {"detector": "grounding_dino"}}),
            "grounding_dino")
        self.assertEqual(
            vision_config.detector_backend(
                {"open_vocab": {"detector": "yoloe-26s-seg"}}),
            "yoloe_26")
        self.assertEqual(
            vision_config.detector_backend(
                {"open_vocab": {"name": "yolov8s-world"}}),
            "yolo_world")
        self.assertEqual(
            vision_config.detector_display_name(
                {"open_vocab": {"name": "yolov8s-world"}}),
            "yolov8s-world")

    def test_unknown_detector_backend_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Unknown detector"):
            vision_config.detector_backend(
                {"open_vocab": {"detector": "not-a-model"}})

    def test_depth_backend_accepts_model_key_and_legacy_name(self):
        self.assertEqual(vision_config.depth_backend({}), "lite_any_stereo")
        self.assertEqual(
            vision_config.depth_backend(
                {"stereo-net": {"model": "rt_igevpp"}}),
            "rt_igevpp")
        self.assertEqual(
            vision_config.depth_backend(
                {"stereo-net": {"name": "fast_foundation_stereo"}}),
            "fast_foundation_stereo")
        self.assertEqual(
            vision_config.depth_backend(
                {"stereo-net": {"name": "sceneflow"}}),
            "rt_igevpp")
        with self.assertRaisesRegex(ValueError, "Unknown dense stereo model"):
            vision_config.depth_backend(
                {"stereo-net": {"model": "not-a-model"}})

    def test_with_model_selection_pins_backends_without_mutating_source(self):
        source = {"open_vocab": {"detector": "yolo_world", "name": "keep"},
                  "stereo-net": {"model": "lite_any_stereo", "name": "keep"}}
        selected = vision_config.with_model_selection(
            source, detector="grounding-dino-tiny", depth="rt_igevpp")
        self.assertEqual(selected["open_vocab"]["detector"],
                         "grounding_dino")
        self.assertEqual(selected["open_vocab"]["name"],
                         "grounding-dino-tiny")
        self.assertEqual(selected["stereo-net"]["model"], "rt_igevpp")
        self.assertEqual(source["open_vocab"]["name"], "keep")
        self.assertEqual(source["stereo-net"]["model"], "lite_any_stereo")

    def test_pilot_capture_dir_is_a_sibling_of_captures(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            normal = vision_config.default_capture_dir()
            pilot = vision_config.default_capture_dir(pilot=True)
        self.assertEqual(os.path.basename(normal), "captures")
        self.assertEqual(os.path.basename(pilot), "pilot_captures")
        self.assertEqual(os.path.dirname(normal), os.path.dirname(pilot))

    def test_pilot_capture_dir_ignores_capture_dir_override(self):
        with mock.patch.dict(os.environ,
                             {"VISION_CAPTURE_DIR": "/tmp/custom_captures"},
                             clear=False):
            self.assertEqual(vision_config.default_capture_dir(),
                             "/tmp/custom_captures")
            self.assertNotEqual(vision_config.default_capture_dir(pilot=True),
                                "/tmp/custom_captures")


if __name__ == "__main__":
    unittest.main()
