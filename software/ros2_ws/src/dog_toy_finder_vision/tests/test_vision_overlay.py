"""Tests for annotated detection images written by offline evaluation."""

import io
import os
import sys
import tempfile
import unittest

from PIL import Image

SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, SRC_ROOT)

from dog_toy_finder_vision import vision_config  # noqa: E402
from dog_toy_finder_vision import vision_overlay  # noqa: E402


class RecordingArtifacts:
    """Minimal tracker double used to verify overlay artifact tracking."""

    def __init__(self):
        self.files = []
        self.directories = []

    def track(self, *paths):
        self.files.extend(paths)

    def track_dir(self, path):
        self.directories.append(path)


def _jpeg(color=(210, 210, 210), size=(320, 240)):
    handle = io.BytesIO()
    Image.new("RGB", size, color).save(handle, format="JPEG")
    return handle.getvalue()


def _rgb_colors(image):
    data = image.tobytes()
    return [(data[i], data[i + 1], data[i + 2])
            for i in range(0, len(data), 3)]


class OverlayTests(unittest.TestCase):
    def setUp(self):
        self.config = vision_config.load_config()
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)

    def test_overlay_path_is_unique_and_sanitised(self):
        path = vision_overlay.overlay_image_path(
            "/tmp/eval", "stereo-net__yolo_world__rt_igevpp",
            os.path.join("run 1", "scene A", "shot_02"), "additional_01",
            "left")
        self.assertEqual(
            path, os.path.join(
                "/tmp/eval", "stereo-net__yolo_world__rt_igevpp", "run_1",
                "scene_A", "shot_02", "additional_01_left.jpg"))

    def test_annotate_draws_box_centre_and_operator_bearing(self):
        entry = {
            "target_visible": True,
            "confidence": 0.91,
            "distance_m": 1.2,
            "lateral_bearing_deg": 0.0,
            "bbox_px": [80, 100, 120, 140],
            "mask_centroid_px": [100, 120],
            "method": "triangulation",
        }
        ground_truth = {"target_visible": True, "lateral_bearing_deg": 0.0}
        image = vision_overlay.annotate_image(
            _jpeg(), "red ball", entry, "left", self.config, ground_truth)
        colors = _rgb_colors(image)
        # Green box, cyan centre cross, and yellow truth guide must all appear.
        self.assertIn(vision_overlay.BOX_COLOR, colors)
        self.assertIn(vision_overlay.CENTER_COLOR, colors)
        self.assertTrue(any(red > 200 and green > 180 and blue < 80
                            for red, green, blue in colors))

    def test_annotate_marks_a_missed_target(self):
        image = vision_overlay.annotate_image(
            _jpeg(), "red ball", {"target_visible": False}, "left",
            self.config, {"target_visible": True})
        found = any(red > 180 and green < 130 and blue < 130
                    for red, green, blue in _rgb_colors(image))
        self.assertTrue(found)

    def test_annotate_marks_correct_absence_and_false_positive(self):
        ground_truth = {"target_visible": False}
        correct = vision_overlay.annotate_image(
            _jpeg(), "none", {"target_visible": False}, "left", self.config,
            ground_truth)
        self.assertFalse(any(red > 180 and green < 130 and blue < 130
                             for red, green, blue in _rgb_colors(correct)))
        self.assertTrue(any(green > 180 and red < 180
                            for red, green, blue in _rgb_colors(correct)))

        false_positive = vision_overlay.annotate_image(
            _jpeg(), "none",
            {"target_visible": True, "confidence": 0.9,
             "bbox_px": [90, 100, 110, 120]},
            "left", self.config, ground_truth)
        self.assertTrue(any(red > 180 and green < 130 and blue < 130
                            for red, green, blue in _rgb_colors(false_positive)))

    def test_write_object_overlays_uses_detected_eyes_and_tracks_files(self):
        shot = os.path.join(self.temp.name, "run1", "cap1")
        os.makedirs(shot)
        for eye in ("left", "right"):
            with open(os.path.join(shot, "%s.jpg" % eye), "wb") as handle:
                handle.write(_jpeg())
        overlays_dir = os.path.join(self.temp.name, "overlays")
        artifacts = RecordingArtifacts()
        object_result = {
            "id": "cap1",
            "object_id": "primary",
            "target": "red ball",
            "pipeline": "triangulation",
            "ground_truth": {"target_visible": True,
                             "lateral_bearing_deg": 0.0},
            "results": [{
                "target_visible": True, "confidence": 0.8,
                "distance_m": 1.0, "lateral_bearing_deg": 0.0,
                "method": "triangulation",
                "bbox_px": [90, 100, 110, 120],
                "bbox_right_px": [70, 100, 90, 120],
            }],
        }
        written = vision_overlay.write_object_overlays(
            shot, object_result, self.config, overlays_dir,
            "triangulation__yolo_world", self.temp.name, artifacts=artifacts)
        self.assertEqual(set(written), {"left", "right"})
        for path in written.values():
            self.assertTrue(os.path.isfile(path), path)
            with Image.open(path) as image:
                self.assertEqual(image.size, (320, 240))
        self.assertEqual(artifacts.files, list(written.values()))
        self.assertIn(os.path.dirname(written["left"]), artifacts.directories)

        stereo_object = dict(
            object_result, results=[dict(object_result["results"][0],
                                         method="stereo-net")])
        stereo_written = vision_overlay.write_object_overlays(
            shot, stereo_object, self.config, overlays_dir,
            "stereo-net__yolo_world__rt_igevpp", self.temp.name)
        self.assertEqual(list(stereo_written), ["left"])


if __name__ == "__main__":
    unittest.main()
