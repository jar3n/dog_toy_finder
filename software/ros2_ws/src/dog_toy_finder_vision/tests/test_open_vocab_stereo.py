"""Tests for open-vocabulary stereo matching and disparity triangulation.

No ultralytics weights or torch are needed: a fake detector client returns the
plain-Python detection dictionaries the host service would return for the red
disc in each synthetic image.
"""

import io
import os
import sys
import unittest

import numpy as np
from PIL import Image

SRC_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, SRC_ROOT)

from dog_toy_finder_vision import open_vocab_stereo  # noqa: E402
from dog_toy_finder_vision import vision_config  # noqa: E402


def _detection(u, v=120, label="red ball", confidence=0.95, width=12,
               center_source="box_center"):
    return {"label": label, "confidence": confidence, "u": u, "v": v,
            "width_px": width, "height_px": width,
            "bbox_px": [u - width / 2, v - width / 2,
                        u + width / 2, v + width / 2],
            "center_source": center_source,
            "mask_centroid_px": ([round(u, 2), round(v, 2)]
                                 if center_source == "mask_centroid" else None),
            "mask_area_px": 100 if center_source == "mask_centroid" else 0}


def _stereo_jpegs(left_x=90, disparity=20, right_visible=True):
    config = vision_config.load_config()
    width = int(config["camera"]["width"])
    height = int(config["camera"]["height"])
    images = []
    for center_x in (left_x, left_x - disparity if right_visible else None):
        image = np.full((height, width, 3), 210, dtype=np.uint8)
        if center_x is not None:
            yy, xx = np.ogrid[:height, :width]
            image[(xx - center_x) ** 2 + (yy - 120) ** 2 <= 36] = [255, 0, 0]
        buffer = io.BytesIO()
        Image.fromarray(image).save(buffer, format="JPEG")
        images.append(buffer.getvalue())
    return images[0], images[1]


class FakeClient:
    """Stand-in for yolo_client.detect: find the red disc in each eye."""

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
                continue
            found.append([_detection(float(xs.mean()), float(ys.mean()),
                                     label=classes[0] if classes else "red ball",
                                      center_source="mask_centroid")])
        return found[0], found[1], "fake-world"


class TargetClassTests(unittest.TestCase):
    def test_text_target_is_split_into_class_prompts(self):
        self.assertEqual(open_vocab_stereo.target_classes("red ball"),
                         ["red ball"])
        self.assertEqual(
            open_vocab_stereo.target_classes("red ball, blue bone"),
            ["red ball", "blue bone"])
        self.assertEqual(
            open_vocab_stereo.target_classes("red ball or blue bone"),
            ["red ball", "blue bone"])
        self.assertEqual(open_vocab_stereo.target_classes("  "), [])

    def test_model_name_prefers_service_then_config_then_default(self):
        config = vision_config.load_config()
        self.assertEqual(open_vocab_stereo.model_name(config, "live-world"),
                         "live-world")
        self.assertEqual(open_vocab_stereo.model_name(config),
                         "yolov8s-world")
        config["open_vocab"]["name"] = ""
        self.assertEqual(open_vocab_stereo.model_name(config),
                         "yolov8s-world")


class TriangulationTests(unittest.TestCase):
    def setUp(self):
        self.config = vision_config.load_config()

    def test_detects_and_triangulates_matched_pair(self):
        left, right = _stereo_jpegs(left_x=90, disparity=20)
        client = FakeClient()

        entry = open_vocab_stereo.detect(
            left, right, "red ball", self.config, client=client)

        self.assertEqual(len(client.calls), 1)
        self.assertEqual(client.calls[0][2], ["red ball"])
        self.assertIs(client.calls[0][0], left)
        self.assertIs(client.calls[0][1], right)
        self.assertTrue(entry["target_visible"])
        self.assertEqual(entry["method"], "open_vocab")
        self.assertEqual(entry["distance_source"], "stereo_triangulation")
        self.assertEqual(entry["model"], "fake-world")
        # fx * baseline / disparity = 250 * 0.08 / 20 = 1.0 m.
        self.assertAlmostEqual(entry["distance_m"], 1.0, places=2)
        self.assertAlmostEqual(entry["disparity_px"], 20.0, delta=1.5)
        # atan((cx - u) / fx) = atan((160 - 90) / 250) about 15.6 deg, +left.
        self.assertAlmostEqual(entry["lateral_bearing_deg"], 15.64, delta=0.5)
        self.assertIn("bbox_px", entry)
        self.assertIn("bbox_right_px", entry)
        self.assertEqual(entry["center_source"], "mask_centroid")
        self.assertAlmostEqual(entry["mask_centroid_px"][0], 90.0, delta=1.0)
        self.assertEqual(entry["mask_area_px"], 100)
        self.assertEqual(entry["size"], "small")

    def test_unmatched_target_reports_not_visible(self):
        left, right = _stereo_jpegs(right_visible=False)
        entry = open_vocab_stereo.detect(
            left, right, "red ball", self.config, client=FakeClient())
        self.assertFalse(entry["target_visible"])
        self.assertEqual(entry["distance_source"], "none")
        self.assertEqual(entry["distance_m"], 0.0)
        self.assertEqual(entry["lateral_bearing_deg"], 0.0)
        self.assertEqual(entry["center_source"], "none")

    def test_empty_target_does_not_call_the_service(self):
        left, right = _stereo_jpegs()
        client = FakeClient()
        entry = open_vocab_stereo.detect(
            left, right, "  ", self.config, client=client)
        self.assertEqual(client.calls, [])
        self.assertFalse(entry["target_visible"])
        self.assertEqual(entry["distance_source"], "none")

    def test_jpeg_size_reads_dimensions(self):
        left, _ = _stereo_jpegs()
        self.assertEqual(open_vocab_stereo.jpeg_size(left), (320, 240))


class MatchFilterTests(unittest.TestCase):
    def setUp(self):
        self.config = vision_config.load_config()

    def _match(self, left_u, right_u, **kwargs):
        return open_vocab_stereo._match(
            [_detection(left_u, **kwargs)],
            [_detection(right_u, **kwargs)], self.config)

    def test_valid_disparity_produces_a_pair(self):
        pair = self._match(100, 80)
        self.assertIsNotNone(pair)
        self.assertAlmostEqual(pair["disparity_px"], 20.0)
        self.assertAlmostEqual(pair["distance_m"], 1.0)

    def test_label_and_row_mismatch_are_rejected(self):
        self.assertIsNone(open_vocab_stereo._match(
            [_detection(100)],
            [_detection(80, label="blue bone")], self.config))
        self.assertIsNone(open_vocab_stereo._match(
            [_detection(100, v=100)],
            [_detection(80, v=130)], self.config))

    def test_out_of_range_disparities_are_rejected(self):
        # 250 * 0.08 / 5 = 4 m, beyond camera.depth_max_m = 3 m.
        self.assertIsNone(self._match(100, 95))
        # Larger than open_vocab.max_disparity = 96 px.
        self.assertIsNone(self._match(150, 10))

    def test_higher_confidence_pair_wins(self):
        left = [_detection(100, confidence=0.5),
                _detection(110, confidence=0.95)]
        right = [_detection(80, confidence=0.5),
                 _detection(90, confidence=0.95)]
        pair = open_vocab_stereo._match(left, right, self.config)
        self.assertAlmostEqual(pair["u_left"], 110.0)
        self.assertAlmostEqual(pair["confidence"], 0.95)

    def test_center_source_follows_detector_output(self):
        pair = self._match(100, 80, center_source="mask_centroid")
        self.assertEqual(pair["center_source"], "mask_centroid")
        self.assertEqual(pair["mask_centroid_px"], [100.0, 120.0])

        pair = self._match(100, 80)
        self.assertEqual(pair["center_source"], "box_center")

        mixed = open_vocab_stereo._match(
            [_detection(100, center_source="mask_centroid")],
            [_detection(80)], self.config)
        self.assertEqual(mixed["center_source"], "mixed")


if __name__ == "__main__":
    unittest.main()
