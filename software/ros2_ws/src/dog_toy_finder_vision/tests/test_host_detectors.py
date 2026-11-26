"""Tests for the three host-only open-vocabulary detector backends.

All heavy libraries (ultralytics, torch, transformers) are replaced with small
fakes: these tests check backend selection, prompt formatting, and conversion
into the detection dictionaries consumed by the container's stereo matcher.
"""

import contextlib
import os
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[5]
HOST_VISION = REPO_ROOT / "software" / "host_vision"
sys.path.insert(0, str(HOST_VISION))
import detectors  # noqa: E402


class FakeYOLO:
    def __init__(self, path):
        self.path = path
        self.device = None

    def to(self, device):
        self.device = device


class FakeYOLOE(FakeYOLO):
    pass


class FakeBatch(dict):
    input_ids = ["input-ids"]

    def to(self, device):
        self.device = device
        return self


class FakeProcessor:
    def __init__(self, path):
        self.path = path
        self.queries = []

    def __call__(self, images, text, return_tensors):
        self.queries.append(text)
        return FakeBatch()

    def post_process_grounded_object_detection(self, outputs, input_ids,
                                               box_threshold, text_threshold,
                                               target_sizes):
        self.thresholds = (box_threshold, text_threshold, target_sizes)
        return [{
            "boxes": [[80.0, 110.0, 100.0, 130.0]],
            "scores": [0.91],
            "labels": ["red ball"],
        }]


class FakeModernProcessor(FakeProcessor):
    """Current transformers uses ``threshold`` instead of ``box_threshold``."""

    def post_process_grounded_object_detection(self, outputs, input_ids,
                                               threshold, text_threshold,
                                               target_sizes):
        self.thresholds = (threshold, text_threshold, target_sizes)
        return [{
            "boxes": [[80.0, 110.0, 100.0, 130.0]],
            "scores": [0.91],
            "labels": ["red ball"],
        }]


class FakeDinoModel:
    def __init__(self, path):
        self.path = path
        self.device = None

    def to(self, device):
        self.device = device

    def __call__(self, **inputs):
        return "outputs"


class _Boxes:
    def __init__(self):
        self.xyxy = np.asarray([[80, 110, 100, 130], [180, 110, 200, 130]])
        self.conf = np.asarray([0.9, 0.7])
        self.cls = np.asarray([0, 1])

    def __len__(self):
        return len(self.xyxy)


class FakeMasks:
    def __init__(self, data):
        self.data = data
        self.xy = None


class FakeResult:
    boxes = _Boxes()
    names = {0: "red ball", 1: "blue bone"}

    def __init__(self, masks=None):
        self.masks = masks


class FakeYoloModel:
    def __init__(self, masks=None):
        self.classes = []
        self.kwargs = None
        self.masks = masks

    def set_classes(self, classes):
        self.classes = list(classes)

    def __call__(self, rgb, **kwargs):
        self.kwargs = kwargs
        return [FakeResult(self.masks)]


def fake_modules():
    torch = types.SimpleNamespace(no_grad=contextlib.nullcontext)
    transformers = types.SimpleNamespace(
        AutoProcessor=type("AutoProcessor", (), {
            "from_pretrained": classmethod(
                lambda cls, path: FakeProcessor(path))}),
        AutoModelForZeroShotObjectDetection=type(
            "AutoModelForZeroShotObjectDetection", (), {
                "from_pretrained": classmethod(
                    lambda cls, path: FakeDinoModel(path))}))
    ultralytics = types.SimpleNamespace(YOLO=FakeYOLO, YOLOE=FakeYOLOE)
    return torch, transformers, ultralytics


class BackendSelectionTests(unittest.TestCase):
    def setUp(self):
        detectors._CACHE.clear()

    def test_unknown_backend_is_rejected(self):
        with self.assertRaises(ValueError):
            detectors.detector_name({"detector": "yolo26"})
        self.assertEqual(detectors.detector_name({}), detectors.YOLO_WORLD)

    def test_yolo_world_and_yoloe_26_load_distinct_classes(self):
        _, _, ultralytics = fake_modules()
        with mock.patch.dict(sys.modules, {"ultralytics": ultralytics}):
            backend, loaded = detectors.load_detector({
                "detector": "yoloe_26", "model_path": "weights.pt",
                "device": "cpu"})
            self.assertEqual(backend, detectors.YOLOE_26)
            self.assertIsInstance(loaded, FakeYOLOE)
            self.assertEqual(loaded.device, "cpu")

            backend, loaded = detectors.load_detector({
                "detector": "yolo_world", "model_path": "weights.pt"})
            self.assertEqual(backend, detectors.YOLO_WORLD)
            self.assertIsInstance(loaded, FakeYOLO)

    def test_grounding_dino_loads_processor_and_network(self):
        torch, transformers, _ = fake_modules()
        with mock.patch.dict(sys.modules, {
                "torch": torch, "transformers": transformers}):
            backend, loaded = detectors.load_detector({
                "detector": "grounding_dino",
                "model_path": "IDEA-Research/grounding-dino-tiny",
                "device": "cpu"})
        self.assertEqual(backend, detectors.GROUNDING_DINO)
        processor, network = loaded
        self.assertIsInstance(processor, FakeProcessor)
        self.assertIsInstance(network, FakeDinoModel)
        self.assertEqual(network.device, "cpu")

    def test_injected_model_skips_library_imports(self):
        fake = FakeYoloModel()
        backend, loaded = detectors.load_detector(
            {"detector": "yoloe_26"}, model=fake)
        self.assertEqual(backend, detectors.YOLOE_26)
        self.assertIs(loaded, fake)


class DetectionConversionTests(unittest.TestCase):
    def test_yolo_detections_use_names_and_pass_thresholds(self):
        model = FakeYoloModel()
        entries = detectors.detections_for_image(
            (detectors.YOLO_WORLD, model), np.zeros((240, 320, 3), dtype=np.uint8),
            ["red ball", "blue bone"],
            {"confidence_threshold": 0.4, "max_detections": 5, "device": "cpu"})
        self.assertEqual(model.classes, ["red ball", "blue bone"])
        self.assertEqual(model.kwargs["conf"], 0.4)
        self.assertEqual(model.kwargs["max_det"], 5)
        self.assertEqual([entry["label"] for entry in entries],
                         ["red ball", "blue bone"])
        self.assertAlmostEqual(entries[0]["u"], 90.0)
        self.assertAlmostEqual(entries[0]["width_px"], 20.0)
        self.assertEqual(entries[0]["center_source"], "box_center")
        self.assertEqual(entries[0]["mask_area_px"], 0)
        self.assertIsNone(entries[0]["mask_centroid_px"])

    def test_mask_centroid_replaces_box_centre(self):
        # A triangular mask inside the 80..100 x 110..130 box: its pixel mean is
        # left of the box centre, so the test proves the mask actually drives u/v.
        mask = np.zeros((240, 320), dtype=np.uint8)
        for x in range(80, 101):
            mask[110:110 + (101 - x) // 2, x] = 1
        ys, xs = np.nonzero(mask)
        model = FakeYoloModel(masks=FakeMasks(np.asarray([mask, mask * 0])))
        entries = detectors.detections_for_image(
            (detectors.YOLOE_26, model), np.zeros((240, 320, 3), dtype=np.uint8),
            ["red ball"], {"confidence_threshold": 0.25})

        first = entries[0]
        self.assertEqual(first["center_source"], "mask_centroid")
        self.assertAlmostEqual(first["u"], float(xs.mean()), places=5)
        self.assertAlmostEqual(first["v"], float(ys.mean()), places=5)
        self.assertLess(first["u"], 90.0)  # box centre would be 90
        self.assertEqual(first["mask_centroid_px"],
                         [round(first["u"], 2), round(first["v"], 2)])
        self.assertEqual(first["mask_area_px"], len(xs))
        self.assertEqual(len(first["depth_sample_pixels"]), len(xs))
        self.assertAlmostEqual(first["depth_sample_pixels"][0][0],
                               float(xs[0]), places=2)
        self.assertAlmostEqual(first["bbox_px"][0], 80.0)
        self.assertAlmostEqual(first["bbox_px"][2], 100.0)
        # A second detection with an empty mask falls back to its box centre.
        self.assertEqual(entries[1]["center_source"], "box_center")
        self.assertAlmostEqual(entries[1]["u"], 190.0)
        self.assertEqual(entries[1]["depth_sample_pixels"], [])

    def test_mask_at_network_resolution_is_scaled_to_image_pixels(self):
        mask = np.zeros((120, 160), dtype=np.uint8)  # half of 240x320
        mask[55:60, 40:50] = 1                      # image x 80..100, y 110..120
        ys, xs = np.nonzero(mask)
        model = FakeYoloModel(masks=FakeMasks(np.asarray([mask])))
        entries = detectors.detections_for_image(
            (detectors.YOLOE_26, model), np.zeros((240, 320, 3), dtype=np.uint8),
            ["red ball"], {})
        self.assertEqual(entries[0]["center_source"], "mask_centroid")
        self.assertAlmostEqual(entries[0]["u"], float(xs.mean()) * 2.0, places=5)
        self.assertAlmostEqual(entries[0]["v"], float(ys.mean()) * 2.0, places=5)
        self.assertEqual(entries[0]["mask_area_px"], len(xs) * 4)

    def test_grounding_dino_query_and_thresholds(self):
        self.assertEqual(detectors._dino_query(["Red Ball", "blue bone."]),
                         "red ball. blue bone.")

        torch, _, _ = fake_modules()
        processor = FakeProcessor("fake-id")
        network = FakeDinoModel("fake-id")
        rgb = np.zeros((240, 320, 3), dtype=np.uint8)
        with mock.patch.dict(sys.modules, {"torch": torch}):
            entries = detectors.detections_for_image(
                (detectors.GROUNDING_DINO, (processor, network)), rgb,
                ["Red Ball"], {"confidence_threshold": 0.4,
                               "text_threshold": 0.3, "device": "cpu"})
        self.assertEqual(processor.queries, ["red ball."])
        self.assertEqual(processor.thresholds, (0.4, 0.3, [(240, 320)]))
        self.assertEqual(len(entries), 1)
        # The lowercase model label is mapped back to the operator's prompt.
        self.assertEqual(entries[0]["label"], "Red Ball")
        self.assertAlmostEqual(entries[0]["u"], 90.0)
        self.assertAlmostEqual(entries[0]["confidence"], 0.91)
        self.assertEqual(entries[0]["center_source"], "box_center")

    def test_grounding_dino_accepts_modern_threshold_keyword(self):
        torch, _, _ = fake_modules()
        processor = FakeModernProcessor("fake-id")
        network = FakeDinoModel("fake-id")
        rgb = np.zeros((240, 320, 3), dtype=np.uint8)
        with mock.patch.dict(sys.modules, {"torch": torch}):
            entries = detectors.detections_for_image(
                (detectors.GROUNDING_DINO, (processor, network)), rgb,
                ["Red Ball"], {"confidence_threshold": 0.4,
                               "text_threshold": 0.3, "device": "cpu"})
        self.assertEqual(processor.thresholds, (0.4, 0.3, [(240, 320)]))
        self.assertEqual(len(entries), 1)
        self.assertEqual(entries[0]["label"], "Red Ball")


if __name__ == "__main__":
    unittest.main()
