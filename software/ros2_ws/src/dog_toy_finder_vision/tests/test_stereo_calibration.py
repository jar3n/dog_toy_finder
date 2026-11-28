import json
import os
import unittest

from dog_toy_finder_vision.stereo_calibration import load_stereo_calibration


def _matrix(rows, cols, start=0.0):
    return [[start + row * cols + col for col in range(cols)]
            for row in range(rows)]


def _calibration():
    return {
        "kind": "dog_toy_finder_stereo_calibration",
        "width": 320,
        "height": 240,
        "baseline_m": 0.052,
        "rotation": _matrix(3, 3),
        "translation": [0.052, 0.01, 0.0],
        "left": {
            "k": _matrix(3, 3, 1),
            "d": [0.0] * 5,
            "r": _matrix(3, 3, 2),
            "p": _matrix(3, 4, 3),
        },
        "right": {
            "k": _matrix(3, 3, 10),
            "d": [0.0] * 5,
            "r": _matrix(3, 3, 20),
            "p": _matrix(3, 4, 30),
        },
    }


class TestStereoCalibration(unittest.TestCase):
    def _write(self, directory, data):
        path = os.path.join(directory, "stereo.json")
        with open(path, "w") as handle:
            json.dump(data, handle)
        return path

    def test_load_keeps_independent_k_and_r(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            loaded = load_stereo_calibration(
                self._write(directory, _calibration()))
        self.assertNotEqual(loaded["left"]["k"], loaded["right"]["k"])
        self.assertNotEqual(loaded["left"]["r"], loaded["right"]["r"])
        self.assertEqual(loaded["right"]["p"][0][0], 30)
        self.assertEqual(loaded["baseline_m"], 0.052)

    def test_empty_path_returns_none(self):
        self.assertIsNone(load_stereo_calibration(""))

    def test_missing_explicit_file_is_an_error(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(FileNotFoundError):
                load_stereo_calibration(
                    os.path.join(directory, "missing.json"))

    def test_wrong_kind_is_rejected(self):
        import tempfile
        data = _calibration()
        data["kind"] = "something_else"
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                load_stereo_calibration(self._write(directory, data))

    def test_malformed_projection_is_rejected(self):
        import tempfile
        data = _calibration()
        data["right"]["p"] = _matrix(3, 3)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                load_stereo_calibration(self._write(directory, data))


if __name__ == "__main__":
    unittest.main()
