import os
import tempfile
import unittest

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
                "fx": 299.72,
                "fy": 300.92,
                "cx": 175.61,
                "cy": 127.57,
                "baseline_m": 0.0431,
                "depth_min_m": 0.25,
                "depth_max_m": 3.0,
            },
        })
        self.assertNotIn("target", config)
        self.assertEqual(config["camera"]["fx"], 299.72)


if __name__ == "__main__":
    unittest.main()
