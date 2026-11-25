"""Regression test: the removed live vision trial collector stays removed."""

import os
import unittest

PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


class RemovedVisionTrialCollectorTests(unittest.TestCase):
    def test_module_and_launch_file_are_absent(self):
        module = os.path.join(
            PKG_ROOT, "dog_toy_finder_data", "vision_trial_collector.py")
        launch = os.path.join(
            PKG_ROOT, "launch", "collect_llm_trials.launch.py")
        self.assertFalse(os.path.exists(module))
        self.assertFalse(os.path.exists(launch))

    def test_setup_does_not_register_the_collector(self):
        with open(os.path.join(PKG_ROOT, "setup.py")) as handle:
            setup_text = handle.read()
        self.assertNotIn("vision_trial_collector", setup_text)
        self.assertNotIn("collect_llm_trials.launch.py", setup_text)


if __name__ == "__main__":
    unittest.main()
