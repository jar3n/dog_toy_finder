"""Regression test: the trial collector must be launched as a package node.

Executing the collector source file directly breaks its relative import of
``llm_trials``, so the launch file must use the installed console script.
"""

import unittest
from pathlib import Path

LAUNCH_FILE = (Path(__file__).resolve().parents[1] / "launch"
               / "collect_llm_trials.launch.py")


class CollectLlmTrialsLaunchTests(unittest.TestCase):
    def test_uses_installed_console_script(self):
        text = LAUNCH_FILE.read_text()
        self.assertIn('package="dog_toy_finder_data"', text)
        self.assertIn('executable="vision_trial_collector"', text)
        self.assertNotIn("dog_toy_finder_data/vision_trial_collector.py", text)


if __name__ == "__main__":
    unittest.main()
