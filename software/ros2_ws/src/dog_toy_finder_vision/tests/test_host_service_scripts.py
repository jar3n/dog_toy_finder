"""Tests for the easy host-service stop scripts.

The stop scripts must only kill the process recorded in their PID file, must
tolerate a stale PID file, and must refuse to kill an unrelated process that
happens to reuse a PID.
"""

import os
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[5]
SCRIPTS = REPO_ROOT / "scripts"


def _spawn_fake_service(marker):
    """Return a sleeping process whose cmdline contains ``marker``."""
    command = [sys.executable, "-c", "import time; time.sleep(30)", marker]
    process = subprocess.Popen(command, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)
    # Give the interpreter time to reach sleep so /proc/<pid>/cmdline is set.
    time.sleep(0.2)
    return process


class StopScriptTests(unittest.TestCase):
    def _run_stop(self, script, pid_file):
        environment = dict(os.environ)
        environment["DTF_YOLO_PID_FILE" if "yolo" in script else
                     "DTF_STEREO_PID_FILE"] = str(pid_file)
        return subprocess.run(
            [str(SCRIPTS / script)], env=environment, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)

    def _stop_script_for(self, script, marker, env_key):
        with tempfile.TemporaryDirectory() as directory:
            pid_file = Path(directory) / "service.pid"
            process = _spawn_fake_service(marker)
            self.addCleanup(self._reap, process)
            pid_file.write_text(str(process.pid))
            environment = dict(os.environ)
            environment[env_key] = str(pid_file)
            result = subprocess.run(
                [str(SCRIPTS / script)], env=environment, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertFalse(pid_file.exists())
            self.assertIsNotNone(process.poll(), "fake service was not stopped")

    @staticmethod
    def _reap(process):
        if process.poll() is None:
            process.kill()
            process.wait()

    def test_stop_yolo_script_stops_its_recorded_service(self):
        self._stop_script_for("stop_yolo_service.sh", "yolo_service.py",
                              "DTF_YOLO_PID_FILE")

    def test_stop_stereo_script_stops_its_recorded_service(self):
        self._stop_script_for("stop_stereo_service.sh", "stereo_service.py",
                              "DTF_STEREO_PID_FILE")

    def test_stale_pid_file_is_removed_without_error(self):
        with tempfile.TemporaryDirectory() as directory:
            pid_file = Path(directory) / "service.pid"
            pid_file.write_text("999999")
            result = self._run_stop("stop_yolo_service.sh", pid_file)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("not running", result.stdout)
            self.assertFalse(pid_file.exists())

    def test_unrelated_process_is_not_killed(self):
        process = _spawn_fake_service("some_other_program.py")
        self.addCleanup(self._reap, process)
        with tempfile.TemporaryDirectory() as directory:
            pid_file = Path(directory) / "service.pid"
            pid_file.write_text(str(process.pid))
            result = self._run_stop("stop_yolo_service.sh", pid_file)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("refusing to stop", result.stderr)
            self.assertIsNone(process.poll(), "unrelated process was killed")
            self.assertTrue(pid_file.exists())


if __name__ == "__main__":
    unittest.main()
