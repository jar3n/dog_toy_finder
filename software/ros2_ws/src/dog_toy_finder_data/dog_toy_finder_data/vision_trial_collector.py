"""Record one row per vision model for every /vision/ask comparison.

The vision node already asks every model in its ``models`` parameter about the
same stereo photo and publishes all answers in one ``/vision/result`` JSON.
This collector turns that stream into an annotatable CSV:

    <output_dir>/<run_id>/vision_trials.csv
    <output_dir>/<run_id>/manifest.json

The automated columns (model, latency, answer, image path) are filled here.
The ``gt_*`` ground-truth columns stay empty for a spreadsheet, or are filled
immediately when the ask JSON carries a ``ground_truth`` object::

    {"id": "1", "target": "red ball",
     "ground_truth": {"target_visible": true, "position": "left",
                      "size": "medium", "distance_m": 0.7,
                      "obstacle_ahead": "none"}}

Run the collector alongside the vision node, for example with
``LLM_TRIALS=true ./scripts/launch_all.sh`` or directly::

    ros2 launch dog_toy_finder_data collect_llm_trials.launch.py

Subscribes  /vision/ask      std_msgs/String (JSON, optional ground_truth)
            /vision/result   std_msgs/String (JSON with one result per model)

Parameters
----------
output_dir          : str   trial root directory
run_id              : str   label for this comparison run
distance_tolerance  : float metres, used when scoring a distance ground truth
"""

import json
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .llm_trials import (
    KIND_VISION,
    VISION_FIELDS,
    append_csv,
    vision_row,
)

PENDING_MAX_AGE_S = 120.0


class VisionTrialCollector(Node):
    """Subscribe to vision results and append one CSV row per model answer."""

    def __init__(self):
        super().__init__("vision_trial_collector")
        self.declare_parameter("output_dir", "/tmp/llm_trials")
        self.declare_parameter("run_id", "vision1")
        self.declare_parameter("distance_tolerance", 0.25)

        self.output_dir = self.get_parameter("output_dir").value
        self.run_id = self.get_parameter("run_id").value
        self.distance_tolerance = float(
            self.get_parameter("distance_tolerance").value)

        self.run_dir = os.path.join(self.output_dir, self.run_id)
        os.makedirs(self.run_dir, exist_ok=True)
        self.csv_path = os.path.join(self.run_dir, "vision_trials.csv")

        self._lock = threading.Lock()
        # ask id -> {"target": str, "ground_truth": dict, "received": float}
        self._pending = {}
        self._rows = 0
        self._models = set()
        self.started = time.time()

        self.create_subscription(String, "/vision/ask", self.on_ask, 10)
        self.create_subscription(String, "/vision/result", self.on_result,
                                 qos_profile_sensor_data)
        self._write_manifest()
        self.get_logger().info(
            "Vision trial collector writing %s" % self.csv_path)

    def _write_manifest(self, finished=False):
        """Write run metadata, updating the row count and models seen."""
        manifest = {
            "kind": KIND_VISION,
            "run_id": self.run_id,
            "csv": self.csv_path,
            "distance_tolerance": self.distance_tolerance,
            "rows": self._rows,
            "models": sorted(self._models),
            "started": self.started,
        }
        if finished:
            manifest["finished"] = time.time()
        path = os.path.join(self.run_dir, "manifest.json")
        try:
            with open(path, "w") as handle:
                json.dump(manifest, handle, indent=2)
        except OSError as error:
            self.get_logger().warn("Could not write manifest: %s" % error)

    def on_ask(self, msg):
        """Remember a request's target and optional ground truth by ask id."""
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ask_id = str(request.get("id", ""))
        if not ask_id:
            return
        now = time.time()
        with self._lock:
            self._pending[ask_id] = {
                "target": request.get("target", ""),
                "ground_truth": request.get("ground_truth"),
                "received": now,
            }
            # Drop old asks whose result never arrived so the dict cannot grow.
            for key in [k for k, value in self._pending.items()
                        if now - value["received"] > PENDING_MAX_AGE_S]:
                self._pending.pop(key, None)

    def on_result(self, msg):
        """Append one CSV row per model answer in a /vision/result message."""
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        ask_id = str(result.get("id", ""))
        with self._lock:
            pending = self._pending.pop(ask_id, {}) if ask_id else {}
        ground_truth = pending.get("ground_truth")
        models = result.get("models") or []
        if not isinstance(models, list):
            models = [models]

        entries = result.get("results")
        if not isinstance(entries, list) or not entries:
            # Older/single-model results put the answer at the top level.
            entries = [{
                key: result.get(key)
                for key in ("description", "target_visible", "position",
                            "size", "distance_m", "obstacle_ahead",
                            "path_clear", "seconds", "error")
                if key in result
            }]

        for index, entry in enumerate(entries):
            if not isinstance(entry, dict):
                continue
            model = entry.get("model")
            if not model:
                model = models[index] if index < len(models) else (
                    models[0] if models else "unknown")
            row = vision_row(result, entry, model, self.run_id,
                             self.distance_tolerance, ground_truth)
            append_csv(self.csv_path, row, VISION_FIELDS)
            self._rows += 1
            self._models.add(str(model))
        self._write_manifest()

    def destroy_node(self):
        """Finalise the manifest when the collector stops."""
        self._write_manifest(finished=True)
        super().destroy_node()


def main(args=None):
    """Start the vision trial collector node."""
    rclpy.init()
    node = VisionTrialCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
