"""Vision node answering ``/vision/ask`` with Ollama vision models.

The same rectified stereo pair is sent to one or more Ollama vision models.
Models are loaded and unloaded one at a time, even when ``models`` lists
several, so each model gets all of the Ollama host's VRAM.

The node reads ``vision_detectors.yaml`` for camera intrinsics, baseline, valid
depth range, and the toy's real-world measurements; those numbers are included
in the model prompt.

Subscribes  /stereo/{left,right}/image_rect_color  sensor_msgs/Image
             /vision/ask                         std_msgs/String
Publishes   /vision/result                       std_msgs/String (JSON)

Parameters
----------
config_path      : str   detector YAML path (from full_system_eval.conf)
ml_enabled       : bool  run the Ollama model(s)
model            : str   fallback single model when `models` is empty
models           : str   comma-separated list of Ollama vision models
ollama_url       : str   Ollama endpoint
"""

import io
import json
import os
import queue
import threading
import time

import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

from . import ollama_vlm, vision_answer, vision_config

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "images")
FRESH_PHOTO_TIMEOUT = 5.0

class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("ml_enabled", True)
        self.declare_parameter("left_rect_topic", "")
        self.declare_parameter("right_rect_topic", "")
        self.declare_parameter("model", os.environ.get("VISION_MODEL", "gemma4"))
        self.declare_parameter("models", "")
        self.declare_parameter("ollama_url", os.environ.get(
            "OLLAMA_URL", "http://localhost:11434"))

        self.config = vision_config.load_config(
            self.get_parameter("config_path").value or None)
        self.ml_enabled = (
            bool(self.get_parameter("ml_enabled").value)
            and bool(vision_config.get(self.config, "ml", "enabled", default=True)))
        if not self.ml_enabled:
            raise ValueError("Vision ML path is disabled; check the detector config")

        left_topic = self.get_parameter("left_rect_topic").value or \
            vision_config.get(self.config, "camera", "left_rectified_topic")
        right_topic = self.get_parameter("right_rect_topic").value or \
            vision_config.get(self.config, "camera", "right_rectified_topic")

        self.base_url = self.get_parameter("ollama_url").value.rstrip("/")
        self.chat_url = self.base_url + "/api/chat"
        self.generate_url = self.base_url + "/api/generate"
        models_param = self.get_parameter("models").value.strip()
        if models_param:
            self.models = [m.strip() for m in models_param.split(",") if m.strip()]
        else:
            self.models = [self.get_parameter("model").value]
        # A single model is normal robot operation: keep it warm. Multiple
        # models are a comparison run: unload each one before loading the next.
        self.compare_models = len(self.models) > 1

        self.lock = threading.Lock()
        self.latest = None          # (left_msg, right_msg)
        self.questions = queue.Queue()

        self.result_pub = self.create_publisher(String, "/vision/result", 10)
        self.create_subscription(Image, left_topic, self.on_left,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, right_topic, self.on_right,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/vision/ask", self.on_ask, 10)
        threading.Thread(target=self.worker, daemon=True).start()
        os.makedirs(RESULTS, exist_ok=True)
        self.get_logger().info(
            "Vision node ready: rectified stereo %s + %s, ML models=%s at %s"
            % (left_topic, right_topic,
               self.models if self.ml_enabled else "disabled", self.chat_url))

    def on_left(self, msg):
        with self.lock:
            if isinstance(self.latest, tuple):
                self.latest = (msg, self.latest[1])
            else:
                self.latest = (msg, None)

    def on_right(self, msg):
        with self.lock:
            if isinstance(self.latest, tuple):
                self.latest = (self.latest[0], msg)
            else:
                self.latest = (None, msg)

    def on_ask(self, msg):
        try:
            request = json.loads(msg.data)
        except json.JSONDecodeError:
            request = {"target": msg.data}
        request.setdefault("id", str(int(time.time() * 1000)))
        request["target"] = str(request.get("target", "")).strip()
        self.questions.put((request, self.get_clock().now()))

    def fresh_photo(self, asked_at):
        end = time.time() + FRESH_PHOTO_TIMEOUT
        while time.time() < end:
            with self.lock:
                snap = self.latest
            if (isinstance(snap, tuple) and snap[0] is not None
                    and snap[1] is not None):
                left_stamp = Time.from_msg(snap[0].header.stamp)
                right_stamp = Time.from_msg(snap[1].header.stamp)
                if left_stamp >= asked_at or right_stamp >= asked_at:
                    return snap
            time.sleep(0.05)
        return None

    def _to_jpeg(self, msg):
        img = PilImage.frombytes("RGB", (msg.width, msg.height), bytes(msg.data))
        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=85)
        return buf.getvalue()

    def prompt(self, target):
        return vision_answer.prompt_for(target, self.config)

    def worker(self):
        while rclpy.ok():
            request, asked_at = self.questions.get()
            result = {
                "id": request["id"],
                "target": request["target"],
                "models": [],
                "stereo": True,
                "image_source": "rectified_color",
            }
            try:
                photo = self.fresh_photo(asked_at)
                if photo is None:
                    result["error"] = (
                        "No new rectified stereo photo. Is the stereo pipeline running?")
                else:
                    left_msg, right_msg = photo
                    jpegs = [self._to_jpeg(left_msg), self._to_jpeg(right_msg)]
                    result["image"] = self.save(left_msg, request["id"])

                    entries = []
                    if self.ml_enabled:
                        # Load and unload one model at a time. Ollama would
                        # otherwise keep several models resident, and the later
                        # ones could run with less VRAM than the first.
                        for model in self.models:
                            try:
                                entries.append(self.ask_model(
                                    model, jpegs, request["target"]))
                            finally:
                                if self.compare_models:
                                    self.unload_model(model)

                    result["results"] = entries
                    result["models"] = [entry.get("model", "unknown")
                                        for entry in entries]
                    if len(entries) == 1:
                        single = entries[0]
                        for key in ("description", "target_visible", "position",
                                    "lateral_bearing_deg", "size", "distance_m",
                                    "obstacle_ahead", "path_clear", "seconds",
                                    "method", "distance_source"):
                            if key in single:
                                result[key] = single[key]
            except Exception as error:
                result["error"] = f"Vision node error: {error}"
            self.result_pub.publish(String(data=json.dumps(result)))
            self.get_logger().info(
                "[%s] %d model answer(s) done"
                % (request["target"] or "scene", len(result.get("results", []))))

    def save(self, msg, request_id):
        os.makedirs(RESULTS, exist_ok=True)
        name = f"{time.strftime('%Y%m%d_%H%M%S')}_{request_id[-6:]}.jpg"
        with open(os.path.join(RESULTS, name), "wb") as f:
            f.write(self._to_jpeg(msg))
        return os.path.join("vision", "results", "images", name)

    def unload_model(self, model):
        """Ask Ollama to evict a model so the next one gets all the VRAM."""
        return ollama_vlm.unload_model(model, self.generate_url,
                                       logger=self.get_logger())

    def ask_model(self, model, jpegs, target):
        return ollama_vlm.ask_model(
            model, jpegs, target, self.prompt(target), self.chat_url,
            compare_models=self.compare_models, config=self.config)


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
