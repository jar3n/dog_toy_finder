"""Vision node with two independent paths over the same rectified stereo pair.

Path 1 (classical) is deterministic: colour/edge segmentation, connected
components, and rectified-stereo patch matching with a calibrated baseline. No
learned model is involved.

Path 2 (ML) sends the same rectified images to one or more Ollama vision models.
Models are loaded and unloaded one at a time, even when ``models`` lists
several, so each model gets all of the Ollama host's VRAM.

Both paths read ``vision_detectors.yaml`` for the camera intrinsics, baseline,
valid depth range, and the toy's real-world measurements.

Subscribes  /stereo/{left,right}/image_rect_color  sensor_msgs/Image
             /vision/ask                         std_msgs/String
Publishes   /vision/result                       std_msgs/String (JSON)

Parameters
----------
config_path      : str   detector YAML path (from full_system_eval.conf)
classical_enabled: bool  run the deterministic detector
ml_enabled       : bool  run the Ollama model(s)
model            : str   fallback single model when `models` is empty
models           : str   comma-separated list of Ollama vision models
ollama_url       : str   Ollama endpoint
"""

import base64
import io
import json
import os
import queue
import threading
import time
import urllib.request

import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

from . import classical_detector, vision_config

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "images")
FRESH_PHOTO_TIMEOUT = 5.0

ANSWER_SCHEMA = {
    "type": "object",
    "properties": {
        "description": {"type": "string"},
        "target_visible": {"type": "boolean"},
        "position": {"type": "string", "enum": ["left", "center", "right", "none"]},
        "size": {"type": "string", "enum": ["small", "medium", "large", "none"]},
        "distance_m": {"type": "number", "minimum": 0},
        "obstacle_ahead": {"type": "string"},
    },
    "required": ["description", "target_visible", "position", "size",
                 "distance_m", "obstacle_ahead"],
}

COMMON = (
    "You are the forward-facing stereo camera of a small robot car that drives on the "
    "floor. You are given TWO RECTIFIED images: the first is the LEFT eye, the second "
    "is the RIGHT eye, captured together. Rectification means matching rows have the "
    "same y coordinate, so horizontal image shift is disparity. Use the pair, the "
    "camera measurements below, and the toy's real size to judge distance in metres, "
    "not just appearance. Answer in this order. "
    "description: {description} "
)

TARGET_DESC = ("one or two sentences naming every object you see, its color, and "
               "whether it is on the left, in the center, or on the right of the image.")
SCENE_DESC = ("two or three sentences naming every object you see, its color, and "
              "whether it is on the left, in the center, or on the right of the image.")

TARGET_PROMPT = COMMON.format(description=TARGET_DESC) + (
    "target_visible: true only if the target ({target}) appears in your description. "
    "position: which third of the image the target's center is in: left, center, or right "
    "(none if not visible). "
    "size: how wide the target looks compared with the image width: small (less than one "
    "tenth), medium, or large (more than one quarter); none if not visible. "
    "distance_m: your best estimate of the target's distance from the camera in metres; "
    "0 if not visible or unknown. "
    "obstacle_ahead: the name of the object, other than the target, that stands in the "
    "center third of the image where the robot would drive into it if it drove straight "
    "ahead; write none if there is no such object. Walls do not count."
)

SCENE_PROMPT = COMMON.format(description=SCENE_DESC) + (
    "There is no target: set target_visible false, position none, size none, "
    "distance_m 0. "
    "obstacle_ahead: the name of the object that stands in the center third of the image "
    "where the robot would drive into it if it drove straight ahead; write none if "
    "there is no such object. Walls do not count."
)


def _as_bool(value, default=False):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("true", "1", "yes", "y", "on")
    return bool(value) if value is not None else default


def tidy(answer):
    """Normalise an answer while preserving detector-specific extra fields."""
    result = dict(answer)
    obstacle = str(result.get("obstacle_ahead") or "").strip().strip(".")
    if obstacle.lower() in ("", "none", "nothing", "no", "n/a"):
        obstacle = "none"
    result["obstacle_ahead"] = obstacle
    result["path_clear"] = obstacle == "none"
    result["target_visible"] = _as_bool(result.get("target_visible"), False)
    try:
        distance = float(result.get("distance_m") or 0.0)
    except (TypeError, ValueError):
        distance = 0.0
    result["distance_m"] = max(0.0, distance)
    if not result["target_visible"]:
        result["position"] = "none"
        result["size"] = "none"
        result["distance_m"] = 0.0
    return result


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("classical_enabled", True)
        self.declare_parameter("ml_enabled", True)
        self.declare_parameter("left_rect_topic", "")
        self.declare_parameter("right_rect_topic", "")
        self.declare_parameter("model", os.environ.get("VISION_MODEL", "gemma4"))
        self.declare_parameter("models", "")
        self.declare_parameter("ollama_url", os.environ.get(
            "OLLAMA_URL", "http://localhost:11434"))

        self.config = vision_config.load_config(
            self.get_parameter("config_path").value or None)
        self.classical_enabled = (
            bool(self.get_parameter("classical_enabled").value)
            and bool(vision_config.get(self.config, "classical", "enabled",
                                       default=True)))
        self.ml_enabled = (
            bool(self.get_parameter("ml_enabled").value)
            and bool(vision_config.get(self.config, "ml", "enabled", default=True)))
        if not self.classical_enabled and not self.ml_enabled:
            raise ValueError("Both vision paths are disabled; check the detector config")

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
            "Vision node ready: rectified stereo %s + %s, classical=%s, "
            "ML models=%s at %s"
            % (left_topic, right_topic, self.classical_enabled,
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
        base = TARGET_PROMPT.format(target=target) if target else SCENE_PROMPT
        summary = vision_config.physical_summary(self.config)
        return base + (" " + summary if summary else "")

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
                    if self.classical_enabled:
                        start = time.time()
                        detection = classical_detector.detect(
                            classical_detector.message_to_rgb(left_msg),
                            classical_detector.message_to_rgb(right_msg),
                            request["target"], self.config)
                        detection["model"] = "classical"
                        detection["seconds"] = round(time.time() - start, 3)
                        entries.append(tidy(detection))

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
                                    "size", "distance_m", "obstacle_ahead",
                                    "path_clear", "seconds", "method",
                                    "distance_source", "confidence"):
                            if key in single:
                                result[key] = single[key]
            except Exception as error:
                result["error"] = f"Vision node error: {error}"
            self.result_pub.publish(String(data=json.dumps(result)))
            self.get_logger().info(
                "[%s] %d detector/model answer(s) done"
                % (request["target"] or "scene", len(result.get("results", []))))

    def save(self, msg, request_id):
        os.makedirs(RESULTS, exist_ok=True)
        name = f"{time.strftime('%Y%m%d_%H%M%S')}_{request_id[-6:]}.jpg"
        with open(os.path.join(RESULTS, name), "wb") as f:
            f.write(self._to_jpeg(msg))
        return os.path.join("vision", "results", "images", name)

    def unload_model(self, model):
        """Ask Ollama to evict a model so the next one gets all the VRAM."""
        body = {"model": model, "keep_alive": 0}
        try:
            req = urllib.request.Request(
                self.generate_url, data=json.dumps(body).encode(),
                headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=15) as reply:
                reply.read()
        except Exception as error:
            self.get_logger().warn(
                f"Could not unload {model}; it may still hold VRAM: {error}")

    def ask_model(self, model, jpegs, target):
        images_b64 = [base64.b64encode(j).decode() for j in jpegs]
        body = {
            "model": model,
            "stream": False,
            "think": False,
            "format": ANSWER_SCHEMA,
            "keep_alive": 0 if self.compare_models else "30m",
            "options": {"temperature": 0, "num_ctx": 8192},
            "messages": [{"role": "user", "content": self.prompt(target),
                          "images": images_b64}],
        }
        start = time.time()
        try:
            req = urllib.request.Request(self.chat_url, data=json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=180) as reply:
                answer = json.loads(json.load(reply)["message"]["content"])
            answer = tidy({key: answer[key] for key in ANSWER_SCHEMA["required"]
                           if key in answer})
        except Exception as error:
            return {"model": model, "method": "ml", "error":
                    f"Vision model failed: {error}",
                    "seconds": round(time.time() - start, 2)}
        answer["model"] = model
        answer["method"] = "ml"
        answer["distance_source"] = "vlm_estimate"
        answer["seconds"] = round(time.time() - start, 2)
        return answer


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
