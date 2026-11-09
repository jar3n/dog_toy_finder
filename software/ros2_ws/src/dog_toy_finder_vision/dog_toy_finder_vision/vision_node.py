"""Vision node: answers questions about the stereo camera image with one or more
Ollama vision models, so different LLMs can be compared on the same photo.

Subscribes  /stereo/left/image_raw     sensor_msgs/Image
            /stereo/right/image_raw    sensor_msgs/Image
            /vision/ask                std_msgs/String   JSON {"id", "target"}
Publishes   /vision/result             std_msgs/String   JSON (see below)

The node always uses the stereo pair: both eyes are sent to the VLM so it can
reason about depth and distance, not just appearance.

Parameters
----------
models      : str   comma-separated list of Ollama vision models to run
model       : str   fallback single model when `models` is empty
ollama_url  : str   Ollama endpoint
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
    "floor. You are given TWO images: the first is the LEFT eye, the second is the "
    "RIGHT eye, taken at the same instant. Use the pair to judge depth and distance, "
    "not just appearance. Answer in this order. "
    "description: {description} ")

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
    "distance_m: your best estimate of the target's distance from the camera in metres, "
    "using the stereo pair to judge depth; 0 if not visible or unknown. "
    "obstacle_ahead: the name of the object, other than the target, that stands in the "
    "center third of the image where the robot would drive into it if it drove straight "
    "ahead; write none if there is no such object. Walls do not count.")

SCENE_PROMPT = COMMON.format(description=SCENE_DESC) + (
    "There is no target: set target_visible false, position none, size none, "
    "distance_m 0. "
    "obstacle_ahead: the name of the object that stands in the center third of the image "
    "where the robot would drive into it if it drove straight ahead; write none if there "
    "is no such object. Walls do not count."
)


def tidy(answer):
    obstacle = answer["obstacle_ahead"].strip().strip(".")
    if obstacle.lower() in ("", "none", "nothing", "no", "n/a"):
        obstacle = "none"
    answer["obstacle_ahead"] = obstacle
    answer["path_clear"] = obstacle == "none"
    try:
        distance = float(answer.get("distance_m") or 0.0)
    except (TypeError, ValueError):
        distance = 0.0
    answer["distance_m"] = max(0.0, distance)
    if not answer["target_visible"]:
        answer["position"] = answer["size"] = "none"
        answer["distance_m"] = 0.0
    return answer


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.declare_parameter("model", os.environ.get("VISION_MODEL", "gemma3"))
        self.declare_parameter("models", "")
        self.declare_parameter("ollama_url", os.environ.get(
            "OLLAMA_URL", "http://localhost:11434"))
        self.chat_url = self.get_parameter("ollama_url").value.rstrip("/") + "/api/chat"
        models_param = self.get_parameter("models").value.strip()
        if models_param:
            self.models = [m.strip() for m in models_param.split(",") if m.strip()]
        else:
            self.models = [self.get_parameter("model").value]

        self.lock = threading.Lock()
        self.latest = None          # (left_msg, right_msg)
        self.questions = queue.Queue()

        self.result_pub = self.create_publisher(String, "/vision/result", 10)
        self.create_subscription(Image, "/stereo/left/image_raw",
                                 self.on_left, qos_profile_sensor_data)
        self.create_subscription(Image, "/stereo/right/image_raw",
                                 self.on_right, qos_profile_sensor_data)
        self.create_subscription(String, "/vision/ask", self.on_ask, 10)
        threading.Thread(target=self.worker, daemon=True).start()
        os.makedirs(RESULTS, exist_ok=True)
        self.get_logger().info(
            f"Vision node ready: stereo eyes, models {self.models} at {self.chat_url}")

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

    def worker(self):
        while rclpy.ok():
            request, asked_at = self.questions.get()
            result = {"id": request["id"], "target": request["target"],
                      "models": self.models, "stereo": True}
            try:
                photo = self.fresh_photo(asked_at)
                if photo is None:
                    result["error"] = "No new stereo photo. Is the stereo pipeline running?"
                else:
                    left_msg, right_msg = photo
                    left_jpeg = self._to_jpeg(left_msg)
                    right_jpeg = self._to_jpeg(right_msg)
                    jpegs = [left_jpeg, right_jpeg]
                    result["image"] = self.save(left_msg, request["id"])
                    result["results"] = [
                        self.ask_model(m, jpegs, request["target"])
                        for m in self.models
                    ]
                    if len(self.models) == 1:
                        single = result["results"][0]
                        for key in ("description", "target_visible", "position",
                                    "size", "distance_m", "obstacle_ahead",
                                    "path_clear", "seconds"):
                            if key in single:
                                result[key] = single[key]
            except Exception as error:
                result["error"] = f"Vision node error: {error}"
            self.result_pub.publish(String(data=json.dumps(result)))
            self.get_logger().info(
                f"[{request['target'] or 'scene'}] "
                f"{len(self.models)} model(s) done")

    def save(self, msg, request_id):
        os.makedirs(RESULTS, exist_ok=True)
        name = f"{time.strftime('%Y%m%d_%H%M%S')}_{request_id[-6:]}.jpg"
        with open(os.path.join(RESULTS, name), "wb") as f:
            f.write(self._to_jpeg(msg))
        return os.path.join("vision", "results", "images", name)

    def ask_model(self, model, jpegs, target):
        prompt = TARGET_PROMPT.format(target=target) if target else SCENE_PROMPT
        images_b64 = [base64.b64encode(j).decode() for j in jpegs]
        body = {
            "model": model,
            "stream": False,
            "think": False,
            "format": ANSWER_SCHEMA,
            "keep_alive": "30m",
            "options": {"temperature": 0, "num_ctx": 8192},
            "messages": [{"role": "user", "content": prompt,
                          "images": images_b64}],
        }
        start = time.time()
        try:
            req = urllib.request.Request(self.chat_url, data=json.dumps(body).encode(),
                                         headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=180) as reply:
                answer = json.loads(json.load(reply)["message"]["content"])
            answer = tidy({k: answer[k] for k in ANSWER_SCHEMA["required"]})
        except Exception as error:
            return {"model": model, "error": f"Vision model failed: {error}",
                    "seconds": round(time.time() - start, 2)}
        answer["model"] = model
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
