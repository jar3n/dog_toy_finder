"""Vision node answering ``/vision/ask`` with a selectable pipeline.

Both pipelines take the operator's text target and the rectified left/right
JPEGs, then return one answer entry with the object's location. The node reads
``vision_detectors.yaml`` for the camera intrinsics, baseline, valid depth
range, and open-vocabulary detector settings.

Subscribes  /stereo/{left,right}/image_rect_color  sensor_msgs/Image
             /vision/ask                         std_msgs/String
Publishes   /vision/result                       std_msgs/String (JSON)

Parameters
----------
config_path       : str   detector YAML path (from full_system_eval.conf)
pipeline          : str   triangulation|stereo-net (empty = VISION_PIPELINE / YAML)
yolo_service_url  : str   host detector service (empty = env / YAML / default)
stereo_service_url: str   host dense-depth service (empty = env / YAML / default)
left_rect_topic   : str   override the left rectified-color topic
right_rect_topic : str   override the right rectified-color topic
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

from . import depth_client, vision_config, vision_pipelines, yolo_client

RESULTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results", "images")
FRESH_PHOTO_TIMEOUT = 5.0

# Copied from a single pipeline answer to the top level of /vision/result for
# the mapping and planning consumers that expect them there.
TOP_LEVEL_KEYS = (
    "description", "target_visible", "lateral_bearing_deg",
    "center_offset_m", "distance_m", "obstacle_ahead", "path_clear", "seconds",
    "method", "distance_source", "model", "detector_model", "bbox_px",
    "bbox_right_px", "disparity_px", "center_source", "mask_centroid_px",
    "mask_area_px", "depth_sample_source", "depth_sample_count",
    "depth_valid_count", "depth_disparity_px",
)


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.declare_parameter("config_path", "")
        self.declare_parameter("pipeline", "")
        self.declare_parameter("yolo_service_url", "")
        self.declare_parameter("stereo_service_url", "")
        self.declare_parameter("left_rect_topic", "")
        self.declare_parameter("right_rect_topic", "")

        self.config = vision_config.load_config(
            self.get_parameter("config_path").value or None)
        service_url = self.get_parameter("yolo_service_url").value
        if service_url:
            self.config.setdefault("open_vocab", {})["service_url"] = service_url
        stereo_url = self.get_parameter("stereo_service_url").value
        if stereo_url:
            self.config.setdefault("stereo-net", {})["service_url"] = stereo_url
        self.pipeline = vision_pipelines.pipeline_name(
            self.get_parameter("pipeline").value, self.config)

        left_topic = self.get_parameter("left_rect_topic").value or \
            vision_config.get(self.config, "camera", "left_rectified_topic")
        right_topic = self.get_parameter("right_rect_topic").value or \
            vision_config.get(self.config, "camera", "right_rectified_topic")

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
            "Vision node ready: pipeline=%s, rectified stereo %s + %s"
            % (self.pipeline, left_topic, right_topic))
        try:
            self.configure_host_models()
        except Exception as error:
            # The services may start after this node; each ask retries configure.
            self.get_logger().warning(
                "Could not configure host vision models yet: %s" % error)

    def configure_host_models(self):
        """Select this node's detector/depth models on the running services."""
        yolo_client.configure(self.config)
        if self.pipeline == vision_pipelines.STEREO_NET:
            depth_client.configure(self.config)

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
            result = {
                "id": request["id"],
                "target": request["target"],
                "pipeline": self.pipeline,
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
                    self.configure_host_models()
                    entries = vision_pipelines.run_pipeline(
                        self.pipeline, request["target"],
                        jpegs[0], jpegs[1], self.config)
                    result["results"] = entries
                    result["models"] = [entry.get("model", "unknown")
                                        for entry in entries]
                    if len(entries) == 1:
                        single = entries[0]
                        for key in TOP_LEVEL_KEYS:
                            if key in single:
                                result[key] = single[key]
            except Exception as error:
                result["error"] = f"Vision node error: {error}"
            self.result_pub.publish(String(data=json.dumps(result)))
            self.get_logger().info(
                "[%s] %s produced %d answer(s)"
                % (request["target"] or "scene", self.pipeline,
                   len(result.get("results", []))))

    def save(self, msg, request_id):
        os.makedirs(RESULTS, exist_ok=True)
        name = f"{time.strftime('%Y%m%d_%H%M%S')}_{request_id[-6:]}.jpg"
        with open(os.path.join(RESULTS, name), "wb") as f:
            f.write(self._to_jpeg(msg))
        return os.path.join("vision", "results", "images", name)


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
