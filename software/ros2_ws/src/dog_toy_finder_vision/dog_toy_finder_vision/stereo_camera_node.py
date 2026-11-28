"""Stereo camera node: two ESP32-CAM eyes -> /stereo/{left,right}/image_*.

Each eye is an ESP32-CAM photo server (firmware/esp_cam). This node downloads
a JPEG from each eye on a timer, decodes it with Pillow (no OpenCV needed on
the Python side), and publishes for every eye:

  * image_raw  sensor_msgs/Image  rgb8   -- colour frames for the vision pipeline
  * image_mono sensor_msgs/Image  mono8  -- greyscale frames for rectification
  * camera_info sensor_msgs/CameraInfo   -- intrinsics + rectification

Both image topics carry the same stamp, so image_proc rectification pairs the
mono frame with its camera_info immediately. The C++ stereo_image_proc nodes
then turn the rectified pair into disparity and a point cloud.

When ``calibration_file`` points at a JSON saved by scripts/calibrate_stereo.py,
each eye publishes its own solved K/D/R/P. Without it, the node still publishes
two different K matrices from the per-eye rectified intrinsics below and uses
identity rectification.

Parameters
----------
left_url, right_url : str   ESP32-CAM /capture URLs
rate                : float photos per second (per eye)
width, height       : int   expected image size (must match the ESP32-CAM)
left_fx, left_fy, left_cx, left_cy : float left rectified intrinsics in pixels
right_fx, right_fy, right_cx, right_cy : float right rectified intrinsics in pixels
baseline            : float distance between the two optical centres, metres
calibration_file    : str JSON with per-eye K/D/R/P (optional)
left_frame, right_frame : str optical frame ids
"""

import io
import threading
import time
import urllib.request

import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from .stereo_calibration import load_stereo_calibration


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__("stereo_camera_node")

        def param(name):
            return self.get_parameter(name).value

        self.declare_parameter("left_url", "http://esp32cam-left.local/capture")
        self.declare_parameter("right_url", "http://esp32cam-right.local/capture")
        self.declare_parameter("rate", 2.0)
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        for side in ("left", "right"):
            self.declare_parameter("%s_fx" % side, 250.0)
            self.declare_parameter("%s_fy" % side, 250.0)
            self.declare_parameter("%s_cx" % side, 160.0)
            self.declare_parameter("%s_cy" % side, 120.0)
        self.declare_parameter("baseline", 0.08)
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("left_frame", "left_camera_optical_frame")
        self.declare_parameter("right_frame", "right_camera_optical_frame")

        self.left_url = str(param("left_url"))
        self.right_url = str(param("right_url"))
        self.period = 1.0 / float(param("rate"))
        self.width = int(param("width"))
        self.height = int(param("height"))
        self.intrinsics = {
            side: {field: float(param("%s_%s" % (side, field)))
                   for field in ("fx", "fy", "cx", "cy")}
            for side in ("left", "right")
        }
        self.baseline = float(param("baseline"))
        self.left_frame = str(param("left_frame"))
        self.right_frame = str(param("right_frame"))

        calibration_path = str(param("calibration_file")).strip()
        self.calibration = load_stereo_calibration(calibration_path)
        if self.calibration:
            if ((self.calibration["width"], self.calibration["height"])
                    != (self.width, self.height)):
                self.get_logger().warning(
                    "calibration_file is %dx%d but launch geometry is %dx%d; "
                    "using the calibration image size" % (
                        self.calibration["width"], self.calibration["height"],
                        self.width, self.height))
            self.width = self.calibration["width"]
            self.height = self.calibration["height"]

        # Reliable QoS so image_proc (which subscribes with default/reliable
        # QoS) actually receives the frames; best-effort subscribers can still
        # receive from a reliable publisher.
        self.left_pub = self.create_publisher(Image, "/stereo/left/image_raw", 10)
        self.right_pub = self.create_publisher(Image, "/stereo/right/image_raw", 10)
        self.left_mono_pub = self.create_publisher(Image, "/stereo/left/image_mono", 10)
        self.right_mono_pub = self.create_publisher(Image, "/stereo/right/image_mono", 10)
        self.left_info_pub = self.create_publisher(CameraInfo,
                                                  "/stereo/left/camera_info", 10)
        self.right_info_pub = self.create_publisher(CameraInfo,
                                                   "/stereo/right/camera_info", 10)

        if self.calibration:
            self.left_info = self._calibrated_info(
                self.left_frame, self.calibration["left"])
            self.right_info = self._calibrated_info(
                self.right_frame, self.calibration["right"])
            self.get_logger().info(
                "Loaded full stereo calibration from %s (baseline %.4f m)" % (
                    calibration_path, self.calibration["baseline_m"]))
        else:
            self.left_info = self._camera_info(
                self.left_frame, self.intrinsics["left"], 0.0)
            self.right_info = self._camera_info(
                self.right_frame, self.intrinsics["right"],
                -self.intrinsics["left"]["fx"] * self.baseline)
            self.get_logger().info(
                "Using per-eye rectified intrinsics (left cx %.2f, right cx %.2f, "
                "baseline %.4f m)" % (
                    self.intrinsics["left"]["cx"],
                    self.intrinsics["right"]["cx"], self.baseline))

        self._left_latest = None
        self._right_latest = None
        self._lock = threading.Lock()
        self.photos = 0
        self.failures = 0

        threading.Thread(target=self._http_loop, args=(self.left_url, True), daemon=True).start()
        threading.Thread(target=self._http_loop, args=(self.right_url, False), daemon=True).start()
        self.create_timer(self.period, self._publish_pair)

        self.get_logger().info(
            f"Stereo camera node ready: {self.left_url} / {self.right_url} "
            f"-> /stereo/{{left,right}}/image_raw at {param('rate')} Hz"
        )

    def _camera_info(self, frame_id, intrinsics, tx):
        """Build identity-rectification CameraInfo from one eye's intrinsics."""
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [intrinsics["fx"], 0.0, intrinsics["cx"],
                  0.0, intrinsics["fy"], intrinsics["cy"],
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [intrinsics["fx"], 0.0, intrinsics["cx"], tx,
                  0.0, intrinsics["fy"], intrinsics["cy"], 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def _calibrated_info(self, frame_id, eye):
        """Publish the solved per-eye K/D/R/P from a calibration file."""
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"
        info.d = list(eye["d"])
        info.k = [cell for row in eye["k"] for cell in row]
        info.r = [cell for row in eye["r"] for cell in row]
        info.p = [cell for row in eye["p"] for cell in row]
        return info

    def _http_loop(self, url, is_left):
        while rclpy.ok():
            start = time.time()
            try:
                with urllib.request.urlopen(url, timeout=5.0) as reply:
                    jpeg = reply.read()
                image = PilImage.open(io.BytesIO(jpeg)).convert("RGB")
                if image.size != (self.width, self.height):
                    image = image.resize((self.width, self.height))
                raw = image.tobytes()
                mono = image.convert("L").tobytes()
                with self._lock:
                    if is_left:
                        self._left_latest = (raw, mono, self.get_clock().now().to_msg())
                    else:
                        self._right_latest = (raw, mono, self.get_clock().now().to_msg())
                if self.failures:
                    self.get_logger().info("Camera reachable again")
                self.failures = 0
            except Exception as error:
                self.failures += 1
                if self.failures in (1, 10) or self.failures % 60 == 0:
                    self.get_logger().warn(f"No photo from {url}: {error}")
            time.sleep(max(0.0, self.period - (time.time() - start)))

    def _publish_pair(self):
        with self._lock:
            left = self._left_latest
            right = self._right_latest
            self._left_latest = None
            self._right_latest = None
        if left is None or right is None:
            return
        left_raw, left_mono, left_stamp = left
        right_raw, right_mono, _ = right
        # One stamp for the pair keeps the rectifier and stereo sync happy even
        # though the two HTTP fetches happened at slightly different times.
        stamp = left_stamp
        self._publish_image(self.left_pub, self.left_frame, stamp, "rgb8", 3, left_raw)
        self._publish_image(self.right_pub, self.right_frame, stamp, "rgb8", 3, right_raw)
        self._publish_image(self.left_mono_pub, self.left_frame, stamp, "mono8", 1, left_mono)
        self._publish_image(self.right_mono_pub, self.right_frame, stamp, "mono8", 1, right_mono)

        self.left_info.header.stamp = stamp
        self.right_info.header.stamp = stamp
        self.left_info_pub.publish(self.left_info)
        self.right_info_pub.publish(self.right_info)

        self.photos += 1
        if self.photos == 1:
            self.get_logger().info(f"First stereo pair published ({len(left_raw) // 1024} KB/eye)")

    def _publish_image(self, publisher, frame_id, stamp, encoding, channels, data):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = self.height
        msg.width = self.width
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = self.width * channels
        msg.data = data
        publisher.publish(msg)


def main():
    rclpy.init()
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
