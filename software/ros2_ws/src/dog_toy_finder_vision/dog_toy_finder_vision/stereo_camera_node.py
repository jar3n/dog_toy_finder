"""Stereo camera node: two ESP32-CAM eyes -> /stereo/{left,right}/image_raw.

Each eye is an ESP32-CAM photo server (firmware/esp_cam). This node downloads
a JPEG from each eye on a timer, decodes it with Pillow (no OpenCV needed on
the Python side), and publishes a raw sensor_msgs/Image plus a CameraInfo
carrying the intrinsics and the stereo baseline. The C++ stereo_image_proc
nodes then turn those into disparity and a point cloud.

Parameters
----------
left_url, right_url : str   ESP32-CAM /capture URLs
rate                : float photos per second (per eye)
width, height       : int   expected image size (must match the ESP32-CAM)
fx, fy, cx, cy      : float intrinsics in pixels
baseline            : float distance between the two optical centres, metres
left_frame, right_frame : str optical frame ids
"""

import io
import threading
import time
import urllib.request

import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class StereoCameraNode(Node):
    def __init__(self):
        super().__init__("stereo_camera_node")

        def param(name):
            return self.get_parameter(name).value

        self.declare_parameter("left_url", "http://esp32cam-left.local/capture")
        self.declare_parameter("right_url", "http://esp32cam-right.local/capture")
        self.declare_parameter("rate", 2.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fx", 500.0)
        self.declare_parameter("fy", 500.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("baseline", 0.08)
        self.declare_parameter("left_frame", "left_camera_optical_frame")
        self.declare_parameter("right_frame", "right_camera_optical_frame")

        self.left_url = str(param("left_url"))
        self.right_url = str(param("right_url"))
        self.period = 1.0 / float(param("rate"))
        self.width = int(param("width"))
        self.height = int(param("height"))
        self.fx = float(param("fx"))
        self.fy = float(param("fy"))
        self.cx = float(param("cx"))
        self.cy = float(param("cy"))
        self.baseline = float(param("baseline"))
        self.left_frame = str(param("left_frame"))
        self.right_frame = str(param("right_frame"))

        self.left_pub = self.create_publisher(Image, "/stereo/left/image_raw",
                                              qos_profile_sensor_data)
        self.right_pub = self.create_publisher(Image, "/stereo/right/image_raw",
                                              qos_profile_sensor_data)
        self.left_info_pub = self.create_publisher(CameraInfo,
                                                  "/stereo/left/camera_info",
                                                  qos_profile_sensor_data)
        self.right_info_pub = self.create_publisher(CameraInfo,
                                                   "/stereo/right/camera_info",
                                                   qos_profile_sensor_data)

        self.left_info = self._camera_info(self.left_frame, self.cx, self.cy, 0.0)
        self.right_info = self._camera_info(self.right_frame, self.cx, self.cy,
                                           -self.fx * self.baseline)

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

    def _camera_info(self, frame_id, cx, cy, tx):
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [self.fx, 0.0, cx,
                  0.0, self.fy, cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, cx, tx,
                  0.0, self.fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
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
                with self._lock:
                    if is_left:
                        self._left_latest = (raw, self.get_clock().now().to_msg())
                    else:
                        self._right_latest = (raw, self.get_clock().now().to_msg())
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
        left_raw, left_stamp = left
        right_raw, right_stamp = right
        stamp = left_stamp

        left_msg = Image()
        left_msg.header.stamp = stamp
        left_msg.header.frame_id = self.left_frame
        left_msg.height = self.height
        left_msg.width = self.width
        left_msg.encoding = "rgb8"
        left_msg.is_bigendian = 0
        left_msg.step = self.width * 3
        left_msg.data = left_raw
        self.left_pub.publish(left_msg)

        right_msg = Image()
        right_msg.header.stamp = stamp
        right_msg.header.frame_id = self.right_frame
        right_msg.height = self.height
        right_msg.width = self.width
        right_msg.encoding = "rgb8"
        right_msg.is_bigendian = 0
        right_msg.step = self.width * 3
        right_msg.data = right_raw
        self.right_pub.publish(right_msg)

        self.left_info.header.stamp = stamp
        self.right_info.header.stamp = stamp
        self.left_info_pub.publish(self.left_info)
        self.right_info_pub.publish(self.right_info)

        self.photos += 1
        if self.photos == 1:
            self.get_logger().info(f"First stereo pair published ({len(left_raw) // 1024} KB/eye)")


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
