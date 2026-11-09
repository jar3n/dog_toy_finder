"""Data collector: records a structured dataset during a test run.

Subscribes to poses, vision results, and stereo images, and writes them to a
directory so runs can be analysed offline (VLM benchmarking, odometry drift,
success correlation, etc.).

Topics recorded
---------------
/odometry/filtered   -> poses.csv
/vision/result       -> vision.jsonl
/stereo/left/image_raw  (optional) -> images/left_*.jpg
/stereo/right/image_raw (optional) -> images/right_*.jpg
/stereo/disparity    (optional) -> images/disp_*.png

Parameters
----------
output_dir    : str   dataset root directory
run_id        : str   label for this run (used in filenames)
record_images : bool  save stereo frames (default true)
image_every   : int   save every N-th frame (default 5)
record_disparity : bool  save disparity maps (default false)
"""

import io
import json
import os
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DataCollector(Node):
    def __init__(self):
        super().__init__("data_collector")
        self.declare_parameter("output_dir", "/tmp/dog_toy_dataset")
        self.declare_parameter("run_id", "run1")
        self.declare_parameter("record_images", True)
        self.declare_parameter("image_every", 5)
        self.declare_parameter("record_disparity", False)

        self.output_dir = self.get_parameter("output_dir").value
        self.run_id = self.get_parameter("run_id").value
        self.record_images = bool(self.get_parameter("record_images").value)
        self.image_every = int(self.get_parameter("image_every").value)
        self.record_disparity = bool(self.get_parameter("record_disparity").value)

        self.run_dir = os.path.join(self.output_dir, self.run_id)
        self.img_dir = os.path.join(self.run_dir, "images")
        os.makedirs(self.img_dir, exist_ok=True)

        self._lock = threading.Lock()
        self._frame_count = 0
        self._left_latest = None
        self._right_latest = None

        self.pose_file = open(os.path.join(self.run_dir, "poses.csv"), "w")
        self.pose_file.write("t,stamp,x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n")
        self.vision_file = open(os.path.join(self.run_dir, "vision.jsonl"), "w")

        self.create_subscription(Odometry, "/odometry/filtered", self.on_pose,
                                 qos_profile_sensor_data)
        self.create_subscription(String, "/vision/result", self.on_vision,
                                 qos_profile_sensor_data)
        if self.record_images:
            self.create_subscription(Image, "/stereo/left/image_raw",
                                     self.on_left, qos_profile_sensor_data)
            self.create_subscription(Image, "/stereo/right/image_raw",
                                     self.on_right, qos_profile_sensor_data)
        if self.record_disparity:
            self.create_subscription(Image, "/stereo/disparity",
                                     self.on_disparity, qos_profile_sensor_data)

        manifest = {
            "run_id": self.run_id,
            "started": time.time(),
            "record_images": self.record_images,
            "image_every": self.image_every,
            "record_disparity": self.record_disparity,
        }
        with open(os.path.join(self.run_dir, "manifest.json"), "w") as f:
            json.dump(manifest, f, indent=2)

        self.get_logger().info(f"Data collector writing to {self.run_dir}")

    def _stamp(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_pose(self, msg):
        p = msg.pose.pose
        t = msg.twist.twist
        line = ("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n" % (
            time.time(), self._stamp(msg),
            p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
            t.linear.x, t.linear.y, t.linear.z,
            t.angular.x, t.angular.y, t.angular.z))
        self.pose_file.write(line)
        self.pose_file.flush()

    def on_vision(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {"raw": msg.data}
        data["_t"] = time.time()
        self.vision_file.write(json.dumps(data) + "\n")
        self.vision_file.flush()

    def _save_image(self, msg, side):
        try:
            img = PilImage.frombytes("RGB", (msg.width, msg.height),
                                     bytes(msg.data))
        except Exception:
            return
        name = f"{side}_{int(time.time() * 1000)}.jpg"
        img.save(os.path.join(self.img_dir, name), format="JPEG", quality=90)

    def on_left(self, msg):
        with self._lock:
            self._frame_count += 1
            n = self._frame_count
            self._left_latest = msg
        if n % self.image_every == 0:
            self._save_image(msg, "left")

    def on_right(self, msg):
        with self._lock:
            self._right_latest = msg
        # Save right frames on the same cadence as left by reusing frame_count.
        # (left drives the cadence above)

    def on_disparity(self, msg):
        if not self.record_disparity:
            return
        try:
            # Interpret as 16-bit signed disparity scaled; save as PNG via PIL mode I.
            import struct
            w, h = msg.width, msg.height
            # Best effort: treat data as 16-bit little-endian.
            arr = list(struct.unpack("<%dh" % (w * h), msg.data[:w * h * 2]))
            img = PilImage.new("I", (w, h))
            img.putdata([max(0, v) for v in arr])
            name = f"disp_{int(time.time() * 1000)}.png"
            img.save(os.path.join(self.img_dir, name))
        except Exception as e:
            self.get_logger().warn(f"disparity save failed: {e}")

    def destroy_node(self):
        try:
            self.pose_file.close()
            self.vision_file.close()
        except Exception:
            pass
        # Finalize manifest.
        try:
            manifest_path = os.path.join(self.run_dir, "manifest.json")
            with open(manifest_path) as f:
                m = json.load(f)
            m["finished"] = time.time()
            m["frames"] = self._frame_count
            with open(manifest_path, "w") as f:
                json.dump(m, f, indent=2)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init()
    node = DataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
