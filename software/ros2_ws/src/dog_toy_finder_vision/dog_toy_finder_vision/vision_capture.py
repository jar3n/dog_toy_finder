"""Capture one fresh rectified stereo pair with operator-entered ground truth.

Capturing and evaluating are deliberately separate: this node only waits for a
new left/right rectified-color pair from the running stereo pipeline and writes
it to disk. No model runs at capture time, so the robot can be positioned and
the target measured without Ollama loaded.

Usage inside the ROS 2 container::

    ros2 run dog_toy_finder_vision vision_capture \
        --target "red ball" --depth-m 1.15 --bearing-deg +12 --run-id run1

Each capture is stored as::

    <output_dir>/<run_id>/<capture_id>/
        left.jpg  right.jpg  meta.json

``output_dir`` defaults to ``$VISION_CAPTURE_DIR`` when set, otherwise the
repository's ``data/vision/captures`` directory. The ground-truth depth in
metres and lateral bearing in degrees (positive = robot-left) are entered by
the operator, not estimated by a detector under test.
"""

import argparse
import io
import json
import os
import re
import sys
import time
from datetime import datetime

import rclpy
from PIL import Image as PilImage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from . import vision_config


def image_to_jpeg(msg):
    """Convert a sensor_msgs/Image rgb8/bgr8 message to JPEG bytes."""
    modes = {"rgb8": "RGB", "rgb": "RGB", "bgr8": "BGR"}
    mode = modes.get(msg.encoding)
    if mode is None:
        raise ValueError("vision_capture needs rgb8/bgr8 images, got %s"
                         % msg.encoding)
    expected = msg.width * msg.height * 3
    if len(msg.data) != expected:
        raise ValueError("image has %d bytes, expected %d for %dx%d"
                         % (len(msg.data), expected, msg.width, msg.height))
    image = PilImage.frombytes(mode, (msg.width, msg.height), bytes(msg.data))
    buffer = io.BytesIO()
    image.convert("RGB").save(buffer, format="JPEG", quality=90)
    return buffer.getvalue(), (msg.width, msg.height)


def _stamp_seconds(msg):
    stamp = msg.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class StereoCapture(Node):
    """Subscribe to rectified stereo and expose the first fresh matched pair."""

    def __init__(self, left_topic, right_topic, pair_tolerance_s=0.5):
        super().__init__("vision_capture")
        self._started = time.monotonic()
        self._pair_tolerance = float(pair_tolerance_s)
        self._left = None
        self._right = None
        self.create_subscription(Image, left_topic, self.on_left,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, right_topic, self.on_right,
                                 qos_profile_sensor_data)

    def on_left(self, msg):
        self._left = (time.monotonic(), msg)

    def on_right(self, msg):
        self._right = (time.monotonic(), msg)

    def fresh_pair(self):
        """Return (left, right, stamps) once both eyes are new and matched."""
        if self._left is None or self._right is None:
            return None
        left_at, left = self._left
        right_at, right = self._right
        if left_at <= self._started or right_at <= self._started:
            return None
        left_stamp, right_stamp = _stamp_seconds(left), _stamp_seconds(right)
        if left_stamp > 0.0 and right_stamp > 0.0:
            if abs(left_stamp - right_stamp) > self._pair_tolerance:
                return None
        return left, right, (left_stamp, right_stamp)


def write_capture(directory, left, right, meta):
    """Write both rectified JPEGs and the metadata; return the metadata."""
    os.makedirs(directory, exist_ok=True)
    left_jpeg, left_size = image_to_jpeg(left)
    right_jpeg, right_size = image_to_jpeg(right)
    with open(os.path.join(directory, "left.jpg"), "wb") as handle:
        handle.write(left_jpeg)
    with open(os.path.join(directory, "right.jpg"), "wb") as handle:
        handle.write(right_jpeg)
    meta["images"] = {"left": "left.jpg", "right": "right.jpg",
                      "size_px": [list(left_size), list(right_size)]}
    with open(os.path.join(directory, "meta.json"), "w") as handle:
        json.dump(meta, handle, indent=2)
    return meta


def _prompt_float(label, default=None):
    suffix = " [%.2f]" % default if default is not None else ""
    raw = input("%s%s: " % (label, suffix)).strip()
    if not raw and default is not None:
        return float(default)
    return float(raw)


def _build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", default="red ball",
                        help="operator's description of the toy to measure")
    parser.add_argument("--depth-m", type=float, default=None,
                        help="ground-truth distance from the camera in metres")
    parser.add_argument("--bearing-deg", type=float, default=None,
                        help="ground-truth lateral bearing in degrees, +left")
    parser.add_argument("--not-visible", action="store_true",
                        help="capture a frame where the target is absent")
    parser.add_argument("--size", default="",
                        help="optional operator size label: small/medium/large")
    parser.add_argument("--obstacle", default="none",
                        help="optional operator obstacle label in the centre")
    parser.add_argument("--notes", default="", help="free-form operator notes")
    parser.add_argument("--run-id", default=datetime.now().strftime("run_%Y%m%d"),
                        help="dataset run label (a directory below output-dir)")
    parser.add_argument("--capture-id", default=None,
                        help="capture label (default: timestamp)")
    parser.add_argument("--output-dir", default=None,
                        help="capture root (default: %s)" % vision_config.default_capture_dir())
    parser.add_argument("--config", default=None, help="detector YAML path")
    parser.add_argument("--left-topic", default=None,
                        help="override the left rectified-color topic")
    parser.add_argument("--right-topic", default=None,
                        help="override the right rectified-color topic")
    parser.add_argument("--pair-tolerance", type=float, default=0.5,
                        help="maximum timestamp difference between eyes (s)")
    parser.add_argument("--timeout", type=float, default=20.0,
                        help="how long to wait for a fresh stereo pair (s)")
    return parser


def main(argv=None):
    """Capture one labelled rectified stereo pair."""
    args = _build_parser().parse_args(argv)
    config = vision_config.load_config(args.config)
    output_dir = os.path.abspath(os.path.expanduser(
        args.output_dir or vision_config.default_capture_dir()))

    if args.not_visible:
        target_visible = False
        depth = 0.0
        bearing = 0.0
    else:
        target_visible = True
        depth = args.depth_m
        bearing = args.bearing_deg
        if depth is None or bearing is None:
            if not sys.stdin.isatty():
                raise SystemExit(
                    "--depth-m and --bearing-deg are required for a visible "
                    "target when not running interactively")
            print("Enter ground truth measured for this capture:")
            if depth is None:
                depth = _prompt_float("distance_m (metres)")
            if bearing is None:
                bearing = _prompt_float("lateral_bearing_deg (+left)")
        if depth < 0.0:
            raise SystemExit("--depth-m must be non-negative")
        if abs(bearing) > 90.0:
            raise SystemExit("--bearing-deg must be within +/-90")

    left_topic = args.left_topic or vision_config.get(
        config, "camera", "left_rectified_topic")
    right_topic = args.right_topic or vision_config.get(
        config, "camera", "right_rectified_topic")
    capture_id = args.capture_id or datetime.now().strftime(
        "%Y%m%d_%H%M%S_%f")[:-3]
    capture_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", capture_id)
    run_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", args.run_id)
    directory = os.path.join(output_dir, run_id, capture_id)

    rclpy.init()
    node = StereoCapture(left_topic, right_topic, args.pair_tolerance)
    try:
        deadline = time.monotonic() + max(0.0, args.timeout)
        pair = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            pair = node.fresh_pair()
            if pair is not None:
                break
        if pair is None:
            raise RuntimeError(
                "No fresh matched rectified pair on %s / %s within %.1f s; "
                "is stereo_vision.launch.py running?"
                % (left_topic, right_topic, args.timeout))
        left, right, stamps = pair
        meta = {
            "kind": "vision_capture",
            "capture_id": capture_id,
            "run_id": run_id,
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "target": args.target,
            "ground_truth": {
                "target_visible": target_visible,
                "position": vision_config.position_from_bearing(bearing, config)
                if target_visible else "none",
                "lateral_bearing_deg": round(float(bearing), 2),
                "distance_m": round(float(depth), 3),
                "size": args.size or "none",
                "obstacle_ahead": args.obstacle or "none",
                "notes": args.notes,
            },
            "image_source": "rectified_color",
            "image_topics": {"left": left_topic, "right": right_topic},
            "stamps_s": {"left": stamps[0], "right": stamps[1]},
            "camera": {
                key: config.get("camera", {}).get(key)
                for key in ("width", "height", "fx", "fy", "cx", "cy",
                            "baseline_m")
            },
            "config_path": config.get("config_path"),
            "captured_by": "vision_capture",
        }
        write_capture(directory, left, right, meta)
    except Exception as error:
        print("vision_capture failed: %s" % error, file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps({"capture_dir": directory, "meta": meta}, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
