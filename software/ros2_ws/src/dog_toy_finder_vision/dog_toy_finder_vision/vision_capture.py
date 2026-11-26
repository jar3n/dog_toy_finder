"""Capture fresh rectified stereo shots with operator-entered ground truth.

Capturing and evaluating are deliberately separate: this node waits for new
left/right rectified-color pairs from the running stereo pipeline and writes
them to disk. No detector runs at capture time, so the robot can be placed
and the target measured without loading a vision model.

Usage inside the ROS 2 container::

    ros2 run dog_toy_finder_vision vision_capture \
        --target "red ball" --depth-m 1.15 --center-offset-m 0.24 --run-id run1

One invocation captures one scene with ``--shots`` fresh pairs (default 3)::

    <output_dir>/<run_id>/<capture_id>/
        scene.json
        shot_01/left.jpg  shot_01/right.jpg  shot_01/meta.json
        shot_02/left.jpg  shot_02/right.jpg  shot_02/meta.json
        shot_03/left.jpg  shot_03/right.jpg  shot_03/meta.json

``--shots 1`` keeps the legacy flat layout with ``left.jpg``, ``right.jpg``,
and ``meta.json`` directly in the scene directory. ``vision_eval`` discovers
either layout and writes one row per model per shot.

``output_dir`` defaults to ``$VISION_CAPTURE_DIR`` when set, otherwise the
repository's ``data/vision/captures`` directory. ``--pilot`` selects the sibling
``data/vision/pilot_captures`` directory instead; an explicit ``--output-dir``
still wins. In a TTY the operator enters
the signed offset from the optical-axis centre in the optical plane (positive =
robot-left) and the distance along the normal to that plane; the lateral bearing
is computed as ``atan(offset / depth)``. These are operator measurements, not
estimates from a detector under test, and are shared by every shot of the
scene. A suggested ``capture_id`` is then built from the toy name and those
measured distances; pressing Enter accepts it, and the operator may type a
different ID. Non-interactive callers get the same generated ID, and scripts
may pass ``--bearing-deg`` directly instead of the offset.
"""

import argparse
import io
import json
import math
import os
import re
import shutil
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
    """Subscribe to rectified stereo and expose fresh matched shot pairs."""

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

    def fresh_pair(self, after=None):
        """Return (left, right, stamps, received_at) for a new matched pair.

        ``after`` is the receipt time of the previous shot, so consecutive
        shots cannot reuse the same camera frames.
        """
        if self._left is None or self._right is None:
            return None
        left_at, left = self._left
        right_at, right = self._right
        if left_at <= self._started or right_at <= self._started:
            return None
        if after is not None and (left_at <= after or right_at <= after):
            return None
        left_stamp, right_stamp = _stamp_seconds(left), _stamp_seconds(right)
        if left_stamp > 0.0 and right_stamp > 0.0:
            if abs(left_stamp - right_stamp) > self._pair_tolerance:
                return None
        return left, right, (left_stamp, right_stamp), max(left_at, right_at)


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


def shot_dir_name(index):
    """Return the directory name for a numbered shot."""
    return "shot_%02d" % int(index)


def _clear_previous_shots(directory):
    """Remove numbered shots left by an earlier capture with the same ID."""
    if not os.path.isdir(directory):
        return
    for name in os.listdir(directory):
        if not re.match(r"^shot_\d+$", name):
            continue
        path = os.path.join(directory, name)
        if os.path.islink(path):
            os.unlink(path)
        elif os.path.isdir(path):
            shutil.rmtree(path)


def _wait_for_pair(node, timeout, after=None):
    """Spin until a fresh matched pair newer than ``after`` arrives."""
    deadline = time.monotonic() + max(0.0, timeout)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        pair = node.fresh_pair(after=after)
        if pair is not None:
            return pair
    return None


def _prompt_float(label, default=None):
    suffix = " [%.2f]" % default if default is not None else ""
    raw = input("%s%s: " % (label, suffix)).strip()
    if not raw and default is not None:
        return float(default)
    return float(raw)


def _prompt_required_text(label):
    """Ask for a non-empty line of text, reprompting on blank input."""
    while True:
        value = input("%s: " % label).strip()
        if value:
            return value
        print("%s must not be empty" % label)


def _id_slug(value):
    """Return a lowercase filesystem-safe slug for an operator label."""
    slug = re.sub(r"[^A-Za-z0-9]+", "_", str(value).strip().lower()).strip("_")
    return slug or "target"


def _id_number(value, signed=False):
    """Render a measurement for an ID: ``.`` becomes ``p``, sign becomes p/m."""
    number = float(value)
    text = ("%.3f" % abs(number)).rstrip("0").rstrip(".") or "0"
    text = text.replace(".", "p")
    if signed:
        return ("m" if number < 0.0 else "p") + text
    return text


def capture_id_from_ground_truth(target, depth, center_offset,
                                 target_visible=True):
    """Build a capture ID from the toy name and measured ground truth."""
    slug = _id_slug(target)
    if not target_visible:
        return "%s_absent" % slug
    depth_token = _id_number(depth)
    offset_token = _id_number(center_offset, signed=True)
    return "%s_d%s_o%s" % (slug, depth_token, offset_token)


def capture_id_for_run(interactive=None, default=None):
    """Prompt for a capture ID, accepting ``default`` on an empty line."""
    if interactive is None:
        interactive = sys.stdin.isatty()
    if not interactive:
        return default or datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    if default:
        value = input("capture_id [%s]: " % default).strip()
        return value or default
    return _prompt_required_text("capture_id")


def _build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", default="red ball",
                        help="operator's description of the toy to measure")
    parser.add_argument("--depth-m", type=float, default=None,
                        help="distance along the optical axis (normal to the "
                             "optical plane) in metres; stored as distance_m")
    parser.add_argument("--center-offset-m", type=float, default=None,
                        help="signed distance from the optical-axis centre in "
                             "the optical plane (metres, +left); lateral "
                             "bearing is computed from this and --depth-m")
    parser.add_argument("--bearing-deg", type=float, default=None,
                        help="explicit lateral bearing in degrees, +left; "
                             "mutually exclusive with --center-offset-m")
    parser.add_argument("--not-visible", action="store_true",
                        help="capture a frame where the target is absent")
    parser.add_argument("--obstacle", default="none",
                        help="optional operator obstacle label in the centre")
    parser.add_argument("--notes", default="", help="free-form operator notes")
    parser.add_argument("--shots", type=int, default=3,
                        help="fresh stereo pairs to capture for this scene "
                             "(default: 3; 1 keeps the legacy flat layout)")
    parser.add_argument("--run-id", default=datetime.now().strftime("run_%Y%m%d"),
                        help="dataset run label (a directory below output-dir)")
    parser.add_argument("--output-dir", default=None,
                        help="capture root (default: %s, or %s with --pilot)"
                             % (vision_config.default_capture_dir(),
                                vision_config.default_capture_dir(pilot=True)))
    parser.add_argument("--pilot", action="store_true",
                        help="use the pilot capture root next to captures "
                             "(an explicit --output-dir wins)")
    parser.add_argument("--config", default=None, help="detector YAML path")
    parser.add_argument("--left-topic", default=None,
                        help="override the left rectified-color topic")
    parser.add_argument("--right-topic", default=None,
                        help="override the right rectified-color topic")
    parser.add_argument("--pair-tolerance", type=float, default=0.5,
                        help="maximum timestamp difference between eyes (s)")
    parser.add_argument("--timeout", type=float, default=20.0,
                        help="how long to wait for each fresh stereo pair (s)")
    return parser


def main(argv=None):
    """Capture one labelled scene with one or more fresh stereo shots."""
    args = _build_parser().parse_args(argv)
    if args.shots < 1:
        raise SystemExit("--shots must be at least 1")
    config = vision_config.load_config(args.config)
    output_dir = os.path.abspath(os.path.expanduser(
        args.output_dir or vision_config.default_capture_dir(pilot=args.pilot)))

    interactive = sys.stdin.isatty()
    if interactive and not args.not_visible:
        print("Enter ground truth measured for this scene:")

    if args.not_visible:
        target_visible = False
        depth = 0.0
        bearing = 0.0
        center_offset = None
    else:
        target_visible = True
        depth = args.depth_m
        center_offset = args.center_offset_m
        bearing = args.bearing_deg
        if center_offset is not None and bearing is not None:
            raise SystemExit(
                "use --center-offset-m or --bearing-deg, not both")
        if depth is None or (center_offset is None and bearing is None):
            if not interactive:
                raise SystemExit(
                    "--depth-m and either --center-offset-m or --bearing-deg "
                    "are required for a visible target when not running "
                    "interactively")
            if center_offset is None and bearing is None:
                center_offset = _prompt_float(
                    "center_offset_m along optical plane (+left)")
            if depth is None:
                depth = _prompt_float(
                    "distance_m along optical axis (normal)")
        if depth is None or depth < 0.0:
            raise SystemExit("--depth-m must be non-negative")
        if center_offset is not None:
            bearing = vision_config.bearing_deg_from_center_offset(
                center_offset, depth)
        else:
            bearing = float(bearing)
        if abs(bearing) > 90.0:
            raise SystemExit("--bearing-deg must be within +/-90")
        if center_offset is None:
            center_offset = depth * math.tan(math.radians(bearing))

    default_capture_id = capture_id_from_ground_truth(
        args.target, depth, center_offset, target_visible)
    if interactive:
        print("Enter capture details for this scene:")
    capture_id = re.sub(r"[^A-Za-z0-9_.-]+", "_",
                        capture_id_for_run(interactive, default_capture_id))

    left_topic = args.left_topic or vision_config.get(
        config, "camera", "left_rectified_topic")
    right_topic = args.right_topic or vision_config.get(
        config, "camera", "right_rectified_topic")
    run_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", args.run_id)
    directory = os.path.join(output_dir, run_id, capture_id)
    os.makedirs(directory, exist_ok=True)
    _clear_previous_shots(directory)

    ground_truth = {
        "target_visible": target_visible,
        "lateral_bearing_deg": round(float(bearing), 2),
        "distance_m": round(float(depth), 3),
        "obstacle_ahead": args.obstacle or "none",
        "notes": args.notes,
    }
    if target_visible and center_offset is not None:
        ground_truth["center_offset_m"] = round(float(center_offset), 3)
    common_meta = {
        "kind": "vision_capture",
        "capture_id": capture_id,
        "scene_id": capture_id,
        "run_id": run_id,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "target": args.target,
        "ground_truth": ground_truth,
        "image_source": "rectified_color",
        "image_topics": {"left": left_topic, "right": right_topic},
        "camera": {
            key: config.get("camera", {}).get(key)
            for key in ("width", "height", "fx", "fy", "cx", "cy",
                        "baseline_m")
        },
        "config_path": config.get("config_path"),
        "captured_by": "vision_capture",
        "shot_count": args.shots,
    }
    if args.shots > 1:
        scene_meta = dict(common_meta, shot_count=args.shots)
        with open(os.path.join(directory, "scene.json"), "w") as handle:
            json.dump(scene_meta, handle, indent=2)

    rclpy.init()
    node = StereoCapture(left_topic, right_topic, args.pair_tolerance)
    saved = []
    try:
        after = None
        for index in range(1, args.shots + 1):
            pair = _wait_for_pair(node, args.timeout, after=after)
            if pair is None:
                raise RuntimeError(
                    "No fresh matched rectified pair for shot %d/%d on %s / %s "
                    "within %.1f s; is stereo_vision.launch.py running?"
                    % (index, args.shots, left_topic, right_topic,
                       args.timeout))
            left, right, stamps, received_at = pair
            shot_meta = dict(common_meta, shot_index=index)
            shot_meta["stamps_s"] = {
                "left": stamps[0], "right": stamps[1]}
            if args.shots > 1:
                shot_dir = os.path.join(directory, shot_dir_name(index))
            else:
                shot_dir = directory
            write_capture(shot_dir, left, right, shot_meta)
            saved.append({"shot_dir": shot_dir, "meta": shot_meta})
            after = received_at
            print("vision_capture: saved shot %d/%d to %s"
                  % (index, args.shots, shot_dir), file=sys.stderr)
    except Exception as error:
        print("vision_capture failed: %s" % error, file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps({
        "capture_dir": directory,
        "shots": len(saved),
        "meta": common_meta,
        "captures": saved,
    }, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
