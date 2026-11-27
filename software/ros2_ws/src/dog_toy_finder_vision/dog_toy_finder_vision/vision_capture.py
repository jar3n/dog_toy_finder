"""Capture fresh rectified stereo shots with operator-entered ground truth.

Capturing and evaluating are deliberately separate: this node waits for new
left/right rectified-color pairs from a capture-only stereo publisher and writes
them to disk. It is an offline data-collection tool, not part of the live robot
vision processing stack. No detector or depth model runs at capture time, so the
robot can be placed and the target measured without loading a vision model.

Usage inside the ROS 2 container::

    ros2 run dog_toy_finder_vision vision_capture \
        --target "red ball" --depth-m 1.15 --center-offset-m 0.24 --run-id run1

An interactive invocation captures a whole run grouped by toy. It prompts for
a toy and how many scenes contain that toy, then asks for the distance and
signed centre offset of each scene. Each scene gets ``--shots`` fresh pairs
(default 3)::

    <output_dir>/<run_id>/<capture_id>/
        scene.json
        shot_01/left.jpg  shot_01/right.jpg  shot_01/meta.json
        shot_02/left.jpg  shot_02/right.jpg  shot_02/meta.json
        shot_03/left.jpg  shot_03/right.jpg  shot_03/meta.json

After that toy's scenes are saved, the session prompts for another toy. Type
``q``, ``quit``, ``done``, or ``exit`` at the toy, scene-count, measurement, or
capture-ID prompt to finish the session. Scripted invocations that pass
``--depth-m`` plus ``--center-offset-m`` (or ``--bearing-deg``) still capture
one scene and exit.

``--shots 1`` keeps the legacy flat layout with ``left.jpg``, ``right.jpg``,
and ``meta.json`` directly in the scene directory. ``vision_eval`` discovers
either layout and writes one row per model per shot.

``output_dir`` defaults to ``$VISION_CAPTURE_DIR`` when set, otherwise the
repository's ``data/vision/captures`` directory. ``--pilot`` selects the sibling
``data/vision/pilot_captures`` directory instead; an explicit ``--output-dir``
still wins. In a TTY the operator enters, for every scene, the distance along
the normal to the optical plane and the signed offset from the optical-axis
centre (positive = robot-left); the lateral bearing is computed as
``atan(offset / depth)``. These are operator measurements, not estimates from a
detector under test, and are shared by every shot of the scene. The operator
is then asked to add brief notes on the scene (``--notes`` supplies the
default). A suggested ``capture_id`` is built from the toy name and those
measured distances; pressing Enter accepts it, and the operator may type a
different ID. The operator must then press Enter at ``Press Enter to capture
stereo pair`` before any shots are saved. Non-interactive callers get the same
generated ID, and scripts may pass ``--bearing-deg`` directly instead of the
offset.
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


EXIT_COMMANDS = ("q", "quit", "done", "exit")


class CaptureFinished(Exception):
    """Raised when the operator asks to end an interactive capture session."""


def _is_exit_command(value):
    """Return True when ``value`` asks to finish the capture session."""
    return str(value).strip().lower() in EXIT_COMMANDS


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
    """Ask for a measurement, accepting ``default`` on an empty line."""
    suffix = " [%.2f]" % default if default is not None else ""
    while True:
        raw = input("%s%s: " % (label, suffix)).strip()
        if _is_exit_command(raw):
            raise CaptureFinished()
        if not raw and default is not None:
            return float(default)
        try:
            return float(raw)
        except ValueError:
            print("%s must be a number (or q to finish)" % label)


def _prompt_text(label, default=None):
    """Ask for text, accepting ``default`` on an empty line."""
    suffix = " [%s]" % default if default else ""
    raw = input("%s%s: " % (label, suffix)).strip()
    if _is_exit_command(raw):
        raise CaptureFinished()
    return raw or default or ""


def _prompt_required_text(label):
    """Ask for a non-empty line of text, reprompting on blank input."""
    while True:
        value = _prompt_text(label)
        if value:
            return value
        print("%s must not be empty" % label)


def _prompt_int(label, default=None):
    """Ask for a positive integer, accepting ``default`` on an empty line."""
    suffix = " [%d]" % default if default is not None else ""
    while True:
        raw = input("%s%s: " % (label, suffix)).strip()
        if _is_exit_command(raw):
            raise CaptureFinished()
        if not raw and default is not None:
            return int(default)
        try:
            value = int(raw)
        except ValueError:
            print("%s must be a whole number (or q to finish)" % label)
            continue
        if value < 1:
            print("%s must be at least 1 (or q to finish)" % label)
            continue
        return value


def _prompt_toy_group(args):
    """Prompt for a toy and how many visible scenes to capture with it."""
    print("Describe a toy and how many scenes to capture with it (q to finish):")
    toy = _prompt_text("toy", default=args.target)
    count = _prompt_int("scenes with this toy", default=1)
    return toy, count


def _prompt_notes(default=None):
    """Ask for brief scene notes, accepting ``default`` on an empty line."""
    return _prompt_text("Add in brief notes on the scene", default)


def _prompt_capture_ready():
    """Wait for the operator to confirm the scene is ready to capture."""
    raw = input("Press Enter to capture stereo pair (q to finish): ").strip()
    if _is_exit_command(raw):
        raise CaptureFinished()


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
    """Prompt for a capture ID, accepting ``default`` on an empty line.

    An exit command finishes the interactive session instead of saving a scene.
    """
    if interactive is None:
        interactive = sys.stdin.isatty()
    if not interactive:
        return default or datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    if default:
        return _prompt_text("capture_id", default)
    return _prompt_required_text("capture_id")


def _build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", default="red ball",
                        help="default toy for the first interactive group, or "
                             "the toy for a scripted scene")
    parser.add_argument("--depth-m", type=float, default=None,
                        help="distance along the optical axis (normal to the "
                             "optical plane) in metres; an interactive prompt "
                             "offers this as the default")
    parser.add_argument("--center-offset-m", type=float, default=None,
                        help="signed distance from the optical-axis centre in "
                             "the optical plane (metres, +left); an interactive "
                             "prompt offers this as the default")
    parser.add_argument("--bearing-deg", type=float, default=None,
                        help="explicit lateral bearing in degrees, +left; "
                             "mutually exclusive with --center-offset-m")
    parser.add_argument("--not-visible", action="store_true",
                        help="capture an absent target in a scripted run; "
                             "interactive toy groups assume the toy is visible")
    parser.add_argument("--obstacle", default="none",
                        help="operator obstacle label applied to every scene")
    parser.add_argument("--notes", default="",
                        help="default brief scene notes offered by the "
                             "interactive prompt; used directly in scripted "
                             "runs")
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


def _scene_ground_truth(args, interactive, target=None, target_visible=None):
    """Return target, visibility, depth, bearing, and signed centre offset."""
    if args.center_offset_m is not None and args.bearing_deg is not None:
        raise SystemExit("use --center-offset-m or --bearing-deg, not both")

    if target is None:
        target = args.target
    if target_visible is None:
        target_visible = not args.not_visible

    if not target_visible:
        return target, False, 0.0, 0.0, None

    depth = args.depth_m
    center_offset = args.center_offset_m
    bearing = args.bearing_deg
    if interactive:
        print("Enter ground truth measured for this scene (q to finish):")
        depth = _prompt_float("distance_m along optical axis (normal)",
                              depth)
        if center_offset is None and bearing is not None:
            center_offset = depth * math.tan(math.radians(bearing))
        center_offset = _prompt_float(
            "center_offset_m along optical plane (+left)", center_offset)
        bearing = None
    elif depth is None or (center_offset is None and bearing is None):
        raise SystemExit(
            "--depth-m and either --center-offset-m or --bearing-deg "
            "are required for a visible target when not running "
            "interactively")

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
    return target, True, depth, bearing, center_offset


def _prepare_scene(args, config, output_dir, run_id, left_topic, right_topic,
                   interactive, target=None, target_visible=None):
    """Prompt for ground truth and initialise one scene in the capture run."""
    target, target_visible, depth, bearing, center_offset = \
        _scene_ground_truth(args, interactive, target=target,
                            target_visible=target_visible)
    notes = args.notes
    if interactive:
        notes = _prompt_notes(notes)
    default_capture_id = capture_id_from_ground_truth(
        target, depth, center_offset, target_visible)
    capture_id = re.sub(r"[^A-Za-z0-9_.-]+", "_",
                        capture_id_for_run(interactive, default_capture_id))
    if interactive:
        _prompt_capture_ready()

    directory = os.path.join(output_dir, run_id, capture_id)
    os.makedirs(directory, exist_ok=True)
    _clear_previous_shots(directory)

    ground_truth = {
        "target_visible": target_visible,
        "lateral_bearing_deg": round(float(bearing), 2),
        "distance_m": round(float(depth), 3),
        "obstacle_ahead": args.obstacle or "none",
        "notes": notes,
    }
    if target_visible and center_offset is not None:
        ground_truth["center_offset_m"] = round(float(center_offset), 3)
    common_meta = {
        "kind": "vision_capture",
        "capture_id": capture_id,
        "scene_id": capture_id,
        "run_id": run_id,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "target": target,
        "ground_truth": ground_truth,
        "image_source": "rectified_color",
        "image_topics": {"left": left_topic, "right": right_topic},
        "camera": {
            "width": config.get("camera", {}).get("width"),
            "height": config.get("camera", {}).get("height"),
            "baseline_m": config.get("camera", {}).get("baseline_m"),
            "left": vision_config.eye_intrinsics(config, "left"),
            "right": vision_config.eye_intrinsics(config, "right"),
        },
        "config_path": config.get("config_path"),
        "captured_by": "vision_capture",
        "shot_count": args.shots,
    }
    if args.shots > 1:
        scene_meta = dict(common_meta, shot_count=args.shots)
        with open(os.path.join(directory, "scene.json"), "w") as handle:
            json.dump(scene_meta, handle, indent=2)
    return {"capture_id": capture_id, "directory": directory,
            "common_meta": common_meta}


def _capture_scene(node, args, scene, after=None):
    """Save every shot of one scene; return its answers and latest receipt."""
    directory = scene["directory"]
    common_meta = scene["common_meta"]
    left_topic = common_meta["image_topics"]["left"]
    right_topic = common_meta["image_topics"]["right"]
    saved = []
    for index in range(1, args.shots + 1):
        pair = _wait_for_pair(node, args.timeout, after=after)
        if pair is None:
            raise RuntimeError(
                "No fresh matched rectified pair for shot %d/%d of %s on %s / "
                "%s within %.1f s; is stereo_vision.launch.py running?"
                % (index, args.shots, scene["capture_id"], left_topic,
                   right_topic, args.timeout))
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
        print("vision_capture: saved shot %d/%d of %s to %s"
              % (index, args.shots, scene["capture_id"], shot_dir),
              file=sys.stderr)
    return saved, after


def main(argv=None):
    """Capture one labelled scene, or an interactive run of several scenes."""
    args = _build_parser().parse_args(argv)
    if args.shots < 1:
        raise SystemExit("--shots must be at least 1")
    config = vision_config.load_config(args.config)
    output_dir = os.path.abspath(os.path.expanduser(
        args.output_dir or vision_config.default_capture_dir(pilot=args.pilot)))
    left_topic = args.left_topic or vision_config.get(
        config, "camera", "left_rectified_topic")
    right_topic = args.right_topic or vision_config.get(
        config, "camera", "right_rectified_topic")
    run_id = re.sub(r"[^A-Za-z0-9_.-]+", "_", args.run_id)
    run_dir = os.path.join(output_dir, run_id)
    interactive = sys.stdin.isatty()

    rclpy.init()
    node = StereoCapture(left_topic, right_topic, args.pair_tolerance)
    scenes = []
    after = None
    try:
        while True:
            try:
                if interactive:
                    toy, count = _prompt_toy_group(args)
                    for scene_index in range(1, count + 1):
                        print("Scene %d/%d for %s" % (scene_index, count, toy))
                        scene = _prepare_scene(
                            args, config, output_dir, run_id, left_topic,
                            right_topic, interactive, target=toy,
                            target_visible=True)
                        saved, after = _capture_scene(
                            node, args, scene, after=after)
                        scenes.append({
                            "capture_dir": scene["directory"],
                            "capture_id": scene["capture_id"],
                            "shots": len(saved),
                            "meta": scene["common_meta"],
                            "captures": saved,
                        })
                else:
                    scene = _prepare_scene(args, config, output_dir, run_id,
                                           left_topic, right_topic, interactive)
                    saved, after = _capture_scene(
                        node, args, scene, after=after)
                    scenes.append({
                        "capture_dir": scene["directory"],
                        "capture_id": scene["capture_id"],
                        "shots": len(saved),
                        "meta": scene["common_meta"],
                        "captures": saved,
                    })
                    break
            except CaptureFinished:
                break

        payload = {
            "run_dir": run_dir,
            "scene_count": len(scenes),
            "scenes": scenes,
        }
        if len(scenes) == 1:
            payload.update({
                "capture_dir": scenes[0]["capture_dir"],
                "shots": scenes[0]["shots"],
                "meta": scenes[0]["meta"],
                "captures": scenes[0]["captures"],
            })
        print(json.dumps(payload, indent=2))
        return 0
    except Exception as error:
        print("vision_capture failed: %s" % error, file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
