#!/usr/bin/env python3
"""Simple stereo-vision test (no ROSA, no motion).

Publishes one /vision/ask request for a target and prints the /vision/result
JSON. Run inside the ROS 2 container while stereo_vision.launch.py is
running:

    python3 /workspace/scripts/vision_test.py "red ball"
    python3 /workspace/scripts/vision_test.py ""   # describe the scene

Pass --gt-json to attach ground truth to the request so the LLM trial
collector fills the gt_* columns instead of leaving them for manual entry:

    python3 /workspace/scripts/vision_test.py "red ball" --gt-json gt.json

The JSON object uses the short keys ``target_visible``, ``position``,
``lateral_bearing_deg``, ``size``, ``distance_m``, ``obstacle_ahead``, and
``notes``. Bearing is in degrees from the optical axis, positive to the
robot's left. ``distance_m`` may be a number or a list of repeated
measurements, e.g. ``[0.7, 0.72]``; score uses their mean. ``bearing_deg`` /
``lateral_bearing`` and ``distances_m`` are accepted as aliases.
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisionTester(Node):
    def __init__(self, target, ground_truth=None):
        super().__init__("vision_test")
        self.target = target
        self.ground_truth = ground_truth
        self.result = None
        self.pub = self.create_publisher(String, "/vision/ask", 10)
        self.create_subscription(String, "/vision/result", self._on_result, 10)

    def _on_result(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get("target", "") == self.target:
            self.result = data

    def ask(self, timeout=120.0):
        req_id = str(int(time.time() * 1000))
        payload = {"id": req_id, "target": self.target}
        if self.ground_truth is not None:
            payload["ground_truth"] = self.ground_truth
        payload = json.dumps(payload)
        deadline = time.time() + timeout
        last_send = 0.0
        while time.time() < deadline and rclpy.ok():
            if self.result is not None:
                return self.result
            # Wait for discovery before the first publish: a request sent
            # before the vision node has matched the publisher is dropped.
            # Resend sparingly in case the first one was lost.
            if (self.pub.get_subscription_count() > 0
                    and time.time() - last_send >= 20.0):
                self.pub.publish(String(data=payload))
                if last_send == 0.0:
                    print("vision_node found; waiting for /vision/result ...",
                          file=sys.stderr, flush=True)
                last_send = time.time()
            rclpy.spin_once(self, timeout_sec=0.2)
        return None


def parse_args(argv=None):
    """Parse the target and optional ground-truth file."""
    parser = argparse.ArgumentParser(description="Ask the vision node once")
    parser.add_argument("target", nargs="?", default="red ball",
                        help='toy description, or "" to describe the scene')
    parser.add_argument("--gt-json", "--gt", dest="gt_json", default=None,
                        help="JSON file with ground truth for this photo")
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="seconds to wait for /vision/result")
    return parser.parse_args(argv)


def main():
    args = parse_args()
    ground_truth = None
    if args.gt_json:
        try:
            with open(args.gt_json) as handle:
                ground_truth = json.load(handle)
        except (OSError, json.JSONDecodeError) as error:
            print("Could not read --gt-json: %s" % error, file=sys.stderr)
            sys.exit(2)
        if not isinstance(ground_truth, dict):
            print("--gt-json must contain a JSON object", file=sys.stderr)
            sys.exit(2)
    rclpy.init()
    node = VisionTester(args.target, ground_truth)
    try:
        result = node.ask(timeout=args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if result is None:
        print("TIMEOUT: no /vision/result received", file=sys.stderr)
        sys.exit(1)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
