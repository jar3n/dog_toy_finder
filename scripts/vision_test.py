#!/usr/bin/env python3
"""Simple stereo-vision test (no ROSA, no motion).

Publishes one /vision/ask request for a target and prints the /vision/result
JSON. Run inside the ROS 2 container while stereo_vision.launch.py is
running:

    python3 /workspace/scripts/vision_test.py "red ball"
    python3 /workspace/scripts/vision_test.py ""   # describe the scene
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisionTester(Node):
    def __init__(self, target):
        super().__init__("vision_test")
        self.target = target
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
        payload = json.dumps({"id": req_id, "target": self.target})
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


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "red ball"
    rclpy.init()
    node = VisionTester(target)
    try:
        result = node.ask()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if result is None:
        print("TIMEOUT: no /vision/result received", file=sys.stderr)
        sys.exit(1)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
