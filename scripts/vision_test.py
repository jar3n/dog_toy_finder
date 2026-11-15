#!/usr/bin/env python3
"""Simple stereo-vision test (no ROSA, no motion).

Publishes one /vision/ask request for a target and prints the /vision/result
JSON. Run inside the ROS 2 container while stereo.launch.py + vision.launch.py
are running:

    python3 /workspace/scripts/vision_test.py "red ball"
    python3 /workspace/scripts/vision_test.py ""   # describe the scene
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class VisionTester(Node):
    def __init__(self, target):
        super().__init__("vision_test")
        self.target = target
        self.result = None
        self.event = __import__("threading").Event()
        self.pub = self.create_publisher(String, "/vision/ask", 10)
        self.create_subscription(String, "/vision/result", self._on_result,
                                 qos_profile_sensor_data)

    def _on_result(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get("target", "") == self.target:
            self.result = data
            self.event.set()

    def ask(self, timeout=60.0):
        req_id = str(int(time.time() * 1000))
        self.pub.publish(String(data=json.dumps({"id": req_id, "target": self.target})))
        if not self.event.wait(timeout=timeout):
            return None
        return self.result


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "red ball"
    rclpy.init()
    node = VisionTester(target)
    import threading
    ex = rclpy.executors.SingleThreadedExecutor()
    ex.add_node(node)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()
    result = node.ask()
    ex.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    if result is None:
        print("TIMEOUT: no /vision/result received", file=sys.stderr)
        sys.exit(1)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
