"""Shared ROS 2 session used by the vision tools.

The vision node owns the camera and the VLM; these helpers only publish
/vision/ask and collect the matching /vision/result. Keeping the rclpy node
here means the tool descriptions stay readable.
"""

import json
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String


class VisionSession:
    """A spinning node that talks to /vision/ask and /vision/result."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        import rclpy as _rclpy
        from rclpy.node import Node
        self.node = Node("rosa_vision")
        self.ask_pub = self.node.create_publisher(String, "/vision/ask", 10)
        self.node.create_subscription(String, "/vision/result", self._on_result,
                                     10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self._lock = threading.Lock()
        self._pending = None
        self._event = threading.Event()

    def _on_result(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            if self._pending is not None and data.get("id") == self._pending:
                self._last = data
                self._event.set()

    def ask(self, target, timeout=30.0):
        """Ask the vision node about `target` ("" = describe the scene)."""
        req_id = str(int(time.time() * 1000))
        with self._lock:
            self._pending = req_id
            self._last = None
            self._event.clear()
        self.ask_pub.publish(String(data=json.dumps(
            {"id": req_id, "target": target})))
        if not self._event.wait(timeout=timeout):
            with self._lock:
                self._pending = None
            return None
        with self._lock:
            result = self._last
            self._pending = None
        return result

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        rclpy.try_shutdown()


_session = None


def get_vision():
    global _session
    if _session is None:
        _session = VisionSession()
    return _session


def close_vision():
    global _session
    if _session is not None:
        _session.close()
        _session = None
