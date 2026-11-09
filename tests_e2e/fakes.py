"""Shared fake robot and vision nodes for the search workflow tests.

FakeRobot integrates /cmd_vel into /odometry/filtered along the commanded
heading, so Mover straight/turn loops close properly.

FakeVision answers /vision/ask from a scripted list (last answer repeats),
echoing the requested target like the real vision node.

FakeVisionPerTarget answers each target differently (counts asks per target),
for multi-target tests.
"""

import json
import math
import threading

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String


class FakeRobot(Node):
    """Integrates /cmd_vel into /odometry/filtered; IR always clear."""

    def __init__(self, node_name="fake_robot"):
        super().__init__(node_name)
        self.odom_pub = self.create_publisher(
            Odometry, "/odometry/filtered", 10)
        self.ir_pub = self.create_publisher(
            Bool, "/ir_obstacle/detected", 10)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.lock = threading.Lock()
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(0.05, self.tick)

    def on_cmd(self, msg):
        with self.lock:
            self.v = float(msg.linear.x)
            self.w = float(msg.angular.z)

    def tick(self):
        with self.lock:
            self.x += self.v * math.cos(self.theta) * 0.05
            self.y += self.v * math.sin(self.theta) * 0.05
            self.theta += self.w * 0.05
            th, x, y = self.theta, self.x, self.y
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = math.sin(th / 2.0)
        msg.pose.pose.orientation.w = math.cos(th / 2.0)
        self.odom_pub.publish(msg)
        self.ir_pub.publish(Bool())


class FakeVision(Node):
    """Answers /vision/ask from a scripted list, echoing the target."""

    def __init__(self, answers, node_name="fake_vision"):
        super().__init__(node_name)
        self.answers = list(answers)
        self.result_pub = self.create_publisher(String, "/vision/result", 10)
        self.create_subscription(String, "/vision/ask", self.on_ask,
                                 qos_profile_sensor_data)

    def on_ask(self, msg):
        answer = (self.answers.pop(0) if len(self.answers) > 1
                  else dict(self.answers[-1]))
        result = {"id": "fake", "target_visible": False, "position": "none",
                  "size": "none", "distance_m": 0.0, "obstacle_ahead": "none",
                  "path_clear": True, "seconds": 0.01}
        result.update(answer)
        self.result_pub.publish(String(data=json.dumps(result)))


class FakeVisionPerTarget(FakeVision):
    """Answers each target from its own script, counting asks per target."""

    def __init__(self, per_target, node_name="fake_vision"):
        super().__init__([{}], node_name)
        self.per_target = {k: list(v) for k, v in per_target.items()}
        self.counts = {}

    def on_ask(self, msg):
        req = json.loads(msg.data)
        target = req.get("target", "")
        self.counts[target] = self.counts.get(target, 0) + 1
        script = self.per_target.get(target, [{}])
        idx = self.counts[target] - 1
        answer = dict(script[idx] if idx < len(script) else script[-1])
        result = {"id": req.get("id", "f"), "target": target,
                  "target_visible": False, "position": "none",
                  "size": "none", "distance_m": 0.0,
                  "obstacle_ahead": "none", "path_clear": True,
                  "seconds": 0.01}
        result.update(answer)
        self.result_pub.publish(String(data=json.dumps(result)))
