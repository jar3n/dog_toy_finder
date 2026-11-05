"""Feedback motion primitives shared by test scripts and the ROSA agent.

The Mover subscribes to the fused pose on /odometry/filtered and publishes
Twist commands on /cmd_vel. Every move is closed-loop on the odometry estimate:
the robot stops when the estimate says the requested distance or angle is done.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .transforms import distance_and_bearing, wrap_angle

CMD_TOPIC = "/cmd_vel"
FEEDBACK_TOPIC = "/odometry/filtered"

MAX_SPEED = 0.15          # m/s
MIN_SPEED = 0.05          # m/s, slow enough to stop accurately
MAX_TURN = 0.6            # rad/s
MIN_TURN = 0.3            # rad/s
DIST_TOL = 0.005          # m
ANGLE_TOL = math.radians(1.0)
# The real robot keeps moving briefly after a stop command (Wi-Fi delay,
# estimate delay, and motor spin-down). Stop early by speed x COAST_TIME.
COAST_TIME = 0.06         # s


def yaw_from_quaternion(q) -> float:
    """Extract the yaw angle in radians from a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def local_change(start, end):
    """Motion from start to end in the start frame: forward, left, turn (deg)."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    c, s = math.cos(start[2]), math.sin(start[2])
    return (
        dx * c + dy * s,
        -dx * s + dy * c,
        math.degrees(wrap_angle(end[2] - start[2])),
    )


class Mover(Node):
    """ROS 2 node with closed-loop straight, turn, and go-to moves."""

    def __init__(
        self,
        cmd_topic: str = CMD_TOPIC,
        node_name: str = "mover",
        feedback_topic: str = FEEDBACK_TOPIC,
    ):
        super().__init__(node_name)
        self.lock = threading.Lock()
        self.feedback_topic = feedback_topic
        self.coast_time = COAST_TIME
        self.rates = (0.0, 0.0)
        self.poses = {"odom": None, "wheel": None}

        self.cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(
            Odometry, feedback_topic, self._on_feedback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry, "/wheel/odom", self._on_wheel_odom, qos_profile_sensor_data
        )

    def _store_pose(self, key: str, pose) -> None:
        """Record one pose under a key, protected by the node lock."""
        with self.lock:
            self.poses[key] = (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )

    def _on_feedback(self, msg: Odometry) -> None:
        """Store the fused pose and its forward speed / turn rate."""
        self._store_pose("odom", msg.pose.pose)
        with self.lock:
            self.rates = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_wheel_odom(self, msg: Odometry) -> None:
        """Store the wheel-only pose for comparison."""
        self._store_pose("wheel", msg.pose.pose)

    def pose(self, key: str = "odom"):
        """Return (x, y, theta) for a pose key, or None when not received yet."""
        with self.lock:
            return self.poses.get(key)

    def send(self, forward: float, turn: float) -> None:
        """Publish one Twist command."""
        msg = Twist()
        msg.linear.x = float(forward)
        msg.angular.z = float(turn)
        self.cmd.publish(msg)

    def stop(self) -> None:
        """Publish several zero commands so the Pico watchdog sees the stop."""
        for _ in range(5):
            self.send(0.0, 0.0)
            time.sleep(0.05)

    def wait_for_odom(self, timeout: float = 10.0) -> None:
        """Block until the fused pose arrives; raise RuntimeError on timeout."""
        end = time.time() + timeout
        while self.pose() is None and time.time() < end:
            time.sleep(0.05)
        if self.pose() is None:
            raise RuntimeError(
                "No %s received. Is the bringup launch running and the "
                "micro-ROS agent connected?" % self.feedback_topic
            )

    def lead(self, index: int, direction: float) -> float:
        """Distance or angle the robot will coast after a stop command."""
        with self.lock:
            rate = self.rates[index]
        return max(0.0, direction * rate) * self.coast_time

    def straight(self, distance: float) -> None:
        """Drive forward (positive) or backward (negative) in metres."""
        start = self.pose()
        if start is None:
            return
        x0, y0, theta0 = start
        sign = 1.0 if distance >= 0.0 else -1.0
        deadline = time.time() + abs(distance) / MIN_SPEED * 1.5 + 5.0
        while time.time() < deadline:
            x, y, theta = self.pose()
            along = (x - x0) * math.cos(theta0) + (y - y0) * math.sin(theta0)
            remaining = abs(distance) - sign * along
            if remaining <= DIST_TOL + self.lead(0, sign):
                break
            speed = sign * min(MAX_SPEED, max(MIN_SPEED, 1.5 * remaining))
            turn = max(-0.5, min(0.5, 2.0 * wrap_angle(theta0 - theta)))
            self.send(speed, turn)
            time.sleep(0.05)
        self.stop()

    def turn(self, angle: float) -> None:
        """Turn in place by angle radians (positive = left / counter-clockwise)."""
        start = self.pose()
        if start is None:
            return
        target = wrap_angle(start[2] + angle)
        deadline = time.time() + abs(angle) / MIN_TURN * 1.5 + 5.0
        while time.time() < deadline:
            error = wrap_angle(target - self.pose()[2])
            if abs(error) <= ANGLE_TOL + self.lead(1, math.copysign(1.0, error)):
                break
            rate = math.copysign(
                min(MAX_TURN, max(MIN_TURN, 1.5 * abs(error))), error
            )
            self.send(0.0, rate)
            time.sleep(0.05)
        self.stop()

    def goto(self, goal_x: float, goal_y: float, tolerance: float = 0.05) -> None:
        """Drive to a world-frame point, turning first and correcting on the way."""
        deadline = time.time() + 30.0
        while time.time() < deadline:
            pose = self.pose()
            if pose is None:
                return
            distance, bearing = distance_and_bearing(pose, goal_x, goal_y)
            if distance <= tolerance:
                break
            self.turn(wrap_angle(bearing))
            self.straight(distance)
        self.stop()

    def square(self, side: float, clockwise: bool = False) -> None:
        """Drive a square of the requested side length."""
        for _ in range(4):
            self.straight(side)
            self.turn(-math.pi / 2.0 if clockwise else math.pi / 2.0)
