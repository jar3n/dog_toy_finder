"""Feedback motion primitives shared by test scripts and the ROSA agent.

The Mover subscribes only to the fused pose on /odometry/filtered and publishes
Twist commands on /cmd_vel_auto. The cmd_vel_mux node forwards one source at a
time to the Pico on /cmd_vel, so manual drive can take exclusive control. Every
move is closed-loop on the EKF estimate: the robot stops when the estimate says
the requested distance or angle is done. Raw /wheel/odom feeds the EKF but is
never used as motion feedback, and a non-zero command is refused when the EKF
estimate is missing or stale.

The Mover also subscribes to the Pico IR sensor on /ir_obstacle/detected. The
Pico runs a local reflex failsafe: it brakes and blocks forward/turn commands
while an obstacle is within range, allowing only stepped reverse. Because the
reflex and this topic share the same debounced sensor, ROS knows the failsafe
engaged when /ir_obstacle/detected is true during a move. Forward moves and
turns abort promptly in that case (instead of driving against the brake until
the move deadline), and ``failsafe_hit()`` reports that it happened.
"""

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

from .feedback_gate import (
    FEEDBACK_TIMEOUT,
    FEEDBACK_TOPIC,
    FeedbackGate,
    validate_feedback_topic,
)
from .transforms import distance_and_bearing, wrap_angle

# Autonomous commands go to the mux input, never straight to the Pico.
# cmd_vel_mux owns /cmd_vel so a manual driver can override without two
# publishers fighting over the motors.
CMD_TOPIC = "/cmd_vel_auto"
IR_TOPIC = "/ir_obstacle/detected"

MAX_SPEED = 1.0          # m/s
MIN_SPEED = 0.10          # m/s, safely above the measured ~0.05 m/s motor dead band
MAX_TURN = 2.0            # rad/s
MIN_TURN = 0.7            # rad/s
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
        ir_topic: str = IR_TOPIC,
        feedback_timeout: float = FEEDBACK_TIMEOUT,
    ):
        # Validate before creating the node so a bad topic cannot leave a
        # half-initialised ROS node behind.
        validate_feedback_topic(feedback_topic)
        super().__init__(node_name)
        self.lock = threading.Lock()
        self.feedback_topic = feedback_topic
        self.ir_topic = ir_topic
        self.feedback_gate = FeedbackGate(timeout=feedback_timeout)
        self.coast_time = COAST_TIME
        self.rates = (0.0, 0.0)
        self.poses = {"odom": None}
        # Latest Pico IR state and whether the failsafe interrupted a move.
        # False until the first /ir_obstacle/detected arrives, so a missing
        # sensor degrades to the old behaviour instead of blocking every move.
        self._obstacle_ahead = False
        self._failsafe_hit = False

        self.cmd = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(
            Odometry, feedback_topic, self._on_feedback, qos_profile_sensor_data
        )
        self.create_subscription(
            Bool, ir_topic, self._on_ir, qos_profile_sensor_data
        )

    def _on_feedback(self, msg: Odometry) -> None:
        """Store the EKF pose, speed, and arrival time."""
        with self.lock:
            self.poses["odom"] = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw_from_quaternion(msg.pose.pose.orientation),
            )
            self.rates = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
            self.feedback_gate.update()

    def _on_ir(self, msg: Bool) -> None:
        """Store the Pico IR obstacle state (the failsafe signal)."""
        with self.lock:
            self._obstacle_ahead = bool(msg.data)

    def obstacle_ahead(self) -> bool:
        """Return the latest IR obstacle state from the Pico, or False."""
        with self.lock:
            return self._obstacle_ahead

    def failsafe_hit(self) -> bool:
        """Return true when the Pico failsafe interrupted the last move."""
        with self.lock:
            return self._failsafe_hit

    def clear_failsafe(self) -> None:
        """Forget that the failsafe hit; called at the start of every move."""
        with self.lock:
            self._failsafe_hit = False

    def _note_failsafe(self, what: str) -> None:
        """Latch the failsafe flag and warn, so ROS knows the Pico stopped us."""
        with self.lock:
            self._failsafe_hit = True
        self.get_logger().warn(
            "Pico failsafe: IR obstacle ahead during %s; move aborted. "
            "Driver must reverse in small steps or wait for the path to clear."
            % what
        )

    def feedback_fresh(self) -> bool:
        """Return True when a recent EKF pose is available for a move."""
        return self.pose() is not None and self.feedback_gate.fresh()

    def pose(self, key: str = "odom"):
        """Return the fused (x, y, theta), or None before the first EKF pose."""
        if key != "odom":
            raise ValueError("only the fused EKF pose 'odom' is available")
        with self.lock:
            return self.poses.get(key)

    def send(self, forward: float, turn: float) -> None:
        """Publish one Twist command; non-zero motion requires fresh EKF data."""
        if (forward != 0.0 or turn != 0.0) and not self.feedback_fresh():
            raise RuntimeError(
                "Refusing motion: no fresh %s (EKF) feedback. Is bringup "
                "running and is the micro-ROS agent connected?" % FEEDBACK_TOPIC
            )
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
        """Block until fresh fused odometry arrives; raise RuntimeError otherwise."""
        end = time.time() + timeout
        while not self.feedback_fresh() and time.time() < end:
            time.sleep(0.05)
        if not self.feedback_fresh():
            raise RuntimeError(
                "No fresh %s (EKF) received. Is the bringup launch running "
                "and the micro-ROS agent connected?" % self.feedback_topic
            )

    def _abort_for_stale_feedback(self, what: str) -> None:
        """Warn that the EKF stopped updating and a move was aborted."""
        self.get_logger().warn(
            "No fresh %s (EKF) feedback; aborted %s." % (FEEDBACK_TOPIC, what)
        )

    def lead(self, index: int, direction: float) -> float:
        """Distance or angle the robot will coast after a stop command."""
        with self.lock:
            rate = self.rates[index]
        return max(0.0, direction * rate) * self.coast_time

    def straight(self, distance: float) -> None:
        """Drive forward (positive) or backward (negative) in metres.

        Forward moves abort as soon as the Pico IR failsafe reports an
        obstacle ahead. Backward moves do not abort on the IR signal: the Pico
        allows stepped reverse, so the move can still make progress.
        """
        self.clear_failsafe()
        if not self.feedback_fresh():
            self._abort_for_stale_feedback("straight")
            return
        start = self.pose()
        x0, y0, theta0 = start
        sign = 1.0 if distance >= 0.0 else -1.0
        deadline = time.time() + abs(distance) / MIN_SPEED * 1.5 + 5.0
        while time.time() < deadline:
            if not self.feedback_fresh():
                self._abort_for_stale_feedback("straight")
                break
            if sign > 0.0 and self.obstacle_ahead():
                self._note_failsafe("forward straight")
                break
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
        """Turn in place by angle radians (positive = left / counter-clockwise).

        Turns abort as soon as the Pico IR failsafe reports an obstacle
        ahead: the Pico blocks turn-in-place while blocked (except during the
        pause between reverse steps), so continuing would only drive against
        the brake until the deadline.
        """
        self.clear_failsafe()
        if not self.feedback_fresh():
            self._abort_for_stale_feedback("turn")
            return
        start = self.pose()
        target = wrap_angle(start[2] + angle)
        deadline = time.time() + abs(angle) / MIN_TURN * 1.5 + 5.0
        while time.time() < deadline:
            if not self.feedback_fresh():
                self._abort_for_stale_feedback("turn")
                break
            if self.obstacle_ahead():
                self._note_failsafe("turn")
                break
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
        self.clear_failsafe()
        deadline = time.time() + 30.0
        while time.time() < deadline:
            if self.failsafe_hit():
                break
            if not self.feedback_fresh():
                self._abort_for_stale_feedback("goto")
                break
            pose = self.pose()
            distance, bearing = distance_and_bearing(pose, goal_x, goal_y)
            if distance <= tolerance:
                break
            self.turn(wrap_angle(bearing))
            if self.failsafe_hit():
                break
            self.straight(distance)
        self.stop()

    def square(self, side: float, clockwise: bool = False) -> None:
        """Drive a square of the requested side length."""
        for _ in range(4):
            self.straight(side)
            self.turn(-math.pi / 2.0 if clockwise else math.pi / 2.0)
