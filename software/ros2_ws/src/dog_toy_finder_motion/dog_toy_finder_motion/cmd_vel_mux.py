"""Arbitrate autonomous and manual velocity commands onto the Pico's topic.

The Pico subscribes to ``/cmd_vel`` and is the only consumer of drive commands.
Several ROS 2 nodes can produce commands (the autonomous search planner, ROSA's
``Mover``, test moves, and the keyboard manual drive), and if they publish to
``/cmd_vel`` directly the last writer wins. During a manual drive that is a
real conflict: an autonomous node can overwrite the human's command, or the
human can fight an in-progress move.

This node owns the single publisher on ``/cmd_vel``:

    /cmd_vel_manual  --\
                        >-- cmd_vel_mux --> /cmd_vel --> Pico
    /cmd_vel_auto    --/

Manual commands always win while they are fresh (``manual_timeout``). When the
manual source goes quiet, the mux holds zero for ``release_timeout`` before it
lets autonomous commands through again, so losing the manual link does not
hand control back mid-motion. If neither source is publishing, the mux keeps
sending zero at ``publish_rate`` so the Pico's 500 ms command watchdog stays
satisfied and the robot cannot keep the last command.

Run it with the bringup launch (which starts one instance):

    ros2 run dog_toy_finder_motion cmd_vel_mux

``/cmd_vel_source`` reports the current winner: ``manual``, ``auto``,
``released`` (manual just stopped), or ``none``.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

MANUAL_TOPIC = "/cmd_vel_manual"
AUTO_TOPIC = "/cmd_vel_auto"
OUTPUT_TOPIC = "/cmd_vel"
SOURCE_TOPIC = "/cmd_vel_source"

SOURCE_MANUAL = "manual"
SOURCE_AUTO = "auto"
SOURCE_RELEASED = "released"
SOURCE_NONE = "none"

DEFAULT_PUBLISH_RATE = 20.0      # Hz, keeps the Pico watchdog fed
DEFAULT_MANUAL_TIMEOUT = 0.5     # s without a manual command before it expires
DEFAULT_AUTO_TIMEOUT = 0.5       # s without an auto command before it expires
DEFAULT_RELEASE_TIMEOUT = 0.5    # s of zero after manual control ends


class CommandSelector:
    """Pure source-selection logic for the velocity mux.

    Keeping the timing rules out of the ROS node makes the priority and
    timeout behaviour directly testable. All times are seconds from any
    monotonic clock; the caller passes ``now`` so tests control it.
    """

    def __init__(
        self,
        manual_timeout: float = DEFAULT_MANUAL_TIMEOUT,
        auto_timeout: float = DEFAULT_AUTO_TIMEOUT,
        release_timeout: float = DEFAULT_RELEASE_TIMEOUT,
    ):
        """Store the timeouts and start with no command from either source."""
        self.manual_timeout = max(0.0, float(manual_timeout))
        self.auto_timeout = max(0.0, float(auto_timeout))
        self.release_timeout = max(0.0, float(release_timeout))

        self.manual = (0.0, 0.0)
        self.auto = (0.0, 0.0)
        self.last_manual = None
        self.last_auto = None
        self.manual_active = False
        self.release_until = None

    def update_manual(
        self, linear: float, angular: float, now: float = None
    ) -> None:
        """Record the latest manual command."""
        if now is None:
            now = time.monotonic()
        self.manual = (float(linear), float(angular))
        self.last_manual = now

    def update_auto(self, linear: float, angular: float, now: float = None) -> None:
        """Record the latest autonomous command."""
        if now is None:
            now = time.monotonic()
        self.auto = (float(linear), float(angular))
        self.last_auto = now

    def select(self, now: float = None):
        """Return ``(source, linear, angular)`` for this instant.

        Manual wins while fresh. After it expires the output is zero for
        ``release_timeout`` seconds, so autonomous motion cannot resume in the
        same instant the human stopped. Autonomous commands are used only when
        they are fresh themselves; otherwise the output is zero.
        """
        if now is None:
            now = time.monotonic()

        manual_fresh = (
            self.last_manual is not None
            and (now - self.last_manual) <= self.manual_timeout
        )
        if manual_fresh:
            self.manual_active = True
            self.release_until = None
            return (SOURCE_MANUAL,) + self.manual

        if self.manual_active:
            # Manual just went quiet: insert a short hands-off gap.
            self.manual_active = False
            self.release_until = now + self.release_timeout

        if self.release_until is not None and now < self.release_until:
            return (SOURCE_RELEASED, 0.0, 0.0)
        self.release_until = None

        auto_fresh = (
            self.last_auto is not None
            and (now - self.last_auto) <= self.auto_timeout
        )
        if auto_fresh:
            return (SOURCE_AUTO,) + self.auto

        return (SOURCE_NONE, 0.0, 0.0)


class CmdVelMux(Node):
    """Publish exactly one command source on /cmd_vel with manual priority."""

    def __init__(self):
        """Read parameters, subscribe to both sources, and start the timer."""
        super().__init__("cmd_vel_mux")

        self.declare_parameter("manual_topic", MANUAL_TOPIC)
        self.declare_parameter("auto_topic", AUTO_TOPIC)
        self.declare_parameter("output_topic", OUTPUT_TOPIC)
        self.declare_parameter("source_topic", SOURCE_TOPIC)
        self.declare_parameter("publish_rate", DEFAULT_PUBLISH_RATE)
        self.declare_parameter("manual_timeout", DEFAULT_MANUAL_TIMEOUT)
        self.declare_parameter("auto_timeout", DEFAULT_AUTO_TIMEOUT)
        self.declare_parameter("release_timeout", DEFAULT_RELEASE_TIMEOUT)

        manual_topic = str(self.get_parameter("manual_topic").value)
        auto_topic = str(self.get_parameter("auto_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self.source_topic = str(self.get_parameter("source_topic").value)
        publish_rate = max(
            1.0, float(self.get_parameter("publish_rate").value))

        self.selector = CommandSelector(
            manual_timeout=float(self.get_parameter("manual_timeout").value),
            auto_timeout=float(self.get_parameter("auto_timeout").value),
            release_timeout=float(
                self.get_parameter("release_timeout").value),
        )

        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.source_pub = self.create_publisher(String, self.source_topic, 10)
        self.create_subscription(Twist, manual_topic, self._on_manual, 10)
        self.create_subscription(Twist, auto_topic, self._on_auto, 10)
        self.create_timer(1.0 / publish_rate, self._on_timer)

        self.last_source = None
        self.get_logger().info(
            "cmd_vel mux: %s (manual) + %s (auto) -> %s, manual priority"
            % (manual_topic, auto_topic, output_topic))

    def _on_manual(self, msg: Twist) -> None:
        """Feed a manual command to the selector."""
        self.selector.update_manual(msg.linear.x, msg.angular.z)

    def _on_auto(self, msg: Twist) -> None:
        """Feed an autonomous command to the selector."""
        self.selector.update_auto(msg.linear.x, msg.angular.z)

    def _on_timer(self) -> None:
        """Publish the selected command and its source at a fixed rate."""
        source, linear, angular = self.selector.select()

        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.cmd_pub.publish(command)
        self.source_pub.publish(String(data=source))

        if source != self.last_source:
            self.get_logger().info("cmd_vel source: %s" % source)
            self.last_source = source

    def stop(self) -> None:
        """Publish several zero commands so the Pico stops before shutdown."""
        zero = Twist()
        for _ in range(5):
            self.cmd_pub.publish(zero)
            time.sleep(0.02)


def main(args=None) -> None:
    """Run the mux and always leave the robot stopped on exit."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CmdVelMux()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
