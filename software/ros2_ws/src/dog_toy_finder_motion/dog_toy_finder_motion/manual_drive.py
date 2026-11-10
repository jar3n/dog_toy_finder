"""Keyboard manual drive for the Dog Toy Finder.

Hold W/S (or Up/Down) to drive forward/backward and A/D (or Left/Right) to
turn in place. Press space to stop and Q to quit. The robot never drives and
turns at the same time: every command sets exactly one axis and clears the
other, so a Twist never has both a linear and an angular component.

There is no key-release event on a terminal, so holding a key works through
the keyboard's auto-repeat. If no drive key arrives for ``key_timeout``
seconds (default 0.6 s), the node publishes a stop; releasing the key therefore
stops the robot instead of leaving it driving.

Run it while the bringup stack is running:

    ros2 run dog_toy_finder_motion manual_drive
    ros2 run dog_toy_finder_motion manual_drive --ros-args \
        -p linear_speed:=0.10 -p angular_speed:=0.4

The Pico's IR reflex and 500 ms /cmd_vel watchdog still apply in manual mode:
the Pico brakes locally on an obstacle and stops if this node stops publishing.
"""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

CMD_TOPIC = "/cmd_vel"
DEFAULT_LINEAR_SPEED = 0.12   # m/s
DEFAULT_ANGULAR_SPEED = 0.5   # rad/s
DEFAULT_KEY_TIMEOUT = 0.6     # s without a key before stopping
DEFAULT_PUBLISH_RATE = 20.0   # Hz

# Terminal escape sequences for the arrow keys. Most terminals send ESC [ X;
# a few use ESC O X (application cursor mode).
ARROW_KEYS = {
    "\x1b[A": "up",
    "\x1b[B": "down",
    "\x1b[C": "right",
    "\x1b[D": "left",
    "\x1bOA": "up",
    "\x1bOB": "down",
    "\x1bOC": "right",
    "\x1bOD": "left",
}

HELP = """
Manual drive (hold a key; release stops after a moment)

  W / Up      drive forward
  S / Down    drive backward
  A / Left    turn left
  D / Right   turn right
  Space       stop now
  Q           stop and quit

The robot drives or turns, never both at once. The Pico IR reflex and the
500 ms /cmd_vel watchdog remain active.
"""


def decode_keys(data: str, pending: str = ""):
    """Turn raw terminal input into logical keys.

    Returns ``(keys, pending)`` where ``keys`` is a list such as
    ``["w", "up", "left"]`` and ``pending`` holds an incomplete escape
    sequence to prepend to the next read. Unknown escape sequences are
    discarded; ordinary characters are returned unchanged (the caller decides
    which ones are bindings).
    """
    text = pending + data
    keys = []
    index = 0

    while index < len(text):
        char = text[index]

        if char == "\x1b":
            # An escape sequence can be split across reads; keep the tail.
            if index + 1 >= len(text):
                return keys, text[index:]
            second = text[index + 1]
            if second in ("[", "O"):
                if index + 2 >= len(text):
                    return keys, text[index:]
                sequence = text[index:index + 3]
                if sequence in ARROW_KEYS:
                    keys.append(ARROW_KEYS[sequence])
                index += 3
                continue
            # A lone Escape (or Alt+key): drop the escape and handle the key.
            index += 1
            continue

        keys.append(char)
        index += 1

    return keys, ""


def command_for_key(key: str, linear_speed: float, angular_speed: float):
    """Return ``(linear_x, angular_z)`` for one key, or None if unbound.

    Driving keys produce a non-zero linear component with zero turn, and
    turning keys produce a non-zero angular component with zero drive. Space
    produces an explicit stop. This is what guarantees the robot never drives
    and turns at the same time in manual mode.
    """
    key = key.lower()

    if key in ("w", "up"):
        return (linear_speed, 0.0)
    if key in ("s", "down"):
        return (-linear_speed, 0.0)
    if key in ("a", "left"):
        return (0.0, angular_speed)
    if key in ("d", "right"):
        return (0.0, -angular_speed)
    if key == " ":
        return (0.0, 0.0)
    return None


class ManualDrive(Node):
    """Publish exclusive drive-or-turn Twists from keyboard input."""

    def __init__(self):
        """Read parameters, create the publisher, and prepare the terminal."""
        super().__init__("manual_drive")

        self.declare_parameter("linear_speed", DEFAULT_LINEAR_SPEED)
        self.declare_parameter("angular_speed", DEFAULT_ANGULAR_SPEED)
        self.declare_parameter("key_timeout", DEFAULT_KEY_TIMEOUT)
        self.declare_parameter("publish_rate", DEFAULT_PUBLISH_RATE)
        self.declare_parameter("cmd_topic", CMD_TOPIC)

        self.linear_speed = float(
            self.get_parameter("linear_speed").value)
        self.angular_speed = float(
            self.get_parameter("angular_speed").value)
        self.key_timeout = float(self.get_parameter("key_timeout").value)
        self.publish_rate = max(
            1.0, float(self.get_parameter("publish_rate").value))
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)

        self.cmd = self.create_publisher(Twist, self.cmd_topic, 10)
        self.linear = 0.0
        self.angular = 0.0
        self.last_key_time = time.monotonic()
        self.pending_input = ""
        self.stdin_fd = sys.stdin.fileno()
        self.saved_termios = None
        self.timed_out = False

    def enable_raw_input(self) -> None:
        """Switch the terminal to non-canonical, no-echo input.

        Raises RuntimeError when stdin is not a terminal, for example when the
        node is started without a TTY in Docker.
        """
        if not sys.stdin.isatty():
            raise RuntimeError(
                "manual_drive needs an interactive terminal; run it with a "
                "TTY (for example docker compose run --rm -it ros2 ...)")
        self.saved_termios = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)
        attributes = termios.tcgetattr(self.stdin_fd)
        attributes[6][termios.VMIN] = 0
        attributes[6][termios.VTIME] = 0
        termios.tcsetattr(self.stdin_fd, termios.TCSANOW, attributes)

    def restore_terminal(self) -> None:
        """Restore the terminal settings saved by enable_raw_input()."""
        if self.saved_termios is not None:
            termios.tcsetattr(
                self.stdin_fd, termios.TCSANOW, self.saved_termios)
            self.saved_termios = None

    def set_command(self, linear: float, angular: float) -> None:
        """Replace the current command; one axis is always zero."""
        self.linear = float(linear)
        self.angular = float(angular)

    def update_timeout(self, now=None) -> bool:
        """Stop when no drive key has arrived for ``key_timeout`` seconds.

        Returns True when this call stopped the robot. The dead-man behaviour
        means releasing a key stops the robot even though terminals do not
        report key releases.
        """
        if now is None:
            now = time.monotonic()

        moving = self.linear != 0.0 or self.angular != 0.0
        if not moving or (now - self.last_key_time) < self.key_timeout:
            return False

        self.set_command(0.0, 0.0)
        if not self.timed_out:
            self.timed_out = True
            self.get_logger().info("manual drive: keys released; stopped")
        return True

    def publish_command(self) -> None:
        """Publish the current command as a Twist."""
        message = Twist()
        message.linear.x = self.linear
        message.angular.z = self.angular
        self.cmd.publish(message)

    def stop(self) -> None:
        """Stop and publish a few zero commands for the Pico watchdog."""
        self.set_command(0.0, 0.0)
        for _ in range(5):
            self.publish_command()
            time.sleep(0.01)

    def handle_keys(self, keys) -> bool:
        """Apply decoded keys; return False when the user asked to quit."""
        for key in keys:
            if key.lower() == "q":
                return False

            command = command_for_key(
                key, self.linear_speed, self.angular_speed)
            if command is None:
                continue

            self.set_command(*command)
            self.last_key_time = time.monotonic()
            if self.timed_out:
                self.timed_out = False
                self.get_logger().info("manual drive: moving again")

        return True

    def pump_input(self) -> bool:
        """Read and apply available input; return False when quitting."""
        ready, _, _ = select.select(
            [self.stdin_fd], [], [], 1.0 / self.publish_rate)
        if not ready:
            return True

        try:
            data = os.read(self.stdin_fd, 64).decode("utf-8", "replace")
        except OSError:
            return True

        keys, self.pending_input = decode_keys(data, self.pending_input)
        return self.handle_keys(keys)

    def run(self) -> None:
        """Drive until Q, Ctrl-C, or the terminal closes."""
        self.enable_raw_input()
        print(HELP, flush=True)
        self.get_logger().info(
            "manual drive ready: %.2f m/s, %.2f rad/s, stop after %.2f s "
            "without a key" % (
                self.linear_speed, self.angular_speed, self.key_timeout))

        self.last_key_time = time.monotonic()
        try:
            while rclpy.ok():
                if not self.pump_input():
                    break

                self.update_timeout()
                self.publish_command()
        finally:
            self.restore_terminal()


def main(args=None) -> int:
    """Run the manual drive node and always stop the robot on exit."""
    # Keep Python's KeyboardInterrupt handling; the node stops in finally.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = ManualDrive()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        print("error: %s" % error, file=sys.stderr)
        return 1
    finally:
        node.stop()
        node.restore_terminal()
        node.destroy_node()
        rclpy.try_shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
