"""Drive known test moves using the fused estimate as feedback.

Run while dog_toy_finder_bringup is running:
    ros2 run dog_toy_finder_motion mover_test straight 1.0
    ros2 run dog_toy_finder_motion mover_test turn 90        # + = left
    ros2 run dog_toy_finder_motion mover_test square 1.0
    ros2 run dog_toy_finder_motion mover_test goto 1.0 0.5

Commands go to /cmd_vel and feedback comes from /odometry/filtered. Press
Ctrl+C at any time to stop the robot.
"""

import argparse
import sys
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .mover import Mover, local_change
from .transforms import wrap_angle


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line parser for the test moves."""
    parser = argparse.ArgumentParser(
        description="Drive test moves using fused odometry as feedback"
    )
    subparsers = parser.add_subparsers(dest="move", required=True)
    subparsers.add_parser("straight").add_argument("meters", type=float)
    subparsers.add_parser("turn").add_argument("degrees", type=float)
    square = subparsers.add_parser("square")
    square.add_argument("side", type=float)
    square.add_argument("--cw", action="store_true", help="clockwise")
    goto = subparsers.add_parser("goto")
    goto.add_argument("x", type=float)
    goto.add_argument("y", type=float)
    return parser


def main(args=None) -> None:
    """Run one test move and report what the fused estimate measured."""
    parsed, ros_args = build_parser().parse_known_args(args=args)

    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = Mover(node_name="mover_test")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    try:
        node.wait_for_odom()
        start = node.pose()
        if parsed.move == "straight":
            node.straight(parsed.meters)
        elif parsed.move == "turn":
            node.turn(wrap_angle(parsed.degrees * 3.141592653589793 / 180.0))
        elif parsed.move == "square":
            node.square(parsed.side, clockwise=parsed.cw)
        elif parsed.move == "goto":
            node.goto(parsed.x, parsed.y)
        end = node.pose()
        if start is not None and end is not None:
            forward, left, turned = local_change(start, end)
            print(
                "Fused estimate measured: forward %.3f m, left %.3f m, "
                "turned %+.1f deg" % (forward, left, turned)
            )
    except RuntimeError as error:
        print("error: %s" % error, file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        node.stop()
        executor.shutdown()
        thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
