"""Shared ROS 2 session used by the ROSA tools.

ROSA tools are plain Python functions, so this module owns the rclpy node and
spins it on a background thread. Keeping it separate from ``tools.py`` means
the tool descriptions stay readable and the ROS lifecycle lives in one place.
"""

import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from dog_toy_finder_motion.mover import Mover


class RobotSession:
    """A spinning Mover node that the agent can call without blocking ROS."""

    def __init__(self):
        if not rclpy.ok():
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.mover = Mover(node_name="rosa_motion")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.mover)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        self.waited = False

    def ready(self, timeout: float = 8.0) -> bool:
        """Return true once fused odometry has arrived."""
        if not self.waited:
            try:
                self.mover.wait_for_odom(timeout)
            except RuntimeError:
                return False
            self.waited = True
        return self.mover.pose() is not None

    def pose(self):
        """Return the latest fused (x, y, theta) or None."""
        return self.mover.pose()

    def close(self) -> None:
        """Stop the robot, stop spinning, and release the ROS context."""
        self.mover.stop()
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.mover.destroy_node()
        rclpy.try_shutdown()


_session = None


def get_robot() -> RobotSession:
    """Return the process-wide robot session, creating it on first use."""
    global _session
    if _session is None:
        _session = RobotSession()
    return _session


def close_robot() -> None:
    """Stop and release the process-wide robot session."""
    global _session
    if _session is not None:
        _session.close()
        _session = None
