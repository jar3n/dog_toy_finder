"""Enforce EKF-only feedback for motion commands.

The Pico's ``/wheel/odom`` is an input to the EKF, not a motion feedback source.
Every closed-loop move must use the fused ``/odometry/filtered`` pose instead,
and must not keep driving if that estimate stops arriving. Keeping the small
topic/timeout policy here makes the invariant directly testable without a ROS
graph.
"""

import time

# The only odometry topic allowed to close the motion loop.
FEEDBACK_TOPIC = "/odometry/filtered"

# A command is allowed only while the EKF has published within this window.
FEEDBACK_TIMEOUT = 1.0


def validate_feedback_topic(topic: str) -> None:
    """Reject any feedback source other than the EKF output.

    ``Mover`` used to accept a ``feedback_topic`` parameter, which made it
    possible to accidentally close the loop on raw wheel odometry. The check is
    intentionally strict: raw odometry feeds the filter and must never be used
    as motion feedback.
    """
    if topic != FEEDBACK_TOPIC:
        raise ValueError(
            "motion feedback must be the EKF topic %r, got %r"
            % (FEEDBACK_TOPIC, topic)
        )


class FeedbackGate:
    """Track whether fused odometry is present and recent enough to move."""

    def __init__(self, timeout: float = FEEDBACK_TIMEOUT):
        """Store the maximum allowed age of the last EKF message."""
        self.timeout = max(0.0, float(timeout))
        self.last_update = None

    def update(self, now: float = None) -> None:
        """Record receipt of a fresh fused odometry message."""
        self.last_update = time.monotonic() if now is None else float(now)

    def fresh(self, now: float = None) -> bool:
        """Return True only after a recent EKF update."""
        if self.last_update is None:
            return False
        if now is None:
            now = time.monotonic()
        return 0.0 <= now - self.last_update <= self.timeout
