"""ROSA tools that move the robot using the fused odometry estimate.

The tools validate every argument, ask for approval before a move by default,
and report what the fused estimate actually measured. ROS 2 publishing and
spinning live in ``robot.py``; this file only contains the agent-facing layer.
"""

import math
import os
import time

from langchain_core.tools import tool

from dog_toy_finder_motion.mover import local_change
from dog_toy_finder_motion.transforms import distance_and_bearing, wrap_angle

from .robot import get_robot

MAX_DISTANCE = 2.0        # m per drive command
MAX_TURN_DEGREES = 180.0  # deg per turn command
MAX_GOAL_RANGE = 3.0      # m from the current pose for go_to
APPROVAL_ENV = "ROSA_REQUIRE_APPROVAL"
REQUIRE_APPROVAL = os.environ.get(APPROVAL_ENV, "1").strip().lower() not in (
    "0",
    "false",
    "no",
    "off",
)


def _number(text):
    """Parse the first token of a value into a finite float, or return None."""
    try:
        value = float(str(text).strip().split()[0])
    except (ValueError, IndexError):
        return None
    return value if math.isfinite(value) else None


def _word(text, allowed):
    """Return a lower-case word when it is one of the allowed words, else None."""
    word = str(text).strip().lower()
    return word if word in allowed else None


def _approved(description: str) -> bool:
    """Ask the human to approve a move unless approval is disabled."""
    if not REQUIRE_APPROVAL:
        return True
    try:
        answer = input(
            "\n[APPROVAL NEEDED] %s. Allow? [y/N] " % description
        )
    except (EOFError, KeyboardInterrupt):
        return False
    return answer.strip().lower() in ("y", "yes")


def _fmt(pose) -> str:
    """Format a pose for a human-readable tool result."""
    return "x = %.3f m, y = %.3f m, heading = %.1f deg" % (
        pose[0],
        pose[1],
        math.degrees(pose[2]),
    )


def _ready_or_error():
    """Return (robot, None) when odometry is flowing, else (None, message)."""
    robot = get_robot()
    if not robot.ready():
        return None, (
            "No /odometry/filtered. Is the bringup launch and micro-ROS agent "
            "running? The robot did not move."
        )
    return robot, None


def _report(robot, start, description: str) -> str:
    """Describe the measured motion and the current fused pose."""
    time.sleep(0.4)   # let the estimate and the motors settle
    end = robot.pose()
    if end is None:
        return "%s. The fused estimate stopped updating." % description
    forward, left, turned = local_change(start, end)
    text = (
        "%s. Fused estimate measured: forward %.3f m, left %.3f m, "
        "turned %+.1f deg. Fused pose now: %s."
        % (description, forward, left, turned, _fmt(end))
    )
    if robot.mover.failsafe_hit():
        text += (
            " Pico failsafe engaged: the IR sensor saw an obstacle ahead "
            "(/ir_obstacle/detected), so the Pico braked and blocked the "
            "command. The move was aborted early; reverse in small steps or "
            "turn once the path is clear."
        )
    return text


@tool
def get_pose() -> str:
    """Report the robot's fused position and heading: x, y in metres, heading in degrees."""
    robot, error = _ready_or_error()
    if error is not None:
        return error
    return "Fused pose: %s (start pose = 0, 0, 0)." % _fmt(robot.pose())


@tool
def drive_distance(direction: str, meters: str) -> str:
    """Drive straight and report the measured motion.

    direction: 'forward' or 'backward'.
    meters: the distance exactly as the user wrote it, for example '1.0'.
    """
    distance = _number(meters)
    word = _word(direction, ("forward", "backward"))
    if word is None or distance is None:
        return (
            "Invalid drive: direction must be 'forward' or 'backward' and "
            "meters must be a number. The robot did not move."
        )
    if not 0.0 < abs(distance) <= MAX_DISTANCE:
        return (
            "Invalid drive: 0 < meters <= %.1f. The robot did not move."
            % MAX_DISTANCE
        )

    signed = abs(distance) if word == "forward" else -abs(distance)
    robot, error = _ready_or_error()
    if error is not None:
        return error
    if not _approved("Drive %s %.2f m" % (word, abs(distance))):
        return "The human declined. The robot did not move."

    start = robot.pose()
    robot.mover.straight(signed)
    return _report(robot, start, "Drove %s %.2f m" % (word, abs(distance)))


@tool
def turn(direction: str, degrees: str) -> str:
    """Turn in place and report the measured heading change.

    direction: 'left' or 'right'.
    degrees: the angle exactly as the user wrote it, for example '90'.
    """
    angle = _number(degrees)
    word = _word(direction, ("left", "right"))
    if word is None or angle is None:
        return (
            "Invalid turn: direction must be 'left' or 'right' and degrees "
            "must be a number. The robot did not move."
        )
    if not 0.0 < abs(angle) <= MAX_TURN_DEGREES:
        return (
            "Invalid turn: 0 < degrees <= %.0f. The robot did not move."
            % MAX_TURN_DEGREES
        )

    signed = abs(angle) if word == "left" else -abs(angle)
    robot, error = _ready_or_error()
    if error is not None:
        return error
    if not _approved("Turn %s %.1f deg" % (word, abs(angle))):
        return "The human declined. The robot did not move."

    start = robot.pose()
    robot.mover.turn(wrap_angle(math.radians(signed)))
    return _report(robot, start, "Turned %s %.1f deg" % (word, abs(angle)))


@tool
def go_to(x: str, y: str) -> str:
    """Drive to a world-frame point and report the final pose.

    x and y: metres from the start pose, exactly as the user wrote them.
    """
    goal_x = _number(x)
    goal_y = _number(y)
    if goal_x is None or goal_y is None:
        return "Invalid goal: x and y must be numbers. The robot did not move."

    robot, error = _ready_or_error()
    if error is not None:
        return error
    distance, _bearing = distance_and_bearing(robot.pose(), goal_x, goal_y)
    if distance > MAX_GOAL_RANGE:
        return (
            "Invalid goal: %.2f m away, limit is %.1f m. The robot did not move."
            % (distance, MAX_GOAL_RANGE)
        )
    if not _approved("Drive to x %.2f m, y %.2f m" % (goal_x, goal_y)):
        return "The human declined. The robot did not move."

    start = robot.pose()
    robot.mover.goto(goal_x, goal_y)
    return _report(
        robot, start, "Drove to x %.2f m, y %.2f m" % (goal_x, goal_y)
    )


@tool
def check_failsafe() -> str:
    """Report whether the Pico failsafe currently sees an obstacle ahead.

    Reads the latest /ir_obstacle/detected value. When true the Pico has
    braked and is blocking forward/turn commands, allowing only stepped
    reverse until the path clears.
    """
    robot = get_robot()
    if robot.mover.obstacle_ahead():
        return (
            "Pico failsafe is ACTIVE: /ir_obstacle/detected is true, so the "
            "Pico is braking and blocking forward/turn commands. Reverse in "
            "small steps or wait for the path to clear."
        )
    if robot.mover.failsafe_hit():
        return (
            "Pico failsafe interrupted the last move, but the path is clear "
            "now. You may drive again."
        )
    return "Pico failsafe is not active: no obstacle ahead on /ir_obstacle/detected."


@tool
def stop_robot() -> str:
    """Stop the robot immediately. Use this if a move fails or the user asks to stop."""
    robot = get_robot()
    robot.mover.stop()
    return "Stop command sent; /cmd_vel is zero."
