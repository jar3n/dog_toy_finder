"""2D coordinate transforms between the odom frame and the robot frame.

pose = (x, y, theta): position in the world, heading in radians,
counter-clockwise from world +x. Robot frame: +x forward, +y left.
"""

import math


def wrap_angle(angle: float) -> float:
    """Wrap an angle in radians into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def robot_to_world(pose, px: float, py: float):
    """Map a point in the robot frame to the world frame."""
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    return x + px * c - py * s, y + px * s + py * c


def world_to_robot(pose, gx: float, gy: float):
    """Map a point in the world frame to the robot frame."""
    x, y, theta = pose
    dx, dy = gx - x, gy - y
    c, s = math.cos(theta), math.sin(theta)
    return dx * c + dy * s, -dx * s + dy * c


def distance_and_bearing(pose, gx: float, gy: float):
    """Return distance (m) and bearing (rad, + = left) to a world point."""
    xr, yr = world_to_robot(pose, gx, gy)
    return math.hypot(xr, yr), math.atan2(yr, xr)
