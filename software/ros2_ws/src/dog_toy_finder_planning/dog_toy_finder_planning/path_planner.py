"""Pure path-planning helpers (no ROS dependency).

These are kept separate so they can be unit-tested without a ROS 2 runtime.
"""

import math


def expanding_square_waypoints(side=0.5, increment=0.5, loops=3):
    """Return a list of (x, y) waypoints for an expanding square search.

    The robot is assumed to start at (0, 0) facing +x. Each loop adds
    `increment` to the side length. Waypoints are the corner positions.
    """
    waypoints = []
    x, y = 0.0, 0.0
    heading = 0.0  # 0=+x, 1=+y, 2=-x, 3=-y (counter-clockwise turns)
    cur_side = side
    for _ in range(loops):
        for _ in range(4):
            if heading == 0:
                x += cur_side
            elif heading == 1:
                y += cur_side
            elif heading == 2:
                x -= cur_side
            else:
                y -= cur_side
            waypoints.append((round(x, 6), round(y, 6)))
            heading = (heading + 1) % 4
        cur_side += increment
    return waypoints


def lawnmower_waypoints(width=2.0, height=2.0, spacing=0.5):
    """Return (x, y) waypoints for a boustrophedon (lawnmower) sweep."""
    waypoints = []
    y = 0.0
    direction = 1
    while y <= height + 1e-9:
        if direction == 1:
            waypoints.append((0.0, round(y, 6)))
            waypoints.append((width, round(y, 6)))
        else:
            waypoints.append((width, round(y, 6)))
            waypoints.append((0.0, round(y, 6)))
        y += spacing
        direction *= -1
    return waypoints


def a_star(grid, start, goal):
    """A* on a 2D grid of 0 (free) / 1 (obstacle). Returns list of (r,c) or None.

    grid: list of lists of 0/1. start, goal: (r, c) tuples.
    """
    rows = len(grid)
    cols = len(grid[0]) if rows else 0
    if not (0 <= start[0] < rows and 0 <= start[1] < cols):
        return None
    if not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
        return None
    if grid[start[0]][start[1]] == 1 or grid[goal[0]][goal[1]] == 1:
        return None

    def h(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = {start}
    came_from = {}
    g_score = {start: 0}
    f_score = {start: h(start, goal)}

    while open_set:
        current = min(open_set, key=lambda n: f_score.get(n, float("inf")))
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        open_set.discard(current)
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nb = (current[0] + dr, current[1] + dc)
            if not (0 <= nb[0] < rows and 0 <= nb[1] < cols):
                continue
            if grid[nb[0]][nb[1]] == 1:
                continue
            tentative = g_score[current] + 1
            if tentative < g_score.get(nb, float("inf")):
                came_from[nb] = current
                g_score[nb] = tentative
                f_score[nb] = tentative + h(nb, goal)
                open_set.add(nb)
    return None
