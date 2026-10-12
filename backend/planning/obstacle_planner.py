"""
Obstacle-aware routing between two setpoints.

Works on the height grid measured by the overhead depth camera (tallest
surface per cell), so it needs no model of the scene. For each leg it picks
the cheapest of:
  * the straight line, if it clears everything under it,
  * climbing over what is in the way,
  * going around at the current altitude (A* on the grid).
"""
from __future__ import annotations

import heapq
import math
from typing import List, Optional, Tuple

import numpy as np

Point3 = Tuple[float, float, float]

_INFLATE_M = 0.25        # horizontal margin around every obstacle
_CLEAR_M = 0.30          # vertical margin above an obstacle
_GOAL_CLEAR_M = 0.12     # margin at the goal itself (it may be a hover above an object)


def _inflate(heights: np.ndarray, cells: int) -> np.ndarray:
    if cells <= 0:
        return heights
    import cv2
    k = 2 * cells + 1
    return cv2.dilate(heights, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))


class HeightGrid:
    def __init__(self, heights: np.ndarray, origin: Tuple[float, float], cell: float, inflate_m: float = _INFLATE_M):
        self.raw = heights
        self.cell = cell
        self.origin = origin
        self.h = _inflate(heights, int(round(inflate_m / cell)))
        self.nx, self.ny = heights.shape

    def index(self, x: float, y: float) -> Tuple[int, int]:
        i = int((x - self.origin[0]) / self.cell)
        j = int((y - self.origin[1]) / self.cell)
        return min(max(i, 0), self.nx - 1), min(max(j, 0), self.ny - 1)

    def centre(self, i: int, j: int) -> Tuple[float, float]:
        return self.origin[0] + (i + 0.5) * self.cell, self.origin[1] + (j + 0.5) * self.cell

    def height_at(self, x: float, y: float, inflated: bool = True) -> float:
        i, j = self.index(x, y)
        return float((self.h if inflated else self.raw)[i, j])

    def max_along(self, a: Tuple[float, float], b: Tuple[float, float], skip_end_m: float = 0.0) -> float:
        """Tallest inflated cell under the segment a-b, optionally ignoring the last stretch."""
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(length / (self.cell * 0.5)) + 1)
        top = 0.0
        for k in range(n + 1):
            t = k / n
            if skip_end_m and (1 - t) * length < skip_end_m:
                break
            top = max(top, self.height_at(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        return top

    def free(self, i: int, j: int, z: float) -> bool:
        return self.h[i, j] + _CLEAR_M <= z


def _astar(grid: HeightGrid, start: Tuple[int, int], goal: Tuple[int, int], z: float) -> Optional[List[Tuple[int, int]]]:
    if not grid.free(*goal, z) or not grid.free(*start, z):
        return None
    open_set = [(0.0, start)]
    came = {}
    cost = {start: 0.0}
    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    while open_set:
        _, cur = heapq.heappop(open_set)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        for di, dj, step in moves:
            nxt = (cur[0] + di, cur[1] + dj)
            if not (0 <= nxt[0] < grid.nx and 0 <= nxt[1] < grid.ny) or not grid.free(*nxt, z):
                continue
            new = cost[cur] + step
            if new < cost.get(nxt, float("inf")):
                cost[nxt] = new
                came[nxt] = cur
                heapq.heappush(open_set, (new + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1]), nxt))
    return None


def _simplify(grid: HeightGrid, pts: List[Tuple[float, float]], z: float) -> List[Tuple[float, float]]:
    """Drop intermediate points wherever a straight shortcut is clear."""
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and grid.max_along(pts[i], pts[j]) + _CLEAR_M > z:
            j -= 1
        out.append(pts[j])
        i = j
    return out


def _length(points: List[Point3]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points[:-1], points[1:]))


def plan_path(
    start: Point3,
    goal: Point3,
    grid_spec,
    z_limits: Tuple[float, float],
) -> Tuple[Optional[List[Point3]], str]:
    """Waypoints from start to goal (goal included, start excluded).

    Returns (waypoints, how). waypoints is None when no safe route exists;
    `how` names the strategy or the reason for failure.
    """
    if grid_spec is None:
        return [goal], "direct (no obstacle map)"
    grid = HeightGrid(*grid_spec)
    z_min, z_max = z_limits
    a, b = (start[0], start[1]), (goal[0], goal[1])

    if goal[2] < grid.height_at(b[0], b[1], inflated=False) + _GOAL_CLEAR_M:
        return None, f"goal altitude {goal[2]:.2f} m is inside an obstacle ({grid.height_at(*b, inflated=False):.2f} m tall there)"

    # The final approach may end above the target object itself, so the last
    # stretch is checked against the goal altitude only at the goal cell.
    cruise = max(start[2], goal[2])
    blocking = grid.max_along(a, b, skip_end_m=_INFLATE_M + grid.cell)
    if blocking + _CLEAR_M <= min(start[2], goal[2]):
        return [goal], "direct"

    options: List[Tuple[float, List[Point3], str]] = []

    z_over = max(cruise, grid.max_along(a, b) + _CLEAR_M)
    if z_over <= z_max:
        over: List[Point3] = []
        if z_over > start[2] + 0.02:
            over.append((start[0], start[1], round(z_over, 3)))
        over.append((goal[0], goal[1], round(z_over, 3)))
        if abs(z_over - goal[2]) > 0.02:
            over.append(goal)
        options.append((_length([start] + over), over, f"over at {z_over:.2f} m"))

    z_around = max(cruise, z_min)
    cells = _astar(grid, grid.index(*a), grid.index(*b), z_around)
    if cells:
        pts = [a] + [grid.centre(i, j) for i, j in cells[1:-1]] + [b]
        pts = _simplify(grid, pts, z_around)
        around: List[Point3] = []
        if z_around > start[2] + 0.02:
            around.append((start[0], start[1], round(z_around, 3)))
        around += [(round(p[0], 3), round(p[1], 3), round(z_around, 3)) for p in pts[1:]]
        if abs(z_around - goal[2]) > 0.02:
            around.append(goal)
        options.append((_length([start] + around), around, f"around at {z_around:.2f} m"))

    if not options:
        return None, "no clear route within the altitude limits"
    options.sort(key=lambda o: o[0])
    return options[0][1], options[0][2]
