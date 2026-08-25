"""Closed XY paths for named 2D shapes inside the indoor workspace."""
from __future__ import annotations

import math
import re
from typing import List, Tuple

from backend.schemas.mission import SkillPrimitive

Point = Tuple[float, float]
_R = 0.55

_N_GON_NAMES = {
    "triangle": 3,
    "equilateral": 3,
    "square": 4,
    "pentagon": 5,
    "hexagon": 6,
    "hex": 6,
    "heptagon": 7,
    "octagon": 8,
    "nonagon": 9,
    "decagon": 10,
    "dodecagon": 12,
}


def wants_closed_path(command: str) -> bool:
    cmd = command.lower()
    if re.search(r"\bcircl|\bround\b|\boval\b|\bellipse\b", cmd):
        return True
    if re.search(r"\bfly in\b|\bshape\b|\bpolygon\b|\bgon\b|\bpath\b|\bfigure\b", cmd):
        return True
    return any(name in cmd for name in list(_N_GON_NAMES) + [
        "rectangle", "rhombus", "diamond", "star", "heart",
        "cross", "plus", "spiral", "infinity", "trapezoid",
        "parallelogram", "l-shape", "lshape",
    ])


def parse_shape_name(command: str) -> str:
    cmd = command.lower()
    m = re.search(r"fly in (?:an?\s+)?(?:a\s+)?([a-z0-9\-]+)", cmd)
    if m:
        return m.group(1)
    for name in sorted(_N_GON_NAMES, key=len, reverse=True):
        if name in cmd:
            return name
    for name in (
        "rectangle", "rhombus", "diamond", "star", "heart", "circle",
        "oval", "ellipse", "cross", "plus", "spiral", "infinity", "eight",
        "trapezoid", "parallelogram",
    ):
        if name in cmd:
            return name
    return "polygon"


def regular_polygon(n: int, radius: float = _R, start_deg: float = 90.0) -> List[Point]:
    n = max(3, min(int(n), 16))
    pts: List[Point] = []
    for i in range(n):
        ang = math.radians(start_deg + 360.0 * i / n)
        pts.append((radius * math.cos(ang), radius * math.sin(ang)))
    return pts


def shape_corners(name: str, radius: float = _R) -> List[Point]:
    key = (name or "polygon").lower().strip()
    key = re.sub(r"[^a-z0-9\-]", "", key)

    if key in ("rectangle", "rect"):
        return [(0.70, 0.35), (0.70, -0.35), (-0.70, -0.35), (-0.70, 0.35)]
    if key in ("rhombus", "diamond"):
        return [(0.60, 0.0), (0.0, 0.50), (-0.60, 0.0), (0.0, -0.50)]
    if key in ("square",):
        return [(0.50, 0.50), (0.50, -0.50), (-0.50, -0.50), (-0.50, 0.50)]
    if key in ("star", "pentagram"):
        return _star(5, radius, radius * 0.40)
    if key in ("heart",):
        return _heart(12, radius)
    if key in ("circle", "round", "oval", "ellipse"):
        rx, ry = (0.70, 0.35) if key in ("oval", "ellipse") else (radius, radius)
        return [(rx * math.cos(a), ry * math.sin(a)) for a in
                [math.radians(i * 30) for i in range(12)]]
    if key in ("figure8", "figure-8", "eight", "infinity"):
        return _figure8()
    if key in ("cross", "plus"):
        s = 0.50
        t = 0.16
        return [
            (t, t), (s, t), (s, -t), (t, -t), (t, -s), (-t, -s),
            (-t, -t), (-s, -t), (-s, t), (-t, t), (-t, s), (t, s),
        ]
    if key in ("lshape", "l-shape", "l"):
        return [(0.55, 0.55), (0.55, -0.55), (-0.15, -0.55), (-0.15, -0.15), (-0.55, -0.15), (-0.55, 0.55)]
    if key in ("trapezoid", "trapezium"):
        return [(0.35, 0.45), (0.70, -0.40), (-0.70, -0.40), (-0.35, 0.45)]
    if key in ("parallelogram",):
        return [(0.55, 0.35), (0.75, -0.35), (-0.55, -0.35), (-0.75, 0.35)]
    if key in ("spiral",):
        return _spiral()

    m = re.match(r"(\d+)\s*-?\s*gon", key)
    if m:
        return regular_polygon(int(m.group(1)), radius)
    if key in _N_GON_NAMES:
        return regular_polygon(_N_GON_NAMES[key], radius)
    # Unknown name: closed hexagon so the drone still flies a path.
    return regular_polygon(6, radius)


def closed_move_skills(z: float, corners: List[Point]) -> List[SkillPrimitive]:
    if not corners:
        return []
    pts = list(corners) + [corners[0]]
    return [SkillPrimitive(skill="MOVE_TO", params={"x": round(x, 3), "y": round(y, 3), "z": z}) for x, y in pts]


def clamp_z(z: float, z_min: float, z_max: float) -> float:
    return max(z_min, min(z, z_max))


def clip_skills_to_workspace(skills: List[SkillPrimitive], workspace) -> List[SkillPrimitive]:
    pad = 0.05
    xmin, xmax = workspace.x_min + pad, workspace.x_max - pad
    ymin, ymax = workspace.y_min + pad, workspace.y_max - pad
    zmin, zmax = workspace.z_min, workspace.z_max
    for skill in skills:
        p = skill.params
        if "x" in p:
            p["x"] = max(xmin, min(float(p["x"]), xmax))
        if "y" in p:
            p["y"] = max(ymin, min(float(p["y"]), ymax))
        if "z" in p:
            p["z"] = max(zmin, min(float(p["z"]), zmax))
    return skills


def _star(points: int, r_outer: float, r_inner: float) -> List[Point]:
    pts: List[Point] = []
    for i in range(points * 2):
        r = r_outer if i % 2 == 0 else r_inner
        ang = math.radians(-90 + i * 180.0 / points)
        pts.append((r * math.cos(ang), r * math.sin(ang)))
    return pts


def _heart(n: int, scale: float) -> List[Point]:
    raw: List[Point] = []
    for i in range(n):
        t = 2 * math.pi * i / n
        x = 16 * math.sin(t) ** 3
        y = 13 * math.cos(t) - 5 * math.cos(2 * t) - 2 * math.cos(3 * t) - math.cos(4 * t)
        raw.append((x, y))
    max_abs = max(max(abs(x), abs(y)) for x, y in raw) or 1.0
    k = scale / max_abs
    return [(x * k, y * k) for x, y in raw]


def _figure8() -> List[Point]:
    pts: List[Point] = []
    r = 0.28
    for i in range(8):
        a = math.radians(i * 45)
        pts.append((-0.32 + r * math.cos(a), r * math.sin(a)))
    for i in range(8):
        a = math.radians(i * 45)
        pts.append((0.32 + r * math.cos(a), r * math.sin(a)))
    return pts


def _spiral() -> List[Point]:
    pts: List[Point] = []
    for i in range(10):
        a = math.radians(i * 40)
        r = 0.12 + 0.045 * i
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts
