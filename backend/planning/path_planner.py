"""Indoor path planner: densify hops inside the geofence (NeLV path stage)."""
from __future__ import annotations

import math
from typing import List

from backend.planning.shape_paths import clip_skills_to_workspace
from backend.schemas.mission import SkillPrimitive

_MAX_HOP_M = 1.05


def densify_move_tos(skills: List[SkillPrimitive], workspace) -> List[SkillPrimitive]:
    """Split long MOVE_TO segments so the plant can track without 30s timeouts."""
    out: List[SkillPrimitive] = []
    last_xy = None
    last_z = None
    for item in skills:
        name = item.skill.upper()
        params = dict(item.params or {})
        if name == "TAKEOFF":
            last_z = float(params.get("z", last_z or 0.5))
            last_xy = last_xy or (0.0, 0.0)
            out.append(item)
            continue
        if name != "MOVE_TO":
            out.append(item)
            continue
        x = float(params.get("x", 0.0))
        y = float(params.get("y", 0.0))
        z = float(params.get("z", last_z or 0.5))
        if last_xy is None:
            out.append(item)
            last_xy, last_z = (x, y), z
            continue
        dx = x - last_xy[0]
        dy = y - last_xy[1]
        dist = math.hypot(dx, dy)
        if dist <= _MAX_HOP_M + 1e-6:
            out.append(item)
        else:
            n = max(2, int(math.ceil(dist / _MAX_HOP_M)))
            for i in range(1, n + 1):
                t = i / n
                out.append(
                    SkillPrimitive(
                        skill="MOVE_TO",
                        params={
                            "x": round(last_xy[0] + t * dx, 3),
                            "y": round(last_xy[1] + t * dy, 3),
                            "z": round(last_z + t * (z - last_z), 3),
                            "speed": params.get("speed", 0.4),
                        },
                    )
                )
        last_xy, last_z = (x, y), z
    return clip_skills_to_workspace(out, workspace)
