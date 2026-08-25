"""Control platform: takeoff/landing sandwich + loiter (NeLV executable trajectory)."""
from __future__ import annotations

from typing import Any, Dict, List, Optional

from backend.planning.intent_router import ParsedIntent
from backend.planning.path_planner import densify_move_tos
from backend.planning.route_planner import resolve_perception_skills, spec_from_intent
from backend.planning.shape_paths import clip_skills_to_workspace
from backend.schemas.mission import MissionSpec, SkillPrimitive


def sandwich_takeoff_land(
    skills: List[SkillPrimitive],
    *,
    altitude: float,
    takeoff_first: bool,
    land_at_end: bool,
) -> List[SkillPrimitive]:
    if not skills:
        return skills
    names = [s.skill.upper() for s in skills]
    out = list(skills)
    needs_air = any(n in {"MOVE_TO", "CIRCLE", "FIND", "INSPECT", "HOVER", "ROTATE"} for n in names)
    if takeoff_first and needs_air and names[0] != "TAKEOFF":
        out = [SkillPrimitive(skill="TAKEOFF", params={"z": altitude, "speed": 0.4})] + out
    if land_at_end and out[-1].skill.upper() != "LAND":
        out = out + [SkillPrimitive(skill="LAND", params={})]
    return out


def insert_loiter(skills: List[SkillPrimitive], spec: Optional[MissionSpec]) -> List[SkillPrimitive]:
    if not spec or not spec.pois:
        return skills
    loiter_by_xy = {
        (round(p.x, 2), round(p.y, 2)): p.loiter_s
        for p in spec.pois
        if p.loiter_s and p.loiter_s > 0
    }
    if not loiter_by_xy:
        return skills
    out: List[SkillPrimitive] = []
    for item in skills:
        out.append(item)
        if item.skill.upper() == "MOVE_TO":
            key = (round(float(item.params.get("x", 0)), 2), round(float(item.params.get("y", 0)), 2))
            t = loiter_by_xy.get(key)
            if t:
                out.append(SkillPrimitive(skill="HOVER", params={"t": t}))
    return out


def finalize_skills(
    skills: List[SkillPrimitive],
    workspace,
    context: Optional[Dict[str, Any]] = None,
    intent: Optional[ParsedIntent] = None,
) -> tuple[List[SkillPrimitive], MissionSpec, List[str]]:
    """parser params → route POIs → path densify → takeoff/land pattern."""
    stages = ["parser", "route", "path", "control"]
    spec = spec_from_intent(intent, context) if intent else MissionSpec()
    routed = resolve_perception_skills(skills, context)
    if intent and intent.use_code and intent.intent == "vision" and spec.pois:
        poi = spec.pois[0]
        routed = [
            SkillPrimitive(skill="TAKEOFF", params={"z": spec.altitude_m, "speed": 0.4}),
            SkillPrimitive(
                skill="FIND",
                params={"target": poi.label, "x": poi.x, "y": poi.y, "z": poi.z},
            ),
            SkillPrimitive(
                skill="MOVE_TO",
                params={"x": poi.x, "y": poi.y, "z": poi.z, "speed": 0.4},
            ),
        ]
    pathed = densify_move_tos(routed, workspace)
    land = bool(intent and intent.intent in {"return", "land"})
    takeoff = not (intent and intent.intent == "land")
    alt = spec.altitude_m
    controlled = sandwich_takeoff_land(
        pathed, altitude=alt, takeoff_first=takeoff, land_at_end=land
    )
    controlled = insert_loiter(controlled, spec if intent and intent.intent == "vision" else None)
    return clip_skills_to_workspace(controlled, workspace), spec, stages
