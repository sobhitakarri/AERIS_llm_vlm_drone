"""Indoor route planner: POIs from workspace objects (NeLV route stage, no GIS)."""
from __future__ import annotations

from typing import Any, Dict, List, Optional

from backend.planning.intent_router import ParsedIntent
from backend.schemas.mission import MissionSpec, PointOfInterest


def match_object(target: str, objects: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    t = (target or "").lower().strip()
    if not objects:
        return None
    for obj in objects:
        label = str(obj.get("label") or "").lower()
        if t and (t in label or label in t):
            return obj
        words = [w for w in t.split() if w not in {"the", "a", "an", "red", "blue"}]
        if label and any(w in label for w in words):
            return obj
    color_hint = None
    if "red" in t:
        color_hint = "red"
    elif "blue" in t:
        color_hint = "blue"
    if color_hint:
        for obj in objects:
            if color_hint in str(obj.get("label") or "").lower():
                return obj
    return objects[0]


def spec_from_intent(intent: ParsedIntent, context: Optional[Dict[str, Any]] = None) -> MissionSpec:
    ctx = context or {}
    objects = list(ctx.get("objects") or [])
    z = float(intent.slots.get("z", 0.5))
    z = max(0.2, min(1.5, z))

    if intent.intent == "vision":
        obj = match_object(intent.target or "", objects) or {
            "label": intent.target or "object",
            "x": 0.5,
            "y": 0.5,
            "z": 0.0,
        }
        standoff = 0.35
        px, py = float(obj.get("x", 0.5)), float(obj.get("y", 0.5))
        return MissionSpec(
            mission_type="inspect",
            altitude_m=z,
            takeoff_first=True,
            land_at_end=False,
            pois=[
                PointOfInterest(
                    x=px - standoff,
                    y=py,
                    z=z,
                    label=str(obj.get("label") or "object"),
                    loiter_s=2.0,
                )
            ],
        )

    if intent.intent == "goto":
        return MissionSpec(
            mission_type="goto",
            altitude_m=z,
            pois=[
                PointOfInterest(
                    x=float(intent.slots.get("x", 0.0)),
                    y=float(intent.slots.get("y", 0.0)),
                    z=z,
                    label="goto",
                )
            ],
        )

    if intent.intent == "return":
        return MissionSpec(
            mission_type="return",
            altitude_m=z,
            pois=[PointOfInterest(x=0.0, y=0.0, z=z, label="home")],
            land_at_end=True,
        )

    return MissionSpec(mission_type=intent.intent or "custom", altitude_m=z)


def resolve_perception_skills(
    skills: List,
    context: Optional[Dict[str, Any]] = None,
) -> List:
    """Fill FIND/INSPECT x,y,z from known objects (indoor POI table)."""
    from backend.schemas.mission import SkillPrimitive

    objects = list((context or {}).get("objects") or [])
    out: List[SkillPrimitive] = []
    for item in skills:
        name = item.skill.upper()
        params = dict(item.params or {})
        if name in {"FIND", "INSPECT"} and not all(k in params for k in ("x", "y", "z")):
            obj = match_object(str(params.get("target") or ""), objects)
            if obj:
                z = float(params.get("z") or 0.5)
                params["x"] = float(obj.get("x", 0.5)) - 0.35
                params["y"] = float(obj.get("y", 0.5))
                params["z"] = z
                params["target"] = obj.get("label") or params.get("target")
        out.append(SkillPrimitive(skill=item.skill, params=params))
    return out
