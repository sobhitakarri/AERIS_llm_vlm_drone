"""Deterministic skill plans for intents that do not need an LLM."""
from __future__ import annotations

from typing import Any, Dict, List

from backend.planning.intent_router import ParsedIntent, parse_intent
from backend.planning.shape_paths import closed_move_skills, parse_shape_name, shape_corners
from backend.schemas.mission import SkillPrimitive


def local_plan(command: str, context: Dict[str, Any] | None = None) -> List[SkillPrimitive]:
    intent = parse_intent(command, context)
    return plan_from_intent(intent, context)


def plan_from_intent(intent: ParsedIntent, context: Dict[str, Any] | None = None) -> List[SkillPrimitive]:
    ctx = context or {}
    pose = ctx.get("pose") or {}
    z = float(intent.slots.get("z", pose.get("z", 0.5)))
    z = max(0.2, min(1.5, z))
    hover_t = float(intent.slots.get("t", 2.0))

    if intent.intent == "takeoff":
        return [
            SkillPrimitive(skill="TAKEOFF", params={"z": z, "speed": 0.4}),
            SkillPrimitive(skill="HOVER", params={"t": hover_t}),
        ]
    if intent.intent == "land":
        return [SkillPrimitive(skill="LAND", params={})]
    if intent.intent == "hover":
        return [SkillPrimitive(skill="HOVER", params={"t": hover_t})]
    if intent.intent == "return":
        return [
            SkillPrimitive(skill="MOVE_TO", params={"x": 0.0, "y": 0.0, "z": z, "speed": 0.4}),
            SkillPrimitive(skill="LAND", params={}),
        ]
    if intent.intent == "goto":
        x = float(intent.slots.get("x", 0.0))
        y = float(intent.slots.get("y", 0.0))
        return [
            SkillPrimitive(skill="TAKEOFF", params={"z": z, "speed": 0.4}),
            SkillPrimitive(skill="MOVE_TO", params={"x": x, "y": y, "z": z, "speed": 0.4}),
        ]
    if intent.intent == "relative":
        x = float(intent.slots.get("x", pose.get("x", 0.0)))
        y = float(intent.slots.get("y", pose.get("y", 0.0)))
        return [
            SkillPrimitive(skill="MOVE_TO", params={"x": x, "y": y, "z": z, "speed": 0.4}),
        ]
    if intent.intent == "rotate":
        return [SkillPrimitive(skill="ROTATE", params={"angle": float(intent.slots.get("angle", 90.0))})]
    if intent.intent == "circle":
        r = float(intent.slots.get("radius", 0.4))
        r = max(0.2, min(0.8, r))
        skills = [SkillPrimitive(skill="TAKEOFF", params={"z": z, "speed": 0.4})]
        skills.append(SkillPrimitive(skill="CIRCLE", params={"target": "center", "radius": r}))
        return skills
    if intent.intent == "shape":
        name = intent.shape or parse_shape_name(intent.normalized)
        corners = shape_corners(name)
        skills = [SkillPrimitive(skill="TAKEOFF", params={"z": z, "speed": 0.4})]
        skills.extend(closed_move_skills(z, corners))
        if "land" in intent.normalized:
            skills.append(SkillPrimitive(skill="LAND", params={}))
        return skills
    return []
