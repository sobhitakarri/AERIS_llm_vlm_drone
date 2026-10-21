"""Deterministic skill plans for intents that do not need an LLM."""
from __future__ import annotations

from typing import Any, Dict, List

from backend.planning.intent_router import ParsedIntent, parse_intent
from backend.planning.shape_paths import closed_move_skills, parse_shape_name, shape_corners
from backend.schemas.mission import SkillPrimitive

# Cruise height when the command names none and the vehicle is on the ground
# (the UAV-VLA pipeline uses the same). Holding the current height there would
# mean flying at the floor limit, under anything taller than about 0.25 m.
_DEFAULT_ALTITUDE_M = 0.6


def local_plan(command: str, context: Dict[str, Any] | None = None) -> List[SkillPrimitive]:
    intent = parse_intent(command, context)
    return plan_from_intent(intent, context)


def plan_from_intent(intent: ParsedIntent, context: Dict[str, Any] | None = None) -> List[SkillPrimitive]:
    ctx = context or {}
    pose = ctx.get("pose") or {}
    if "z" in intent.slots:
        want = float(intent.slots["z"])
    else:
        here = float(pose.get("z", 0.0) or 0.0)
        want = here if here > 0.2 else _DEFAULT_ALTITUDE_M
    z = max(0.2, min(1.5, want))
    if z != want and "z" in intent.slots:
        from backend.core import trace
        trace.event("workspace_clip", skill="code-planner", param="z", requested=round(want, 3), clipped=z)
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
            SkillPrimitive(skill="RETURN", params={}),
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
    if intent.intent == "vision":
        # No coordinates here: FIND is resolved by the perception stack at
        # run time, and fails if the target cannot be seen.
        return [
            SkillPrimitive(skill="TAKEOFF", params={"z": z, "speed": 0.4}),
            SkillPrimitive(skill="FIND", params={"target": intent.target or "object", "t": hover_t}),
        ]
    return []
