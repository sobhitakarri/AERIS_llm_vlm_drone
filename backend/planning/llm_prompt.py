"""Natural-language → Skill DSL prompt for the live LLM planner."""
from __future__ import annotations

from typing import Any, Dict, List, Optional

from backend.schemas.capabilities import RobotCapabilities
from backend.schemas.mission import SkillPrimitive


def is_takeoff_only_command(command: str) -> bool:
    cmd = command.lower()
    has_takeoff = any(k in cmd for k in ["take off", "takeoff", "take-off"])
    extra = (
        "fly", "go ", "goto", "move", "navigate", "land", "return", "home",
        "hover", "hold", "rotate", "yaw", "spin", "circle", "inspect",
        "find", "look", "search", "left", "right", "forward", "back",
        "around", "square", "rect", "triangle", "hex", "star", "path",
        "then ", " and ",
    )
    return has_takeoff and not any(k in cmd for k in extra)


def plan_is_too_thin(command: str, skills: List[SkillPrimitive]) -> bool:
    if not skills:
        return True
    names = {s.skill.upper() for s in skills}
    if names <= {"TAKEOFF"}:
        return not is_takeoff_only_command(command)
    return False


def build_planner_prompt(
    raw_command: str,
    capabilities: RobotCapabilities,
    context: Optional[Dict[str, Any]] = None,
) -> str:
    ctx = context or {}
    pose = ctx.get("pose") or {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    objects = ctx.get("objects") or [{"label": "red_bottle", "x": 0.5, "y": 0.5, "z": 0.0}]
    ws = capabilities.workspace
    skills = ", ".join(capabilities.supported_skills)

    return f"""You are the mission parser and planner for a LiteWing indoor micro-UAV
(NeLV-style: extract mission parameters, then output executable skills).
The operator can say ANYTHING. First infer mission_type, altitude, and
points of interest in workspace meters (not GPS). Then translate into
an ordered list of low-level skills. Invent waypoints if needed.
Do not output TAKEOFF-only unless the operator only asked to take off.

Workspace (meters, home is 0,0,0):
  x in [{ws.x_min}, {ws.x_max}], y in [{ws.y_min}, {ws.y_max}], z in [{ws.z_min}, {ws.z_max}]
Current pose: x={pose.get("x", 0):.2f} y={pose.get("y", 0):.2f} z={pose.get("z", 0):.2f} yaw={pose.get("yaw", 0):.1f} deg
Known objects: {objects}

Allowed skills: {skills}
Each item is {{"skill": "NAME", "params": {{...}}}} with params filled in:
  TAKEOFF {{"z": meters}}
  MOVE_TO {{"x": m, "y": m, "z": m}}   world frame, origin at home
  HOVER {{"t": seconds}}
  ROTATE {{"angle": degrees}}         relative yaw
  CIRCLE {{"target": "name", "radius": m}}
  FIND / INSPECT {{"target": "name"}}
  LAND {{}}
  RETURN {{}}                         go above home (0,0)

Rules:
- If the task needs flight and z is near 0, start with TAKEOFF.
- Relative words (forward/back/left/right) are offsets from CURRENT pose, still as MOVE_TO world coordinates.
- Geometric paths: YOU choose vertices that match the request, then close the loop. Stay inside the workspace.
- "inspect/find <object>": FIND or INSPECT that target, MOVE_TO a safe hover near it (about 0.4 m away), HOVER.
- "land" / "come down": LAND. "go home": RETURN.
- Never command coordinates outside the workspace.
- reasoning: one or two sentences of why this skill list matches the command.

Operator command: "{raw_command}"

Return JSON only:
{{
  "reasoning": "why this sequence",
  "skills": [
    {{"skill": "TAKEOFF", "params": {{"z": 1.0}}}}
  ]
}}
"""
