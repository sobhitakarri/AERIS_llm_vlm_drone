"""
UAV-VLA (Vision-Language-Action) mission pipeline.

Follows the three stages of https://github.com/Sautenich/UAV-VLA, indoors and
in metres:
  1. GoalExtractor     what to do and which object the command refers to
  2. ObjectSearch      find that object in the scene camera and ground it
  3. ActionsGenerator  turn goal + grounded position into Skill DSL steps

Nothing here knows where any object is. A target that cannot be seen and
grounded produces no plan: the mission is refused rather than flown to a
guessed position.
"""
from dataclasses import dataclass, field
import math
import re
from typing import Any, List, Optional, Tuple

import numpy as np

from backend.schemas.mission import MissionPlan, SkillPrimitive
from backend.schemas.capabilities import RobotCapabilities
from backend.schemas.perception import DetectedObject
from backend.planning.safety_validator import SafetyValidator
from backend.planning.shape_paths import clip_skills_to_workspace
from backend.core import trace
from backend.core.logger import get_logger

logger = get_logger("UAV-VLA")

_DEFAULT_ALTITUDE_M = 0.6
_DEFAULT_STANDOFF_M = 0.5
_VIEW_CLEARANCE_M = 0.30      # height above whatever is under a viewing position

_ACTION = (
    r"find|locate|search for|look for|go to|fly to|move to|navigate to|approach|inspect|"
    r"look at|point at|face|hover (?:above|over|at|near|beside|next to|facing)|"
    r"circle(?: around)?|orbit(?: around)?|land (?:on|near|next to|beside)"
)
_TARGET = re.compile(rf"\b(?:{_ACTION})\s+(?:the\s+|a\s+|an\s+)?(?P<target>.+)$")
# Where the referring expression ends and the next instruction begins.
# Relational clauses ("left of the blue cube") stay in: the VLM resolves them.
_TARGET_END = re.compile(
    r"\s*(?:,|\band\s+(?:then\s+)?(?:hover|land|inspect|circle|orbit|return|look|fly|go|take|rotate|face|come|stay|wait)\b"
    r"|\bthen\b|\bfrom\s+\d|\bat\s+\d|\bfor\s+\d|\bwith\s+(?:a\s+)?radius\b|\bwhile\b|\bbefore\b|\bafter\b).*$"
)
_CONDITIONAL = re.compile(
    rf"\b(?:{_ACTION})\s+(?:the\s+)?(?P<primary>.+?)\s+if\s+(?P<cond>.+?),?\s+(?:otherwise|else)\s+"
    rf"(?:(?:{_ACTION})\s+)?(?:the\s+)?(?P<fallback>.+)$"
)


@dataclass
class VLAMissionGoal:
    raw_command: str
    task_type: str                      # HOVER_ABOVE | VIEW | CIRCLE | LAND_NEAR
    target_object: Optional[str] = None
    fallback_object: Optional[str] = None
    object_types: List[str] = field(default_factory=list)
    target_altitude: Optional[float] = None
    hover_duration: float = 3.0
    land_at_end: bool = False
    facing_target: bool = False
    standoff_distance: float = 0.0
    above_center: bool = False
    radius: float = 0.5
    condition: Optional[str] = None

    def __post_init__(self):
        if not self.object_types:
            self.object_types = [t for t in (self.target_object, self.fallback_object) if t]


def _clean_phrase(text: str) -> str:
    text = _TARGET_END.sub("", " " + (text or "")).strip()
    text = re.sub(r"\s+(?:it|them|there)$", "", text)
    return text.strip(" .,!?")


class GoalExtractor:
    """Stage 1: natural-language command to a structured goal."""

    def __init__(self, llm_provider=None):
        self.llm = llm_provider

    def extract_goal(self, command: str) -> VLAMissionGoal:
        cmd = re.sub(r"\s+", " ", (command or "").lower().strip())

        standoff = 0.0
        m = re.search(r"(\d+(?:\.\d+)?)\s*(?:m|meters?|metres?)\s*(?:away|back|distance)", cmd)
        if m:
            standoff = float(m.group(1))
        without_standoff = cmd if not m else cmd.replace(m.group(0), " ")

        altitude = None
        m = re.search(r"(?:\bat|\bto|altitude|height)\s*(?:of\s*)?(\d+(?:\.\d+)?)\s*(?:m|meters?|metres?)\b", without_standoff)
        if m:
            altitude = float(m.group(1))

        hover_s = 3.0
        m = re.search(r"(\d+(?:\.\d+)?)\s*(?:s|sec|secs|seconds?)\b", cmd)
        if m:
            hover_s = float(m.group(1))

        radius = 0.5
        m = re.search(r"radius\s*(?:of\s*)?(\d+(?:\.\d+)?)", cmd)
        if m:
            radius = float(m.group(1))

        target, fallback, condition = None, None, None
        m = _CONDITIONAL.search(cmd)
        if m:
            primary = _clean_phrase(m.group("primary"))
            cond = re.sub(r"^(?:it is|it's|its|that is|there is)\s+", "", m.group("cond").strip())
            target = f"{primary} that is {cond}"
            fallback = _clean_phrase(m.group("fallback"))
            condition = cond
        else:
            m = _TARGET.search(cmd)
            if m:
                target = _clean_phrase(m.group("target")) or None

        circle = bool(re.search(r"\b(?:circle|orbit)\b", cmd))
        facing = bool(re.search(r"\b(?:facing|face|look at|point at)\b", cmd))
        inspect = bool(re.search(r"\binspect\b", cmd))
        above = bool(re.search(r"\b(?:above|over|on top of)\b", cmd))
        land_at_end = bool(re.search(r"\bland\b", cmd))

        if circle:
            task = "CIRCLE"
        elif standoff > 0 or (inspect and not above) or (facing and not above):
            task = "VIEW"
        else:
            task = "HOVER_ABOVE"

        goal = VLAMissionGoal(
            raw_command=command, task_type=task, target_object=target, fallback_object=fallback,
            target_altitude=altitude, hover_duration=hover_s, land_at_end=land_at_end,
            facing_target=facing or task == "VIEW", standoff_distance=standoff,
            above_center=above, radius=radius, condition=condition,
        )
        logger.info(
            f"[UAV-VLA: GoalExtractor] task={goal.task_type} target='{goal.target_object}' "
            f"fallback='{goal.fallback_object}' standoff={goal.standoff_distance} alt={goal.target_altitude}"
        )
        return goal


class ObjectSearchVLM:
    """Stage 2: find the target in the scene camera and ground it in metres."""

    def __init__(self, vlm_provider=None, perception=None):
        self.vlm = vlm_provider
        self.perception = perception
        self.last_bbox = None
        self.last_label = None
        self.last_detection: Optional[DetectedObject] = None

    def search_and_ground(self, frame: Optional[np.ndarray], description: str,
                          z_altitude: float = _DEFAULT_ALTITUDE_M) -> Optional[DetectedObject]:
        """Returns the grounded detection, or None if the object is not there."""
        self.last_label, self.last_bbox, self.last_detection = description, None, None
        if frame is None:
            logger.warning("[UAV-VLA: ObjectSearch] No camera frame available.")
            return None
        if self.perception is None or not hasattr(self.perception, "locate"):
            logger.warning("[UAV-VLA: ObjectSearch] No perception stack attached.")
            return None
        det = self.perception.locate(frame, description, z_altitude=z_altitude)
        if det is None or det.world_x is None:
            logger.info(f"[UAV-VLA: ObjectSearch] '{description}' not found in view.")
            return None
        self.last_bbox, self.last_detection = det.bbox, det
        logger.info(
            f"[UAV-VLA: ObjectSearch] '{description}' at ({det.world_x:.2f}, {det.world_y:.2f}) m, "
            f"top {det.top_z if det.top_z is not None else float('nan'):.2f} m [{det.grounding}]"
        )
        return det


class ActionsGenerator:
    """Stage 3: goal + grounded target to a validated skill sequence."""

    def __init__(self, validator: Optional[SafetyValidator] = None, grounding=None):
        self.validator = validator or SafetyValidator()
        self.grounding = grounding

    def _surface_below(self, x: float, y: float) -> float:
        if self.grounding is not None and getattr(self.grounding, "has_depth", False):
            return self.grounding.obstacle_height(x, y, radius=0.2)
        return 0.0

    def generate_actions(self, goal: VLAMissionGoal, det: DetectedObject,
                         capabilities: RobotCapabilities, pose: Optional[dict] = None) -> MissionPlan:
        ws = capabilities.workspace
        pose = pose or {}
        px, py, pz = float(pose.get("x", 0.0)), float(pose.get("y", 0.0)), float(pose.get("z", 0.0))
        airborne = pz > ws.z_min * 0.6
        tx, ty = float(det.world_x), float(det.world_y)
        top = float(det.top_z) if det.top_z is not None else 0.0
        clamp_z = lambda z: round(max(ws.z_min, min(ws.z_max, z)), 2)  # noqa: E731
        cruise = clamp_z(goal.target_altitude if goal.target_altitude else _DEFAULT_ALTITUDE_M)

        skills: List[SkillPrimitive] = []
        if not airborne:
            skills.append(SkillPrimitive(skill="TAKEOFF", params={"z": cruise, "speed": 0.4}))

        # Unit vector from the target toward where the drone is now: the side
        # a stand-off or orbit entry point is placed on.
        ax, ay = px - tx, py - ty
        norm = math.hypot(ax, ay)
        ax, ay = (ax / norm, ay / norm) if norm > 1e-3 else (-1.0, 0.0)

        note = ""
        if goal.task_type == "VIEW":
            d = goal.standoff_distance or _DEFAULT_STANDOFF_M
            vx, vy = tx + d * ax, ty + d * ay
            vx = max(ws.x_min, min(ws.x_max, vx))
            vy = max(ws.y_min, min(ws.y_max, vy))
            vz = clamp_z(max(goal.target_altitude or 0.0, top + 0.10, self._surface_below(vx, vy) + _VIEW_CLEARANCE_M))
            yaw = math.degrees(math.atan2(ty - vy, tx - vx))
            skills.append(SkillPrimitive(skill="MOVE_TO", params={"x": round(vx, 2), "y": round(vy, 2), "z": vz, "speed": 0.4}))
            skills.append(SkillPrimitive(skill="ROTATE", params={"yaw": round(yaw, 1)}))
            skills.append(SkillPrimitive(skill="HOVER", params={"t": goal.hover_duration}))
            note = f"view from {d:.2f} m, heading {yaw:.0f} deg"
        elif goal.task_type == "CIRCLE":
            r = max(0.25, min(1.2, goal.radius))
            cz = clamp_z(max(cruise, top + _VIEW_CLEARANCE_M))
            skills.append(SkillPrimitive(skill="MOVE_TO", params={"x": round(tx + r * ax, 2), "y": round(ty + r * ay, 2), "z": cz, "speed": 0.4}))
            skills.append(SkillPrimitive(skill="CIRCLE", params={"target": goal.target_object, "x": round(tx, 2), "y": round(ty, 2), "radius": r}))
            skills.append(SkillPrimitive(skill="HOVER", params={"t": 2.0}))
            note = f"orbit radius {r:.2f} m"
        else:
            hz = clamp_z(max(cruise, float(det.world_z) if det.world_z is not None else cruise))
            skills.append(SkillPrimitive(skill="MOVE_TO", params={"x": round(tx, 2), "y": round(ty, 2), "z": hz, "speed": 0.4}))
            if goal.facing_target:
                skills.append(SkillPrimitive(skill="ROTATE", params={"yaw": round(math.degrees(math.atan2(-ay, -ax)), 1)}))
            skills.append(SkillPrimitive(skill="HOVER", params={"t": goal.hover_duration}))
            note = f"hover at {hz:.2f} m above it"

        if goal.land_at_end:
            skills.append(SkillPrimitive(skill="LAND", params={}))
        skills = clip_skills_to_workspace(skills, ws)

        how = {"depth": "depth camera", "floor": "floor projection", "nominal": "uncalibrated camera"}.get(det.grounding or "", "camera")
        reasoning = (
            f"Target '{det.label}' grounded by {how} at ({tx:.2f}, {ty:.2f}) m"
            + (f", top {top:.2f} m" if det.top_z is not None else "")
            + (f", standing on a {det.support_z:.2f} m surface" if det.support_z else "")
            + f". Plan: {note}."
        )
        if goal.condition:
            reasoning += f" Condition checked: '{goal.condition}'."

        plan = MissionPlan(
            raw_command=goal.raw_command, reasoning=reasoning, skills=skills, source="uav-vla",
            pipeline=["uav-vla-goal-extractor", "uav-vla-object-search", "uav-vla-actions-generator"],
        )
        self.validator.capabilities = capabilities
        validation = self.validator.validate_plan(plan)
        if not validation.is_valid:
            logger.warning(f"[UAV-VLA: ActionsGenerator] Plan failed validation: {validation.errors}")
        return plan


class UAVVLAPipeline:
    """Goal extraction, object search and action generation in sequence."""

    def __init__(self, vlm_provider=None, llm_provider=None, perception=None, grounding=None):
        self.goal_extractor = GoalExtractor(llm_provider)
        self.object_search = ObjectSearchVLM(vlm_provider=vlm_provider, perception=perception)
        self.actions_generator = ActionsGenerator(grounding=grounding)

    def attach(self, perception=None, grounding=None) -> None:
        if perception is not None:
            self.object_search.perception = perception
        if grounding is not None:
            self.actions_generator.grounding = grounding

    def process(self, raw_command: str, camera_frame: Optional[np.ndarray],
                capabilities: RobotCapabilities, pose: Optional[dict] = None,
                goal: Optional[VLAMissionGoal] = None) -> MissionPlan:
        goal = goal or self.goal_extractor.extract_goal(raw_command)
        stages = ["uav-vla-goal-extractor", "uav-vla-object-search"]
        if not goal.target_object:
            return MissionPlan(raw_command=raw_command, source="uav-vla", pipeline=stages[:1], skills=[],
                               reasoning="Could not work out which object the command refers to.")

        z = goal.target_altitude or _DEFAULT_ALTITUDE_M
        with trace.stage("object_search"):
            det = self.object_search.search_and_ground(camera_frame, goal.target_object, z)
        tried = [goal.target_object]
        if det is None and goal.fallback_object:
            tried.append(goal.fallback_object)
            with trace.stage("object_search"):
                det = self.object_search.search_and_ground(camera_frame, goal.fallback_object, z)
        trace.set_planner(goal_task=goal.task_type, goal_target=goal.target_object, goal_fallback=goal.fallback_object,
                          searched=tried, grounded=det.label if det is not None else None,
                          grounding=det.grounding if det is not None else None,
                          grounded_xy=[det.world_x, det.world_y] if det is not None else None)
        if det is None:
            why = "no camera frame" if camera_frame is None else "not found in the camera view, or nothing physical is at the detected location"
            return MissionPlan(
                raw_command=raw_command, source="uav-vla", pipeline=stages, skills=[],
                reasoning=f"Target {' / '.join(repr(t) for t in tried)}: {why}. No plan generated.",
            )
        with trace.stage("actions"):
            return self.actions_generator.generate_actions(goal, det, capabilities, pose)
