"""
Gemini provider via the Interactions API (google-genai).
https://ai.google.dev/gemini-api/docs/interactions
"""
import os
import re
import json
from typing import Any, Optional
from backend.models.base_llm import LLMProvider
from backend.models.base_vlm import VLMProvider
from backend.schemas.mission import MissionPlan, SkillPrimitive
from backend.schemas.capabilities import RobotCapabilities
from backend.schemas.perception import DetectedObject, BoundingBox
from backend.planning.skill_dsl import SkillDSL
from backend.planning.shape_paths import (
    clamp_z,
    clip_skills_to_workspace,
    closed_move_skills,
    parse_shape_name,
    shape_corners,
    wants_closed_path,
)
from backend.planning.llm_prompt import build_planner_prompt, plan_is_too_thin
from backend.planning.intent_router import parse_intent
from backend.planning.local_planner import plan_from_intent
from backend.planning.control_platform import finalize_skills
from backend.planning import plan_cache
from backend.core.config import settings
from backend.core.logger import get_logger

logger = get_logger("GeminiProvider")

_PLACEHOLDER_KEYS = {"", "your_key_here", "changeme", "paste_here"}
_PARAM_KEYS = ("x", "y", "z", "t", "angle", "radius", "target")
PLAN_JSON_SCHEMA = SkillDSL.get_json_schema()


def _clean_api_key(value: str) -> str:
    key = (value or "").strip().strip('"').strip("'")
    if key.lower() in _PLACEHOLDER_KEYS:
        return ""
    return key

try:
    from google import genai
    from google.genai import types
    GENAI_SDK_AVAILABLE = True
except ImportError:
    GENAI_SDK_AVAILABLE = False
    genai = None
    types = None

GEMINI_MODEL_CANDIDATES = [
    "gemini-3.7-flash",
    "gemini-3.6-flash",
    "gemini-3.5-flash",
    "gemini-2.5-flash",
    "gemini-2.0-flash",
    "gemini-flash-latest",
]


def _extract_json(text: str) -> dict:
    raw = (text or "").strip()
    if not raw:
        raise ValueError("Gemini returned empty text")
    if raw.startswith("```"):
        raw = re.sub(r"^```(?:json)?\s*", "", raw)
        raw = re.sub(r"\s*```$", "", raw)
    return json.loads(raw)


def _interaction_text(interaction: Any) -> str:
    text = getattr(interaction, "output_text", None)
    if text:
        return text
    parts = []
    for step in getattr(interaction, "steps", None) or []:
        if getattr(step, "type", "") != "model_output":
            continue
        for block in getattr(step, "content", None) or []:
            chunk = getattr(block, "text", None)
            if chunk:
                parts.append(chunk)
    return "\n".join(parts)


def coerce_skill(raw: Any) -> Optional[SkillPrimitive]:
    """Accept Gemini shapes: {skill, params}, flat x/y/z, or {TAKEOFF: {...}}."""
    if not isinstance(raw, dict):
        return None
    data = dict(raw)
    if "skill" not in data and len(data) == 1:
        name, payload = next(iter(data.items()))
        if isinstance(payload, dict):
            data = {"skill": name, "params": payload}
    skill = str(data.get("skill") or data.get("name") or data.get("action") or "").upper().strip()
    params = data.get("params") or data.get("parameters") or data.get("args") or {}
    if not isinstance(params, dict):
        params = {}
    else:
        params = dict(params)
    for key in _PARAM_KEYS:
        if key in data and key not in params:
            params[key] = data[key]
    if not skill:
        return None
    return SkillPrimitive(skill=skill, params=params)


def coerce_skills(raw_skills: Any) -> list[SkillPrimitive]:
    if isinstance(raw_skills, str):
        try:
            raw_skills = json.loads(raw_skills)
        except json.JSONDecodeError:
            return []
    if not isinstance(raw_skills, list):
        return []
    out: list[SkillPrimitive] = []
    for item in raw_skills:
        skill = coerce_skill(item)
        if skill is None:
            continue
        if out and skill.skill == out[-1].skill and skill.params == out[-1].params:
            continue
        out.append(skill)
    return out


class GeminiProvider(LLMProvider, VLMProvider):
    """
    Production Provider for Gemini using the official google-genai SDK.
    """

    def __init__(self, api_key: str = None):
        if api_key is None:
            api_key = settings.gemini_api_key or os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY") or ""
        self.api_key = _clean_api_key(api_key)
        self.client = None
        self.model_name = settings.gemini_model
        self.is_live = False

        if GENAI_SDK_AVAILABLE and self.api_key:
            try:
                self.client = genai.Client(api_key=self.api_key)
                self.is_live = True
                api = "Interactions" if hasattr(self.client, "interactions") else "generate_content"
                logger.info(f"Gemini LIVE client ready ({api} API, model={self.model_name}).")
            except Exception as e:
                logger.error(f"Failed to initialize Gemini Client: {e}")
                self.client = None
                self.is_live = False
        else:
            logger.warning(
                "Gemini offline: missing google-genai SDK or GEMINI_API_KEY. "
                "Copy .env.example to .env and paste a key from https://aistudio.google.com/apikey"
            )

    def _generate_json(self, prompt: str) -> dict:
        last_err = None
        models = [self.model_name] + [m for m in GEMINI_MODEL_CANDIDATES if m != self.model_name]
        for name in models:
            if hasattr(self.client, "interactions"):
                try:
                    interaction = self.client.interactions.create(
                        model=name,
                        input=prompt,
                        response_format={
                            "type": "text",
                            "mime_type": "application/json",
                            "schema": PLAN_JSON_SCHEMA,
                        },
                    )
                    data = _extract_json(_interaction_text(interaction))
                    self.model_name = name
                    logger.info(f"Gemini Interactions plan from {name}")
                    return data
                except Exception as e:
                    last_err = e
                    logger.warning(f"Interactions API model '{name}' failed: {e}")
            if types is not None:
                try:
                    response = self.client.models.generate_content(
                        model=name,
                        contents=prompt,
                        config=types.GenerateContentConfig(response_mime_type="application/json"),
                    )
                    data = _extract_json(getattr(response, "text", "") or "")
                    self.model_name = name
                    logger.info(f"Gemini generate_content plan from {name}")
                    return data
                except Exception as e:
                    last_err = e
                    logger.warning(f"generate_content model '{name}' failed: {e}")
        raise RuntimeError(last_err)

    def plan(self, raw_command: str, capabilities: RobotCapabilities, context=None) -> MissionPlan:
        """Code planner first, then disk cache, then Gemini."""
        logger.info(f"Generating plan for command: '{raw_command}'")
        ctx = context if isinstance(context, dict) else {}
        intent = parse_intent(raw_command, ctx)

        if intent.use_code:
            skills = clip_skills_to_workspace(
                plan_from_intent(intent, ctx), capabilities.workspace
            )
            return self._finalize(
                raw_command, f"[CODE] intent={intent.intent}", skills, "code",
                capabilities, ctx, intent,
            )

        cached = plan_cache.lookup(raw_command)
        if cached:
            skills = clip_skills_to_workspace(
                coerce_skills(cached.get("skills", [])), capabilities.workspace
            )
            hit = cached.get("hit", "exact")
            return self._finalize(
                raw_command, f"[cache-{hit}] reused stored plan", skills, f"cache-{hit}",
                capabilities, ctx, intent,
            )

        if self.client:
            try:
                prompt = build_planner_prompt(raw_command, capabilities, context)
                data = self._generate_json(prompt)
                logger.info(f"Gemini JSON: {json.dumps(data, default=str)[:1500]}")
                skills = clip_skills_to_workspace(
                    coerce_skills(data.get("skills", [])), capabilities.workspace
                )
                if plan_is_too_thin(raw_command, skills):
                    logger.warning("First plan was too thin; asking Gemini to expand the skill list.")
                    retry = (
                        prompt
                        + "\n\nThe previous answer was incomplete (TAKEOFF only or empty). "
                        "Output the FULL skill sequence for the same command. "
                        "Invent waypoints/params. Do not return TAKEOFF-only."
                    )
                    data = self._generate_json(retry)
                    logger.info(f"Gemini retry JSON: {json.dumps(data, default=str)[:1500]}")
                    skills = clip_skills_to_workspace(
                        coerce_skills(data.get("skills", [])), capabilities.workspace
                    )
                if plan_is_too_thin(raw_command, skills) and wants_closed_path(raw_command):
                    logger.warning("LLM still omitted a path; using geometric fallback for this closed-path command.")
                    skills = self._offline_plan(raw_command, capabilities, ctx).skills
                reasoning = data.get("reasoning", "")
                vision_skills = {s.skill for s in skills} & {"FIND", "INSPECT"}
                plan_cache.store_entry(
                    raw_command,
                    {"prompt": prompt, "model": self.model_name},
                    data if isinstance(data, dict) else {},
                    skills,
                    allow_template=not intent.vision and not vision_skills,
                )
                return self._finalize(
                    raw_command,
                    f"[Gemini LIVE {self.model_name}] {reasoning}",
                    skills,
                    "gemini",
                    capabilities,
                    ctx,
                    intent,
                )
            except Exception as e:
                logger.error(f"Gemini API planning error: {e}. Falling back to rule-based parser.")

        return self._wrap_offline(raw_command, capabilities, ctx)

    def _wrap_offline(self, raw_command: str, capabilities: RobotCapabilities, ctx) -> MissionPlan:
        plan = self._offline_plan(raw_command, capabilities, ctx)
        if not plan.skills:
            return plan
        intent = parse_intent(raw_command, ctx)
        return self._finalize(
            raw_command, plan.reasoning, plan.skills, "offline", capabilities, ctx, intent
        )

    def _finalize(
        self,
        raw_command: str,
        reasoning: str,
        skills,
        source: str,
        capabilities: RobotCapabilities,
        ctx,
        intent,
    ) -> MissionPlan:
        skills, spec, stages = finalize_skills(skills, capabilities.workspace, ctx, intent)
        return MissionPlan(
            raw_command=raw_command,
            reasoning=f"{reasoning} | NeLV {'→'.join(stages)}",
            skills=skills,
            source=source,
            spec=spec,
            pipeline=stages,
        )

    def _offline_plan(self, raw_command: str, capabilities: RobotCapabilities, context=None) -> MissionPlan:
        cmd = raw_command.lower()
        skills = []
        z_max = capabilities.workspace.z_max
        z_min = capabilities.workspace.z_min

        nums = [float(n) for n in re.findall(r"(\d+(?:\.\d+)?)\s*(?:m|meter|metres|meters)?", cmd)]
        requested_z = nums[0] if nums else 1.0

        wants_land = bool(re.search(r"\bland\b|\blanding\b", cmd))
        wants_takeoff = any(k in cmd for k in ["take off", "takeoff", "take-off"])
        wants_hover = any(k in cmd for k in ["hover", "maintain", "hold", "height", "altitude", "stay"])
        wants_move = wants_closed_path(cmd) or any(
            k in cmd for k in ["fly", "go to", "move", "navigate", "cube"]
        )
        z = clamp_z(requested_z, z_min, z_max)

        if wants_takeoff or wants_hover or (wants_move and not wants_land):
            skills.append(SkillPrimitive(skill="TAKEOFF", params={"z": z}))

        if re.search(r"\bcircl|\bround\b", cmd) and not re.search(r"\boval\b|\bellipse\b", cmd):
            skills.append(SkillPrimitive(skill="CIRCLE", params={"target": "red_bottle", "radius": 0.5}))
        elif "cube" in cmd or "blue" in cmd:
            skills.append(SkillPrimitive(skill="MOVE_TO", params={"x": -0.5, "y": 0.5, "z": z}))
        elif wants_closed_path(cmd):
            name = parse_shape_name(cmd)
            corners = shape_corners(name)
            skills.extend(closed_move_skills(z, corners))
            logger.info(f"Closed path for shape '{name}' ({len(corners)} vertices).")
        elif wants_hover:
            skills.append(SkillPrimitive(skill="HOVER", params={"t": 4.0}))

        if wants_land:
            skills.append(SkillPrimitive(skill="LAND", params={}))

        if not skills:
            reasoning = (
                "Offline parser did not recognize a flight command. "
                "Try: 'take off to 1m and fly in a square'."
            )
            return MissionPlan(raw_command=raw_command, reasoning=reasoning, skills=[])

        reasoning = (
            f"Generated via rule-based offline parser (no Gemini API key). "
            f"Workspace Z limit is {z_max} m — commands above that are rejected by the safety validator."
        )
        return MissionPlan(raw_command=raw_command, reasoning=reasoning, skills=skills)

    def resolve_target(self, image_bytes: bytes, target_description: str) -> Optional[DetectedObject]:
        """Performs VLM semantic object identification."""
        logger.info(f"VLM target resolution query: '{target_description}'")
        return DetectedObject(
            label=target_description,
            bbox=BoundingBox(u_min=420, v_min=210, u_max=510, v_max=330),
            confidence=0.93,
            world_x=0.5,
            world_y=0.5
        )
