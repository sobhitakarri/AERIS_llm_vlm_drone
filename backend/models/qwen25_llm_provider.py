"""
Qwen2.5-3B Local LLM Planning Provider via Ollama.
Used exclusively for natural language - skill plan decomposition.
VLM visual grounding stays on QwenProvider (Qwen2-VL-2B).

Why separate:
- Qwen2-VL-2B is a vision-language model: good at bounding boxes, poor at JSON planning
- Qwen2.5-3B is a pure-text instruct model: much better instruction following and JSON output
- Both fit in 4GB VRAM (never loaded simultaneously - Ollama swaps them)
"""
import json
import re
from typing import Any, Dict, List, Optional

from backend.models.base_llm import LLMProvider
from backend.schemas.mission import MissionPlan, SkillPrimitive
from backend.schemas.capabilities import RobotCapabilities
from backend.core.logger import get_logger
from backend.planning.intent_router import parse_intent
from backend.planning.local_planner import plan_from_intent

import urllib.request

logger = get_logger("Qwen25LLM")

_DEFAULT_MODEL = "qwen2.5:3b"
_OLLAMA_URL = "http://localhost:11434"


def _call_ollama(
    model: str,
    messages: List[Dict[str, Any]],
    *,
    temperature: float = 0.05,
    num_predict: int = 1200,
) -> str:
    payload = {
        "model": model,
        "messages": messages,
        "stream": False,
        "format": "json",
        "options": {
            "temperature": temperature,
            "num_predict": num_predict,
        },
    }
    body = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        f"{_OLLAMA_URL}/api/chat",
        data=body,
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=90) as resp:
        data = json.loads(resp.read().decode())
    return data.get("message", {}).get("content", "")


def _build_system_prompt(capabilities: RobotCapabilities, context: Optional[Dict]) -> str:
    ctx = context or {}
    pose = ctx.get("pose") or {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    objects = ctx.get("objects") or []
    ws = capabilities.workspace
    obj_str = json.dumps(objects) if objects else "[]"

    return f"""You are the flight planner for an indoor micro-UAV (quadrotor).
Your ONLY job: read the operator command and output a JSON flight plan.

Workspace (meters, home = origin 0,0,0):
  x in [{ws.x_min:.1f}, {ws.x_max:.1f}],  y in [{ws.y_min:.1f}, {ws.y_max:.1f}],  z in [{ws.z_min:.1f}, {ws.z_max:.1f}]
Current pose: x={pose.get('x', 0):.2f} y={pose.get('y', 0):.2f} z={pose.get('z', 0):.2f} yaw={pose.get('yaw', 0):.1f} deg
Known objects in scene: {obj_str}

Allowed skills:
  TAKEOFF  {{"z": meters, "speed": 0.4}}
  MOVE_TO  {{"x": m, "y": m, "z": m, "speed": 0.4}}
  HOVER    {{"t": seconds}}
  ROTATE   {{"angle": degrees}}
  CIRCLE   {{"radius": m}}
  FIND     {{"target": "name"}}
  INSPECT  {{"target": "name"}}
  LAND     {{}}
  RETURN   {{}}

Rules:
1. Always start airborne tasks with TAKEOFF if z near 0.
2. Geometric paths: generate ALL vertices. A square = 5 MOVE_TO (4 corners + close).
3. patrol perimeter: 4 MOVE_TO corner waypoints then LAND.
4. survey/scan area: lawnmower S-path across workspace (6-8 MOVE_TO rows).
5. if you see X, circle it: FIND X, then CIRCLE radius 0.4.
6. circle/orbit X: FIND X, MOVE_TO object, CIRCLE radius 0.4.
7. inspect all objects: TAKEOFF, then per object: FIND + MOVE_TO + HOVER.
8. NEVER output coordinates outside workspace.
9. reasoning: 1-2 sentences.

Output JSON only:
{{
  "reasoning": "...",
  "skills": [{{"skill": "NAME", "params": {{...}}}}]
}}"""


class Qwen25LLMProvider(LLMProvider):
    """
    Qwen2.5-3B instruct model for LLM planning.
    Handles compound commands, high-level reasoning, conditional logic,
    novel shapes, patrol/survey/inspect-all patterns.
    Does NOT handle VLM (bounding boxes) - that stays on QwenProvider.
    """

    def __init__(
        self,
        endpoint_url: str = _OLLAMA_URL,
        model_name: str = _DEFAULT_MODEL,
    ):
        self.endpoint_url = endpoint_url.rstrip("/")
        self.model_name = model_name
        self.is_live = False
        self._check_connection()

    def _check_connection(self) -> bool:
        try:
            req = urllib.request.Request(f"{self.endpoint_url}/api/tags")
            with urllib.request.urlopen(req, timeout=3) as resp:
                data = json.loads(resp.read().decode())
                models = [m.get("name") for m in data.get("models", [])]
                for candidate in [self.model_name, "qwen2.5:3b", "qwen2.5:7b"]:
                    if candidate in models:
                        self.model_name = candidate
                        self.is_live = True
                        logger.info(f"[Qwen25LLM] Connected. Model: '{self.model_name}'")
                        return True
                for m in models:
                    if "qwen2.5" in m.lower():
                        self.model_name = m
                        self.is_live = True
                        logger.info(f"[Qwen25LLM] Connected (fallback). Model: '{self.model_name}'")
                        return True
                logger.warning(f"[Qwen25LLM] No qwen2.5 model found. Available: {models}")
                return False
        except Exception as e:
            logger.warning(f"[Qwen25LLM] Ollama check failed: {e}")
            return False

    def plan(self, raw_command: str, capabilities: RobotCapabilities, context=None) -> MissionPlan:
        logger.info(f"[Qwen25LLM] Planning: '{raw_command}'")

        # Step 1: Deterministic intent routing first
        try:
            intent = parse_intent(raw_command, context)
            logger.info(f"[Qwen25LLM] Intent: '{intent.intent}', use_code={intent.use_code}")
            if intent.use_code and intent.intent != "unknown":
                skills = plan_from_intent(intent, context)
                if skills:
                    return MissionPlan(
                        raw_command=raw_command,
                        reasoning=(
                            f"Deterministic plan: intent='{intent.intent}'"
                            + (f", shape='{intent.shape}'" if intent.shape else "")
                            + f". {len(skills)} skill(s) via local planner."
                        ),
                        skills=skills,
                        source="local-planner",
                        pipeline=["intent-router", "local-planner"],
                    )
        except Exception as e:
            logger.warning(f"[Qwen25LLM] Intent routing error: {e}")

        # Step 2: Qwen2.5-3B for unknown/compound/high-level
        if not self.is_live:
            logger.warning("[Qwen25LLM] Model offline, returning minimal fallback.")
            return self._minimal_fallback(raw_command)

        system_prompt = _build_system_prompt(capabilities, context)
        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user",   "content": f"Operator command: \"{raw_command}\""},
        ]

        try:
            content = _call_ollama(self.model_name, messages)
            logger.info(f"[Qwen25LLM] Response: {content[:300]}")
            data = json.loads(content)
            skills = [
                SkillPrimitive(skill=item["skill"].upper(), params=item.get("params", {}))
                for item in data.get("skills", [])
                if "skill" in item
            ]
            reasoning = data.get("reasoning", "Generated by Qwen2.5-3B.")

            # Sanity check + one retry for geometric commands
            geo_kw = ["square", "triangle", "pentagon", "hexagon", "star",
                      "patrol", "survey", "perimeter", "lawnmower", "figure"]
            is_geo = any(kw in raw_command.lower() for kw in geo_kw)
            move_count = sum(1 for s in skills if s.skill == "MOVE_TO")
            if is_geo and move_count < 3:
                logger.warning(f"[Qwen25LLM] Only {move_count} MOVE_TO for geometric cmd - retrying.")
                messages.append({"role": "assistant", "content": content})
                messages.append({
                    "role": "user",
                    "content": (
                        "Your answer was incomplete. A square needs 5 MOVE_TO steps. "
                        "Patrol needs 4 border waypoints. Output the COMPLETE corrected JSON now."
                    ),
                })
                content2 = _call_ollama(self.model_name, messages)
                data2 = json.loads(content2)
                skills2 = [
                    SkillPrimitive(skill=item["skill"].upper(), params=item.get("params", {}))
                    for item in data2.get("skills", [])
                    if "skill" in item
                ]
                if sum(1 for s in skills2 if s.skill == "MOVE_TO") > move_count:
                    skills, reasoning = skills2, data2.get("reasoning", reasoning) + " [retry]"

            return MissionPlan(
                raw_command=raw_command,
                reasoning=reasoning,
                skills=skills,
                source="qwen2.5-3b",
                pipeline=["intent-router", "qwen2.5-3b-ollama"],
            )

        except Exception as e:
            logger.error(f"[Qwen25LLM] LLM call failed: {e}")
            return self._minimal_fallback(raw_command)

    def _minimal_fallback(self, raw_command: str) -> MissionPlan:
        alt = 1.0
        m = re.search(r"(\d+(?:\.\d+)?)\s*m", raw_command.lower())
        if m:
            alt = float(m.group(1))
        return MissionPlan(
            raw_command=raw_command,
            reasoning="Minimal fallback: Qwen2.5-3B unavailable.",
            skills=[
                SkillPrimitive(skill="TAKEOFF", params={"z": alt, "speed": 0.4}),
                SkillPrimitive(skill="HOVER",   params={"t": 3.0}),
                SkillPrimitive(skill="LAND",    params={}),
            ],
            source="qwen25-fallback",
        )
