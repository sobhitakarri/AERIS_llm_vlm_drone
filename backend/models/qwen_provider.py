"""
Local Qwen3 / Qwen3-VL Provider Stub (OpenAI-Compatible Local Endpoint).
"""
from typing import Optional
from backend.models.base_llm import LLMProvider
from backend.models.base_vlm import VLMProvider
from backend.schemas.mission import MissionPlan, SkillPrimitive
from backend.schemas.capabilities import RobotCapabilities
from backend.schemas.perception import DetectedObject
from backend.core.logger import get_logger

logger = get_logger("QwenProvider")


class QwenProvider(LLMProvider, VLMProvider):
    """
    Local foundation model provider stub for Qwen3 / Qwen3-VL.
    Designed for local GPU edge deployments via vLLM / SGLang OpenAI-compatible serving interfaces.
    """

    def __init__(self, endpoint_url: str = "http://localhost:8000/v1"):
        self.endpoint_url = endpoint_url
        logger.info(f"Initialized Local Qwen3 Provider endpoint: {self.endpoint_url}")

    def plan(self, raw_command: str, capabilities: RobotCapabilities, context=None) -> MissionPlan:
        logger.info(f"[Qwen Local] Planning command: '{raw_command}'")
        skills = [
            SkillPrimitive(skill="TAKEOFF", params={"z": 1.0}),
            SkillPrimitive(skill="HOVER", params={"t": 2.0}),
            SkillPrimitive(skill="LAND", params={})
        ]
        return MissionPlan(
            raw_command=raw_command,
            reasoning="Generated via local Qwen3 model backend.",
            skills=skills
        )

    def resolve_target(self, image_bytes: bytes, target_description: str) -> Optional[DetectedObject]:
        logger.info(f"[Qwen3-VL Local] Grounding target: '{target_description}'")
        return None
