"""
Abstract LLM Provider Base Class.
"""
from typing import Optional, Dict, Any
from abc import ABC, abstractmethod
from backend.schemas.mission import MissionPlan
from backend.schemas.capabilities import RobotCapabilities


class LLMProvider(ABC):
    """
    Abstract interface for LLM task planning and skill decomposition.
    Framework code depends on this abstraction, making foundation models pluggable.
    """

    @abstractmethod
    def plan(
        self,
        raw_command: str,
        capabilities: RobotCapabilities,
        context: Optional[Dict[str, Any]] = None,
    ) -> MissionPlan:
        """Translates any natural-language command into a Skill DSL MissionPlan."""
        pass
