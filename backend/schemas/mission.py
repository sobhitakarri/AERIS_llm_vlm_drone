"""
Skill Primitive & Mission Plan Schemas.
"""
from typing import List, Dict, Any, Optional
from pydantic import BaseModel, Field


class SkillPrimitive(BaseModel):
    skill: str = Field(..., description="Name of the skill, e.g., TAKEOFF, MOVE_TO, HOVER")
    params: Dict[str, Any] = Field(default_factory=dict, description="Parameters for the skill")


class PointOfInterest(BaseModel):
    x: float
    y: float
    z: float = 0.5
    label: str = ""
    loiter_s: float = 0.0


class MissionSpec(BaseModel):
    """LLM-as-Parser style mission parameters (indoor meters, not lat/lon)."""
    mission_type: str = "custom"
    altitude_m: float = 0.5
    pois: List[PointOfInterest] = Field(default_factory=list)
    land_at_end: bool = False
    takeoff_first: bool = True


class MissionPlan(BaseModel):
    raw_command: str = Field(..., description="Original user natural language command")
    reasoning: str = Field(default="", description="Chain-of-thought planning explanation")
    skills: List[SkillPrimitive] = Field(default_factory=list, description="Sequence of skill primitives")
    source: Optional[str] = Field(default=None, description="code, cache-exact, cache-template, or gemini")
    spec: Optional[MissionSpec] = Field(default=None, description="Parsed mission parameters")
    pipeline: List[str] = Field(default_factory=list, description="NeLV-style stage names that ran")


class PlanValidationResult(BaseModel):
    is_valid: bool = Field(..., description="Whether the plan passed safety checks")
    errors: List[str] = Field(default_factory=list, description="List of safety rejection reasons")
    validated_plan: Optional[MissionPlan] = Field(default=None, description="Plan after validation")
