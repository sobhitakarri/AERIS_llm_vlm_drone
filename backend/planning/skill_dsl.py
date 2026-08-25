"""
Drone Skill Domain-Specific Language (DSL) & Primitive Definitions.
"""
from typing import Dict, Any, List
from backend.schemas.mission import SkillPrimitive


class SkillDSL:
    """
    Standardized Drone Skill Action Space.
    Central bridge between AI foundation models and low-level drone interfaces.
    """

    ALLOWED_SKILLS = {
        "TAKEOFF": ["z"],
        "LAND": [],
        "HOVER": ["t"],
        "MOVE_TO": ["x", "y", "z"],
        "ROTATE": ["angle"],
        "CIRCLE": ["target", "radius"],
        "FIND": ["target"],
        "INSPECT": ["target"],
        "RETURN": []
    }

    @classmethod
    def create_skill(cls, name: str, **kwargs) -> SkillPrimitive:
        name_upper = name.upper()
        if name_upper not in cls.ALLOWED_SKILLS:
            raise ValueError(f"Unknown skill '{name}'. Allowed: {list(cls.ALLOWED_SKILLS.keys())}")

        expected_params = cls.ALLOWED_SKILLS[name_upper]
        filtered_params = {k: v for k, v in kwargs.items() if k in expected_params}
        return SkillPrimitive(skill=name_upper, params=filtered_params)

    @classmethod
    def get_json_schema(cls) -> Dict[str, Any]:
        """JSON schema for Gemini Interactions structured output."""
        return {
            "type": "object",
            "properties": {
                "reasoning": {"type": "string"},
                "skills": {
                    "type": "array",
                    "minItems": 1,
                    "items": {
                        "type": "object",
                        "properties": {
                            "skill": {
                                "type": "string",
                                "enum": list(cls.ALLOWED_SKILLS.keys()),
                            },
                            "params": {
                                "type": "object",
                                "properties": {
                                    "x": {"type": "number"},
                                    "y": {"type": "number"},
                                    "z": {"type": "number"},
                                    "t": {"type": "number"},
                                    "angle": {"type": "number"},
                                    "radius": {"type": "number"},
                                    "target": {"type": "string"},
                                },
                            },
                        },
                        "required": ["skill", "params"],
                    },
                },
            },
            "required": ["reasoning", "skills"],
        }
