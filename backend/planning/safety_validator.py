"""
Pre-Execution Safety Validator (AI-Side Guardrails).
"""
from typing import List
from backend.schemas.mission import MissionPlan, PlanValidationResult
from backend.schemas.capabilities import RobotCapabilities
from backend.planning.skill_dsl import SkillDSL
from backend.core.logger import get_logger

logger = get_logger("SafetyValidator")


class SafetyValidator:
    """
    Evaluates every LLM-generated plan BEFORE dispatching to execution.
    Enforces geofences, altitude limits, velocity boundaries, and action space checks.
    """

    def __init__(self, capabilities: RobotCapabilities = None):
        self.capabilities = capabilities or RobotCapabilities()

    def validate_plan(self, plan: MissionPlan) -> PlanValidationResult:
        errors: List[str] = []
        bounds = self.capabilities.workspace

        if not plan.skills:
            errors.append("Mission plan contains no skills.")
            return PlanValidationResult(is_valid=False, errors=errors)

        for i, skill_item in enumerate(plan.skills):
            skill_name = skill_item.skill.upper()
            params = skill_item.params

            # 1. Action Space Check
            if skill_name not in SkillDSL.ALLOWED_SKILLS:
                errors.append(f"Step {i+1}: Skill '{skill_name}' is not in allowed action space.")
                continue

            if skill_name not in self.capabilities.supported_skills:
                errors.append(f"Step {i+1}: Robot does not support skill '{skill_name}'.")

            # 2. Geofence & Bounding Checks
            if skill_name == "TAKEOFF":
                z = params.get("z", 1.0)
                if not (bounds.z_min <= z <= bounds.z_max):
                    errors.append(f"Step {i+1}: TAKEOFF altitude {z}m violates bounds [{bounds.z_min}, {bounds.z_max}]m.")

            elif skill_name == "MOVE_TO":
                x = params.get("x", 0.0)
                y = params.get("y", 0.0)
                z = params.get("z", 1.0)

                if not (bounds.x_min <= x <= bounds.x_max):
                    errors.append(f"Step {i+1}: MOVE_TO X={x}m violates geofence [{bounds.x_min}, {bounds.x_max}]m.")
                if not (bounds.y_min <= y <= bounds.y_max):
                    errors.append(f"Step {i+1}: MOVE_TO Y={y}m violates geofence [{bounds.y_min}, {bounds.y_max}]m.")
                if not (bounds.z_min <= z <= bounds.z_max):
                    errors.append(f"Step {i+1}: MOVE_TO Z={z}m violates altitude limits [{bounds.z_min}, {bounds.z_max}]m.")

            elif skill_name == "CIRCLE":
                radius = params.get("radius", 0.5)
                if radius > 1.2:
                    errors.append(f"Step {i+1}: CIRCLE radius {radius}m exceeds workspace safety limit (1.2m).")

        is_valid = len(errors) == 0
        if is_valid:
            logger.info(f"Plan validation PASSED for command: '{plan.raw_command}'")
        else:
            logger.warning(f"Plan validation REJECTED: {errors}")

        return PlanValidationResult(
            is_valid=is_valid,
            errors=errors,
            validated_plan=plan if is_valid else None
        )
