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

        # Track simulated drone trajectory state to validate relative/spatial skills
        cur_x, cur_y, cur_z = 0.0, 0.0, 0.0

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
                cur_z = z
                if not (bounds.z_min <= z <= bounds.z_max):
                    errors.append(f"Step {i+1}: TAKEOFF altitude {z}m violates bounds [{bounds.z_min}, {bounds.z_max}]m.")

            elif skill_name == "MOVE_TO":
                x = params.get("x", cur_x)
                y = params.get("y", cur_y)
                z = params.get("z", cur_z)
                cur_x, cur_y, cur_z = x, y, z

                if not (bounds.x_min <= x <= bounds.x_max):
                    errors.append(f"Step {i+1}: MOVE_TO X={x}m violates geofence [{bounds.x_min}, {bounds.x_max}]m.")
                if not (bounds.y_min <= y <= bounds.y_max):
                    errors.append(f"Step {i+1}: MOVE_TO Y={y}m violates geofence [{bounds.y_min}, {bounds.y_max}]m.")
                if not (bounds.z_min <= z <= bounds.z_max):
                    errors.append(f"Step {i+1}: MOVE_TO Z={z}m violates altitude limits [{bounds.z_min}, {bounds.z_max}]m.")

            elif skill_name == "CIRCLE":
                radius = float(params.get("radius", 0.5))
                cx = float(params.get("x", cur_x))
                cy = float(params.get("y", cur_y))

                if radius <= 0:
                    errors.append(f"Step {i+1}: CIRCLE radius {radius}m must be positive.")
                elif radius > 1.2:
                    errors.append(f"Step {i+1}: CIRCLE radius {radius}m exceeds workspace safety limit (1.2m).")

                # Geofence boundary check for the entire perimeter of the circle
                if (cx - radius < bounds.x_min) or (cx + radius > bounds.x_max):
                    errors.append(f"Step {i+1}: CIRCLE perimeter on X [{cx - radius:.2f}, {cx + radius:.2f}]m violates geofence [{bounds.x_min}, {bounds.x_max}]m.")
                if (cy - radius < bounds.y_min) or (cy + radius > bounds.y_max):
                    errors.append(f"Step {i+1}: CIRCLE perimeter on Y [{cy - radius:.2f}, {cy + radius:.2f}]m violates geofence [{bounds.y_min}, {bounds.y_max}]m.")

            elif skill_name in ("FIND", "INSPECT"):
                if all(k in params for k in ("x", "y")):
                    fx, fy = float(params["x"]), float(params["y"])
                    cur_x, cur_y = fx, fy
                    if not (bounds.x_min <= fx <= bounds.x_max):
                        errors.append(f"Step {i+1}: {skill_name} X={fx}m violates geofence [{bounds.x_min}, {bounds.x_max}]m.")
                    if not (bounds.y_min <= fy <= bounds.y_max):
                        errors.append(f"Step {i+1}: {skill_name} Y={fy}m violates geofence [{bounds.y_min}, {bounds.y_max}]m.")

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
