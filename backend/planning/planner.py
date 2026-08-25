"""
Mission Planner Finite State Machine (FSM).
"""
from enum import Enum
from typing import Optional, List
from backend.schemas.mission import MissionPlan, PlanValidationResult
from backend.planning.safety_validator import SafetyValidator
from backend.core.logger import get_logger

logger = get_logger("MissionPlanner")


class PlannerState(str, Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    REPLANNING = "REPLANNING"
    DONE = "DONE"
    ERROR = "ERROR"


class MissionPlanner:
    """
    High-level Mission Planner FSM.
    Coordinates natural language prompt decomposition, plan validation,
    execution triggering, and closed-loop disturbance replanning.
    """

    def __init__(self, validator: SafetyValidator = None):
        self.validator = validator or SafetyValidator()
        self.state = PlannerState.IDLE
        self.current_plan: Optional[MissionPlan] = None
        self.replan_count = 0

    def start_mission(self, raw_command: str) -> PlannerState:
        logger.info(f"FSM State Transition: {self.state.value} -> {PlannerState.PLANNING.value}")
        self.state = PlannerState.PLANNING
        self.replan_count = 0
        return self.state

    def process_generated_plan(self, plan: MissionPlan) -> PlanValidationResult:
        """Processes an LLM-generated plan through the safety validator."""
        validation = self.validator.validate_plan(plan)
        if validation.is_valid:
            self.current_plan = validation.validated_plan
            self.state = PlannerState.EXECUTING
            logger.info(f"FSM State Transition: -> {PlannerState.EXECUTING.value}")
        else:
            self.state = PlannerState.ERROR
            logger.error(f"FSM State Transition: -> {PlannerState.ERROR.value} (Validation Failed)")
        return validation

    def trigger_replan(self, reason: str) -> PlannerState:
        """Triggers closed-loop replanning when visual disturbance is detected."""
        self.replan_count += 1
        logger.warning(f"FSM State Transition: {self.state.value} -> {PlannerState.REPLANNING.value}. Reason: {reason} (Replan #{self.replan_count})")
        self.state = PlannerState.REPLANNING
        return self.state

    def finish_mission(self) -> PlannerState:
        logger.info(f"FSM State Transition: {self.state.value} -> {PlannerState.DONE.value}")
        self.state = PlannerState.DONE
        return self.state

    def reset(self) -> None:
        self.state = PlannerState.IDLE
        self.current_plan = None
        self.replan_count = 0
        logger.info(f"FSM Reset -> {PlannerState.IDLE.value}")
