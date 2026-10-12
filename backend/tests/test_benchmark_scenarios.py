"""
The eight benchmark prompts, checked for what the software can decide from the
text alone: routing and goal structure. Where the drone actually goes is a
property of perception and flight, and is measured against the digital twin
by backend/scripts/run_twin_benchmark.py, not asserted here.
"""
import pytest

from backend.planning.intent_router import parse_intent
from backend.planning.local_planner import plan_from_intent
from backend.planning.vla_pipeline import GoalExtractor, UAVVLAPipeline
from backend.schemas.capabilities import RobotCapabilities

VISUAL_PROMPTS = [
    "Find cardboard box and hover it",
    "Find the shiny red bottle next to the blue cube and hover facing it",
    "Find the bottle that is left of the blue cube and inspect it from 0.4 m away",
    "Find the red bottle if it is in front of the cube, otherwise look at the cube",
    "Look at the cylindrical object and hover above its center",
    "Hover at the end of the wall",
]
SHAPE_PROMPTS = ["Fly 0.5m and make a square in air", "Fly in a star pattern"]


@pytest.mark.parametrize("prompt", VISUAL_PROMPTS)
def test_visual_prompts_route_to_the_vla_pipeline(prompt):
    intent = parse_intent(prompt)
    assert intent.intent == "vision" and intent.vision
    assert GoalExtractor().extract_goal(prompt).target_object


@pytest.mark.parametrize("prompt", SHAPE_PROMPTS)
def test_shape_prompts_are_planned_without_a_model(prompt):
    intent = parse_intent(prompt)
    assert intent.intent == "shape" and not intent.vision
    skills = plan_from_intent(intent)
    assert skills[0].skill == "TAKEOFF"
    assert sum(s.skill == "MOVE_TO" for s in skills) >= 4


def test_square_honours_the_requested_altitude():
    skills = plan_from_intent(parse_intent("Fly 0.5m and make a square in air"))
    assert skills[0].params["z"] == 0.5
    assert all(s.params["z"] == 0.5 for s in skills if s.skill == "MOVE_TO")


@pytest.mark.parametrize("prompt", VISUAL_PROMPTS)
def test_no_visual_prompt_yields_a_plan_without_perception(prompt):
    """With nothing to see with, a visual mission must be refused, never guessed."""
    plan = UAVVLAPipeline().process(prompt, None, RobotCapabilities())
    assert plan.skills == []


def test_code_planner_defers_visual_targets_to_run_time():
    skills = plan_from_intent(parse_intent("find the red bottle"))
    assert [s.skill for s in skills] == ["TAKEOFF", "FIND"]
    assert not {"x", "y"} & set(skills[1].params)
