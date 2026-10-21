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


def test_twin_benchmark_covers_these_prompts_with_real_targets():
    import json
    from pathlib import Path
    from backend.scripts.run_twin_benchmark import SCENARIOS

    config = Path(__file__).resolve().parents[2] / "matlab" / "twin" / "config"
    scenes = {name: {o["name"] for o in json.loads((config / f"{name}.json").read_text(encoding="utf-8"))["objects"]}
              for name in ("scene_lab", "scene_distractor")}
    ids = [s["id"] for s in SCENARIOS]
    assert len(ids) == len(set(ids)) and 9 not in ids          # 9 is the replan test
    assert set(VISUAL_PROMPTS + SHAPE_PROMPTS) <= {s["prompt"] for s in SCENARIOS}
    for s in SCENARIOS:
        for scene in s.get("scenes", scenes):
            names = scenes[scene]
            expect = s.get("expect") or []
            if isinstance(expect, str):
                assert expect in names, (s["id"], scene, expect)
            elif expect:
                assert set(expect) & names, (s["id"], scene)
            if s.get("avoid") and s.get("scenes"):
                assert s["avoid"] in names, (s["id"], scene)


def test_distractor_scene_only_adds_look_alikes():
    import json
    import math
    from pathlib import Path

    config = Path(__file__).resolve().parents[2] / "matlab" / "twin" / "config"
    lab = {o["name"]: o for o in json.loads((config / "scene_lab.json").read_text(encoding="utf-8"))["objects"]}
    dis = {o["name"]: o for o in json.loads((config / "scene_distractor.json").read_text(encoding="utf-8"))["objects"]}
    assert set(dis) - set(lab) == {"green_bottle", "red_box"}
    assert all(dis[n] == lab[n] for n in lab)
    # Scenario 22's answer: the green bottle is the one next to the laptop.
    lap = dis["laptop"]["pos"]
    d = {n: math.hypot(dis[n]["pos"][0] - lap[0], dis[n]["pos"][1] - lap[1]) for n in ("green_bottle", "red_bottle")}
    assert d["green_bottle"] < 0.5 < d["red_bottle"]


def test_code_planner_defers_visual_targets_to_run_time():
    skills = plan_from_intent(parse_intent("find the red bottle"))
    assert [s.skill for s in skills] == ["TAKEOFF", "FIND"]
    assert not {"x", "y"} & set(skills[1].params)


def test_shape_without_altitude_flies_at_cruise_height_from_the_ground():
    """Found in the recorded run of case 17: from the pad the star was planned
    at the floor limit (0.25 m) and ran into the 0.23 m crate."""
    on_pad = {"pose": {"x": -1.8, "y": -1.4, "z": 0.0}}
    skills = plan_from_intent(parse_intent("Fly in a star pattern", on_pad), on_pad)
    assert {s.params["z"] for s in skills if s.skill in ("TAKEOFF", "MOVE_TO")} == {0.6}
    airborne = {"pose": {"x": 0.0, "y": 0.0, "z": 1.1}}
    skills = plan_from_intent(parse_intent("Fly in a star pattern", airborne), airborne)
    assert all(s.params["z"] == 1.1 for s in skills if s.skill == "MOVE_TO")
