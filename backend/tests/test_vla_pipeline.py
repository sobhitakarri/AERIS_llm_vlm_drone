"""
UAV-VLA pipeline: goal extraction, grounding hand-off and action generation.
No test here supplies object coordinates through anything but a detection.
"""
import math

import numpy as np
import pytest

from backend.planning.vla_pipeline import UAVVLAPipeline, GoalExtractor, ObjectSearchVLM
from backend.schemas.capabilities import RobotCapabilities, WorkspaceBounds
from backend.schemas.perception import BoundingBox, DetectedObject


class FakePerception:
    """Stands in for FastPerception.locate: returns what a detector would."""

    def __init__(self, objects):
        self.objects = objects
        self.asked = []

    def locate(self, frame, label, z_altitude=0.6):
        self.asked.append(label)
        for key, (x, y, top) in self.objects.items():
            if key in label:
                return DetectedObject(
                    label=label, bbox=BoundingBox(u_min=10, v_min=10, u_max=40, v_max=40),
                    world_x=x, world_y=y, top_z=top, world_z=max(z_altitude, top + 0.35), grounding="depth",
                )
        return None


@pytest.fixture
def caps():
    return RobotCapabilities(workspace=WorkspaceBounds(x_min=-2.3, x_max=2.3, y_min=-1.9, y_max=1.9, z_min=0.25, z_max=2.0))


FRAME = np.zeros((240, 320, 3), np.uint8)


def test_goal_extractor_basic_fields():
    goal = GoalExtractor().extract_goal("find the red bottle at 1.2m and hover for 5 seconds then land")
    assert goal.target_object == "red bottle"
    assert goal.target_altitude == 1.2
    assert goal.hover_duration == 5.0
    assert goal.land_at_end is True
    assert goal.task_type == "HOVER_ABOVE"


@pytest.mark.parametrize("command,target,task", [
    ("Find cardboard box and hover it", "cardboard box", "HOVER_ABOVE"),
    ("Find the shiny red bottle next to the blue cube and hover facing it", "shiny red bottle next to the blue cube", "VIEW"),
    ("Look at the cylindrical object and hover above its center", "cylindrical object", "HOVER_ABOVE"),
    ("Hover at the end of the wall", "end of the wall", "HOVER_ABOVE"),
    ("circle around the traffic cone with radius 0.6 and land", "traffic cone", "CIRCLE"),
    ("approach the potted plant", "potted plant", "HOVER_ABOVE"),
])
def test_goal_extractor_keeps_the_referring_expression(command, target, task):
    goal = GoalExtractor().extract_goal(command)
    assert goal.target_object == target
    assert goal.task_type == task


def test_goal_extractor_standoff_is_not_an_altitude():
    goal = GoalExtractor().extract_goal("Find the bottle that is left of the blue cube and inspect it from 0.4 m away")
    assert goal.standoff_distance == 0.4
    assert goal.target_altitude is None
    assert goal.task_type == "VIEW"
    assert goal.target_object == "bottle that is left of the blue cube"


def test_goal_extractor_conditional_has_a_fallback():
    goal = GoalExtractor().extract_goal("Find the red bottle if it is in front of the cube, otherwise look at the cube")
    assert goal.target_object == "red bottle that is in front of the cube"
    assert goal.fallback_object == "cube"
    assert goal.condition == "in front of the cube"


def test_object_search_without_a_detector_finds_nothing():
    assert ObjectSearchVLM().search_and_ground(FRAME, "red bottle") is None
    assert ObjectSearchVLM(perception=FakePerception({})).search_and_ground(None, "red bottle") is None


def test_unseen_target_produces_no_plan(caps):
    pipeline = UAVVLAPipeline(perception=FakePerception({}))
    plan = pipeline.process("find the barrel and hover", FRAME, caps)
    assert plan.skills == []
    assert "not found" in plan.reasoning


def test_hover_above_uses_the_detected_position(caps):
    pipeline = UAVVLAPipeline(perception=FakePerception({"red bottle": (1.2, 1.2, 1.01)}))
    plan = pipeline.process("find the red bottle and hover for 4s", FRAME, caps, pose={"x": -1.8, "y": -1.4, "z": 0.0})
    names = [s.skill for s in plan.skills]
    assert names == ["TAKEOFF", "MOVE_TO", "HOVER"]
    move = plan.skills[1].params
    assert (move["x"], move["y"]) == (1.2, 1.2)
    assert move["z"] >= 1.01 + 0.3            # clear of the object's top
    assert plan.skills[2].params["t"] == 4.0
    assert plan.source == "uav-vla"


def test_view_stands_off_on_the_approach_side_and_faces_the_target(caps):
    pipeline = UAVVLAPipeline(perception=FakePerception({"blue cube": (1.0, 0.0, 0.3)}))
    plan = pipeline.process("inspect the blue cube from 0.5 m away", FRAME, caps, pose={"x": -1.0, "y": 0.0, "z": 0.0})
    move = next(s for s in plan.skills if s.skill == "MOVE_TO").params
    assert move["x"] == pytest.approx(0.5, abs=0.02) and move["y"] == pytest.approx(0.0, abs=0.02)
    yaw = next(s for s in plan.skills if s.skill == "ROTATE").params["yaw"]
    assert math.cos(math.radians(yaw)) == pytest.approx(1.0, abs=0.01)   # looking along +x at the cube


def test_conditional_falls_back_when_primary_is_absent(caps):
    perception = FakePerception({"cube": (0.5, 0.5, 0.2)})
    perception.objects = {"cube": (0.5, 0.5, 0.2)}
    # The primary phrase also contains "cube"; make the fake reject it.
    real_locate = perception.locate
    perception.locate = lambda frame, label, z_altitude=0.6: None if "bottle" in label else real_locate(frame, label, z_altitude)
    pipeline = UAVVLAPipeline(perception=perception)
    plan = pipeline.process("Find the red bottle if it is in front of the cube, otherwise look at the cube", FRAME, caps)
    assert plan.skills, plan.reasoning
    assert "cube" in plan.reasoning and "bottle" not in plan.reasoning.split("grounded")[0]


def test_plan_stays_inside_the_workspace(caps):
    pipeline = UAVVLAPipeline(perception=FakePerception({"box": (2.25, 1.85, 0.4)}))
    plan = pipeline.process("inspect the box from 1.0 m away", FRAME, caps, pose={"x": 5.0, "y": 5.0, "z": 0.0})
    for s in plan.skills:
        if s.skill == "MOVE_TO":
            assert caps.workspace.x_min <= s.params["x"] <= caps.workspace.x_max
            assert caps.workspace.y_min <= s.params["y"] <= caps.workspace.y_max
            assert caps.workspace.z_min <= s.params["z"] <= caps.workspace.z_max
