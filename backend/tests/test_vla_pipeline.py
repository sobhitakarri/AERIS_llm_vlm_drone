"""
Test UAV-VLA (Vision-Language-Action) Pipeline.
"""
import pytest
import numpy as np
from backend.planning.vla_pipeline import UAVVLAPipeline, GoalExtractor, ObjectSearchVLM, ActionsGenerator
from backend.schemas.capabilities import RobotCapabilities


def test_goal_extractor():
    extractor = GoalExtractor()
    goal = extractor.extract_goal("find the red bottle at 1.2m and hover for 5 seconds then land")
    assert goal.target_object == "red bottle"
    assert goal.target_altitude == 1.2
    assert goal.hover_duration == 5.0
    assert goal.land_at_end is True
    assert goal.task_type == "SEARCH_HOVER"


def test_object_search_mock():
    search = ObjectSearchVLM()
    coords = search.search_and_ground(None, "red bottle")
    assert coords == (0.5, 0.5, 0.0)


def test_uav_vla_end_to_end():
    pipeline = UAVVLAPipeline()
    capabilities = RobotCapabilities()

    dummy_frame = np.zeros((240, 360, 3), dtype=np.uint8)
    plan = pipeline.process("find the barrel at 1.0m and hover for 4s", dummy_frame, capabilities)

    assert len(plan.skills) >= 3
    assert plan.skills[0].skill == "TAKEOFF"
    assert plan.skills[0].params["z"] == 1.0
    assert plan.skills[1].skill == "MOVE_TO"
    assert plan.skills[1].params["x"] == 0.6
    assert plan.skills[1].params["y"] == 0.8
    assert plan.skills[2].skill == "HOVER"
    assert plan.skills[2].params["t"] == 4.0
    assert plan.source == "uav-vla"
