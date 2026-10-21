"""
A target-directed mission whose target disappears must not fly on to where
the target was (found by the remove_target experiment: perception reported
TARGET_LOST, yet the vehicle completed the plan over the empty spot).
"""
import threading

import pytest

from backend.drone.sim_interface import SimInterface
from backend.runtime import mission_executor as me
from backend.runtime.mission_executor import MissionExecutor
from backend.runtime.state_manager import StateManager
from backend.schemas.mission import MissionPlan, SkillPrimitive
from backend.schemas.perception import BoundingBox, DetectedObject

BOX = BoundingBox(u_min=100, v_min=100, u_max=120, v_max=130)

TARGET = "red bottle"


@pytest.fixture
def setup(monkeypatch):
    monkeypatch.setattr(me, "_REACQUIRE_S", 1.0)
    drone = SimInterface()
    drone.connect()
    sm = StateManager()
    sm.update_object(DetectedObject(label=TARGET, bbox=BOX, world_x=1.2, world_y=1.2, world_z=1.0, confidence=0.9))
    ex = MissionExecutor(drone=drone, state_manager=sm)
    ex.perception_live = False
    plan = MissionPlan(raw_command="find the red bottle and hover above it", skills=[
        SkillPrimitive(skill="TAKEOFF", params={"z": 0.6}),
        SkillPrimitive(skill="MOVE_TO", params={"x": 1.2, "y": 1.2, "z": 1.0}),
        SkillPrimitive(skill="HOVER", params={"t": 0.1}),
    ])
    return drone, sm, ex, plan


def _lose_after_takeoff(ex, sm, done, then=None):
    def on_step(skill):
        done.append(skill)
        if skill == "TAKEOFF":
            sm.mark_lost(TARGET)
            ex.note_target_lost(TARGET)
            if then:
                then()
    return on_step


def test_lost_target_stops_the_mission_and_goes_home(setup):
    drone, sm, ex, plan = setup
    ex.set_mission_target(TARGET)
    done = []
    ok = ex.execute_plan(plan, on_step_complete=_lose_after_takeoff(ex, sm, done))
    assert ok is False
    assert done == ["TAKEOFF"]                         # never flew to the target or hovered there
    assert "lost" in ex.last_error
    st = drone.get_state()
    hx, hy = drone.home_position()
    assert abs(st.x - hx) < 0.1 and abs(st.y - hy) < 0.1 and st.z < 0.1
    assert ex.monitor.status == "error"
    assert ex.mission_target is None                   # cleared for the next mission


def test_target_found_again_lets_the_mission_continue(setup):
    drone, sm, ex, plan = setup
    ex.set_mission_target(TARGET)
    found = lambda: threading.Timer(0.3, sm.update_object, [DetectedObject(  # noqa: E731
        label=TARGET, bbox=BOX, world_x=1.2, world_y=1.2, world_z=1.0, confidence=0.9)]).start()
    done = []
    assert ex.execute_plan(plan, on_step_complete=_lose_after_takeoff(ex, sm, done, then=found)) is True
    assert done == ["TAKEOFF", "MOVE_TO", "HOVER"]


def test_missions_without_a_target_ignore_lost_events(setup):
    drone, sm, ex, plan = setup
    ex.set_mission_target(None)                         # e.g. a shape flight, or the open-loop baseline
    assert ex.execute_plan(plan, on_step_complete=_lose_after_takeoff(ex, sm, [])) is True


def test_a_different_label_does_not_stop_the_mission(setup):
    drone, sm, ex, plan = setup
    ex.set_mission_target("blue cube")
    assert ex.execute_plan(plan, on_step_complete=_lose_after_takeoff(ex, sm, [])) is True


def test_reappearing_far_away_is_another_object_not_the_target(setup):
    """Seen in the twin: after the red bottle was removed, the VLM boxed the red
    fire extinguisher 4.6 m away as 'red bottle'. That is not the target found again."""
    drone, sm, ex, plan = setup
    ex.set_mission_target(TARGET, (1.2, 1.2))
    impostor = lambda: threading.Timer(0.3, sm.update_object, [DetectedObject(  # noqa: E731
        label=TARGET, bbox=BOX, world_x=-2.74, world_y=0.30, world_z=1.0, confidence=0.9)]).start()
    done = []
    assert ex.execute_plan(plan, on_step_complete=_lose_after_takeoff(ex, sm, done, then=impostor)) is False
    assert done == ["TAKEOFF"]


def test_found_again_near_an_accepted_replan_position(setup):
    drone, sm, ex, plan = setup
    ex.set_mission_target(TARGET, (1.2, 1.2))
    ex.update_target_position(1.5, 1.0)                 # a replan moved it here
    back = lambda: threading.Timer(0.3, sm.update_object, [DetectedObject(  # noqa: E731
        label=TARGET, bbox=BOX, world_x=1.55, world_y=1.05, world_z=1.0, confidence=0.9)]).start()
    assert ex.execute_plan(plan, on_step_complete=_lose_after_takeoff(ex, sm, [], then=back)) is True
