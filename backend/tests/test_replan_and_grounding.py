"""ReplanManager + homography load/save + mid-mission inject."""
import json
import tempfile
from pathlib import Path

from backend.runtime.event_bus import EventBus, EVENT_OBJECT_DETECTED
from backend.runtime.replan_manager import ReplanManager
from backend.runtime.mission_executor import MissionExecutor
from backend.drone.sim_interface import SimInterface
from backend.schemas.mission import MissionPlan, SkillPrimitive
from backend.vision.spatial_grounding import SpatialGrounding


def test_replan_triggers_on_drift():
    bus = EventBus()
    hits = []
    mgr = ReplanManager(
        event_bus=bus,
        replan_callback=lambda label, info: hits.append(info),
        distance_threshold=0.3,
        cooldown_s=0.0,
        mission_active=lambda: True,
    )
    mgr.start()
    bus.publish(EVENT_OBJECT_DETECTED, {"label": "red_bottle", "world_x": 0.5, "world_y": 0.5})
    for _ in range(3):  # drift must persist to clear the jitter debounce
        bus.publish(EVENT_OBJECT_DETECTED, {"label": "red_bottle", "world_x": 0.9, "world_y": 0.5})
    assert mgr.replan_count == 1
    assert hits and hits[0]["drift"] >= 0.3


def test_replan_ignores_non_actionable_detections():
    """TARGET_LOST detections must not independently generate a MOVE_TO."""
    bus = EventBus()
    hits = []
    mgr = ReplanManager(
        event_bus=bus,
        replan_callback=lambda label, info: hits.append(info),
        distance_threshold=0.3,
        cooldown_s=0.0,
        mission_active=lambda: True,
    )
    mgr.start()
    bus.publish(EVENT_OBJECT_DETECTED, {"label": "red_bottle", "world_x": 0.5, "world_y": 0.5})
    for _ in range(3):
        bus.publish(EVENT_OBJECT_DETECTED, {
            "label": "red_bottle", "world_x": 0.9, "world_y": 0.5, "actionable": False,
        })
    assert hits == []
    assert mgr.replan_count == 0


def test_replan_debounces_single_frame_jitter():
    """One bad detection frame must not trigger a replan."""
    bus = EventBus()
    hits = []
    mgr = ReplanManager(
        event_bus=bus,
        replan_callback=lambda label, info: hits.append(info),
        distance_threshold=0.3,
        cooldown_s=0.0,
        mission_active=lambda: True,
    )
    mgr.start()
    bus.publish(EVENT_OBJECT_DETECTED, {"label": "red_bottle", "world_x": 0.0, "world_y": 0.0})
    bus.publish(EVENT_OBJECT_DETECTED, {"label": "red_bottle", "world_x": 1.2, "world_y": 0.0})
    bus.publish(EVENT_OBJECT_DETECTED, {"label": "red_bottle", "world_x": 0.01, "world_y": 0.0})
    assert hits == []
    assert mgr.replan_count == 0


def test_replan_ignored_when_mission_idle():
    bus = EventBus()
    hits = []
    mgr = ReplanManager(
        event_bus=bus,
        replan_callback=lambda label, info: hits.append(info),
        mission_active=lambda: False,
    )
    mgr.start()
    bus.publish(EVENT_OBJECT_DETECTED, {"label": "red_bottle", "world_x": 0.0, "world_y": 0.0})
    bus.publish(EVENT_OBJECT_DETECTED, {"label": "red_bottle", "world_x": 1.0, "world_y": 0.0})
    assert hits == []
    assert mgr.replan_count == 0


def test_homography_roundtrip(tmp_path: Path):
    g = SpatialGrounding(img_w=640, img_h=480)
    path = tmp_path / "homography.json"
    g.save(path)
    g2 = SpatialGrounding(img_w=320, img_h=240)
    g2.load(path)
    assert g2.img_w == 640
    wx, wy, _ = g2.pixel_to_world(320, 240)
    assert abs(wx) < 0.15
    assert abs(wy) < 0.15
    data = json.loads(path.read_text())
    assert "H" in data


def test_executor_inject_replan():
    drone = SimInterface()
    drone.connect()
    executor = MissionExecutor(drone=drone)
    plan = MissionPlan(
        raw_command="hover",
        skills=[
            SkillPrimitive(skill="TAKEOFF", params={"z": 0.5}),
            SkillPrimitive(skill="HOVER", params={"t": 0.05}),
            SkillPrimitive(skill="LAND", params={}),
        ],
    )

    injected = {"n": 0}

    def _inject(_skill):
        if _skill == "HOVER" and injected["n"] == 0:
            injected["n"] += 1
            executor.inject_replan([SkillPrimitive(skill="LAND", params={})])

    ok = executor.execute_plan(plan, on_step_complete=_inject)
    assert ok is True


def test_replan_accumulates_slow_drift():
    """Drift is measured against the anchor, so a target nudged in many small
    steps still crosses the threshold."""
    bus = EventBus()
    hits = []
    mgr = ReplanManager(
        event_bus=bus,
        replan_callback=lambda label, info: hits.append(info),
        distance_threshold=0.3,
        cooldown_s=0.0,
        mission_active=lambda: True,
    )
    mgr.start()
    # 8 cm per frame: no single pair is 0.3 m apart, but the total is 0.32 m.
    for i in range(7):
        bus.publish(EVENT_OBJECT_DETECTED, {"label": "red_bottle", "world_x": i * 0.08, "world_y": 0.0})
    assert mgr.replan_count == 1
    assert hits[0]["drift"] >= 0.3
    assert hits[0]["old_x"] == 0.0  # measured from the anchor, not the last frame


def test_replan_honours_max_replans():
    bus = EventBus()
    hits = []
    mgr = ReplanManager(
        event_bus=bus,
        replan_callback=lambda label, info: hits.append(info),
        distance_threshold=0.3,
        cooldown_s=0.0,
        max_replans=2,
        mission_active=lambda: True,
    )
    mgr.start()
    for i in range(10):
        bus.publish(EVENT_OBJECT_DETECTED, {"label": "red_bottle", "world_x": i * 0.5, "world_y": 0.0})
    assert mgr.replan_count == 2
    assert len(hits) == 2


def test_replan_keeps_remaining_skills():
    """A mid-mission replan must be inserted, not truncate the tail — otherwise
    the closing LAND is silently dropped."""
    drone = SimInterface()
    drone.connect()
    executor = MissionExecutor(drone=drone)
    plan = MissionPlan(
        raw_command="patrol",
        skills=[
            SkillPrimitive(skill="TAKEOFF", params={"z": 0.5}),
            SkillPrimitive(skill="MOVE_TO", params={"x": 0.3, "y": 0.0, "z": 0.5}),
            SkillPrimitive(skill="LAND", params={}),
        ],
    )

    done = []
    fired = {"n": 0}

    def _inject(skill):
        done.append(skill)
        if skill == "TAKEOFF" and fired["n"] == 0:
            fired["n"] += 1
            executor.inject_replan([SkillPrimitive(skill="HOVER", params={"t": 0.05})])

    assert executor.execute_plan(plan, on_step_complete=_inject) is True
    assert done == ["TAKEOFF", "HOVER", "MOVE_TO", "LAND"]


def test_executor_rejects_concurrent_mission():
    drone = SimInterface()
    drone.connect()
    executor = MissionExecutor(drone=drone)
    second = {}

    def _try_second(_skill):
        if "result" not in second:
            second["result"] = executor.execute_plan(
                MissionPlan(raw_command="intruder", skills=[SkillPrimitive(skill="LAND", params={})])
            )

    plan = MissionPlan(
        raw_command="takeoff",
        skills=[SkillPrimitive(skill="TAKEOFF", params={"z": 0.5}), SkillPrimitive(skill="LAND", params={})],
    )
    assert executor.execute_plan(plan, on_step_complete=_try_second) is True
    assert second["result"] is False
    assert executor.is_busy() is False
