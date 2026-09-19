"""
Unit tests for the closed-loop Perception-Action architecture:
- EventBus publish/subscribe
- StateManager synchronized store
- SpatialGrounding dynamic resolution
- SafetyValidator CIRCLE perimeter bounds
- MissionExecutor closed-loop VLM triggering and navigation
"""
import numpy as np
import pytest

from backend.drone.sim_interface import SimInterface
from backend.runtime.state_manager import StateManager
from backend.runtime.event_bus import EventBus, EVENT_OBJECT_DETECTED, EVENT_STEP_COMPLETED
from backend.runtime.mission_executor import MissionExecutor
from backend.vision.spatial_grounding import SpatialGrounding
from backend.planning.safety_validator import SafetyValidator
from backend.schemas.mission import MissionPlan, SkillPrimitive
from backend.schemas.perception import BoundingBox, DetectedObject
from backend.schemas.capabilities import RobotCapabilities


class MockVLM:
    """Simulated VLM model that returns a detected object with bounding box."""
    def __init__(self, target="bottle", bbox=(100, 100, 200, 200)):
        self.target = target
        self.bbox = bbox

    def resolve_target(self, image_bytes: bytes, target_description: str, image_width=640, image_height=480):
        if target_description.lower() in self.target.lower() or self.target.lower() in target_description.lower():
            u_min, v_min, u_max, v_max = self.bbox
            return DetectedObject(
                label=target_description,
                bbox=BoundingBox(u_min=u_min, v_min=v_min, u_max=u_max, v_max=v_max),
                confidence=0.92,
            )
        return None


def test_spatial_grounding_dynamic_resolution():
    """Verify homography computes proper metric world coordinates with dynamic camera resolution."""
    grounding = SpatialGrounding(img_w=1280, img_h=720)
    # Center pixel (640, 360) should map close to workspace center (0, 0)
    wx, wy, wz = grounding.pixel_to_world(640, 360, z_altitude=1.2)
    assert abs(wx) < 0.1
    assert abs(wy) < 0.1
    assert wz == 1.2

    # Dynamic scaling when frame dimensions differ
    wx2, wy2, wz2 = grounding.pixel_to_world(320, 180, z_altitude=1.0, img_w=640, img_h=360)
    assert abs(wx2) < 0.1
    assert abs(wy2) < 0.1


def test_state_manager_thread_safety():
    """Verify StateManager updates, object tracking, and context generation."""
    sm = StateManager()
    det = DetectedObject(
        label="bottle",
        bbox=BoundingBox(u_min=50, v_min=50, u_max=150, v_max=150),
        confidence=0.88,
        world_x=0.4,
        world_y=0.6,
        world_z=1.0,
    )
    sm.update_object(det)
    retrieved = sm.get_object("bottle")
    assert retrieved is not None
    assert retrieved.world_x == 0.4

    context = sm.objects_as_context()
    assert len(context) == 1
    assert context[0]["label"] == "bottle"
    assert context[0]["x"] == 0.4


def test_event_bus():
    """Verify EventBus publish/subscribe functionality."""
    bus = EventBus()
    received = []

    def on_event(data):
        received.append(data)

    bus.subscribe("test_event", on_event)
    bus.publish("test_event", {"msg": "hello"})
    assert len(received) == 1
    assert received[0]["msg"] == "hello"

    bus.unsubscribe("test_event", on_event)
    bus.publish("test_event", {"msg": "ignored"})
    assert len(received) == 1


def test_circle_perimeter_safety_validation():
    """Verify SafetyValidator rejects CIRCLE commands that cross workspace geofence bounds."""
    validator = SafetyValidator()
    # Safe circle
    safe_plan = MissionPlan(
        raw_command="circle center",
        skills=[
            SkillPrimitive(skill="TAKEOFF", params={"z": 1.0}),
            SkillPrimitive(skill="CIRCLE", params={"x": 0.0, "y": 0.0, "radius": 0.5}),
        ],
    )
    res_safe = validator.validate_plan(safe_plan)
    assert res_safe.is_valid is True

    # Dangerous circle exceeding geofence boundaries (x=1.3 + radius 0.5 = 1.8 > 1.5)
    unsafe_plan = MissionPlan(
        raw_command="circle near edge",
        skills=[
            SkillPrimitive(skill="TAKEOFF", params={"z": 1.0}),
            SkillPrimitive(skill="MOVE_TO", params={"x": 1.3, "y": 1.3, "z": 1.0}),
            SkillPrimitive(skill="CIRCLE", params={"radius": 0.5}),
        ],
    )
    res_unsafe = validator.validate_plan(unsafe_plan)
    assert res_unsafe.is_valid is False
    assert any("violates geofence" in err for err in res_unsafe.errors)


def test_perception_action_loop_execution():
    """Verify MissionExecutor runs closed-loop visual grounding on FIND commands."""
    drone = SimInterface()
    drone.connect()
    # Provide a synthetic camera frame
    drone.get_frame = lambda: np.zeros((480, 640, 3), dtype=np.uint8)

    state_mgr = StateManager()
    bus = EventBus()
    detected_events = []
    bus.subscribe(EVENT_OBJECT_DETECTED, lambda d: detected_events.append(d))

    vlm = MockVLM(target="bottle", bbox=(320, 240, 360, 280))
    executor = MissionExecutor(
        drone=drone,
        vlm=vlm,
        state_manager=state_mgr,
        event_bus=bus,
    )

    plan = MissionPlan(
        raw_command="find the bottle",
        skills=[
            SkillPrimitive(skill="TAKEOFF", params={"z": 1.0}),
            SkillPrimitive(skill="FIND", params={"target": "bottle"}),
            SkillPrimitive(skill="LAND", params={}),
        ],
    )

    success = executor.execute_plan(plan)
    assert success is True

    # Check that VLM detected target and stored in StateManager
    stored_obj = state_mgr.get_object("bottle")
    assert stored_obj is not None
    assert stored_obj.world_x is not None
    assert len(detected_events) == 1
    assert detected_events[0]["label"] == "bottle"
