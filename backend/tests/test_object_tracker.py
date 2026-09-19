"""VLM-identify-once + OpenCV-track-continuously pipeline."""
import numpy as np
import pytest

from backend.runtime.mission_executor import MissionExecutor
from backend.runtime.state_manager import StateManager
from backend.schemas.mission import MissionPlan, SkillPrimitive
from backend.schemas.perception import BoundingBox, DetectedObject
from backend.vision.fast_cv import FastPerception
from backend.vision.object_tracker import ObjectTracker, TargetLock, TrackerManager
from backend.drone.sim_interface import SimInterface

cv2 = pytest.importorskip("cv2")

W, H = 320, 240
# Blurred so the patch has spatial structure like a real object. Per-pixel white
# noise is a pathological case for correlation and not representative.
_OBJ = cv2.GaussianBlur(
    np.random.default_rng(3).integers(120, 255, (40, 40, 3)).astype(np.uint8), (7, 7), 0
)


def scene(cx, cy, occlude=False):
    """Textured background plus a colourless textured target, so nothing here is
    findable by hue alone."""
    rng = np.random.default_rng(11)
    img = rng.integers(40, 90, (H, W, 3)).astype(np.uint8)
    img = cv2.GaussianBlur(img, (9, 9), 0)
    img[cy - 20:cy + 20, cx - 20:cx + 20] = _OBJ
    if occlude:
        cv2.rectangle(img, (cx - 35, cy - 35), (cx + 35, cy + 35), (25, 25, 25), -1)
    return img


def box_at(cx, cy):
    return BoundingBox(u_min=cx - 20, v_min=cy - 20, u_max=cx + 20, v_max=cy + 20)


class FakeVLM:
    """Stands in for Qwen2-VL; counts how often it is consulted."""

    def __init__(self, cx=80, cy=120, found=True):
        self.calls = 0
        self.cx, self.cy = cx, cy
        self.found = found

    def resolve_target(self, image_bytes, target, image_width=0, image_height=0):
        self.calls += 1
        if not self.found:
            return None
        return DetectedObject(label=target, bbox=box_at(self.cx, self.cy), confidence=0.8)


def test_tracks_colourless_object_across_frames():
    """The target has no distinctive colour, so only an appearance tracker works."""
    tr = ObjectTracker(label="widget")
    assert tr.start(scene(80, 120), box_at(80, 120)) is True

    centers = []
    for cx in range(83, 140, 3):
        upd = tr.update(scene(cx, 120))
        assert upd.ok, f"lost at cx={cx}: {upd.reason}"
        centers.append(upd.bbox.center_pixel[0])

    assert centers[-1] > centers[0] + 30  # followed the motion
    assert abs(centers[-1] - 137) < 15    # and stayed on the object


def test_reports_loss_and_does_not_silently_drift():
    """Full occlusion must surface as a loss, since that is what escalates to the
    VLM. A tracker that keeps claiming success is the dangerous failure."""
    tr = ObjectTracker(label="widget")
    tr.start(scene(120, 120), box_at(120, 120))
    for _ in range(12):
        upd = tr.update(scene(120, 120, occlude=True))
        if not upd.ok:
            break
    assert tr.alive is False
    assert upd.ok is False
    assert upd.reason


def test_health_check_rejects_box_outside_frame():
    tr = ObjectTracker(label="widget")
    tr.start(scene(80, 120), box_at(80, 120))

    class Runaway:
        def update(self, _frame):
            return True, (10_000.0, 10_000.0, 40.0, 40.0)

    tr._tracker = Runaway()
    upd = tr.update(scene(80, 120))
    assert upd.ok is False
    assert tr.alive is False


def test_health_check_rejects_scale_explosion():
    """CSRT drifting onto background usually balloons the box before its own
    ok flag ever flips."""
    tr = ObjectTracker(label="widget")
    tr.start(scene(80, 120), box_at(80, 120))

    class Ballooning:
        def update(self, _frame):
            return True, (10.0, 10.0, 300.0, 220.0)

    tr._tracker = Ballooning()
    upd = tr.update(scene(80, 120))
    assert upd.ok is False
    assert "scale" in upd.reason or "aspect" in upd.reason


def test_health_check_rejects_implausible_jump():
    tr = ObjectTracker(label="widget")
    tr.start(scene(40, 40), box_at(40, 40))

    class Teleport:
        def update(self, _frame):
            return True, (270.0, 190.0, 40.0, 40.0)

    tr._tracker = Teleport()
    upd = tr.update(scene(40, 40))
    assert upd.ok is False
    assert "jump" in upd.reason


def test_vlm_called_once_then_tracker_takes_over():
    vlm = FakeVLM(cx=80, cy=120)
    fp = FastPerception(vlm=vlm)
    fp.set_targets(["widget"])

    for i in range(10):
        res = fp.process_frame(scene(80 + i * 3, 120), z_altitude=1.0)
        assert res.objects, f"no detection on frame {i}"
        assert res.objects[0].world_x is not None

    assert vlm.calls == 1, f"VLM should identify once, was called {vlm.calls}x"


def test_vlm_is_rate_limited_when_target_absent():
    """An absent target must not burn a VLM call on every loop iteration."""
    vlm = FakeVLM(found=False)
    fp = FastPerception(vlm=vlm)
    fp.trackers = TrackerManager(redetect_cooldown_s=60.0)
    fp.set_targets(["ghost"])

    for _ in range(25):
        fp.process_frame(scene(80, 120), z_altitude=1.0)

    assert vlm.calls == 1, f"expected cooldown to hold at 1 call, got {vlm.calls}"


def test_lost_track_escalates_to_vlm_again():
    vlm = FakeVLM(cx=120, cy=120)
    fp = FastPerception(vlm=vlm)
    fp.trackers = TrackerManager(redetect_cooldown_s=0.0)
    fp.set_targets(["widget"])

    fp.process_frame(scene(120, 120), z_altitude=1.0)
    assert vlm.calls == 1

    for _ in range(12):
        fp.process_frame(scene(120, 120, occlude=True), z_altitude=1.0)

    assert vlm.calls > 1, "loss must trigger re-detection"


def test_set_targets_drops_stale_trackers():
    vlm = FakeVLM()
    fp = FastPerception(vlm=vlm)
    fp.set_targets(["widget"])
    fp.process_frame(scene(80, 120), z_altitude=1.0)
    assert "widget" in fp.trackers.active_labels()

    fp.set_targets(["other"])
    assert "widget" not in fp.trackers.snapshot()


def test_repeated_misses_back_off_exponentially():
    """A target that is simply not in the room must not keep costing VLM calls
    at a fixed rate."""
    mgr = TrackerManager(redetect_cooldown_s=1.0, max_cooldown_s=30.0)
    assert mgr.needs_detection("ghost") is True

    mgr.note_detection_attempt("ghost")
    mgr.note_detection_missed("ghost")
    assert mgr.needs_detection("ghost") is False  # now waiting 2s, not 1s

    for _ in range(9):
        mgr.note_detection_missed("ghost")
    # 1.0 * 2**10 would be 1024s, so the cap must hold it at 30s
    mgr._last_vlm_attempt["ghost"] = __import__("time").time() - 29.0
    assert mgr.needs_detection("ghost") is False
    mgr._last_vlm_attempt["ghost"] = __import__("time").time() - 31.0
    assert mgr.needs_detection("ghost") is True


def test_successful_seed_clears_backoff():
    mgr = TrackerManager(redetect_cooldown_s=1.0)
    for _ in range(4):
        mgr.note_detection_missed("widget")
    assert mgr._misses["widget"] == 4
    mgr.seed("widget", scene(80, 120), box_at(80, 120))
    assert "widget" not in mgr._misses


def test_arbitrary_label_needs_no_colour_prior():
    """Any label works: nothing maps 'orange traffic cone' to a hue range."""
    vlm = FakeVLM(cx=100, cy=100)
    fp = FastPerception(vlm=vlm)
    fp.set_targets(["orange traffic cone"])
    res = fp.process_frame(scene(100, 100), z_altitude=1.0)
    assert res.objects
    assert res.objects[0].label == "orange traffic cone"


def test_csrt_then_vlm_fail_enters_target_lost():
    """Confirmed tracker failure + VLM miss must become TARGET_LOST, not a
    recycled last-known world pose."""
    vlm = FakeVLM(cx=120, cy=120)
    sm = StateManager()
    fp = FastPerception(vlm=vlm, state_manager=sm)
    fp.trackers = TrackerManager(redetect_cooldown_s=0.0)
    fp.set_targets(["widget"])

    first = fp.process_frame(scene(120, 120), z_altitude=1.0)
    assert first.objects
    assert first.objects[0].world_x is not None
    assert fp.trackers.lock_state("widget") == TargetLock.TRACKING
    assert sm.is_actionable("widget") is True

    vlm.found = False
    lost_result = None
    for _ in range(15):
        lost_result = fp.process_frame(scene(120, 120, occlude=True), z_altitude=1.0)
        if "widget" in lost_result.lost_labels:
            break
    assert lost_result is not None
    assert "widget" in lost_result.lost_labels
    assert fp.trackers.lock_state("widget") == TargetLock.TARGET_LOST
    assert sm.is_actionable("widget") is False
    assert sm.get_object("widget").world_x is None
    assert all(o.world_x is None or o.label != "widget" for o in lost_result.objects)


def test_stale_state_cannot_reenter_perception_after_loss():
    """process_frame used to fall back to StateManager and re-emit the last
    pose, which then drove replans."""
    vlm = FakeVLM(found=False)
    sm = StateManager()
    sm.update_object(DetectedObject(
        label="widget", bbox=box_at(80, 120), confidence=0.9,
        world_x=0.7, world_y=-0.4, world_z=1.0,
    ))
    fp = FastPerception(vlm=vlm, state_manager=sm)
    fp.trackers = TrackerManager(redetect_cooldown_s=0.0)
    fp.set_targets(["widget"])

    res = fp.process_frame(scene(80, 120), z_altitude=1.0)
    assert "widget" in res.lost_labels
    assert res.objects == []
    assert sm.is_actionable("widget") is False


def test_executor_holds_when_target_lost():
    """FIND must hover, not fly to the last known world coordinate."""
    drone = SimInterface()
    drone.connect()
    sm = StateManager()
    sm.update_object(DetectedObject(
        label="bottle", bbox=box_at(80, 120), confidence=0.9,
        world_x=0.9, world_y=0.9, world_z=0.5,
    ))
    sm.mark_lost("bottle")
    executor = MissionExecutor(drone=drone, state_manager=sm)
    plan = MissionPlan(
        raw_command="find bottle",
        skills=[
            SkillPrimitive(skill="TAKEOFF", params={"z": 0.5}),
            SkillPrimitive(skill="FIND", params={"target": "bottle"}),
            SkillPrimitive(skill="LAND", params={}),
        ],
    )
    assert executor.execute_plan(plan) is True
    st = drone.get_state()
    assert abs(st.x) < 0.2
    assert abs(st.y) < 0.2


def test_state_manager_lost_object_is_not_in_llm_context():
    sm = StateManager()
    sm.update_object(DetectedObject(
        label="bottle", bbox=box_at(80, 120),
        world_x=0.4, world_y=0.6, world_z=1.0,
    ))
    sm.mark_lost("bottle")
    assert sm.objects_as_context() == []
    assert sm.get_actionable_objects() == []
