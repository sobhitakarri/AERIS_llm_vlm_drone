"""
Regression tests for faults found while recording the twin test cases:
a model that answers in the wrong format, a heading estimate that drifted
until position fixes were refused, and a request to a model that never returned.
"""
import math
import types as pytypes

import numpy as np
import pytest

from backend.models import gemini_provider
from backend.models.gemini_provider import GeminiProvider, _extract_json
from backend.schemas.telemetry import DroneState
from backend.vision.calibration import OverheadCalibration
from backend.vision.scene_service import SceneService

W, H, F, CAM_Z = 960, 720, 400.0, 2.9


# ------------------------------------------------------------------ provider
def test_repeated_key_keeps_the_first_value():
    data = _extract_json('{"label": "box", "box_2d": [1, 2, 3, 4], "box_2d": [5, 6, 7, 8]}')
    assert data["box_2d"] == [1, 2, 3, 4]


def test_a_stray_bracket_after_the_answer_is_ignored():
    assert _extract_json('[{"label": "blue cube", "box": [103, 818, 194, 893]}]}')[0]["box"] == [103, 818, 194, 893]


def test_mixed_scale_numbers_are_not_a_box():
    from backend.vision.bbox_convert import to_pixels as bbox_to_pixels
    assert bbox_to_pixels([7.32, 143, 198, 770], 960, 720, order="yxyx", assume="norm1000") is None
    assert bbox_to_pixels([-413, 237, 342, 310], 960, 720, order="yxyx", assume="norm1000") is None
    assert bbox_to_pixels([103, 818, 194, 893], 960, 720, order="yxyx", assume="norm1000") is not None
    assert bbox_to_pixels([0.1, 0.8, 0.2, 0.9], 960, 720, order="yxyx", assume="norm1000") is not None


class _Models:
    def __init__(self, replies):
        self.replies, self.calls = list(replies), []

    def generate_content(self, model, contents, config=None):
        self.calls.append(model)
        reply = self.replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return pytypes.SimpleNamespace(text=reply)


def _provider(replies) -> GeminiProvider:
    if not gemini_provider.GENAI_SDK_AVAILABLE:
        pytest.skip("google-genai is not installed")
    p = GeminiProvider(api_key="")
    p.client = pytypes.SimpleNamespace(models=_Models(replies))
    p.is_live = True
    p.model_name = "primary"
    return p


GOOD = '{"label": "crate", "box_2d": [400, 400, 500, 500], "confidence": 0.9}'


def test_malformed_answer_is_asked_again_on_the_same_model():
    p = _provider(['{"label": "crate", "box": [1, 2, 3, 4, 5, 6, 7, 8, 9]}', '{"label": "crate", "box_2d": [400, 400,', GOOD])
    det = p.resolve_target(b"jpeg", "crate", W, H)
    assert det is not None and det.label == "crate"
    assert p.client.models.calls == ["primary"] * 3


def test_a_clear_not_visible_answer_is_final():
    p = _provider(['{"label": "none", "box_2d": null, "confidence": 0.0}', GOOD])
    assert p.resolve_target(b"jpeg", "purple elephant", W, H) is None
    assert len(p.client.models.calls) == 1


def test_an_exhausted_model_is_skipped_not_retried():
    p = _provider([RuntimeError("429 RESOURCE_EXHAUSTED"), GOOD])
    det = p.resolve_target(b"jpeg", "crate", W, H)
    assert det is not None
    assert p.client.models.calls[0] == "primary" and p.client.models.calls[1] != "primary"
    assert p.model_name == p.client.models.calls[1]      # later calls start from the model that answered


def test_an_overloaded_model_is_tried_again(monkeypatch):
    monkeypatch.setattr(gemini_provider.time, "sleep", lambda s: None)
    p = _provider([RuntimeError("503 UNAVAILABLE. high demand"), GOOD])
    assert p.resolve_target(b"jpeg", "crate", W, H) is not None
    assert p.client.models.calls == ["primary", "primary"]


def test_requests_carry_a_deadline():
    assert 0 < gemini_provider.REQUEST_TIMEOUT_MS <= 60000


# ------------------------------------------------------------- scene service
class _Grounding:
    def __init__(self):
        K = np.array([[F, 0, W / 2], [0, F, H / 2], [0, 0, 1.0]])
        R = np.diag([1.0, -1.0, -1.0])                    # camera looks straight down
        t = -R @ np.array([0.0, 0.0, CAM_Z])
        self.calibration = OverheadCalibration(K=K, R=R, t=t, H=np.eye(3), image_size=(W, H), marker_ids=[0],
                                               reproj_rms_px=0.1, floor_rms_m=0.001)
        self.calibrated = True
        self.has_depth = True


class _Drone:
    """A vehicle whose estimate can be offset and rotated from where it really is."""
    is_connected = True
    runtime = {"airframe": {"arm_m": 0.045}}

    def __init__(self):
        self.true_xy, self.est_xy, self.z, self.yaw = (0.0, 0.0), (0.0, 0.0), 0.8, 0.0
        self.sent = []

    def get_state(self):
        return DroneState(x=self.est_xy[0], y=self.est_xy[1], z=self.z, yaw=self.yaw, is_armed=True)

    def get_depth_frame(self):
        depth = np.full((H, W), CAM_Z, np.float32)
        u = int(W / 2 + F * self.true_xy[0] / (CAM_Z - self.z))
        v = int(H / 2 - F * self.true_xy[1] / (CAM_Z - self.z))
        depth[v - 6:v + 7, u - 6:u + 7] = CAM_Z - self.z
        return depth

    def send_external_pose(self, x, y, yaw_deg=None):
        self.sent.append((x, y, yaw_deg))
        return True


@pytest.fixture
def scene():
    drone = _Drone()
    return SceneService(drone, _Grounding()), drone


def test_fix_reports_where_the_camera_sees_the_airframe(scene):
    svc, drone = scene
    drone.true_xy, drone.est_xy = (0.50, 0.30), (0.54, 0.27)
    svc._external_fix()
    assert svc.fix_count == 1
    x, y, yaw = drone.sent[-1]
    assert math.hypot(x - 0.50, y - 0.30) < 0.02
    assert yaw is None                                   # nothing yet to measure a heading from


def test_gate_widens_with_distance_flown_since_the_last_fix(scene):
    svc, drone = scene
    svc._external_fix()                                  # fix at the origin
    drone.true_xy, drone.est_xy = (0.05, 0.20), (0.05, 0.0)
    svc._external_fix()                                  # 20 cm off after no travel: refused
    assert svc.fix_rejected == 1
    drone.true_xy, drone.est_xy = (2.0, 0.20), (2.0, 0.0)
    svc._external_fix()                                  # 20 cm off after 2 m of dead reckoning: accepted
    assert svc.fix_count == 2


def test_heading_error_is_measured_from_a_leg_and_fed_back(scene):
    svc, drone = scene
    svc._external_fix()                                  # leg starts where estimate and camera agree
    err = math.radians(6.0)                              # estimate is rotated +6 deg from the truth
    drone.true_xy = (1.5, 0.0)
    drone.est_xy = (1.5 * math.cos(err), 1.5 * math.sin(err))
    svc._external_fix()
    assert svc.yaw_fix_count == 1
    assert svc.last_yaw_fix_deg == pytest.approx(6.0, abs=0.7)
    assert drone.sent[-1][2] == pytest.approx(-6.0, abs=0.7)      # heading the estimator should move to


def test_no_heading_correction_from_a_straight_leg(scene):
    svc, drone = scene
    svc._external_fix()
    drone.true_xy = drone.est_xy = (1.5, 0.0)
    svc._external_fix()
    assert svc.yaw_fix_count == 0 and drone.sent[-1][2] is None
