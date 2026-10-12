"""
Tests for the pieces that sit between the digital twin and the planner:
marker calibration, depth proposals, depth-verified grounding, obstacle
routing and the twin's wire protocol. All use synthetic data; none needs
MATLAB.
"""
import json
import math
import socket
import struct
import threading
import time

import cv2
import numpy as np
import pytest

from backend.planning.obstacle_planner import plan_path
from backend.schemas.perception import BoundingBox, DetectedObject
from backend.vision.calibration import OverheadCalibration, calibrate_overhead
from backend.vision.fast_cv import FastPerception
from backend.vision.scene_proposals import SceneGeometry, find_proposals, match_proposal
from backend.vision.spatial_grounding import SpatialGrounding

W, H, HFOV = 960, 720, 100.0
CAM_Z = 2.92
MARKERS = {
    "dictionary": "DICT_4X4_50", "size_m": 0.30,
    "items": [{"id": 0, "pos": [-2.5, -2.0]}, {"id": 1, "pos": [2.5, -2.0]}, {"id": 2, "pos": [2.55, -0.6]},
              {"id": 3, "pos": [-2.5, 2.0]}, {"id": 4, "pos": [0.0, -2.1]}, {"id": 5, "pos": [-1.2, 2.15]}],
}
BOUNDS = {"x": [-2.9, 2.9], "y": [-2.4, 2.4]}
# name -> (cx, cy, size_x, size_y, base_z, height)
BOXES = {
    "table": (1.5, 1.2, 1.4, 0.7, 0.0, 0.75),
    "bottle": (1.2, 1.2, 0.08, 0.08, 0.75, 0.26),
    "crate": (-0.55, 0.35, 0.40, 0.30, 0.0, 0.24),
    "drum": (1.45, -0.85, 0.40, 0.40, 0.0, 0.60),
}


def true_camera():
    """Nadir camera: image right = world +x, image up = world +y."""
    f = (W / 2) / math.tan(math.radians(HFOV / 2))
    K = np.array([[f, 0, W / 2], [0, f, H / 2], [0, 0, 1.0]])
    R = np.array([[1.0, 0, 0], [0, -1.0, 0], [0, 0, -1.0]])
    t = -R @ np.array([0.0, 0.0, CAM_Z])
    return K, R, t


def render_markers() -> np.ndarray:
    K, R, t = true_camera()
    frame = np.full((H, W, 3), 150, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for item in MARKERS["items"]:
        code = cv2.copyMakeBorder(cv2.aruco.generateImageMarker(dictionary, item["id"], 200), 25, 25, 25, 25,
                                  cv2.BORDER_CONSTANT, value=255)
        half = MARKERS["size_m"] / 2 * (250 / 200)
        cx, cy = item["pos"]
        world = np.array([[cx - half, cy + half, 0], [cx + half, cy + half, 0], [cx + half, cy - half, 0], [cx - half, cy - half, 0]])
        px = (K @ (R @ world.T + t[:, None])).T
        px = (px[:, :2] / px[:, 2:]).astype(np.float32)
        src = np.float32([[0, 0], [250, 0], [250, 250], [0, 250]])
        warped = cv2.warpPerspective(code, cv2.getPerspectiveTransform(src, px), (W, H), borderValue=0)
        mask = cv2.warpPerspective(np.full_like(code, 255), cv2.getPerspectiveTransform(src, px), (W, H))
        frame[mask > 0] = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)[mask > 0]
    return frame


def render_depth(boxes=BOXES) -> np.ndarray:
    """Planar depth of the floor plus box tops, as the overhead camera sees them."""
    K, R, t = true_camera()
    u, v = np.meshgrid(np.arange(W, dtype=np.float64), np.arange(H, dtype=np.float64))
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], axis=-1) @ R
    centre = np.array([0.0, 0.0, CAM_Z])
    depth = np.full((H, W), CAM_Z)
    for cx, cy, sx, sy, base, height in boxes.values():
        d = CAM_Z - (base + height)
        hit = centre + rays * d
        inside = (np.abs(hit[..., 0] - cx) <= sx / 2) & (np.abs(hit[..., 1] - cy) <= sy / 2)
        depth = np.where(inside & (d < depth), d, depth)
    return depth.astype(np.float32)


@pytest.fixture(scope="module")
def calibration() -> OverheadCalibration:
    cal = calibrate_overhead(render_markers(), MARKERS, {"hfov_deg": HFOV})
    assert cal is not None
    return cal


# ---------------------------------------------------------------- calibration
def test_calibration_recovers_the_camera_pose(calibration):
    assert sorted(calibration.marker_ids) == [0, 1, 2, 3, 4, 5]
    assert calibration.reproj_rms_px < 1.0
    assert np.allclose(calibration.camera_position, [0, 0, CAM_Z], atol=0.02)
    assert calibration.floor_rms_m < 0.01


def test_calibration_needs_enough_markers():
    assert calibrate_overhead(np.full((H, W, 3), 150, np.uint8), MARKERS, {"hfov_deg": HFOV}) is None


def test_floor_projection_is_wrong_for_raised_points_and_depth_fixes_it(calibration):
    """The reason the overhead camera is RGB-D: a point on a table projects
    onto the floor far from where it is."""
    x, y, z = 1.2, 1.2, 1.01
    u, v = calibration.world_to_pixel(x, y, z)
    fx, fy = calibration.pixel_to_floor(u, v)
    assert math.hypot(fx - x, fy - y) > 0.5
    wx, wy, wz = calibration.pixel_to_world(u, v, CAM_Z - z)
    assert (wx, wy, wz) == pytest.approx((x, y, z), abs=0.02)


# ------------------------------------------------------------------ proposals
def test_proposals_find_every_object_with_position_and_height(calibration):
    props = find_proposals(render_depth(), SceneGeometry(calibration), bounds=BOUNDS)
    for name, (cx, cy, _, _, base, height) in BOXES.items():
        near = min(props, key=lambda p: math.hypot(p.x - cx, p.y - cy) + abs(p.top_z - (base + height)))
        assert math.hypot(near.x - cx, near.y - cy) < 0.04, name
        assert near.top_z == pytest.approx(base + height, abs=0.03), name


def test_object_on_a_table_is_separate_from_the_table(calibration):
    props = find_proposals(render_depth(), SceneGeometry(calibration), bounds=BOUNDS)
    bottle = min(props, key=lambda p: math.hypot(p.x - 1.2, p.y - 1.2) + abs(p.top_z - 1.01))
    assert bottle.size_m[0] < 0.2 and bottle.size_m[1] < 0.2
    assert bottle.support_z == pytest.approx(0.75, abs=0.04)
    assert not bottle.stands_on_floor


def test_empty_floor_has_no_proposals(calibration):
    assert find_proposals(render_depth({}), SceneGeometry(calibration), bounds=BOUNDS) == []


def test_excluded_region_hides_the_drone(calibration):
    boxes = dict(BOXES, drone=(0.0, 0.0, 0.1, 0.1, 0.95, 0.03))
    geo = SceneGeometry(calibration)
    seen = find_proposals(render_depth(boxes), geo, bounds=BOUNDS)
    hidden = find_proposals(render_depth(boxes), geo, bounds=BOUNDS, exclude=[(0.0, 0.0, 0.98, 0.22)])
    assert any(math.hypot(p.x, p.y) < 0.1 for p in seen)
    assert not any(math.hypot(p.x, p.y) < 0.1 for p in hidden)


def _box_around(calibration, x, y, z, half_px):
    u, v = calibration.world_to_pixel(x, y, z)
    return BoundingBox(u_min=int(u - half_px), v_min=int(v - half_px), u_max=int(u + half_px), v_max=int(v + half_px))


def test_detector_box_selects_the_object_not_the_table_under_it(calibration):
    props = find_proposals(render_depth(), SceneGeometry(calibration), bounds=BOUNDS)
    match = match_proposal(_box_around(calibration, 1.2, 1.2, 1.0, 22), props)   # a loose box round the bottle
    assert match is not None
    assert math.hypot(match.x - 1.2, match.y - 1.2) < 0.05 and match.top_z > 0.9


def test_box_on_bare_floor_matches_nothing(calibration):
    props = find_proposals(render_depth(), SceneGeometry(calibration), bounds=BOUNDS)
    assert match_proposal(_box_around(calibration, -1.5, -1.0, 0.0, 30), props) is None


# ------------------------------------------------------------ grounded stack
class BoxVLM:
    """Returns a fixed pixel box for any query: a model that never refuses."""

    def __init__(self, bbox: BoundingBox):
        self.bbox = bbox
        self.calls = 0

    def resolve_target(self, image_bytes, target_description, image_width=640, image_height=480):
        self.calls += 1
        return DetectedObject(label=target_description, bbox=self.bbox, confidence=0.8)


def _grounding(calibration, boxes=BOXES) -> SpatialGrounding:
    g = SpatialGrounding()
    g.set_calibration(calibration, BOUNDS)
    depth = render_depth(boxes)
    g.depth_source = lambda: depth
    return g


def test_hallucinated_box_is_rejected_by_depth(calibration):
    vlm = BoxVLM(_box_around(calibration, -1.5, -1.0, 0.0, 30))     # bare floor
    fp = FastPerception(grounding=_grounding(calibration), vlm=vlm)
    assert fp.locate(render_markers(), "wine bottle") is None
    assert fp.rejected_ungrounded == 1


def test_real_box_is_grounded_in_three_dimensions(calibration):
    vlm = BoxVLM(_box_around(calibration, 1.45, -0.85, 0.6, 30))    # the drum
    fp = FastPerception(grounding=_grounding(calibration), vlm=vlm, z_max=2.0)
    det = fp.locate(render_markers(), "steel drum")
    assert det is not None and det.grounding == "depth"
    assert (det.world_x, det.world_y) == pytest.approx((1.45, -0.85), abs=0.04)
    assert det.top_z == pytest.approx(0.60, abs=0.03)
    assert det.world_z >= det.top_z + 0.3                          # fly-to height clears it


def test_object_is_followed_in_depth_without_asking_the_model_again(calibration):
    vlm = BoxVLM(_box_around(calibration, -0.55, 0.35, 0.24, 30))   # the crate
    g = _grounding(calibration)
    fp = FastPerception(grounding=g, vlm=vlm)
    assert fp.locate(render_markers(), "crate") is not None
    fp.set_targets(["crate"])

    moved = dict(BOXES, crate=(-0.40, 0.45, 0.40, 0.30, 0.0, 0.24))  # nudged 18 cm
    depth = render_depth(moved)
    g.depth_source = lambda: depth
    g._proposals_t = 0.0
    res = fp.process_frame(render_markers())
    assert vlm.calls == 1
    assert res.objects and (res.objects[0].world_x, res.objects[0].world_y) == pytest.approx((-0.40, 0.45), abs=0.04)

    gone = {k: v for k, v in BOXES.items() if k != "crate"}
    depth2 = render_depth(gone)
    g.depth_source = lambda: depth2
    g._proposals_t = 0.0
    res = fp.process_frame(render_markers())
    assert not any(o.label == "crate" and o.actionable for o in res.objects)


# -------------------------------------------------------------------- routing
def _grid(cells):
    """10 cm grid over x in [-2, 2], y in [-2, 2]; cells = {(x, y): height}."""
    h = np.zeros((40, 40), np.float32)
    for (x, y), z in cells.items():
        h[int((x + 2) / 0.1), int((y + 2) / 0.1)] = z
    return h, (-2.0, -2.0), 0.1


def test_route_is_direct_in_free_space():
    path, how = plan_path((-1, 0, 0.6), (1, 0, 0.6), _grid({}), (0.25, 2.0))
    assert path == [(1, 0, 0.6)] and how == "direct"


def test_route_goes_over_or_around_a_wall():
    wall = {(0.0, y / 10): 1.0 for y in range(-8, 9)}
    path, how = plan_path((-1, 0, 0.6), (1, 0, 0.6), _grid(wall), (0.25, 2.0))
    assert path is not None and path[-1] == (1, 0, 0.6) and how != "direct"
    grid, origin, cell = _grid(wall)
    for a, b in zip([(-1, 0, 0.6)] + path[:-1], path):
        for k in range(21):
            x, y, z = (a[i] + (b[i] - a[i]) * k / 20 for i in range(3))
            assert z > grid[int((x - origin[0]) / cell), int((y - origin[1]) / cell)] + 0.05


def test_route_goes_around_when_it_cannot_go_over():
    wall = {(0.0, y / 10): 3.0 for y in range(-8, 9)}
    path, how = plan_path((-1, 0, 0.6), (1, 0, 0.6), _grid(wall), (0.25, 2.0))
    assert path is not None and how.startswith("around")
    assert max(abs(p[1]) for p in path) > 0.8


def test_goal_inside_an_obstacle_is_refused():
    block = {(1.0, 0.0): 1.2}
    path, how = plan_path((-1, 0, 0.6), (1, 0, 0.6), _grid(block), (0.25, 2.0))
    assert path is None and "obstacle" in how


def test_hovering_above_an_object_is_allowed():
    block = {(1.0, 0.0): 0.6}
    path, _ = plan_path((-1, 0, 0.6), (1, 0, 1.0), _grid(block), (0.25, 2.0))
    assert path is not None and path[-1] == (1, 0, 1.0)


# ------------------------------------------------------------- wire protocol
class FakeTwin:
    """Speaks the twin's UDP/TCP protocol with trivial kinematics."""

    def __init__(self, cmd_port, telem_port, cam_port):
        self.ports = (cmd_port, telem_port, cam_port)
        self.pos = [0.0, 0.0, 0.012]
        self.target = list(self.pos)
        self.armed = False
        self.seq = 0
        self.commands = []
        self.running = True
        self.rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx.bind(("127.0.0.1", cmd_port))
        self.rx.settimeout(0.02)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cam = socket.socket()
        self.cam.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.cam.bind(("127.0.0.1", cam_port))
        self.cam.listen(1)
        self.cam.settimeout(0.2)
        threading.Thread(target=self._loop, daemon=True).start()
        threading.Thread(target=self._frames, daemon=True).start()

    def _loop(self):
        while self.running:
            try:
                c = json.loads(self.rx.recvfrom(4096)[0])
                self.commands.append(c)
                if c["cmd"] == "TAKEOFF":
                    self.target = [self.pos[0], self.pos[1], c["z"]]; self.armed = True; self.seq += 1
                elif c["cmd"] == "MOVE_TO":
                    self.target = [c["x"], c["y"], min(c["z"], 2.0)]; self.seq += 1
                elif c["cmd"] == "LAND":
                    self.target = [self.pos[0], self.pos[1], 0.012]
            except (socket.timeout, OSError):
                pass
            step = [max(-0.05, min(0.05, t - p)) for t, p in zip(self.target, self.pos)]
            self.pos = [p + s for p, s in zip(self.pos, step)]
            if self.target[2] < 0.02 and self.pos[2] < 0.02:
                self.armed = False
            speed = [s / 0.02 if abs(s) >= 0.05 else 0.0 for s in step]
            msg = {"t": time.time(), "x": self.pos[0], "y": self.pos[1], "z": self.pos[2],
                   "vx": speed[0], "vy": speed[1], "vz": speed[2], "roll": 0, "pitch": 0, "yaw": 0,
                   "armed": self.armed, "seq": self.seq, "target": self.target + [0],
                   "battery_v": 4.0, "battery_pct": 90, "truth": {"x": self.pos[0], "y": self.pos[1], "z": self.pos[2]}}
            self.tx.sendto(json.dumps(msg).encode(), ("127.0.0.1", self.ports[1]))

    def _frames(self):
        while self.running:
            try:
                conn, _ = self.cam.accept()
            except (socket.timeout, OSError):
                continue
            rgb = np.zeros((6, 8, 3), np.uint8); rgb[..., 0] = 200       # pure red in RGB
            depth = np.full((6, 8), 1500, np.uint16)
            try:
                while self.running:
                    conn.sendall(struct.pack("<4sBB2xHHf", b"AERS", 1, 0, 8, 6, 1.0) + rgb.tobytes())
                    conn.sendall(struct.pack("<4sBB2xHHf", b"AERS", 2, 1, 8, 6, 1.0) + depth.tobytes())
                    time.sleep(0.05)
            except OSError:
                pass

    def stop(self):
        self.running = False
        for s in (self.rx, self.tx, self.cam):
            s.close()


@pytest.fixture
def twin(monkeypatch, tmp_path):
    from backend.drone import twin_interface as ti
    ports = {"host": "127.0.0.1", "cmd_port": 15005, "telem_port": 15006, "cam_port": 15007}
    runtime = {"net": ports, "streams": {"overhead": 1, "overhead_depth": 2},
               "workspace": {"x": [-2, 2], "y": [-2, 2], "z": [0.25, 2.0]}, "home": {"pos": [0.0, 0.0, 0.012]}}
    path = tmp_path / "twin_runtime.json"
    path.write_text(json.dumps(runtime))
    monkeypatch.setattr(ti, "RUNTIME_FILE", path)
    fake = FakeTwin(ports["cmd_port"], ports["telem_port"], ports["cam_port"])
    iface = ti.TwinInterface()
    iface.SETTLE_S = 0.05
    assert iface.connect() and iface.wait_for_telemetry(3.0)
    yield iface, fake
    iface.disconnect()
    fake.stop()


def test_twin_interface_flies_and_reports_arrival_from_telemetry(twin):
    iface, fake = twin
    assert iface.takeoff(1.0)
    assert iface.state.z == pytest.approx(1.0, abs=0.05) and iface.state.is_armed
    assert iface.move_to(0.5, -0.3, 1.0, velocity=1.0)
    assert (iface.state.x, iface.state.y) == pytest.approx((0.5, -0.3), abs=0.05)
    assert iface.land() and not iface.state.is_armed
    assert [c["cmd"] for c in fake.commands] == ["TAKEOFF", "MOVE_TO", "LAND"]


def test_twin_interface_waits_for_the_clamped_setpoint(twin):
    iface, _ = twin
    assert iface.takeoff(1.0)
    assert iface.move_to(0.0, 0.0, 5.0, velocity=1.0)       # the twin clamps z to 2.0
    assert iface.state.z == pytest.approx(2.0, abs=0.05)


def test_twin_interface_decodes_rgb_and_depth_streams(twin):
    iface, _ = twin
    deadline = time.time() + 3
    while time.time() < deadline and (iface.get_frame() is None or iface.get_depth_frame() is None):
        time.sleep(0.05)
    frame, depth = iface.get_frame(), iface.get_depth_frame()
    assert frame.shape == (6, 8, 3) and frame[0, 0].tolist() == [0, 0, 200]     # BGR for OpenCV
    assert depth.shape == (6, 8) and depth[0, 0] == pytest.approx(1.5)


def test_twin_interface_refuses_to_fly_without_a_twin(monkeypatch, tmp_path):
    from backend.drone import twin_interface as ti
    path = tmp_path / "twin_runtime.json"
    path.write_text(json.dumps({"net": {"host": "127.0.0.1", "cmd_port": 15015, "telem_port": 15016, "cam_port": 15017}}))
    monkeypatch.setattr(ti, "RUNTIME_FILE", path)
    iface = ti.TwinInterface()
    iface.connect()
    try:
        assert iface.takeoff(1.0) is False
        assert iface.move_to(1, 1, 1) is False
    finally:
        iface.disconnect()
