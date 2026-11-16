import math

from backend.planning.gate_nav import GateSpec, GateVLM, gate_skills, locate_gate, pixel_ray

CAM = {"fx": 168.05, "fy": 168.05, "cx": 240, "cy": 180}
POSE = {"x": -1.8, "y": 0.0, "z": 0.5, "yaw": 0.0}


def _project(point, pose=POSE, tilt=20.0):
    """Pixel of a world point, by inverting pixel_ray with a numeric search."""
    best, bp = 1e9, None
    for u in range(0, 480, 4):
        for v in range(0, 360, 4):
            o, r = pixel_ray(u, v, CAM, pose, tilt_down_deg=tilt)
            t = (point[0] - o[0]) / r[0]
            d = math.dist(o + t * r, point)
            if d < best:
                best, bp = d, (u, v)
    return bp


def test_picture_box_locates_gate_within_tolerance():
    gate = (0.4, 1.1, 1.25)
    u, v = _project(gate)
    o, r = pixel_ray(u, v, CAM, POSE)
    depth = math.dist((o[0], o[1]), gate[:2])
    half = CAM["fx"] * 0.25 / depth
    found = locate_gate((u - half, v - half, u + half, v + half), CAM, POSE, GateSpec())
    assert math.hypot(found["x"] - gate[0], found["y"] - gate[1]) < 0.25


def test_skills_pass_through_the_gate_along_the_heading():
    ws = {"x": [-2.3, 2.3], "y": [-1.9, 1.9], "z": [0.25, 2.0]}
    sk = gate_skills({"x": 0.4, "y": 1.1}, POSE, GateSpec(), ws)
    pts = [(s.params["x"], s.params["y"]) for s in sk if s.skill == "MOVE_TO"]
    assert pts[0][0] < 0.4 < pts[2][0]
    assert all(abs(p[1] - 1.1) < 1e-6 for p in pts)


def test_parse_refuses_when_no_gate_fits():
    c = GateVLM._parse({"gates": [{"picture": "cat", "shape": "circle", "picture_box_2d": [0, 0, 500, 500]}],
                        "choice": -1, "reason": "none"}, 480, 360)
    assert c.index == -1 and c.box is None
