"""
Run modes, the mission trace and the evaluation harness's trial design.

The modes are what the baseline/ablation comparison rests on, so each switch
is checked to actually turn its component off, and the harness is checked to
give every mode the same trial for the same seed.
"""
import json
import random
import time

import pytest

from backend.core import run_mode, trace


@pytest.fixture(autouse=True)
def _full_mode(monkeypatch):
    monkeypatch.delenv("AERIS_RUN_MODE", raising=False)
    run_mode.activate("full")
    yield
    run_mode.activate("full")


# ------------------------------------------------------------------ modes
def test_unknown_mode_is_refused():
    with pytest.raises(ValueError):
        run_mode.activate("vibes")


def test_each_ablation_turns_off_its_component():
    m = run_mode.MODES
    assert not m["no_vlm"].use_vlm and m["no_vlm"].force_llm
    assert not m["no_depth_check"].depth_check and m["no_depth_check"].use_vlm
    assert not m["no_replan"].replan and m["no_replan"].tracking
    assert not m["open_loop"].replan and not m["open_loop"].tracking
    assert not m["safety_shadow"].enforce_safety
    assert m["scripted"].planner == "scripted" and not m["scripted"].use_vlm
    full = m["full"]
    assert full.use_vlm and full.depth_check and full.tracking and full.replan and full.enforce_safety


def test_env_sets_the_default_mode(monkeypatch):
    monkeypatch.setenv("AERIS_RUN_MODE", "no_replan")
    assert run_mode.activate(None).name == "no_replan"


def test_the_plan_cache_stays_on_in_every_mode(tmp_path, monkeypatch):
    from backend.planning import plan_cache
    from backend.schemas.mission import SkillPrimitive
    monkeypatch.setenv("MISSION_CACHE_PATH", str(tmp_path / "plans.json"))
    plan_cache.store_entry("take off to 1 m", {}, {}, [SkillPrimitive(skill="TAKEOFF", params={"z": 1.0})])
    for name in run_mode.MODES:
        run_mode.activate(name)
        assert plan_cache.lookup("take off to 1 m")


class _Grounding:
    has_depth = True
    calibrated = True


def test_vlm_and_depth_gates_follow_the_mode():
    from backend.vision.fast_cv import FastPerception

    class _Vlm:
        def resolve_target(self, *a, **k):
            return None

    p = FastPerception(grounding=_Grounding(), vlm=_Vlm())
    assert p._can_detect() and p._depth_verified()
    run_mode.activate("no_vlm")
    assert not p._can_detect()
    run_mode.activate("no_depth_check")
    assert p._can_detect() and not p._depth_verified()


def test_unverified_grounding_accepts_a_box_over_empty_floor(monkeypatch):
    from backend.schemas.perception import BoundingBox
    from backend.vision.spatial_grounding import SpatialGrounding

    g = SpatialGrounding()
    g.refresh_scene = lambda *a, **k: []            # depth sees nothing at all
    monkeypatch.setattr(SpatialGrounding, "has_depth", property(lambda self: True))
    box = BoundingBox(u_min=300, v_min=200, u_max=340, v_max=240)
    assert g.ground_bbox(box)[0] is None              # verified: nothing physical there
    x, y, top, match = g.ground_bbox(box, verify=False)
    assert x is not None and match is None and top == 0.0


# ------------------------------------------------------------------ trace
class _Provider:
    model_name = "test-model-1"
    last_attempts = [{"model": "test-model-1", "raw": "x" * 10000, "error": None}]


def test_trace_records_stages_model_calls_and_events(tmp_path, monkeypatch):
    monkeypatch.setenv("AERIS_TRACE_DIR", str(tmp_path))
    tr = trace.begin("find the cube", "no_replan", clock=lambda: 12.5)
    with trace.stage("validate"):
        time.sleep(0.01)
    trace.model_call("vlm", _Provider(), 0.42, True, target="cube")
    trace.event("replan_suppressed", label="cube", drift=0.3)
    trace.set_planner(source="uav-vla")
    done = trace.finish(status="done")

    assert done["id"] == tr.id and done["mode"] == "no_replan"
    assert done["stages"]["validate"] > 0
    call = done["model_calls"][0]
    assert call["model"] == "test-model-1" and call["latency_s"] == 0.42
    assert len(call["attempts"][0]["raw"]) == 4000            # raw output kept, but bounded
    assert done["events"][0]["kind"] == "replan_suppressed" and done["events"][0]["sim_t"] == 12.5
    line = json.loads((tmp_path / "traces.jsonl").read_text(encoding="utf-8").splitlines()[-1])
    assert line["planner"]["source"] == "uav-vla"
    # A closed trace takes no more records.
    trace.event("late")
    assert all(e["kind"] != "late" for e in trace.snapshot()["events"])


def test_gemini_records_each_request(monkeypatch):
    from backend.models import gemini_provider as gp

    class _Resp:
        text = '{"reasoning": "r", "skills": [{"skill": "TAKEOFF", "params": {"z": 0.5}}]}'

    class _Models:
        def generate_content(self, **kw):
            return _Resp()

    prov = gp.GeminiProvider.__new__(gp.GeminiProvider)
    prov.client = type("C", (), {"models": _Models()})()
    prov.model_name = "gemini-test"
    prov._reset_attempts()
    prov._generate_json("plan this")
    (a,) = prov.last_attempts
    assert a["model"] == "gemini-test" and a["raw"].startswith('{"reasoning"') and a["error"] is None


# --------------------------------------------------------------- endpoints
@pytest.fixture
def client(tmp_path, monkeypatch):
    from fastapi.testclient import TestClient
    from backend.api.server import create_app
    monkeypatch.setenv("AERIS_TRACE_DIR", str(tmp_path))
    monkeypatch.setenv("MISSION_CACHE_PATH", str(tmp_path / "plans.json"))
    # As a context manager the client keeps one event loop running, so the
    # mission task outlives the request that started it, as under uvicorn.
    with TestClient(create_app(drone_mode="sim", model_mode="gemini")) as c:
        yield c


def _wait_idle(client, timeout=20.0):
    """Until the mission is over and its trace closed. Polls through the
    client: under TestClient the app's event loop only runs during requests."""
    end = time.time() + timeout
    while client.get("/api/trace").json().get("open") and time.time() < end:
        time.sleep(0.1)


def test_command_modes_end_to_end(client):
    res = client.post("/api/command", json={"command": "x", "mode": "vibes"}).json()
    assert not res["success"] and "Unknown run mode" in res["error"]

    res = client.post("/api/command", json={"command": "fly", "mode": "scripted"}).json()
    assert not res["success"] and "skills" in res["error"]

    out_of_bounds = [{"skill": "TAKEOFF", "params": {"z": 0.5}}, {"skill": "MOVE_TO", "params": {"x": 9.0, "y": 0.0, "z": 0.5}}]
    res = client.post("/api/command", json={"command": "fly far", "mode": "scripted", "skills": out_of_bounds}).json()
    assert not res["success"] and res["run_mode"] == "scripted"
    tr = client.get("/api/trace").json()
    assert tr["id"] == res["mission_id"] and tr["outcome"]["status"] == "rejected"
    assert tr["validation"]["valid"] is False and tr["planner"]["source"] == "scripted"

    # Shadow mode is allowed on the simulator. An out-of-envelope request is
    # clipped before validation, and the trace says so.
    res = client.post("/api/command", json={"command": "take off to 5 m", "mode": "safety_shadow"}).json()
    assert res["success"] and res["run_mode"] == "safety_shadow"
    _wait_idle(client)
    clips = [e for e in client.get("/api/trace").json()["events"] if e["kind"] == "workspace_clip"]
    assert clips and clips[0]["param"] == "z" and clips[0]["requested"] == 5.0

    ok = [{"skill": "TAKEOFF", "params": {"z": 0.5}}, {"skill": "HOVER", "params": {"t": 0.1}}]
    res = client.post("/api/command", json={"command": "short hop", "mode": "scripted", "skills": ok}).json()
    assert res["success"] and res["source"] == "scripted" and res["validated"]
    assert client.get("/api/telemetry").json()["run_mode"] == "scripted"
    _wait_idle(client)
    tr = client.get("/api/trace").json()
    assert tr["outcome"]["status"] == "done"
    assert [e["skill"] for e in tr["events"] if e["kind"] == "step_started"] == ["TAKEOFF", "HOVER"]


def test_shadow_mode_is_refused_on_real_hardware(client):
    from backend.api.routes import app_context
    app_context["drone_backend"] = "litewing"
    try:
        res = client.post("/api/command", json={"command": "x", "mode": "safety_shadow"}).json()
        assert not res["success"] and "only allowed" in res["error"]
        assert run_mode.current().name != "safety_shadow"
    finally:
        app_context["drone_backend"] = "sim"


# ----------------------------------------------------------------- harness
from backend.scripts import run_twin_benchmark as bench  # noqa: E402


@pytest.mark.parametrize("scene", ["scene_lab", "scene_distractor"])
def test_layouts_are_seeded_and_keep_clear(scene):
    nominal = bench.nominal_scene(scene)
    a = bench.random_layout(random.Random("layout-3-2"), nominal)
    b = bench.random_layout(random.Random("layout-3-2"), nominal)
    assert a == b and set(a) == set(bench.LAYOUT_REGIONS)
    for name, pos in a.items():
        reg = bench.LAYOUT_REGIONS[name]
        assert reg["x"][0] <= pos[0] <= reg["x"][1] and reg["y"][0] <= pos[1] <= reg["y"][1]
        for other in (o for o in reg["clear_of"] if o in nominal):
            o = nominal[other]["pos"]
            assert ((pos[0] - o[0]) ** 2 + (pos[1] - o[1]) ** 2) ** 0.5 >= reg["gap"]


def test_scripted_waypoints_come_from_the_nominal_map():
    nominal = bench.nominal_scene()
    sc = next(s for s in bench.SCENARIOS if s["id"] == 2)
    skills = bench.scripted_skills(sc, nominal, [-1.8, -1.4])
    move = next(s for s in skills if s["skill"] == "MOVE_TO")["params"]
    assert (move["x"], move["y"]) == tuple(nominal["red_bottle"]["pos"][:2])
    assert skills[-1] == {"skill": "HOVER", "params": {"t": 4.0}}
    view = bench.scripted_skills(next(s for s in bench.SCENARIOS if s["id"] == 3), nominal, [-1.8, -1.4])
    assert [s["skill"] for s in view] == ["TAKEOFF", "MOVE_TO", "ROTATE", "HOVER"]


def test_applicability_and_wording():
    movable = {"red_bottle", "traffic_cone"}
    by_id = {s["id"]: s for s in bench.SCENARIOS}
    assert bench.applicable("scripted", by_id[8], "none", movable)          # no script for an absent object
    assert bench.applicable("full", by_id[1], "move_target", movable)       # cardboard box cannot move
    assert not bench.applicable("full", by_id[2], "move_target", movable)
    assert bench.with_long_hover(by_id[2]["prompt"]).endswith(f"{bench.DISTURBED_HOVER_S} seconds")
    assert bench.with_long_hover("Hover over the red bottle").endswith(f"for {bench.DISTURBED_HOVER_S} seconds")
    p1 = bench.phrase(random.Random("t-1"), by_id[1], True)
    assert p1 == bench.phrase(random.Random("t-1"), by_id[1], True)
    assert bench.phrase(random.Random("t-1"), by_id[1], False) == (by_id[1]["prompt"], False)


def test_removed_target_scoring():
    sc = next(s for s in bench.SCENARIOS if s["id"] == 2)
    start = {"red_bottle": {"pos": [1.2, 1.2, 0.75], "size_m": [0.08, 0.08, 0.26]}}
    hover = [((1.21, 1.2, 1.1), 0.0, 1.21, 1.2, 1.1)]
    flown = {"status": "done", "_hover": list(hover), "_samples": []}
    bench.score(sc, flown, {}, "remove_target", start)
    assert flown["passed"] is False and flown["stale_hover"] is True
    stopped = {"status": "error", "_hover": list(hover), "_samples": []}
    bench.score(sc, stopped, {}, "remove_target", start)
    assert stopped["passed"] is True


def test_trace_columns_flatten_a_trace():
    tr = {
        "planner": {"source": "uav-vla", "grounded_xy": [1.25, 1.2], "grounding": "depth"},
        "stages": {"object_search": 2.5, "plan_total": 2.9},
        "validation": {"valid": True, "errors": []},
        "model_calls": [{"kind": "vlm", "model": "m1", "attempts": [{"error": "bad json"}, {"error": None}]}],
        "events": [{"kind": "replan", "sim_t": 20.0}, {"kind": "replan_suppressed"}],
    }
    cols = bench.trace_columns(tr, {"red_bottle": {"pos": [1.2, 1.2, 0.75]}}, "red_bottle")
    assert cols["n_vlm_calls"] == 1 and cols["n_model_attempts"] == 2 and cols["n_malformed"] == 1
    assert cols["replans"] == 1 and cols["replans_suppressed"] == 1 and cols["first_replan_sim_t"] == 20.0
    assert cols["grounding_error_cm"] == 5.0 and cols["t_object_search_s"] == 2.5
    assert set(cols) <= set(bench.CSV_COLUMNS)
