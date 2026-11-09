"""LiteWing console API, end to end on the bench link: a sentence in, a drone flying, plots out."""
import time

import pytest
from fastapi.testclient import TestClient

from backend.api.routes import app_context
from backend.api.server import create_app
from backend.drone.litewing_flight import HARD_MAX_Z
from backend.planning import plan_cache

FAST = {"takeoff_time": 0.5, "stabilize_time": 0.5, "waypoint_stabilization": 0.1, "landing_time": 0.2}


@pytest.fixture
def client(tmp_path, monkeypatch):
    monkeypatch.setenv("MISSION_CACHE_PATH", str(tmp_path / "plans.json"))
    monkeypatch.setenv("AERIS_TRACE_DIR", str(tmp_path / "traces"))
    app = create_app(drone_mode="real", model_mode="gemini", drone_uri="sim://litewing")
    with TestClient(app) as c:
        assert c.post("/api/litewing/params", json={"values": FAST}).json()["success"]
        yield c
    drone = app_context["drone"]
    drone.emergency_stop()
    drone.disconnect()
    plan_cache.set_namespace("")


def wait_idle(client, timeout=60):
    t0 = time.time()
    time.sleep(0.3)
    while time.time() - t0 < timeout:
        if not client.get("/api/litewing/status").json()["mission"]["busy"]:
            return
        time.sleep(0.2)
    raise AssertionError("mission still running")


def test_status_reports_the_envelope(client):
    st = client.get("/api/litewing/status").json()
    assert st["active"] and st["simulated"] and st["connected"]
    assert st["ceiling"] == HARD_MAX_Z
    caps = client.get("/api/capabilities").json()
    assert caps["workspace"]["z_max"] == HARD_MAX_Z


def test_fly_half_a_metre_high_is_limited_to_30_cm_and_says_so(client):
    r = client.post("/api/command", json={"command": "take off to 0.5 m"}).json()
    assert r["success"], r
    assert r["skills"][0]["skill"] == "TAKEOFF" and r["skills"][0]["params"]["z"] == HARD_MAX_Z
    assert any(c["param"] == "z" and c["requested"] == 0.5 and c["clipped"] == HARD_MAX_Z for c in r["clips"])
    wait_idle(client)
    drone = app_context["drone"]
    assert max(c[4] for c in drone.link.sent if c[0] == "hover") <= HARD_MAX_Z + 1e-9


def test_sentence_square_flies_a_half_metre_square_and_lands(client):
    r = client.post("/api/command", json={"command": "fly a square"}).json()
    assert r["success"], r
    moves = [s["params"] for s in r["skills"] if s["skill"] == "MOVE_TO"]
    assert len(moves) == 5 and all(m["z"] <= HARD_MAX_Z for m in moves)
    assert max(abs(m["x"]) for m in moves) <= 0.26 and max(abs(m["y"]) for m in moves) <= 0.26   # 0.5 m box
    wait_idle(client)
    drone = app_context["drone"]
    # the plan ended without LAND: the drone must keep holding, not fall
    assert drone.state.is_armed and drone.get_state().z > 0.2
    n = drone.link.n_hover
    time.sleep(0.5)
    assert drone.link.n_hover > n + 10
    assert drone.link.x ** 2 + drone.link.y ** 2 < 0.35 ** 2
    assert client.post("/api/command", json={"command": "land"}).json()["success"]
    wait_idle(client)
    assert not drone.state.is_armed and drone.get_state().z < 0.06


def test_idle_hold_lands_when_no_command_follows(client):
    client.post("/api/litewing/params", json={"values": {"idle_hold_s": 1.0}})
    client.post("/api/command", json={"command": "take off to 0.3 m"})
    wait_idle(client)
    drone = app_context["drone"]
    assert drone.state.is_armed
    t0 = time.time()
    while drone.state.is_armed and time.time() - t0 < 15:
        time.sleep(0.2)
    assert not drone.state.is_armed and drone.get_state().z < 0.06


def test_explicit_size_in_the_sentence(client):
    r = client.post("/api/command", json={"command": "fly a 0.4 m square at 0.2 m"}).json()
    assert r["success"], r
    moves = [s["params"] for s in r["skills"] if s["skill"] == "MOVE_TO"]
    assert max(abs(m["x"]) for m in moves) == pytest.approx(0.2, abs=0.01)
    assert all(m["z"] == 0.2 for m in moves)
    wait_idle(client)


def test_plans_are_cached_under_the_litewing_namespace(client, tmp_path):
    client.post("/api/command", json={"command": "fly a square"})
    wait_idle(client)
    assert plan_cache.get_namespace() == "litewing"
    store = plan_cache.load_store()
    assert any(k.startswith("litewing|") for k in store["exact"])
    plan_cache.set_namespace("")
    assert plan_cache.lookup("fly a square") is None     # the twin's cache is untouched


def test_console_buttons_build_the_official_square(client):
    r = client.post("/api/litewing/flight", json={"kind": "square", "distance": 0.4}).json()
    assert r["success"], r
    moves = [(s["params"]["x"], s["params"]["y"]) for s in r["skills"] if s["skill"] == "MOVE_TO"]
    assert moves == [(0.0, -0.4), (0.4, -0.4), (0.4, 0.0), (0.0, 0.0)]   # right, forward, left, home
    wait_idle(client)


def test_history_streams_over_the_websocket(client):
    client.post("/api/command", json={"command": "take off to 0.3 m"})
    with client.websocket_connect("/api/litewing/ws") as ws:
        first = ws.receive_json()
        assert first["active"] and "history" in first
        time.sleep(1.0)
        second = ws.receive_json()
        assert second["history"]["seq"] >= first["history"]["seq"]
        assert set(("t", "px", "py", "h", "cx", "vx")) <= set(second["history"])
    wait_idle(client)


def test_params_are_clamped_and_reported(client):
    r = client.post("/api/litewing/params", json={"values": {"target_height": 0.9, "pos_kp": 1.2}}).json()
    assert r["applied"]["target_height"] == HARD_MAX_Z
    assert "target_height" in r["clamped"]
    assert client.post("/api/litewing/params/reset").json()["success"]


def test_stop_lands_a_running_mission_under_control(client):
    assert client.post("/api/litewing/params", json={"values": {"hover_duration": 20}}).json()["success"]
    client.post("/api/litewing/flight", json={"kind": "start_flight", "hover_s": 20})
    time.sleep(2.5)
    assert app_context["drone"].state.is_armed
    assert client.post("/api/litewing/stop").json()["success"]
    wait_idle(client)
    drone = app_context["drone"]
    assert not drone.state.is_armed and drone.get_state().z < 0.06
    assert ("stop",) in drone.link.sent


def test_joystick_session(client):
    assert client.post("/api/litewing/joystick", json={"action": "start"}).json()["success"]
    client.post("/api/litewing/joystick", json={"action": "key", "key": "w", "down": True})
    time.sleep(1.2)
    client.post("/api/litewing/joystick", json={"action": "key", "key": "w", "down": False})
    drone = app_context["drone"]
    assert drone.est.world[0] > 0.05
    assert client.post("/api/litewing/joystick", json={"action": "stop"}).json()["success"]
    assert not drone.state.is_armed


def test_led_endpoints(client):
    assert client.post("/api/litewing/led", json={"action": "set", "r": 1, "g": 2, "b": 3}).json()["success"]
    assert app_context["drone"].link.leds["color"] == (1, 2, 3)
    assert client.post("/api/litewing/led", json={"action": "clear"}).json()["success"]


def test_debug_toggle_and_dry_run_sends_nothing(client):
    assert client.post("/api/litewing/debug", json={"enabled": True}).json()["success"]
    drone = app_context["drone"]
    drone.link.sent.clear()
    client.post("/api/command", json={"command": "take off to 0.3 m"})
    wait_idle(client)
    assert drone.link.sent == []
