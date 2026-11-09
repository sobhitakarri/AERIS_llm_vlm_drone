"""LiteWing flight core and interface, closed loop against the bench link (no drone)."""
import math
import threading
import time

import pytest

from backend.drone.litewing_flight import (
    HARD_MAX_Z,
    DeadReckoner,
    FlightConfig,
    PositionController,
    body_to_world,
    clamp_z,
    to_display,
    world_to_body,
)
from backend.drone.litewing_interface import LiteWingInterface
from backend.drone.litewing_link import FakeLink


# ------------------------------------------------------------------ pure core
def test_ceiling_is_30_cm_whatever_is_asked():
    assert clamp_z(0.5) == HARD_MAX_Z == 0.30
    assert clamp_z(5.0) == 0.30
    cfg = FlightConfig()
    res = cfg.apply({"target_height": 1.0})
    assert cfg.target_height == 0.30
    assert res["clamped"]["target_height"] == {"requested": 1.0, "used": 0.30}


def test_apply_clamps_gains_and_reports_unknown():
    cfg = FlightConfig()
    res = cfg.apply({"max_correction": 5, "pos_kp": 1.4, "nonsense": 1, "use_height_scaling": "false"})
    assert cfg.max_correction == 0.7 and cfg.pos_kp == 1.4
    assert cfg.use_height_scaling is False
    assert res["unknown"] == ["nonsense"]


def test_flow_velocity_matches_official_formula():
    est = DeadReckoner(FlightConfig())
    # delta * height * (5.4 deg in rad) / (30 * 0.01)
    expected = 4 * 0.3 * (5.4 * math.pi / 180) / (30 * 0.01)
    assert est.velocity_from_delta(4, 0.3) == pytest.approx(expected)
    assert est.velocity_from_delta(4, 0.0) == 0.0


def test_official_axis_convention():
    # official: X is left, Y is forward; planner: x forward, y left
    assert to_display(0.5, 0.0) == (0.0, 0.5)
    assert to_display(0.0, 0.5) == (0.5, 0.0)


def test_yaw_frame_roundtrip():
    for psi in (0.0, 0.7, -2.0):
        wx, wy = body_to_world(0.3, -0.2, psi)
        fwd, left = world_to_body(wx, wy, psi)
        assert (fwd, left) == pytest.approx((0.3, -0.2))
    # turned 90 deg CCW, body-forward is world +y (left)
    assert body_to_world(1.0, 0.0, math.pi / 2) == pytest.approx((0.0, 1.0))


def test_controller_pushes_towards_target_and_is_clamped():
    ctl = PositionController(FlightConfig())
    cx, cy = ctl.step((0.5, -0.5), (0.0, 0.0), (0.0, 0.0))
    assert cx > 0 > cy
    far = ctl.step((50.0, 0.0), (0.0, 0.0), (0.0, 0.0))
    assert far[0] == pytest.approx(0.7)
    capped = ctl.step((50.0, 0.0), (0.0, 0.0), (0.0, 0.0), limit=0.2)
    assert capped[0] == pytest.approx(0.2)


def test_dead_reckoner_integrates_only_when_enabled():
    est = DeadReckoner(FlightConfig())
    t = time.time()
    for i in range(10):
        est.ingest(0, 5, 0.3, t + i * 0.01)
    assert est.world == (0.0, 0.0)
    est.enabled = True
    for i in range(10, 40):
        est.ingest(0, 5, 0.3, t + i * 0.01)
    assert est.world[0] > 0.0          # flow Y is forward, planner x


def test_re_anchor_keeps_the_world_position():
    est = DeadReckoner(FlightConfig())
    est.x, est.y = 0.4, -0.2
    est.re_anchor(time.time())
    assert est.world == pytest.approx((0.4, -0.2)) and (est.x, est.y) == (0.0, 0.0)


# ------------------------------------------------------------ closed loop
@pytest.fixture
def drone():
    cfg = FlightConfig()
    cfg.takeoff_time, cfg.stabilize_time, cfg.waypoint_stabilization, cfg.landing_time = 0.6, 0.6, 0.1, 0.2
    d = LiteWingInterface(uri="sim://test", config=cfg, debug=False)
    d.link = FakeLink("sim://test", seed=3)
    assert d.connect()
    yield d
    d.emergency_stop()
    d.disconnect()


def test_takeoff_is_limited_to_the_ceiling(drone):
    assert drone.takeoff(0.5)                    # "fly 0.5 m high"
    st = drone.get_state()
    assert 0.2 < st.z <= HARD_MAX_Z + 0.03
    assert max(c[4] for c in drone.link.sent if c[0] == "hover") <= HARD_MAX_Z + 1e-9
    assert drone.land()
    time.sleep(0.3)                              # last centimetres after the motors cut
    assert drone.get_state().z < 0.06


def test_move_to_arrives_by_dead_reckoning(drone):
    assert drone.takeoff(0.3)
    assert drone.move_to(0.4, -0.2, 0.3, velocity=0.4)
    x, y = drone.est.world
    assert math.hypot(x - 0.4, y + 0.2) < 0.15
    # the vehicle itself (ground truth in the bench link) is close to where it thinks it is
    assert math.hypot(drone.link.x - 0.4, drone.link.y + 0.2) < 0.2
    assert drone.land()


def test_square_returns_to_start(drone):
    assert drone.takeoff(0.3)
    d = 0.4
    for wp in [(0.0, -d), (d, -d), (d, 0.0), (0.0, 0.0)]:
        assert drone.move_to(wp[0], wp[1], 0.3)
    assert math.hypot(drone.link.x, drone.link.y) < 0.2
    assert drone.land()


def test_no_setpoint_ever_exceeds_the_ceiling_even_for_a_wild_waypoint(drone):
    assert drone.takeoff(0.3)
    drone.move_to(0.2, 0.0, 5.0)                 # asks for 5 m up and is clamped
    assert max(c[4] for c in drone.link.sent if c[0] == "hover") <= HARD_MAX_Z + 1e-9
    assert drone.land()


def test_waypoint_outside_the_area_is_limited(drone):
    assert drone.takeoff(0.3)
    assert drone._target == (0.0, 0.0)
    drone.cfg.waypoint_timeout = 3.0
    drone.move_to(9.0, 0.0, 0.3)
    assert drone._target[0] <= 1.0
    drone.land()


def test_stale_sensor_data_trips_and_lands(drone):
    assert drone.takeoff(0.3)
    drone.link.freeze_flow = True
    ok = drone.hover(2.0)
    assert not ok
    assert "stale" in (drone._tripped or "")
    drone.link.freeze_flow = False
    assert drone.land()


def test_emergency_stop_cuts_the_motors_and_stops_loops(drone):
    assert drone.takeoff(0.3)
    drone.emergency_stop()
    assert drone.link.sent[-3][0] in ("setpoint", "stop") or drone.link.sent[-1][0] in ("setpoint", "stop")
    assert not drone.hover(0.5)
    assert not drone.get_state().is_armed


def test_rotate_closed_loop(drone):
    assert drone.takeoff(0.3)
    assert drone.rotate(60)
    assert abs(drone.link.yaw - 60) < 8
    drone.land()


def test_takeoff_refused_when_tof_not_on_the_ground(drone):
    drone.land()
    drone.link.z = 0.6                            # sensor reads 0.6 m before take-off
    time.sleep(0.15)
    assert not drone.takeoff(0.3)
    drone.link.z = 0.0


def test_takeoff_refused_on_low_battery(drone):
    drone.link.battery = 3.2
    time.sleep(0.2)
    assert not drone.takeoff(0.3)


# --------------------------------------------------------------- debug mode
def test_debug_mode_sends_no_motor_command_but_simulates_the_flight():
    cfg = FlightConfig()
    cfg.takeoff_time, cfg.stabilize_time, cfg.waypoint_stabilization, cfg.landing_time = 0.5, 0.5, 0.1, 0.2
    d = LiteWingInterface(uri="sim://dbg", config=cfg, debug=True)
    assert d.connect() and d.debug and d.virtual
    try:
        assert d.takeoff(0.3)
        assert d.move_to(0.3, 0.0, 0.3)
        assert d.land()
        assert d.link.sent == []                  # not one command reached the link
        assert d.get_state().z == 0.0
    finally:
        d.disconnect()


def test_debug_mode_cannot_change_while_flying(drone):
    assert drone.takeoff(0.3)
    assert drone.set_debug(True) is not None
    drone.land()
    assert drone.set_debug(True) is None


def test_height_falls_back_to_the_tof_when_the_state_estimate_reads_zero():
    """Seen on a real LiteWing: range 0.155 m, stateEstimate.z 0.000, so every flow velocity was 0."""
    d = LiteWingInterface(uri="sim://z", debug=True)
    d.link.no_state_z = True
    assert d.connect()
    try:
        assert d.start_sensor_test() is None
        d.link.held, d.link.z = True, 0.155
        d.link.vf = 0.0
        time.sleep(0.4)
        assert d.height == pytest.approx(0.155, abs=0.02)
        # flow counts now turn into velocity because the height is non-zero
        assert d.est.velocity_from_delta(4, d.height) > 0.0
        d.stop_sensor_test()
    finally:
        d.disconnect()


def test_motor_test_is_capped_ground_only_and_always_ends_at_zero_thrust():
    d = LiteWingInterface(uri="sim://mt", debug=False)
    assert d.connect()
    try:
        r = d.motor_test(percent=80, seconds=60)                  # asks for far too much
        assert r["success"] and r["percent"] == 15.0 and r["seconds"] == 3.0
        thrusts = [c[4] for c in d.link.sent if c[0] == "setpoint"]
        assert max(thrusts) <= int(0.15 * 65535) and thrusts[-1] == 0
        assert ("stop",) in d.link.sent and not d.link.armed
        assert max(r["motor_power_peak"]) > 0 and r["motors_log_available"]   # firmware feedback was read
        assert d.phase == "IDLE" and not d.motor_test_active
    finally:
        d.disconnect()


def test_motor_test_refusals():
    d = LiteWingInterface(uri="sim://mt2", debug=True)
    assert d.connect()
    try:
        assert "Debug" in d.motor_test()["error"]                  # debug never sends a motor command
        d.set_debug(False)
        d.link.battery = 3.3
        time.sleep(0.2)
        assert "battery" in d.motor_test()["error"]
        d.link.battery, d.link.held, d.link.z = 4.1, True, 0.4      # lifted off the desk
        time.sleep(0.3)
        assert "ground" in d.motor_test()["error"]
        assert not any(c[0] == "setpoint" and c[4] > 0 for c in d.link.sent)
    finally:
        d.disconnect()


def test_prop_test_reports_per_motor_vibration_and_flags_a_silent_motor():
    d = LiteWingInterface(uri="sim://pt", debug=True)
    assert d.connect()
    try:
        assert "Debug" in d.prop_test(wait_s=2)["error"]
        d.set_debug(False)
        r = d.prop_test(wait_s=2.5)
        assert r["success"] and [m["motor"] for m in r["motors"]] == [1, 2, 3, 4]
        assert all(m["vibration_seen"] and m["pass_flag"] for m in r["motors"])
        assert d.link.params["health.startPropTest"] == "1"
        assert not any(c[0] == "setpoint" for c in d.link.sent)      # the firmware drives the motors, not us
    finally:
        d.disconnect()


def test_motor_test_stops_at_once_on_e_stop():
    d = LiteWingInterface(uri="sim://mt3", debug=False)
    assert d.connect()
    try:
        threading.Timer(0.8, d.emergency_stop).start()
        t0 = time.time()
        r = d.motor_test(percent=10, seconds=3)
        assert not r["success"] and "operator" in r["error"] and time.time() - t0 < 2.0
        assert [c for c in d.link.sent if c[0] == "setpoint"][-1][4] == 0
    finally:
        d.disconnect()


def test_flow_calibration_scales_the_integrated_distance():
    d = LiteWingInterface(uri="sim://cal", debug=True)
    assert d.connect()
    try:
        assert "Sensor Test" in d.calibrate_flow(0.5)["error"]         # needs the test running
        assert d.start_sensor_test() is None
        d.est.x, d.est.y = 0.35, 0.0                                  # sensor says 0.35 m ...
        r = d.calibrate_flow(0.5)                                     # ... the ruler says 0.5 m
        assert r["success"] and r["gain"] == pytest.approx(0.5 / 0.35)
        assert d.est.velocity_from_delta(4, 0.3) == pytest.approx(
            FlightConfig().flow_gain * 0 + 4 * 0.3 * (5.4 * math.pi / 180) / 0.3 * r["gain"])
        assert d.est.world == (0.0, 0.0)                              # reset for a verification slide
        assert not d.calibrate_flow(0.5)["success"]                   # nothing moved yet
        assert not d.calibrate_flow(9.0)["success"]                   # implausible distance
        d.stop_sensor_test()
    finally:
        d.disconnect()


def test_sensor_test_shows_real_sensors_even_in_debug():
    d = LiteWingInterface(uri="sim://s", debug=True)
    assert d.connect()
    try:
        assert d.start_sensor_test() is None
        assert not d.virtual
        d.link.held, d.link.z = True, 0.12        # carried up by hand
        time.sleep(0.4)
        assert d.height == pytest.approx(0.12, abs=0.02)
        d.est.x = 0.3                             # moved 0.3 m: the PID shows how hard it would push back
        time.sleep(0.1)
        assert d._corr[0] < -0.05 and d.link.sent == []
        d.stop_sensor_test()
    finally:
        d.disconnect()


# --------------------------------------------------------------- recorder
def test_recorder_streams_incrementally_and_signals_a_new_flight(drone):
    time.sleep(0.3)
    first = drone.rec.since(0, -1)
    assert first["t"] and first["seq"] > 0
    nxt = drone.rec.since(first["seq"], first["epoch"])
    assert nxt["epoch"] == first["epoch"]
    drone.rec.reset()
    after = drone.rec.since(first["seq"], first["epoch"])
    assert after["epoch"] != first["epoch"]


def test_leds_reach_the_link(drone):
    assert drone.led_color(10, 20, 30) is None
    assert drone.link.leds["color"] == (10, 20, 30)
    assert drone.led_blink(True) is None and drone.link.leds["blink"] is True
    assert drone.led_clear() is None and drone.link.leds["color"] == (0, 0, 0)


def test_csv_log_is_written(drone, tmp_path):
    drone.rec.csv_dir = tmp_path
    path = drone.set_csv(True)
    time.sleep(0.3)
    drone.set_csv(False)
    text = open(path).read().splitlines()
    assert text[0].startswith("time_s") and len(text) > 3
