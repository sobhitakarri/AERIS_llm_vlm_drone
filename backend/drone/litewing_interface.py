"""
LiteWing (ESP32-S3 + Flight Positioning Module) interface.

Closed loop on the vehicle's own sensors, the way Circuit-Digest's
dead-reckoning-maneuvers.py flies it: optical flow (PMW3901) gives velocity,
integrated to a position; the VL53L1X time-of-flight gives height; a position
PID plus a velocity-damping PID turns the error into a hover setpoint at 50 Hz.
Python sends only (vx, vy, yawrate, height). The ESP32 keeps the motors.

  Hardware / firmware : https://github.com/Circuit-Digest/ESP-Drone
  Official scripts    : https://github.com/Circuit-Digest/LiteWing/tree/main/Python-Scripts

Safety, in force whatever the plan, the LLM or the dashboard asks for:
  * height is clamped to HARD_MAX_Z (0.30 m) on every setpoint that leaves here;
  * the flight area is +-HARD_MAX_XY from the take-off point;
  * stale sensor data, a tilt past MAX_TILT, a ceiling breach or a low battery
    trip a controlled landing;
  * debug mode (default on real hardware) sends no motor command at all and
    simulates the motion from the commanded setpoints instead.

Frames: the planner's (x forward, y left, origin at the take-off point). The
dashboard shows the official script's (X left, Y forward) via `to_display`.
"""
from __future__ import annotations

import math
import os
import threading
import time
from typing import Dict, Optional, Tuple

from backend.core.config import settings
from backend.core.logger import get_logger
from backend.drone.base_interface import DroneInterface
from backend.drone.litewing_flight import (
    CONTROL_DT,
    DEG_TO_RAD,
    HARD_MAX_XY,
    HARD_MAX_Z,
    HARD_MIN_Z,
    LIMITS,
    DeadReckoner,
    FlightConfig,
    FlightRecorder,
    OutputLog,
    PositionController,
    clamp_z,
    to_display,
    world_to_body,
)
from backend.drone.litewing_link import CFLIB_AVAILABLE, CflibLink, FakeLink
from backend.schemas.telemetry import DroneState, FlightMode

logger = get_logger("LiteWingInterface")

_DEFAULT_URI = "udp://192.168.43.42"
_NEOPIXEL_PORT = 0x09


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


class LiteWingInterface(DroneInterface):
    DATA_TIMEOUT = 0.3          # s without a motion packet in flight -> land
    MIN_TAKEOFF_V = 3.80        # resting voltage. A unit at 3.74 V sagged to 3.37 V in a 1.5 s, 15 % motor test
    LAND_V = 3.0
    WARN_V = 3.3
    MAX_TILT = 50.0             # deg
    CEILING_TRIP = HARD_MAX_Z + 0.20
    GROUND_MAX_RANGE = 0.25     # a ToF reading above this before take-off is not "on the ground"

    def __init__(self, uri: str = None, link=None, debug: Optional[bool] = None, config: Optional[FlightConfig] = None):
        raw = uri or settings.drone_uri or _DEFAULT_URI
        if raw.startswith("udp://") and not raw.rsplit(":", 1)[-1].isdigit():
            # The official scripts omit the port; cflib 0.1.32 needs it, and the
            # ESP-Drone firmware listens on 2390 (UDP_SERVER_PORT in wifi_esp32.c).
            raw = raw.rstrip("/") + ":2390"
        self.uri = raw
        self.simulated = raw.startswith("sim://") or isinstance(link, FakeLink)
        self.link = link or (FakeLink(raw) if raw.startswith("sim://") else CflibLink(raw))
        if debug is None:
            env = os.getenv("LITEWING_DEBUG")
            debug = (env.strip().lower() in ("1", "true", "yes", "on")) if env is not None else not self.simulated
        self.debug = bool(debug)

        self.cfg = config or FlightConfig()
        self._defaults = FlightConfig().to_dict()
        self.est = DeadReckoner(self.cfg)
        self.ctl = PositionController(self.cfg)
        self.rec = FlightRecorder()
        self.output = OutputLog()

        self.state = DroneState()
        self.is_connected = False
        self._lock = threading.RLock()
        self._abort = threading.Event()
        self._sampler: Optional[threading.Thread] = None
        self._handles = []

        # sensors (real, from the radio)
        self.sensor_ready = False
        self.last_data_t = 0.0
        self.height = 0.0
        self.range_m = 0.0
        self.delta = (0.0, 0.0)
        self.squal: Optional[float] = None       # PMW3901 surface quality, when the firmware logs it
        self.battery_v = 0.0
        self.motors = [0, 0, 0, 0]               # firmware's commanded motor power (0..65535)
        self.motor_test_active = False
        self.roll = self.pitch = self.yaw = 0.0
        self.has_attitude = False
        # control state
        self.phase = "IDLE"
        self.sensor_test = False
        self._target = (0.0, 0.0)
        self._tz = 0.0
        self._corr = (0.0, 0.0)
        self._virt_v = (0.0, 0.0)
        self._tripped: Optional[str] = None
        self._last_tick = 0.0
        self._land_lock = threading.Lock()
        self._hold_thread: Optional[threading.Thread] = None
        self.led_state = {"r": 255, "g": 255, "b": 255, "blinking": False}
        # joystick session
        self._manual = False
        self._manual_thread: Optional[threading.Thread] = None
        self._keys: set = set()
        self.hold_origin = True

    # ------------------------------------------------------------------ helpers
    def log(self, message: str) -> None:
        self.output.add(message)
        logger.info("[LiteWing] %s", message)

    @property
    def virtual(self) -> bool:
        """Motion and height come from the commanded setpoints, not the sensors."""
        return self.debug and not self.sensor_test

    def workspace(self) -> dict:
        return {"x": (-HARD_MAX_XY, HARD_MAX_XY), "y": (-HARD_MAX_XY, HARD_MAX_XY), "z": (0.15, HARD_MAX_Z)}

    @property
    def runtime(self) -> dict:
        return {"airframe": {"name": "LiteWing ESP32-S3 + Positioning Module", "v_max": [0.4, 0.4]}}

    def envelope(self) -> dict:
        """Hints for the offline planner: default height and shape size."""
        return {"default_z": min(self.cfg.target_height, HARD_MAX_Z), "z_min": 0.15, "z_max": HARD_MAX_Z,
                "shape_size": self.cfg.maneuver_distance}

    # --------------------------------------------------------------- connection
    def connect(self) -> bool:
        if not self.simulated and not CFLIB_AVAILABLE:
            logger.error("cflib is not installed. pip install cflib")
            return False
        self.log(f"Connecting to LiteWing at {self.uri}" + (" (bench simulation)" if self.simulated else "") + " ...")
        try:
            self.link.open()
            time.sleep(0.3 if self.simulated else 1.0)
            self._start_logs()
            self.is_connected = True
            self._abort.clear()
            self._start_sampler()
            self.phase = "IDLE"
            deadline = time.time() + 3.0
            while not self.sensor_ready and time.time() < deadline:
                time.sleep(0.05)
            if not self.sensor_ready:
                self.log("WARNING: no motion-sensor packets yet. Check the positioning module is seated.")
            self.log("Connected." + (" DEBUG MODE: motors disabled." if self.debug else ""))
            return True
        except Exception as exc:
            logger.error("[LiteWing] Connection failed to %s: %s", self.uri, exc)
            self.log(f"Connection failed: {exc}")
            self.is_connected = False
            return False

    def _start_logs(self) -> None:
        motion = self.link.make_log(
            "Motion", 10,
            [("motion.deltaX", "int16_t"), ("motion.deltaY", "int16_t"), ("stateEstimate.z", "float"),
             ("range.zrange", "uint16_t"), ("motion.squal", "uint8_t")], self._on_motion)
        motion.start()
        self._handles.append(motion)
        try:
            status = self.link.make_log(
                "Status", 50,
                [("pm.vbat", "float"), ("stateEstimate.roll", "float"), ("stateEstimate.pitch", "float"),
                 ("stateEstimate.yaw", "float")], self._on_status)
            status.start()
            self._handles.append(status)
        except Exception as exc:
            self.log(f"Attitude/battery log unavailable ({exc}); yaw turns will be timed.")
        try:        # what the firmware actually commanded to each motor (feedback for the motor test)
            motors = self.link.make_log(
                "Motors", 50, [(f"motor.m{i}", "uint32_t") for i in (1, 2, 3, 4)], self._on_motors)
            motors.start()
            self._handles.append(motors)
        except Exception as exc:
            self.log(f"Motor output log unavailable ({exc}); the motor test cannot confirm output.")

    def _on_motors(self, data: dict) -> None:
        with self._lock:
            self.motors = [int(data.get(f"motor.m{i}", 0) or 0) for i in (1, 2, 3, 4)]

    def _on_motion(self, data: dict) -> None:
        now = time.time()
        with self._lock:
            self.last_data_t = now
            self.sensor_ready = True
            self.range_m = float(data.get("range.zrange", 0.0) or 0.0) / 1000.0
            dx, dy = float(data.get("motion.deltaX", 0.0)), float(data.get("motion.deltaY", 0.0))
            self.delta = (dx, dy)
            self.squal = data.get("motion.squal", self.squal)
            if self.virtual:
                return
            # stateEstimate.z can sit at 0 on the bench (seen on a real unit: range 0.155 m,
            # estimate 0.000), which zeroes every flow velocity. The ToF range is the
            # sensor behind it, so fall back to that.
            z = float(data.get("stateEstimate.z", 0.0) or 0.0)
            if z > 0.02:
                self.height = z
            else:
                # The firmware's own estimate is an IIR of the ToF (alpha 0.9,
                # position_estimator_altitude.c) but follows the barometer (about 0)
                # until it sees fresh ToF samples. Reproduce that filter on the range.
                self.height = self.range_m if self.height <= 0.0 else 0.9 * self.height + 0.1 * self.range_m
            self.est.ingest(dx, dy, self.height, now, self.yaw)

    def _on_status(self, data: dict) -> None:
        with self._lock:
            if "pm.vbat" in data:
                self.battery_v = float(data["pm.vbat"])
            if "stateEstimate.yaw" in data:
                self.has_attitude = True
                if not self.virtual:
                    self.roll = float(data.get("stateEstimate.roll", self.roll))
                    self.pitch = float(data.get("stateEstimate.pitch", self.pitch))
                    self.yaw = float(data["stateEstimate.yaw"])

    def disconnect(self) -> None:
        self._manual = False
        self.sensor_test = False
        if self.is_connected:
            try:
                self.link.send_setpoint(0, 0, 0, 0)
            except Exception:
                pass
            for h in self._handles:
                try:
                    h.stop()
                except Exception:
                    pass
            self._handles = []
            self.is_connected = False
            self.rec.csv_stop()
            try:
                self.link.close()
            except Exception:
                pass
            self.log("Disconnected.")
        if self._sampler:
            self._sampler.join(timeout=1.0)
            self._sampler = None

    # ------------------------------------------------------------------ sampler
    def _start_sampler(self) -> None:
        if self._sampler and self._sampler.is_alive():
            return
        self._sampler = threading.Thread(target=self._sample_loop, name="litewing-sampler", daemon=True)
        self._sampler.start()

    def _sample_loop(self) -> None:
        """25 Hz history for the dashboard, whether or not the drone is flying."""
        while self.is_connected:
            with self._lock:
                px, py = to_display(*self.est.world)
                vx, vy = to_display(self.est.vx, self.est.vy)
                cx, cy = to_display(*self._corr)
                tx, ty = to_display(*self._target)
                height, rng = self.height, self.range_m
                phase, batt = self.phase, self.battery_v
            self.rec.sample(vx=vx, vy=vy, px=px, py=py, cx=cx, cy=cy, h=height, r=rng, tx=tx, ty=ty,
                            phase=phase, battery=batt)
            time.sleep(0.04)

    # ------------------------------------------------------------------- safety
    def _unsafe(self, in_flight: bool = True) -> Optional[str]:
        if not self.simulated and not self.link.is_connected():
            return "radio link lost"
        if not self.virtual and in_flight and self.sensor_ready and time.time() - self.last_data_t > self.DATA_TIMEOUT:
            return f"sensor data stale ({time.time() - self.last_data_t:.2f} s)"
        if self.battery_v > 0 and self.battery_v < self.LAND_V and not self.virtual:
            return f"battery {self.battery_v:.2f} V below {self.LAND_V:.2f} V"
        if max(abs(self.roll), abs(self.pitch)) > self.MAX_TILT and not self.virtual:
            return f"tilt {max(abs(self.roll), abs(self.pitch)):.0f} deg"
        if self.height > self.CEILING_TRIP:
            return f"height {self.height:.2f} m above the {HARD_MAX_Z:.2f} m ceiling"
        wx, wy = self.est.world
        if abs(wx) > HARD_MAX_XY * 1.3 or abs(wy) > HARD_MAX_XY * 1.3:
            return f"position ({wx:.2f}, {wy:.2f}) outside the flight area"
        return None

    def _trip(self, reason: str) -> None:
        if self._tripped is None:
            self._tripped = reason
            self.log(f"SAFETY: {reason}. Stopping.")
        self._abort.set()

    # ---------------------------------------------------------------- control
    def _tick(self, target: Tuple[float, float], z: float, *, yawrate: float = 0.0,
              limit: Optional[float] = None, phase: Optional[str] = None, force: bool = False) -> bool:
        """One 50 Hz cycle: PID -> hover setpoint. False when the caller must stop."""
        t0 = time.time()
        if phase != "HOLD":
            self._last_tick = t0            # the idle-hold worker watches for anyone else commanding
        if phase:
            self.phase = phase
        if not force:
            if self._abort.is_set():
                return False
            why = self._unsafe()
            if why:
                self._trip(why)
                return False
        z = clamp_z(z) if z > 0 else 0.0
        with self._lock:
            ox, oy = self.est.ox, self.est.oy
            rel = (target[0] - ox, target[1] - oy)
            cx, cy = self.ctl.step(rel, (self.est.x, self.est.y), (self.est.vx, self.est.vy), limit)
            self._corr = (cx, cy)
            psi = (self.yaw - self.est.yaw0) * DEG_TO_RAD
            fwd, left = world_to_body(cx, cy, psi)
            send_vx, send_vy = self.cfg.trim_vx + fwd, self.cfg.trim_vy + left
            if self.virtual:
                a = 1.0 - math.exp(-CONTROL_DT / 0.25)
                vx = self._virt_v[0] + (cx - self._virt_v[0]) * a
                vy = self._virt_v[1] + (cy - self._virt_v[1]) * a
                self._virt_v = (vx, vy)
                self.est.advance_virtual(vx, vy, CONTROL_DT)
                self.height += (z - self.height) * (1.0 - math.exp(-CONTROL_DT / 0.35))
                self.yaw += yawrate * CONTROL_DT
        if not self.virtual:
            try:
                self.link.send_hover(send_vx, send_vy, yawrate, z)
            except Exception as exc:
                self._trip(f"send failed: {exc}")
                return False
        time.sleep(max(0.0, CONTROL_DT - (time.time() - t0)))
        return True

    def _set_mode(self, mode: FlightMode) -> None:
        self.state.flight_mode = mode

    def _preflight(self) -> Optional[str]:
        if not self.is_connected:
            return "drone not connected"
        if self._manual:
            return "joystick session active"
        if self.virtual:
            return None
        if not self.sensor_ready or time.time() - self.last_data_t > 0.5:
            return "no fresh sensor data (run Sensor Test and check the positioning module)"
        if self.battery_v > 0 and self.battery_v < self.MIN_TAKEOFF_V:
            return f"battery {self.battery_v:.2f} V below {self.MIN_TAKEOFF_V:.2f} V"
        if self.range_m > self.GROUND_MAX_RANGE:
            return f"ToF reads {self.range_m:.2f} m before take-off (not on the ground, or sensor blocked)"
        return None

    def takeoff(self, altitude: float = 0.3) -> bool:
        why = self._preflight()
        if why:
            self.log(f"Take-off refused: {why}.")
            return False
        z = clamp_z(altitude if altitude and altitude >= HARD_MIN_Z else self.cfg.target_height)
        if z != float(altitude or 0):
            self.log(f"Take-off height {altitude} m limited to {z:.2f} m (ceiling {HARD_MAX_Z:.2f} m).")
        self._abort.clear()
        self._tripped = None
        self.rec.reset()
        self.output.add("--- new flight ---")
        with self._lock:
            self.est.reset()
            self.est.yaw0 = self.yaw
            self.est.yaw_frame = True          # in flight the frame follows the heading
            self.est.enabled = True
            self.ctl.reset()
            self._virt_v = (0.0, 0.0)
            if self.virtual:
                self.height = 0.0
        self._target, self._tz, self._corr = (0.0, 0.0), z, (0.0, 0.0)
        self.log(f"Take-off to {z:.2f} m" + (" (DEBUG: simulated, no motor output)" if self.virtual else ""))
        try:
            if not self.virtual:
                try:
                    self.link.arm(True)
                    time.sleep(0.5)
                except Exception as exc:
                    self.log(f"Arm request skipped: {exc}")
                self.link.send_setpoint(0, 0, 0, 0)          # official unlock
                time.sleep(0.1)
                try:
                    self.link.set_param("commander.enHighLevel", "1")
                except Exception as exc:
                    self.log(f"enHighLevel not set ({exc}); hover setpoints may be ignored.")
                time.sleep(0.5)
            self.state.is_armed = True
            self._set_mode(FlightMode.TAKING_OFF)
            start_h = self.height
            end = time.time() + self.cfg.takeoff_time
            while time.time() < end:
                frac = 1.0 - max(0.0, end - time.time()) / self.cfg.takeoff_time
                h = start_h + (z - start_h) * frac if self.cfg.takeoff_ramp else z
                if not self._tick((0.0, 0.0), h, phase="TAKEOFF"):
                    return self._fail_flight("take-off interrupted")
            end = time.time() + self.cfg.stabilize_time
            while time.time() < end:
                if not self._tick((0.0, 0.0), z, phase="STABILIZING"):
                    return self._fail_flight("take-off interrupted")
            if not self.virtual and self.height < 0.5 * z:
                self._trip(f"did not climb (height {self.height:.2f} m of {z:.2f} m): check props and battery")
                return self._fail_flight("no climb")
            self._set_mode(FlightMode.HOVERING)
            self.phase = "HOVER"
            self._start_idle_hold()
            return True
        except Exception as exc:
            logger.error("[LiteWing] take-off error: %s", exc)
            return self._fail_flight(f"take-off error: {exc}")

    def _start_idle_hold(self) -> None:
        if self._hold_thread and self._hold_thread.is_alive():
            return
        self._hold_thread = threading.Thread(target=self._idle_hold_loop, name="litewing-hold", daemon=True)
        self._hold_thread.start()

    def _idle_hold_loop(self) -> None:
        """Keep position-hold running whenever no command loop is: the firmware
        cuts the motors if setpoints stop, and a plan need not end with LAND.
        Lands after `idle_hold_s` without a new command."""
        idle_since = None
        while self.is_connected and self.state.is_armed:
            if self._manual or time.time() - self._last_tick < 0.1:
                idle_since = None
                time.sleep(0.02)
                continue
            if self._abort.is_set():                 # tripped or stopped: bring it down
                self.land()
                return
            idle_since = idle_since or time.time()
            if time.time() - idle_since > self.cfg.idle_hold_s:
                self.log(f"No command for {self.cfg.idle_hold_s:.0f} s: landing.")
                self.land()
                return
            if not self._tick(self._target, self._tz, phase="HOLD"):
                self.land()
                return

    def _fail_flight(self, why: str) -> bool:
        self.log(f"Flight stopped: {why}.")
        if self.state.is_armed and not self.virtual and self.height > 0.05:
            self.land()
        elif self.virtual:
            self.state.is_armed = False
            self.height = 0.0
            self._set_mode(FlightMode.IDLE)
            self.phase = "IDLE"
        return False

    def hover(self, duration: float = 2.0) -> bool:
        if not self.is_connected or not self.state.is_armed:
            return False
        duration = max(0.0, min(float(duration), 600.0))
        self._set_mode(FlightMode.HOVERING)
        end = time.time() + duration
        while time.time() < end:
            now = time.time()
            if self.est.periodic_reset_due(now):
                self.est.re_anchor(now)
                self.log("Periodic position re-anchor (drift reset).")
            if not self._tick(self._target, self._tz, phase="HOVER"):
                return False
        return True

    def move_to(self, x: float, y: float, z: float, velocity: float = 0.4) -> bool:
        if not self.is_connected or not self.state.is_armed:
            return False
        tx = max(-HARD_MAX_XY, min(HARD_MAX_XY, float(x)))
        ty = max(-HARD_MAX_XY, min(HARD_MAX_XY, float(y)))
        tz = clamp_z(z)
        if (tx, ty) != (float(x), float(y)) or tz != float(z):
            self.log(f"Waypoint ({x:.2f}, {y:.2f}, {z:.2f}) limited to ({tx:.2f}, {ty:.2f}, {tz:.2f}).")
        self._target, self._tz = (tx, ty), tz
        dx, dy = to_display(tx, ty)
        self.log(f"Moving to waypoint X={dx:.2f} Y={dy:.2f} (left/forward) at {tz:.2f} m")
        self._set_mode(FlightMode.NAVIGATING)
        arrived = None
        t0 = time.time()
        while time.time() - t0 < self.cfg.waypoint_timeout:
            if not self._tick((tx, ty), tz, limit=velocity, phase="MISSION"):
                return False
            wx, wy = self.est.world
            near = math.hypot(wx - tx, wy - ty) < self.cfg.maneuver_threshold and abs(self.height - tz) < 0.08
            if near:
                arrived = arrived or time.time()
                if time.time() - arrived >= self.cfg.waypoint_stabilization:
                    self.log("Waypoint reached.")
                    self._set_mode(FlightMode.HOVERING)
                    return True
            else:
                arrived = None
        self.log(f"Waypoint timeout after {self.cfg.waypoint_timeout:.0f} s at ({self.est.world[0]:.2f}, {self.est.world[1]:.2f}).")
        return False

    def rotate(self, angle_degrees: float) -> bool:
        if not self.is_connected or not self.state.is_armed:
            return False
        angle = float(angle_degrees)
        self.log(f"Yaw {angle:.0f} deg")
        if self.has_attitude or self.virtual:
            goal = self.yaw + angle
            limit_t = abs(angle) / 15.0 + 4.0
            t0 = time.time()
            settled = None
            while time.time() - t0 < limit_t:
                err = _wrap180(goal - self.yaw)
                rate = 0.0 if abs(err) < 3.0 else max(-45.0, min(45.0, 2.0 * err))
                if 0 < abs(rate) < 10.0:
                    rate = math.copysign(10.0, rate)
                if not self._tick(self._target, self._tz, yawrate=rate, phase="YAW"):
                    return False
                if rate == 0.0:
                    settled = settled or time.time()
                    if time.time() - settled > 0.3:
                        return True
                else:
                    settled = None
            self.log("Yaw did not settle within the time limit.")
            return False
        rate = 30.0 if angle >= 0 else -30.0
        end = time.time() + min(abs(angle) / 30.0, 12.0)
        while time.time() < end:
            if not self._tick(self._target, self._tz, yawrate=rate, phase="YAW"):
                return False
        return True

    def land(self) -> bool:
        if not self.is_connected:
            return False
        with self._land_lock:            # the hold worker and the executor may both ask
            if not self.state.is_armed and self.height < 0.06:
                return True
            return self._land_locked()

    def _land_locked(self) -> bool:
        self._manual = False
        self.log("Landing")
        self._set_mode(FlightMode.LANDING)
        try:
            z0 = max(self.height, 0.05)
            t0 = time.time()
            while True:
                z = z0 - self.cfg.descent_rate * (time.time() - t0)
                if z <= 0.04 or time.time() - t0 > 12.0:
                    break
                if not self._tick(self._target, z, phase="LANDING", force=True):
                    break
            end = time.time() + self.cfg.landing_time
            while time.time() < end:
                self._tick(self._target, 0.0, phase="LANDING", force=True)
            if not self.virtual:
                self.link.send_setpoint(0, 0, 0, 0)
                time.sleep(0.1)
                try:
                    self.link.send_stop()
                    self.link.arm(False)
                except Exception:
                    pass
            with self._lock:
                self.est.enabled = False
                if self.virtual:
                    self.height = 0.0
                    self.est.vx = self.est.vy = 0.0
                    self._virt_v = (0.0, 0.0)
            self.state.is_armed = False
            self._set_mode(FlightMode.IDLE)
            self.phase = "COMPLETE"
            self._corr = (0.0, 0.0)
            return True
        except Exception as exc:
            logger.error("[LiteWing] landing error: %s", exc)
            return False

    def emergency_stop(self) -> None:
        self.log("EMERGENCY STOP")
        self._abort.set()
        self._manual = False
        if self.is_connected:
            try:
                self.link.send_setpoint(0, 0, 0, 0)
                self.link.send_stop()
                self.link.arm(False)
            except Exception as exc:
                logger.error("[LiteWing] e-stop error: %s", exc)
        with self._lock:
            self.est.enabled = False
            if self.virtual:
                self.height = 0.0
        self.state.is_armed = False
        self._set_mode(FlightMode.EMERGENCY)
        self.phase = "EMERGENCY"

    # ----------------------------------------------------------------- state
    def get_state(self) -> DroneState:
        with self._lock:
            wx, wy = self.est.world
            st = self.state
            st.x, st.y, st.z = wx, wy, self.height
            st.vx, st.vy = self.est.vx, self.est.vy
            st.roll, st.pitch, st.yaw = self.roll, self.pitch, self.yaw % 360.0
            if self.battery_v > 0:
                st.battery_v = self.battery_v
                st.battery_percentage = int(max(0, min(100, (self.battery_v - 3.3) / (4.2 - 3.3) * 100)))
        return st

    def home_position(self) -> tuple:
        return (0.0, 0.0)

    # ------------------------------------------------------- dashboard hooks
    def apply_params(self, values: dict) -> dict:
        res = self.cfg.apply(values)
        if res["applied"]:
            self.log("Applied: " + ", ".join(f"{k}={v:g}" if not isinstance(v, bool) else f"{k}={v}"
                                              for k, v in res["applied"].items()))
        for k, c in res["clamped"].items():
            self.log(f"{k}: {c['requested']:g} outside the safe range, used {c['used']:g}")
        return res

    def reset_params(self) -> dict:
        res = self.cfg.apply(self._defaults)
        self.log("Parameters reset to the official defaults.")
        return res

    def set_debug(self, enabled: bool) -> Optional[str]:
        if self.state.is_armed:
            return "cannot change debug mode while flying"
        self.debug = bool(enabled)
        self.log("DEBUG MODE ON: motors disabled, motion simulated." if self.debug else
                 "DEBUG MODE OFF: setpoints go to the motors. Props on, clear area.")
        return None

    def reset_battery(self) -> None:
        with self._lock:
            self.battery_v = 0.0
        self.log("Battery reading reset; waiting for the next sample.")

    def start_sensor_test(self, log_csv: bool = False) -> Optional[str]:
        if not self.is_connected:
            return "drone not connected"
        if self.state.is_armed or self._manual:
            return "cannot run the sensor test while flying"
        self.rec.reset()
        self.output.add("--- sensor test ---")
        with self._lock:
            self.est.reset()
            self.est.yaw_frame = False         # same as the official sensor test: body frame
            self.est.enabled = True
            self.sensor_test = True
            self.phase = "SENSOR_TEST"
        if log_csv:
            self.log(f"CSV logging to {self.rec.csv_start()}")
        threading.Thread(target=self._sensor_test_loop, name="litewing-sensor-test", daemon=True).start()
        self.log("Sensor test: carry the drone by hand and watch height, range and flow velocity.")
        return None

    def _sensor_test_loop(self) -> None:
        """Official sensor test: the PID runs against the origin every 20 ms, nothing is sent
        to the motors, so the Control Corrections plot shows what it would command."""
        self.ctl.reset()
        self._target = (0.0, 0.0)
        while self.sensor_test and self.is_connected:
            with self._lock:
                self._corr = self.ctl.step((0.0, 0.0), (self.est.x, self.est.y), (self.est.vx, self.est.vy))
            time.sleep(CONTROL_DT)
        self._corr = (0.0, 0.0)

    # ------------------------------------------------------------- motor test
    MOTOR_TEST_MAX_PCT = 15.0
    MOTOR_TEST_MAX_S = 3.0
    MOTOR_TEST_MIN_V = 3.5

    def motor_test(self, percent: float = 8.0, seconds: float = 1.5) -> dict:
        """Spin all four motors at a low raw thrust, for a short time, on the ground.

        Raw thrust goes through the commander, so the firmware's own watchdog (500 ms)
        cuts it if our packets stop; the per-motor `motorPowerSet` override is not used
        because it would keep spinning if the link dropped. Props should be removed.
        Never part of a flight: refused unless on the ground, debug off, battery ok.
        """
        if not self.is_connected:
            return {"success": False, "error": "drone not connected"}
        if self.debug:
            return {"success": False, "error": "Debug Mode is on (no motor command is ever sent). Untick it for the motor test."}
        if self.state.is_armed or self._manual or self.sensor_test or self.motor_test_active:
            return {"success": False, "error": "busy: land and stop the sensor test / joystick first"}
        if self.battery_v > 0 and self.battery_v < self.MOTOR_TEST_MIN_V:
            return {"success": False, "error": f"battery {self.battery_v:.2f} V is below {self.MOTOR_TEST_MIN_V:.2f} V"}
        if self.height > 0.15 or self.range_m > 0.25:
            return {"success": False, "error": "the drone does not read as sitting on the ground"}
        pct = max(1.0, min(float(percent), self.MOTOR_TEST_MAX_PCT))
        secs = max(0.5, min(float(seconds), self.MOTOR_TEST_MAX_S))
        thrust = int(pct / 100.0 * 65535)
        self._abort.clear()
        self.motor_test_active = True
        self.phase = "MOTOR_TEST"
        v0, peak, vmin, aborted = self.battery_v, [0, 0, 0, 0], self.battery_v or 9.9, None
        self.log(f"MOTOR TEST: all four motors at {pct:.0f}% for {secs:.1f} s (props should be off)")
        try:
            try:
                self.link.arm(True)
                time.sleep(0.3)
            except Exception as exc:
                self.log(f"Arm request skipped: {exc}")
            self.link.send_setpoint(0, 0, 0, 0)          # unlock
            time.sleep(0.1)
            ramp = 0.5
            t0 = time.time()
            while time.time() - t0 < ramp + secs:
                if self._abort.is_set():
                    aborted = "stopped by operator"
                    break
                if self.battery_v > 0 and self.battery_v < self.LAND_V + 0.1:
                    aborted = f"battery {self.battery_v:.2f} V"
                    break
                if max(abs(self.roll), abs(self.pitch)) > 30.0:
                    aborted = "the drone tilted: it is being moved"
                    break
                frac = min(1.0, (time.time() - t0) / ramp)
                self.link.send_setpoint(0, 0, 0, int(thrust * frac))
                with self._lock:
                    peak = [max(a, b) for a, b in zip(peak, self.motors)]
                    if self.battery_v > 0:
                        vmin = min(vmin, self.battery_v)
                time.sleep(CONTROL_DT)
        finally:
            for _ in range(5):                            # zero thrust, repeatedly: it must land
                try:
                    self.link.send_setpoint(0, 0, 0, 0)
                except Exception:
                    pass
                time.sleep(0.02)
            try:
                self.link.send_stop()
                self.link.arm(False)
            except Exception:
                pass
            self.motor_test_active = False
            self.phase = "IDLE"
        if aborted is None and not any(peak):
            # Ran to the end, but the firmware never reported any motor power. Do not call that a pass.
            aborted = ("no motor output reported by the firmware: the thrust was not applied, is below the motors' "
                       "start level (try a higher %), or the firmware has no motor log. Check by eye/ear.")
        self.log("MOTOR TEST " + (f"ended early: {aborted}" if aborted else "done") + f". Motor power seen: {peak}")
        return {"success": aborted is None, "error": aborted, "percent": pct, "seconds": secs,
                "motor_power_peak": peak, "motor_power_expected": thrust,
                "battery_start": v0, "battery_min": None if vmin > 9 else vmin,
                "motors_log_available": any(peak)}

    def prop_test(self, wait_s: float = 7.0) -> dict:
        """The firmware's own propeller self-test (`health.startPropTest`): each motor is
        pulsed for about 50 ms and the IMU vibration and battery drop are measured. This is
        the only per-motor feedback the LiteWing has: there is no RPM or current sensing, and
        `motor.m1..m4` is only the commanded power. The firmware scores a motor as passing
        when its vibration is LOW, so a motor that never turned also passes; the variances
        are what show a motor that did not run (compared against the others)."""
        if not self.is_connected:
            return {"success": False, "error": "drone not connected"}
        if self.debug:
            return {"success": False, "error": "Debug Mode is on (no motor command is ever sent). Untick it first."}
        if self.state.is_armed or self._manual or self.sensor_test or self.motor_test_active:
            return {"success": False, "error": "busy: land and stop the sensor test / joystick first"}
        if self.battery_v > 0 and self.battery_v < self.MOTOR_TEST_MIN_V:
            return {"success": False, "error": f"battery {self.battery_v:.2f} V is below {self.MOTOR_TEST_MIN_V:.2f} V"}
        if self.height > 0.15 or self.range_m > 0.25:
            return {"success": False, "error": "the drone does not read as sitting on the ground"}
        got: Dict[str, float] = {}
        lock = threading.Lock()

        def _take(data: dict) -> None:
            with lock:
                got.update({k: float(v) for k, v in data.items()})

        handles = []
        try:
            for name, vs in (("HealthX", [(f"health.motorVarXM{i}", "float") for i in (1, 2, 3, 4)]),
                             ("HealthY", [(f"health.motorVarYM{i}", "float") for i in (1, 2, 3, 4)]
                              + [("health.motorPass", "uint8_t")])):
                h = self.link.make_log(name, 100, vs, _take)
                h.start()
                handles.append(h)
        except Exception as exc:
            return {"success": False, "error": f"this firmware does not expose the propeller self-test log ({exc})"}
        self._abort.clear()
        self.motor_test_active = True
        self.phase = "PROP_TEST"
        v0 = self.battery_v
        self.log("PROP TEST: the firmware pulses each motor briefly and measures vibration")
        try:
            self.link.set_param("health.startPropTest", "1")
            end = time.time() + max(2.0, float(wait_s))
            while time.time() < end:
                if self._abort.is_set():
                    return {"success": False, "error": "stopped by operator"}
                time.sleep(0.1)
        except Exception as exc:
            return {"success": False, "error": f"could not start the test: {exc}"}
        finally:
            for h in handles:
                try:
                    h.stop()
                except Exception:
                    pass
            self.motor_test_active = False
            self.phase = "IDLE"
        with lock:
            vals = dict(got)
        motors = []
        for i in (1, 2, 3, 4):
            vx, vy = vals.get(f"health.motorVarXM{i}"), vals.get(f"health.motorVarYM{i}")
            motors.append({"motor": i, "variance": None if vx is None or vy is None else vx + vy})
        have = [m["variance"] for m in motors if m["variance"] is not None]
        if not have:
            return {"success": False, "error": "the firmware returned no results (test did not run or log missing)",
                    "motors": motors}
        top = max(have)
        mask = int(vals.get("health.motorPass", 0))
        for m in motors:
            v = m["variance"]
            m["pass_flag"] = bool(mask & (1 << (m["motor"] - 1)))
            # "pass" only means low vibration; no vibration relative to the others means it did not turn
            m["vibration_seen"] = v is not None and top > 0 and v >= 0.2 * top
        quiet = [m["motor"] for m in motors if not m["vibration_seen"]]
        self.log("PROP TEST done: variances " + ", ".join(f"M{m['motor']}={m['variance']}" for m in motors)
                 + (f". Little or no vibration on M{quiet}: did it turn?" if quiet else ""))
        return {"success": not quiet, "error": None if not quiet else f"little or no vibration on motor(s) {quiet}",
                "motors": motors, "motorPass": mask, "battery_start": v0, "battery_end": self.battery_v}

    def calibrate_flow(self, true_distance: float) -> dict:
        """Scale the flow velocity so the displacement integrated since the sensor test
        started (or last calibration) equals the distance the drone was really moved."""
        if not self.sensor_test:
            return {"success": False, "error": "start the Sensor Test, slide the drone a measured distance, then calibrate"}
        try:
            true_distance = float(true_distance)
        except (TypeError, ValueError):
            return {"success": False, "error": "distance must be a number"}
        if not 0.1 <= true_distance <= 3.0:
            return {"success": False, "error": "use a distance between 0.1 and 3 m"}
        with self._lock:
            wx, wy = self.est.world
        measured = math.hypot(wx, wy)
        if measured < 0.05:
            return {"success": False, "error": f"only {measured:.3f} m measured: move the drone first (textured surface, 8-20 cm up)"}
        old = self.cfg.flow_gain
        res = self.cfg.apply({"flow_gain": old * true_distance / measured})
        new = self.cfg.flow_gain
        with self._lock:
            self.est.reset()                         # so the next run can verify it
            self.est.yaw_frame = False
            self.est.enabled = True
        self.rec.reset()
        self.log(f"Flow calibration: measured {measured:.3f} m for {true_distance:.3f} m moved -> gain {old:.3f} -> {new:.3f}")
        return {"success": True, "measured": measured, "true": true_distance, "old_gain": old, "gain": new,
                "clamped": bool(res["clamped"])}

    def stop_sensor_test(self) -> None:
        with self._lock:
            self.sensor_test = False
            self.est.enabled = False
            self.phase = "IDLE"
        if self.rec.csv_active:
            self.rec.csv_stop()
        self.log("Sensor test stopped.")

    def set_csv(self, enabled: bool) -> Optional[str]:
        if enabled:
            self.log(f"CSV logging to {self.rec.csv_start()}")
        else:
            self.rec.csv_stop()
            self.log("CSV logging stopped.")
        return self.rec.csv_path if enabled else None

    # --------------------------------------------------------------- NeoPixel
    def _led_send(self, channel: int, payload: bytes) -> Optional[str]:
        if not self.is_connected:
            return "drone not connected"
        last = None
        for _ in range(3):
            try:
                self.link.send_crtp(_NEOPIXEL_PORT, channel, payload)
                time.sleep(0.02)
                return None
            except Exception as exc:
                last = exc
                time.sleep(0.02)
        return f"LED command failed: {last}"

    def led_color(self, r: int, g: int, b: int) -> Optional[str]:
        r, g, b = (max(0, min(255, int(v))) for v in (r, g, b))
        err = self._led_send(0x00, bytes([0xFF, r, g, b])) or self._led_send(0x01, b"")
        if not err:
            self.led_state.update(r=r, g=g, b=b)
            self.log(f"LEDs set to ({r}, {g}, {b})")
        return err

    def led_blink(self, on: bool, on_ms: int = 500, off_ms: int = 500) -> Optional[str]:
        payload = bytes([1 if on else 0, (on_ms >> 8) & 255, on_ms & 255, (off_ms >> 8) & 255, off_ms & 255]) if on \
            else bytes(5)
        err = self._led_send(0x03, payload)
        if not err:
            self.led_state["blinking"] = bool(on)
            self.log("LEDs blinking" if on else "LED blinking stopped")
        return err

    def led_clear(self) -> Optional[str]:
        err = self._led_send(0x02, b"")
        if not err:
            self.led_state["blinking"] = False
            self.log("LEDs cleared")
        return err

    # ---------------------------------------------------------------- joystick
    @property
    def manual_active(self) -> bool:
        return self._manual

    def manual_start(self) -> Optional[str]:
        if self._manual:
            return "joystick session already running"
        if self.state.is_armed:
            return "already flying; land first"
        if not self.takeoff(self.cfg.target_height):
            return "take-off refused or failed (see Output Log)"
        self._manual = True
        self._keys = set()
        self._manual_thread = threading.Thread(target=self._manual_loop, name="litewing-joystick", daemon=True)
        self._manual_thread.start()
        return None

    def manual_stop(self) -> None:
        was = self._manual
        self._manual = False
        if self._manual_thread and self._manual_thread is not threading.current_thread():
            self._manual_thread.join(timeout=2.0)
        if was or self.state.is_armed:
            self.land()

    def manual_key(self, key: str, down: bool) -> None:
        key = (key or "").lower()
        if key not in ("w", "a", "s", "d"):
            return
        if down:
            self._keys.add(key)
        else:
            self._keys.discard(key)
            if not self._keys:
                x, y = self.est.world
                self.rec.mark_release(*to_display(x, y))
                if self.hold_origin:
                    self._target = (0.0, 0.0)
                else:      # predict where momentum will carry it, then hold there
                    self._target = (x + self.est.vx * 0.10, y + self.est.vy * 0.10)

    def _manual_loop(self) -> None:
        sens = lambda: self.cfg.joystick_sensitivity  # noqa: E731
        while self._manual and self.is_connected:
            tx, ty = self._target
            vf = (1 if "w" in self._keys else 0) - (1 if "s" in self._keys else 0)
            vl = (1 if "a" in self._keys else 0) - (1 if "d" in self._keys else 0)
            if vf or vl:
                wx, wy = self.est.world
                tx = max(-HARD_MAX_XY, min(HARD_MAX_XY, tx + vf * sens() * CONTROL_DT))
                ty = max(-HARD_MAX_XY, min(HARD_MAX_XY, ty + vl * sens() * CONTROL_DT))
                # never let the setpoint run far ahead of the vehicle (wind-up)
                if math.hypot(tx - wx, ty - wy) > 0.25:
                    scale = 0.25 / math.hypot(tx - wx, ty - wy)
                    tx, ty = wx + (tx - wx) * scale, wy + (ty - wy) * scale
                self._target = (tx, ty)
            if not self._tick(self._target, self._tz, limit=0.4, phase="JOYSTICK"):
                break
        if self._manual:                        # a safety trip ended it: come down
            self._manual = False
            self.land()

    # -------------------------------------------------------------- snapshot
    def dashboard_state(self) -> dict:
        with self._lock:
            now = time.time()
            stale = bool(self.sensor_ready and now - self.last_data_t > 1.0 and not self.virtual)
            px, py = to_display(*self.est.world)
            vx, vy = to_display(self.est.vx, self.est.vy)
            cx, cy = to_display(*self._corr)
            tx, ty = to_display(*self._target)
            return {
                "connected": self.is_connected, "uri": self.uri, "simulated": self.simulated,
                "debug": self.debug, "virtual": self.virtual, "phase": self.phase,
                "flight_mode": self.state.flight_mode.value, "armed": self.state.is_armed,
                "sensor_test": self.sensor_test, "tripped": self._tripped,
                "motor_test": self.motor_test_active, "motors": list(self.motors),
                "manual": {"active": self._manual, "hold_origin": self.hold_origin},
                "values": {
                    "flow_dx": self.delta[0], "flow_dy": self.delta[1], "squal": self.squal,
                    "height": self.height, "range": self.range_m, "vx": vx, "vy": vy, "pos_x": px, "pos_y": py,
                    "corr_vx": cx, "corr_vy": cy, "target_x": tx, "target_y": ty,
                    "battery_v": self.battery_v, "roll": self.roll, "pitch": self.pitch, "yaw": self.yaw % 360.0,
                    "stale": stale, "low_battery": 0 < self.battery_v <= self.WARN_V,
                },
                "config": self.cfg.to_dict(),
                "limits": {k: list(v) for k, v in LIMITS.items()},
                "ceiling": HARD_MAX_Z, "area": HARD_MAX_XY,
                "csv": {"active": self.rec.csv_active, "path": self.rec.csv_path},
                "leds": dict(self.led_state),
            }
