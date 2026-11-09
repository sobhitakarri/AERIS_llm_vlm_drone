"""
Radio links for the LiteWing: the real cflib CRTP-over-UDP link, and a bench
stand-in with simple physics (`udp://` vs `sim://`).

The interface talks to either through the same few calls, so the flight code,
the dashboard and the tests run unchanged without a drone.
"""
from __future__ import annotations

import math
import random
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple

from backend.core.logger import get_logger
from backend.drone.litewing_flight import DEG_TO_RAD, DT, FLOW_VELOCITY_CONSTANT

logger = get_logger("LiteWingLink")

try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    CFLIB_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on the machine
    CFLIB_AVAILABLE = False

LogCallback = Callable[[Dict[str, float]], None]


class _Handle:
    def __init__(self, start: Callable[[], None], stop: Callable[[], None]):
        self.start, self.stop = start, stop


class CflibLink:
    """cflib Crazyflie over UDP (the LiteWing's ESP32-S3 access point)."""

    OPEN_TIMEOUT_S = 15.0

    def __init__(self, uri: str):
        self.uri = uri
        self.cf = None
        self.scf = None

    def open(self) -> None:
        if not CFLIB_AVAILABLE:
            raise RuntimeError("cflib is not installed (pip install cflib)")
        cflib.crtp.init_drivers()
        from backend.drone import litewing_udp
        litewing_udp.register()                      # stock cflib 0.1.32 UDP has no checksum
        self.cf = Crazyflie(rw_cache="./cache")
        self.scf = SyncCrazyflie(litewing_udp.to_driver_uri(self.uri), cf=self.cf)
        # open_link() waits forever for the drone's reply (UDP never fails to
        # "connect"), so bound it: a silent drone must not hang the server.
        err: List[BaseException] = []

        def _open():
            try:
                self.scf.open_link()
            except BaseException as exc:             # noqa: BLE001
                err.append(exc)

        t = threading.Thread(target=_open, name="cflib-open", daemon=True)
        t.start()
        t.join(timeout=self.OPEN_TIMEOUT_S)
        if t.is_alive():
            try:
                self.cf.close_link()
            except Exception:
                pass
            raise RuntimeError(
                f"no reply from {self.uri} within {self.OPEN_TIMEOUT_S:.0f} s. Check: PC on the LiteWing Wi-Fi, "
                "drone powered, no other app (phone app, another script) connected to it, and Windows Firewall "
                "allowing Python on this network (set the LiteWing network to Private)."
            )
        if err:
            raise err[0]

    def close(self) -> None:
        if self.scf:
            self.scf.close_link()

    def is_connected(self) -> bool:
        return bool(self.cf) and bool(self.cf.is_connected())

    def send_hover(self, vx: float, vy: float, yawrate: float, z: float) -> None:
        self.cf.commander.send_hover_setpoint(vx, vy, yawrate, z)

    def send_setpoint(self, roll: float, pitch: float, yawrate: float, thrust: int) -> None:
        self.cf.commander.send_setpoint(roll, pitch, yawrate, thrust)

    def send_stop(self) -> None:
        self.cf.commander.send_stop_setpoint()

    def set_param(self, name: str, value: str) -> None:
        self.cf.param.set_value(name, value)

    def arm(self, flag: bool) -> None:
        self.cf.platform.send_arming_request(flag)

    def has_log(self, var: str) -> bool:
        group, name = var.split(".")
        toc = self.cf.log.toc.toc
        return group in toc and name in toc[group]

    def make_log(self, name: str, period_ms: int, variables: List[Tuple[str, str]], callback: LogCallback) -> _Handle:
        from cflib.crazyflie.log import LogConfig
        cfg = LogConfig(name=name, period_in_ms=period_ms)
        added = [(v, t) for v, t in variables if self.has_log(v)]
        for v, t in added:
            cfg.add_variable(v, t)
        if not added:
            raise RuntimeError(f"none of the {name} variables exist on this firmware")
        self.cf.log.add_config(cfg)
        cfg.data_received_cb.add_callback(lambda _ts, data, _lg: callback(dict(data)))
        return _Handle(cfg.start, cfg.stop)

    def send_crtp(self, port: int, channel: int, payload: bytes) -> None:
        from cflib.crtp.crtpstack import CRTPPacket
        pk = CRTPPacket()
        pk.set_header(port, channel)
        pk.data = bytes(payload)
        self.cf.send_packet(pk)


class FakeLink:
    """A LiteWing on the bench: first-order velocity and height response, an
    optical-flow sensor that quantises like the PMW3901, a ToF range, a battery
    that drains, and the firmware's setpoint time-out. Wall-clock, 100 Hz."""

    TAU_V = 0.25
    TAU_Z = 0.35
    SETPOINT_TIMEOUT = 0.5

    def __init__(self, uri: str = "sim://litewing", seed: int = 7, drift: Tuple[float, float] = (0.0, 0.0),
                 flow_noise: float = 0.3):
        self.uri = uri
        self.rng = random.Random(seed)
        self.drift = drift
        self.flow_noise = flow_noise
        self._lock = threading.Lock()
        self._connected = False
        self._thread: Optional[threading.Thread] = None
        self._logs: List[dict] = []
        self.x = self.y = self.z = 0.0
        self.yaw = 0.0
        self.vf = self.vl = 0.0           # body forward / left velocity
        self.roll = self.pitch = 0.0
        self.battery = 4.12
        self.params: Dict[str, str] = {}
        self.leds: Dict[str, object] = {"color": (0, 0, 0), "blink": False}
        self.armed = False
        self.sent: List[tuple] = []        # recent commands, for tests
        self.n_hover = 0                   # hover setpoints received in total
        self._cmd = (0.0, 0.0, 0.0, 0.0)
        self._cmd_t = 0.0
        self._motors_on = False
        self._rem = [0.0, 0.0]
        self.freeze_flow = False           # tests: simulate a stalled sensor stream
        self.held = False                  # tests / sensor test: vehicle carried by hand
        self.no_state_z = False            # firmware whose stateEstimate.z reads 0 on the bench
        self._thrust, self._thrust_t = 0, 0.0
        self._prop_started, self._prop_t = False, 0.0

    # --- link control
    def open(self) -> None:
        self._connected = True
        self._thread = threading.Thread(target=self._run, name="fake-litewing", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._connected = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def is_connected(self) -> bool:
        return self._connected

    # --- commands
    def send_hover(self, vx: float, vy: float, yawrate: float, z: float) -> None:
        with self._lock:
            self._cmd, self._cmd_t, self._motors_on = (vx, vy, yawrate, z), time.time(), True
            self.n_hover += 1
            self.sent.append(("hover", vx, vy, yawrate, z))
            del self.sent[:-500]

    def send_setpoint(self, roll: float, pitch: float, yawrate: float, thrust: int) -> None:
        with self._lock:
            self._thrust, self._thrust_t = int(thrust), time.time()
            if thrust == 0:
                self._motors_on = False
            self.sent.append(("setpoint", roll, pitch, yawrate, thrust))
            del self.sent[:-500]

    def send_stop(self) -> None:
        with self._lock:
            self._motors_on = False
            self.sent.append(("stop",))

    def set_param(self, name: str, value: str) -> None:
        self.params[name] = value
        if name == "health.startPropTest" and str(value) == "1":
            self._prop_started, self._prop_t = True, time.time()

    def arm(self, flag: bool) -> None:
        self.armed = bool(flag)

    def has_log(self, var: str) -> bool:
        return True

    def make_log(self, name: str, period_ms: int, variables: List[Tuple[str, str]], callback: LogCallback) -> _Handle:
        entry = {"vars": [v for v, _ in variables], "period": period_ms / 1000.0, "cb": callback, "next": 0.0, "on": False}
        self._logs.append(entry)

        def start():
            entry["on"] = True

        def stop():
            entry["on"] = False
        return _Handle(start, stop)

    def send_crtp(self, port: int, channel: int, payload: bytes) -> None:
        if port != 0x09:
            return
        if channel == 0x00 and len(payload) >= 4:
            self.leds["color"] = tuple(payload[1:4])
        elif channel == 0x02:
            self.leds["color"] = (0, 0, 0)
            self.leds["blink"] = False
        elif channel == 0x03 and payload:
            self.leds["blink"] = bool(payload[0])

    # --- physics
    def _run(self) -> None:
        last = time.time()
        while self._connected:
            now = time.time()
            dt = now - last
            last = now
            self._step(dt, now)
            for e in self._logs:
                if e["on"] and now >= e["next"] and not (self.freeze_flow and "motion.deltaX" in e["vars"]):
                    e["next"] = now + e["period"]
                    try:
                        e["cb"](self._sample(e["vars"]))
                    except Exception as exc:  # a bad callback must not kill the physics
                        logger.warning("fake log callback failed: %s", exc)
            time.sleep(DT)

    def _step(self, dt: float, now: float) -> None:
        if self.held:                              # carried by hand: the test sets z directly
            return
        with self._lock:
            vx, vy, yawrate, z_cmd = self._cmd
            live = self._motors_on and (now - self._cmd_t) < self.SETPOINT_TIMEOUT
            if not live:
                z_cmd, vx, vy, yawrate = 0.0, 0.0, 0.0, 0.0
            a = 1.0 - math.exp(-dt / self.TAU_V)
            self.vf += (vx - self.vf) * a
            self.vl += (vy - self.vl) * a
            self.z += (z_cmd - self.z) * (1.0 - math.exp(-dt / self.TAU_Z))
            if self.z < 0.004 and z_cmd == 0.0:
                self.z = 0.0
            if self.z < 0.03:                      # on the ground: no horizontal motion
                self.vf = self.vl = 0.0
            self.yaw += yawrate * dt
            c, s = math.cos(self.yaw * DEG_TO_RAD), math.sin(self.yaw * DEG_TO_RAD)
            self.x += (self.vf * c - self.vl * s + self.drift[0]) * dt * (1.0 if self.z > 0.03 else 0.0)
            self.y += (self.vf * s + self.vl * c + self.drift[1]) * dt * (1.0 if self.z > 0.03 else 0.0)
            self.pitch = max(-30.0, min(30.0, -self.vf * 10.0))
            self.roll = max(-30.0, min(30.0, self.vl * 10.0))
            drain = 0.0004 if (live and z_cmd > 0.0) else 0.00002
            self.battery = max(3.0, self.battery - drain * dt * 10.0)

    def _flow_counts(self, v: float, idx: int) -> int:
        """Counts for one 10 ms sample; the fractional part is carried over."""
        if self.z < 0.02:
            return 0
        exact = v / (self.z * FLOW_VELOCITY_CONSTANT) + self.rng.gauss(0.0, self.flow_noise)
        total = exact + self._rem[idx]
        counts = int(round(total))
        self._rem[idx] = total - counts
        return counts

    def _sample(self, names: List[str]) -> Dict[str, float]:
        out = {}
        for n in names:
            if n == "motion.deltaX":
                out[n] = self._flow_counts(self.vl, 0)
            elif n == "motion.deltaY":
                out[n] = self._flow_counts(self.vf, 1)
            elif n == "stateEstimate.z":
                out[n] = 0.0 if self.no_state_z else self.z
            elif n == "range.zrange":
                out[n] = int(max(0.0, self.z + self.rng.gauss(0.0, 0.002)) * 1000)
            elif n.startswith("health.motorVar"):
                out[n] = 0.03 + 0.002 * int(n[-1]) if self._prop_started and time.time() - self._prop_t > 1.0 else 0.0
            elif n == "health.motorPass":
                out[n] = 15 if self._prop_started and time.time() - self._prop_t > 1.0 else 0
            elif n.startswith("motor.m"):       # raw thrust is shared by all four, until the watchdog lapses
                out[n] = self._thrust if time.time() - self._thrust_t < self.SETPOINT_TIMEOUT else 0
            elif n == "motion.squal":
                out[n] = 120 if self.z > 0.02 else 0
            elif n == "pm.vbat":
                out[n] = self.battery
            elif n == "stateEstimate.roll":
                out[n] = self.roll
            elif n == "stateEstimate.pitch":
                out[n] = self.pitch
            elif n == "stateEstimate.yaw":
                out[n] = self.yaw
        return out
