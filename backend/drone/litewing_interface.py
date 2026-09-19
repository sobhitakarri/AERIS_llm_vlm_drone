"""
Real LiteWing (ESP-Drone / ESP32-S3) interface via cflib CRTP over UDP.

Hardware / firmware: https://github.com/Circuit-Digest/ESP-Drone
Official motion math: https://github.com/Circuit-Digest/LiteWing/tree/main/Python-Scripts

Python sends hover setpoints only (vx, vy, yawrate, z). PWM stays on the ESP32.
"""
from __future__ import annotations

import time

from backend.core.config import settings
from backend.core.logger import get_logger
from backend.drone.base_interface import DroneInterface
from backend.drone.litewing_odometry import (
    MAX_HOLD_CORR,
    MAX_HOVER_Z,
    MAX_NAV_CORR,
    MIN_HOVER_Z,
    TRIM_VX,
    TRIM_VY,
    FlowOdometry,
    clamp_hover_z,
    hover_xy,
)
from backend.schemas.telemetry import DroneState, FlightMode

logger = get_logger("LiteWingInterface")

try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    CFLIB_AVAILABLE = True
except ImportError:
    CFLIB_AVAILABLE = False

# Official hello / hold scripts use this URI (no :1988).
_DEFAULT_URI = "udp://192.168.43.42"
_CONTROL_DT = 0.02  # 50 Hz like dead-reckoning-position-hold.py
_MIN_BATTERY_V = 3.40


class LiteWingInterface(DroneInterface):
    def __init__(self, uri: str = None):
        raw = uri or settings.drone_uri or _DEFAULT_URI
        # Strip a mistaken port; Circuit-Digest uses udp://192.168.43.42
        if raw.startswith("udp://") and raw.count(":") > 1 and raw.rsplit(":", 1)[-1].isdigit():
            host = raw.rsplit(":", 1)[0]
            if host.endswith("192.168.43.42") or "192.168.43.42" in host:
                raw = "udp://192.168.43.42"
        self.uri = raw
        self.state = DroneState()
        self.scf = None
        self.cf = None
        self.is_connected = False
        self.odom = FlowOdometry()
        self._pose_log = None
        self._flow_log = None
        self._last_flow_t = time.time()
        self._hl_ok = False

    def connect(self) -> bool:
        if not CFLIB_AVAILABLE:
            logger.error("cflib is not installed. pip install cflib")
            return False
        logger.info("[REAL] Connecting to LiteWing at %s ...", self.uri)
        try:
            cflib.crtp.init_drivers()
            self.cf = Crazyflie(rw_cache="./cache")
            self.scf = SyncCrazyflie(self.uri, cf=self.cf)
            self.scf.open_link()
            time.sleep(1.0)
            # Official unlock: zero attitude/thrust setpoint
            self.cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
            try:
                self.cf.param.set_value("commander.enHighLevel", "1")
                self._hl_ok = True
                logger.info("[REAL] High-level commander enabled.")
            except Exception as exc:
                logger.warning("[REAL] enHighLevel failed (%s). Hover setpoints may be ignored.", exc)
            time.sleep(0.3)
            self.is_connected = True
            self._start_logs()
            logger.info("[REAL] Connected to LiteWing ESP32-S3.")
            return True
        except Exception as e:
            logger.error("[REAL] Connection failed to %s: %s", self.uri, e)
            self.is_connected = False
            return False

    def _start_logs(self) -> None:
        try:
            from cflib.crazyflie.log import LogConfig

            att = LogConfig(name="litewing_att", period_in_ms=50)
            att.add_variable("stateEstimate.z", "float")
            att.add_variable("stateEstimate.roll", "float")
            att.add_variable("stateEstimate.pitch", "float")
            att.add_variable("stateEstimate.yaw", "float")
            att.add_variable("pm.vbat", "float")
            self.cf.log.add_config(att)

            def _on_att(_ts, data, _lg):
                self.state.z = float(data.get("stateEstimate.z", self.state.z))
                self.state.roll = float(data.get("stateEstimate.roll", self.state.roll))
                self.state.pitch = float(data.get("stateEstimate.pitch", self.state.pitch))
                self.state.yaw = float(data.get("stateEstimate.yaw", self.state.yaw))
                vbat = data.get("pm.vbat")
                if vbat is not None:
                    self.state.battery_v = float(vbat)
                self.state.x = self.odom.x
                self.state.y = self.odom.y
                self.state.vx = self.odom.vx
                self.state.vy = self.odom.vy

            att.data_received_cb.add_callback(_on_att)
            att.start()
            self._pose_log = att
            logger.info("[REAL] Logging stateEstimate.z / attitude / pm.vbat.")
        except Exception as e:
            logger.warning("[REAL] Attitude/battery log unavailable: %s", e)

        try:
            from cflib.crazyflie.log import LogConfig

            flow = LogConfig(name="litewing_flow", period_in_ms=10)
            flow.add_variable("motion.deltaX", "int16_t")
            flow.add_variable("motion.deltaY", "int16_t")
            self.cf.log.add_config(flow)

            def _on_flow(_ts, data, _lg):
                now = time.time()
                dt = now - self._last_flow_t
                self._last_flow_t = now
                self.odom.ingest_flow(
                    float(data.get("motion.deltaX", 0)),
                    float(data.get("motion.deltaY", 0)),
                    dt,
                )

            flow.data_received_cb.add_callback(_on_flow)
            flow.start()
            self._flow_log = flow
            logger.info("[REAL] Logging motion.deltaX/Y for dead-reckoning.")
        except Exception as e:
            logger.warning("[REAL] Optical-flow log unavailable: %s", e)

    def disconnect(self) -> None:
        if self.scf and self.is_connected:
            try:
                self.cf.commander.send_setpoint(0, 0, 0, 0)
            except Exception:
                pass
            # Stop log configs first: callbacks firing after close_link raise.
            for cfg in (self._pose_log, self._flow_log):
                try:
                    if cfg:
                        cfg.stop()
                except Exception:
                    pass
            self._pose_log = None
            self._flow_log = None
            self.odom.enabled = False
            self.scf.close_link()
            self.is_connected = False
            logger.info("[REAL] Disconnected from LiteWing.")

    def _battery_ok(self) -> bool:
        v = self.state.battery_v
        if v and v < _MIN_BATTERY_V:
            logger.error("[REAL] Battery %.2f V < %.2f V — refusing flight.", v, _MIN_BATTERY_V)
            return False
        return True

    def _send_hover(self, vx: float, vy: float, yawrate: float, z: float) -> None:
        self.cf.commander.send_hover_setpoint(vx, vy, yawrate, clamp_hover_z(z))

    def _hold_loop(self, target_x: float, target_y: float, z: float, duration: float, max_corr: float) -> bool:
        z = clamp_hover_z(z)
        end = time.time() + duration
        while time.time() < end:
            vx, vy = hover_xy(
                target_x, target_y, self.odom.x, self.odom.y, self.odom.vx, self.odom.vy,
                max_corr=max_corr,
            )
            self._send_hover(vx, vy, 0.0, z)
            self.state.flight_mode = FlightMode.HOVERING
            time.sleep(_CONTROL_DT)
        return True

    def takeoff(self, altitude: float = 0.4) -> bool:
        if not self.is_connected:
            logger.warning("[REAL] Drone not connected.")
            return False
        if not self._battery_ok():
            return False
        z = clamp_hover_z(altitude if altitude >= MIN_HOVER_Z else 0.4)
        logger.info("[REAL] Takeoff to %.2f m (LiteWing safe hover range %.2f–%.2f).", z, MIN_HOVER_Z, MAX_HOVER_Z)
        try:
            try:
                self.cf.platform.send_arming_request(True)
                time.sleep(1.0)
            except Exception as exc:
                logger.warning("[REAL] Arm request skipped: %s", exc)

            self.odom.enabled = False
            self.state.flight_mode = FlightMode.TAKING_OFF
            self.state.is_armed = True
            steps = 5
            for i in range(steps):
                h = (i + 1) * z / steps
                self._send_hover(TRIM_VX, TRIM_VY, 0.0, h)
                time.sleep(0.12)

            t0 = time.time()
            while time.time() - t0 < 1.2:
                self._send_hover(TRIM_VX, TRIM_VY, 0.0, z)
                time.sleep(_CONTROL_DT)

            self.odom.reset()
            self.odom.enabled = True
            self.state.z = z
            self.state.flight_mode = FlightMode.HOVERING
            return True
        except Exception as e:
            logger.error("[REAL] Takeoff error: %s", e)
            return False

    def land(self) -> bool:
        if not self.is_connected:
            return False
        logger.info("[REAL] Landing...")
        try:
            self.odom.enabled = False
            self.state.flight_mode = FlightMode.LANDING
            z0 = max(self.state.z, MIN_HOVER_Z)
            steps = 5
            for i in range(steps):
                h = z0 * (steps - i - 1) / steps
                self._send_hover(TRIM_VX, TRIM_VY, 0.0, max(h, 0.0))
                time.sleep(0.12)
            self.cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.1)
            try:
                self.cf.commander.send_stop_setpoint()
                self.cf.platform.send_arming_request(False)
            except Exception:
                pass
            self.state.z = 0.0
            self.state.is_armed = False
            self.state.flight_mode = FlightMode.IDLE
            return True
        except Exception as e:
            logger.error("[REAL] Landing error: %s", e)
            return False

    def hover(self, duration: float = 2.0) -> bool:
        if not self.is_connected:
            return False
        z = clamp_hover_z(self.state.z if self.state.z >= MIN_HOVER_Z else 0.4)
        logger.info("[REAL] Position-hold hover %.1fs at z=%.2f", duration, z)
        return self._hold_loop(self.odom.x, self.odom.y, z, duration, MAX_HOLD_CORR)

    def move_to(self, x: float, y: float, z: float, velocity: float = 0.4) -> bool:
        if not self.is_connected:
            return False
        z = clamp_hover_z(z)
        vmax = min(max(velocity, 0.05), MAX_NAV_CORR)
        logger.info("[REAL] Dead-reckon MOVE_TO (%.2f, %.2f, %.2f) vmax=%.2f", x, y, z, vmax)
        self.state.flight_mode = FlightMode.NAVIGATING
        self.odom.enabled = True
        t0 = time.time()
        settle = 0.0
        while time.time() - t0 < 20.0:
            vx, vy = hover_xy(
                x, y, self.odom.x, self.odom.y, self.odom.vx, self.odom.vy, max_corr=vmax,
            )
            self._send_hover(vx, vy, 0.0, z)
            err = ((self.odom.x - x) ** 2 + (self.odom.y - y) ** 2 + (self.state.z - z) ** 2) ** 0.5
            if err <= 0.12:
                settle += _CONTROL_DT
                if settle >= 0.35:
                    self.state.flight_mode = FlightMode.HOVERING
                    return True
            else:
                settle = 0.0
            time.sleep(_CONTROL_DT)
        logger.warning("[REAL] MOVE_TO timeout (odom %.2f, %.2f).", self.odom.x, self.odom.y)
        return False

    def rotate(self, angle_degrees: float) -> bool:
        if not self.is_connected:
            return False
        z = clamp_hover_z(self.state.z if self.state.z >= MIN_HOVER_Z else 0.4)
        # Hold a fixed safe rate and fly it for as long as the angle needs,
        # instead of reading the requested angle as a rate.
        rate = 30.0 if angle_degrees >= 0 else -30.0
        duration = min(abs(angle_degrees) / 30.0, 12.0)
        logger.info("[REAL] Yaw %.1f deg at %.0f deg/s (~%.1f s)", angle_degrees, rate, duration)
        t0 = time.time()
        while time.time() - t0 < duration:
            self._send_hover(TRIM_VX, TRIM_VY, rate, z)
            time.sleep(_CONTROL_DT)
        self.state.yaw = (self.state.yaw + angle_degrees) % 360.0
        return True

    def emergency_stop(self) -> None:
        logger.warning("[REAL] EMERGENCY STOP")
        if self.is_connected and self.cf:
            try:
                self.cf.commander.send_setpoint(0, 0, 0, 0)
                self.cf.commander.send_stop_setpoint()
                self.cf.platform.send_arming_request(False)
            except Exception as e:
                logger.error("[REAL] E-stop error: %s", e)
        self.odom.enabled = False
        self.state.flight_mode = FlightMode.EMERGENCY
        self.state.is_armed = False

    def get_state(self) -> DroneState:
        self.state.x = self.odom.x
        self.state.y = self.odom.y
        self.state.vx = self.odom.vx
        self.state.vy = self.odom.vy
        return self.state
