"""
LiteWing body-frame odometry and hover-velocity math.

Matches Circuit-Digest Python-Scripts:
https://github.com/Circuit-Digest/LiteWing/tree/main/Python-Scripts
especially dead-reckoning-position-hold.py

Firmware (ESP-Drone / LiteWing) does not publish GPS-quality stateEstimate.x/y.
XY comes from optical-flow deltas; Z from stateEstimate.z (ToF).
"""
from __future__ import annotations

from dataclasses import dataclass


# Official defaults from dead-reckoning-position-hold.py
OPTICAL_FLOW_SCALE = 3.7
SENSOR_DT = 0.01  # motion log period 10 ms
VELOCITY_SMOOTH_ALPHA = 0.8
VELOCITY_THRESHOLD = 0.005
DRIFT_COMPENSATION_RATE = 0.002
MAX_POSITION_ERROR = 2.0
POSITION_KP = 1.5
VELOCITY_KP = 1.2
TRIM_VX = 0.10
TRIM_VY = -0.02
MAX_HOLD_CORR = 0.10
MAX_NAV_CORR = 0.40
MIN_HOVER_Z = 0.20
MAX_HOVER_Z = 0.80


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def flow_to_velocity(delta: float, scale: float = OPTICAL_FLOW_SCALE, dt: float = SENSOR_DT) -> float:
    """Convert one optical-flow tick to linear velocity (m/s). No height scaling."""
    return float(delta) * scale * dt


def smooth_velocity(new_v: float, prev_v: float, alpha: float = VELOCITY_SMOOTH_ALPHA) -> float:
    sm = new_v * alpha + prev_v * (1.0 - alpha)
    if abs(sm) < VELOCITY_THRESHOLD:
        return 0.0
    return sm


def clamp_hover_z(z: float) -> float:
    return clamp(float(z), MIN_HOVER_Z, MAX_HOVER_Z)


@dataclass
class FlowOdometry:
    """Integrates flow velocities into workspace meters (home = takeoff origin)."""

    x: float = 0.0
    y: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    scale: float = OPTICAL_FLOW_SCALE
    _prev_vx: float = 0.0
    _prev_vy: float = 0.0
    enabled: bool = False

    def reset(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self._prev_vx = 0.0
        self._prev_vy = 0.0

    def ingest_flow(self, delta_x: float, delta_y: float, dt: float) -> None:
        raw_vx = flow_to_velocity(delta_x, self.scale, SENSOR_DT)
        raw_vy = flow_to_velocity(delta_y, self.scale, SENSOR_DT)
        self.vx = smooth_velocity(raw_vx, self._prev_vx)
        self.vy = smooth_velocity(raw_vy, self._prev_vy)
        self._prev_vx, self._prev_vy = self.vx, self.vy
        if not self.enabled or dt <= 0 or dt > 0.1:
            return
        self.x += self.vx * dt
        self.y += self.vy * dt
        speed = (self.vx * self.vx + self.vy * self.vy) ** 0.5
        if speed < VELOCITY_THRESHOLD * 2:
            self.x -= self.x * DRIFT_COMPENSATION_RATE * dt
            self.y -= self.y * DRIFT_COMPENSATION_RATE * dt
        self.x = clamp(self.x, -MAX_POSITION_ERROR, MAX_POSITION_ERROR)
        self.y = clamp(self.y, -MAX_POSITION_ERROR, MAX_POSITION_ERROR)


def hover_xy(
    target_x: float,
    target_y: float,
    pos_x: float,
    pos_y: float,
    vel_x: float,
    vel_y: float,
    *,
    trim_vx: float = TRIM_VX,
    trim_vy: float = TRIM_VY,
    kp: float = POSITION_KP,
    kv: float = VELOCITY_KP,
    max_corr: float = MAX_NAV_CORR,
) -> tuple[float, float]:
    """
    Position+velocity P loop → hover setpoint (vx, vy).

    Official hold script swaps axes when sending:
      send_hover(TRIM_VX + corr_y, TRIM_VY + corr_x, yawrate, z)
    """
    err_x = target_x - pos_x
    err_y = target_y - pos_y
    corr_x = clamp(kp * err_x - kv * vel_x, -max_corr, max_corr)
    corr_y = clamp(kp * err_y - kv * vel_y, -max_corr, max_corr)
    return trim_vx + corr_y, trim_vy + corr_x
