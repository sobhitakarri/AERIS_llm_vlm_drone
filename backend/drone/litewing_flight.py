"""
LiteWing flight core: dead reckoning, cascaded PID and the dashboard recorder.

The maths is the one in Circuit-Digest's dead-reckoning-maneuvers.py
(https://github.com/Circuit-Digest/LiteWing/tree/main/Python-Scripts/Flight_Positioning_Module):
optical-flow deltas scaled by the ToF height, two-point smoothing, integration
with drift compensation, then a position PID plus a velocity-damping PID, sent
as a hover setpoint.

Two deliberate differences, both for safety:
  * every tunable is clamped to LIMITS, and the target height can never exceed
    HARD_MAX_Z whatever the dashboard, the plan or the LLM asks for;
  * the world frame is the planner's (x forward, y left). The official script
    reports x = left, y = forward; `to_display` converts for the dashboard.
"""
from __future__ import annotations

import csv
import math
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import Dict, Optional, Tuple

HARD_MAX_Z = 0.30          # absolute ceiling, metres. Nothing may command above it.
HARD_MIN_Z = 0.10
HARD_MAX_XY = 1.0          # half-width of the flight area from the take-off point, metres
SENSOR_PERIOD_MS = 10
DT = SENSOR_PERIOD_MS / 1000.0
CONTROL_DT = 0.02          # 50 Hz, like the official script
DEG_TO_RAD = math.pi / 180.0
FLOW_VELOCITY_CONSTANT = (5.4 * DEG_TO_RAD) / (30.0 * DT)

# name -> (lo, hi). Values outside are clamped, never rejected, and reported.
LIMITS: Dict[str, Tuple[float, float]] = {
    "target_height": (HARD_MIN_Z, HARD_MAX_Z),
    "takeoff_time": (0.5, 3.0),
    "stabilize_time": (0.5, 6.0),
    "hover_duration": (1.0, 120.0),
    "landing_time": (0.2, 3.0),
    "descent_rate": (0.05, 0.30),
    "idle_hold_s": (1.0, 120.0),
    "smoothing_alpha": (0.1, 1.0),
    "max_correction": (0.05, 0.7),
    "velocity_threshold": (0.0, 0.05),
    "drift_comp_rate": (0.0, 0.05),
    "reset_interval": (5.0, 600.0),
    "max_position_error": (0.3, 2.0),
    "pos_kp": (0.0, 3.0), "pos_ki": (0.0, 0.5), "pos_kd": (0.0, 1.0),
    "vel_kp": (0.0, 3.0), "vel_ki": (0.0, 0.5), "vel_kd": (0.0, 1.0),
    "trim_vx": (-0.3, 0.3), "trim_vy": (-0.3, 0.3),
    "flow_scale": (0.5, 10.0),
    "flow_gain": (0.2, 5.0),
    "maneuver_distance": (0.1, HARD_MAX_XY),
    "maneuver_threshold": (0.03, 0.30),
    "waypoint_stabilization": (0.0, 5.0),
    "waypoint_timeout": (3.0, 60.0),
    "joystick_sensitivity": (0.1, 2.0),
}


@dataclass
class FlightConfig:
    """Runtime-tunable flight parameters. Defaults are the official script's."""
    target_height: float = 0.30
    takeoff_time: float = 1.0
    stabilize_time: float = 3.0
    hover_duration: float = 20.0
    landing_time: float = 0.5
    descent_rate: float = 0.15
    idle_hold_s: float = 30.0        # position-hold between commands, then land
    smoothing_alpha: float = 0.85
    max_correction: float = 0.7
    velocity_threshold: float = 0.005
    drift_comp_rate: float = 0.004
    reset_interval: float = 90.0
    max_position_error: float = 2.0
    pos_kp: float = 1.0
    pos_ki: float = 0.03
    pos_kd: float = 0.0
    vel_kp: float = 0.7
    vel_ki: float = 0.01
    vel_kd: float = 0.0
    trim_vx: float = 0.0
    trim_vy: float = 0.0
    flow_scale: float = 4.4
    flow_gain: float = 1.0           # measured-distance calibration on top of the official scaling
    use_height_scaling: bool = True
    takeoff_ramp: bool = True
    maneuver_distance: float = 0.5
    maneuver_threshold: float = 0.10
    waypoint_stabilization: float = 0.5
    waypoint_timeout: float = 30.0
    joystick_sensitivity: float = 0.2

    def apply(self, values: dict) -> dict:
        """Set known fields from `values`, clamped to LIMITS. Returns what happened."""
        names = {f.name: f for f in fields(self)}
        applied, clamped, unknown = {}, {}, []
        for key, raw in (values or {}).items():
            if key not in names:
                unknown.append(key)
                continue
            if isinstance(getattr(self, key), bool):
                value = raw if isinstance(raw, bool) else str(raw).strip().lower() in ("1", "true", "yes", "on")
                setattr(self, key, value)
                applied[key] = value
                continue
            try:
                value = float(raw)
            except (TypeError, ValueError):
                unknown.append(key)
                continue
            if not math.isfinite(value):
                unknown.append(key)
                continue
            lo, hi = LIMITS.get(key, (-math.inf, math.inf))
            used = max(lo, min(hi, value))
            if used != value:
                clamped[key] = {"requested": value, "used": used}
            setattr(self, key, used)
            applied[key] = used
        return {"applied": applied, "clamped": clamped, "unknown": unknown}

    def to_dict(self) -> dict:
        return asdict(self)


def clamp_z(z: float) -> float:
    """Height command limited to the hardware envelope."""
    return max(HARD_MIN_Z, min(HARD_MAX_Z, float(z)))


def to_display(x: float, y: float) -> Tuple[float, float]:
    """Planner frame (x forward, y left) -> official dashboard frame (X left, Y forward)."""
    return y, x


def body_to_world(fwd: float, left: float, yaw_rad: float) -> Tuple[float, float]:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return fwd * c - left * s, fwd * s + left * c


def world_to_body(wx: float, wy: float, yaw_rad: float) -> Tuple[float, float]:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return wx * c + wy * s, -wx * s + wy * c


class DeadReckoner:
    """Optical-flow velocity and integrated position (planner frame).

    `x`/`y` are relative to the last anchor; `ox`/`oy` carry earlier anchors so
    the world position stays continuous across a periodic reset.
    """

    def __init__(self, cfg: FlightConfig):
        self.cfg = cfg
        self.enabled = False
        # The official script integrates in the body frame. Rotating by yaw is only
        # right in flight (after a ROTATE); carried by hand it just bends the path.
        self.yaw_frame = False
        self.reset()

    def reset(self) -> None:
        self.x = self.y = 0.0
        self.ox = self.oy = 0.0
        self.vx = self.vy = 0.0
        self._hist_fwd = [0.0, 0.0]
        self._hist_left = [0.0, 0.0]
        self._last_t: Optional[float] = None
        self._last_reset = time.time()
        self.yaw0 = 0.0

    @property
    def world(self) -> Tuple[float, float]:
        return self.x + self.ox, self.y + self.oy

    def velocity_from_delta(self, delta: float, height: float) -> float:
        if height <= 0:
            return 0.0
        if self.cfg.use_height_scaling:
            return delta * height * FLOW_VELOCITY_CONSTANT * self.cfg.flow_gain
        return delta * self.cfg.flow_scale * DT * self.cfg.flow_gain

    def _smooth(self, new: float, hist: list) -> float:
        hist[1], hist[0] = hist[0], new
        a = self.cfg.smoothing_alpha
        out = hist[0] * a + hist[1] * (1.0 - a)
        return 0.0 if abs(out) < self.cfg.velocity_threshold else out

    def ingest(self, delta_x: float, delta_y: float, height: float, now: float, yaw_deg: float = 0.0) -> None:
        """One motion-log sample. Flow X is body-left, flow Y is body-forward."""
        left = self._smooth(self.velocity_from_delta(delta_x, height), self._hist_left)
        fwd = self._smooth(self.velocity_from_delta(delta_y, height), self._hist_fwd)
        psi = (yaw_deg - self.yaw0) * DEG_TO_RAD if self.yaw_frame else 0.0
        self.vx, self.vy = body_to_world(fwd, left, psi)
        last, self._last_t = self._last_t, now
        if last is None or not self.enabled:
            return
        dt = now - last
        if not (0.001 <= dt <= 0.1):
            return
        self.x += self.vx * dt
        self.y += self.vy * dt
        if math.hypot(self.vx, self.vy) < self.cfg.velocity_threshold * 2:
            self.x -= self.x * self.cfg.drift_comp_rate * dt
            self.y -= self.y * self.cfg.drift_comp_rate * dt
        lim = self.cfg.max_position_error
        self.x = max(-lim, min(lim, self.x))
        self.y = max(-lim, min(lim, self.y))

    def advance_virtual(self, vx: float, vy: float, dt: float) -> None:
        """Dry-run motion: integrate a commanded world velocity directly."""
        self.vx, self.vy = vx, vy
        self.x += vx * dt
        self.y += vy * dt

    def periodic_reset_due(self, now: float) -> bool:
        return now - self._last_reset >= self.cfg.reset_interval

    def re_anchor(self, now: float) -> None:
        """Official periodic reset, without losing the world position."""
        self.ox += self.x
        self.oy += self.y
        self.x = self.y = 0.0
        self._last_reset = now


class PositionController:
    """Position PID plus velocity-damping PID, summed and clamped (official maths)."""

    def __init__(self, cfg: FlightConfig):
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self.pi = [0.0, 0.0]
        self.vi = [0.0, 0.0]
        self.last_pe = [0.0, 0.0]
        self.last_ve = [0.0, 0.0]
        self.output = (0.0, 0.0)

    def step(self, target: Tuple[float, float], pos: Tuple[float, float], vel: Tuple[float, float],
             limit: Optional[float] = None) -> Tuple[float, float]:
        c = self.cfg
        out = [0.0, 0.0]
        for k in (0, 1):
            pe = target[k] - pos[k]
            ve = -vel[k]
            self.pi[k] = max(-0.1, min(0.1, self.pi[k] + pe * CONTROL_DT))
            self.vi[k] = max(-0.05, min(0.05, self.vi[k] + ve * CONTROL_DT))
            pd = (pe - self.last_pe[k]) / CONTROL_DT
            vd = (ve - self.last_ve[k]) / CONTROL_DT
            self.last_pe[k], self.last_ve[k] = pe, ve
            out[k] = (pe * c.pos_kp + self.pi[k] * c.pos_ki + pd * c.pos_kd
                      + ve * c.vel_kp + self.vi[k] * c.vel_ki + vd * c.vel_kd)
        if math.hypot(target[0] - pos[0], target[1] - pos[1]) < 0.1 and math.hypot(*vel) > 0.05:
            out = [out[0] * 0.8, out[1] * 0.8]
        cap = c.max_correction if limit is None else min(c.max_correction, max(0.05, limit))
        self.output = (max(-cap, min(cap, out[0])), max(-cap, min(cap, out[1])))
        return self.output


_SERIES = ("t", "vx", "vy", "px", "py", "cx", "cy", "h", "r", "tx", "ty")


class FlightRecorder:
    """Thread-safe history for the dashboard plots, the output log and CSV export.

    Positions and velocities are stored in the official display frame
    (X left, Y forward), so the dashboard matches the Circuit-Digest GUI.
    """
    HISTORY = 6000
    LOG_LINES = 800

    def __init__(self, csv_dir: Optional[Path] = None):
        self._lock = threading.Lock()
        self.csv_dir = Path(csv_dir) if csv_dir else Path(__file__).resolve().parents[2] / "reports" / "litewing_logs"
        self._csv_file = None
        self._csv = None
        self.csv_path: Optional[str] = None
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self.epoch = getattr(self, "epoch", 0) + 1     # tells clients to clear their plots
            self._seq = 0
            self._rows: deque = deque(maxlen=self.HISTORY)
            self.release_points: list = []
            self.t0 = time.time()

    def sample(self, **row) -> None:
        with self._lock:
            self._seq += 1
            row["t"] = round(time.time() - self.t0, 3)
            self._rows.append((self._seq, tuple(float(row.get(k, 0.0)) for k in _SERIES)))
            if self._csv:
                try:
                    self._csv.writerow([f"{row['t']:.3f}"] + [f"{row.get(k, 0.0):.4f}" for k in _SERIES[1:]]
                                       + [row.get("phase", ""), f"{row.get('battery', 0.0):.2f}"])
                except Exception:
                    pass

    def since(self, seq: int, epoch: int = -1) -> dict:
        with self._lock:
            if epoch != self.epoch:
                seq = 0                     # client is on an older flight: send it all
            rows = [r for r in self._rows if r[0] > seq]
            out = {k: [r[1][i] for r in rows] for i, k in enumerate(_SERIES)}
            out["seq"] = rows[-1][0] if rows else seq
            out["epoch"] = self.epoch
            out["release"] = list(self.release_points)
            return out

    def mark_release(self, x: float, y: float) -> None:
        with self._lock:
            self.release_points.append((round(x, 3), round(y, 3)))
            del self.release_points[:-50]

    def csv_start(self) -> str:
        self.csv_stop()
        self.csv_dir.mkdir(parents=True, exist_ok=True)
        path = self.csv_dir / time.strftime("litewing_%Y%m%d_%H%M%S.csv")
        f = open(path, "w", newline="")
        w = csv.writer(f)
        w.writerow(["time_s", "vx", "vy", "pos_x_left", "pos_y_fwd", "corr_vx", "corr_vy", "height",
                    "range", "target_x_left", "target_y_fwd", "phase", "battery_v"])
        with self._lock:
            self._csv_file, self._csv, self.csv_path = f, w, str(path)
        return str(path)

    def csv_stop(self) -> None:
        with self._lock:
            f, self._csv_file, self._csv = self._csv_file, None, None
        if f:
            try:
                f.close()
            except Exception:
                pass

    @property
    def csv_active(self) -> bool:
        return self._csv is not None


class OutputLog:
    """The 'Output Log' window: numbered lines, read incrementally."""

    def __init__(self, maxlen: int = FlightRecorder.LOG_LINES):
        self._lock = threading.Lock()
        self._lines: deque = deque(maxlen=maxlen)
        self._seq = 0

    def add(self, message: str) -> None:
        with self._lock:
            self._seq += 1
            self._lines.append((self._seq, time.strftime("%H:%M:%S"), str(message)))

    def clear(self) -> None:
        with self._lock:
            self._lines.clear()

    def since(self, seq: int) -> dict:
        with self._lock:
            lines = [{"n": n, "t": t, "msg": m} for n, t, m in self._lines if n > seq]
            return {"lines": lines, "seq": self._seq}
