"""
Scene camera service for backends that provide an overhead RGB-D camera.

Two background jobs:
  * Calibration. Detects the surveyed floor markers in the overhead image and
    hands the camera pose to SpatialGrounding. Repeats until it succeeds and
    again whenever the backend reconnects.
  * External position fix. Finds the airframe in the depth image near where
    the estimator believes it is and sends that position back, the way a
    motion-capture or overhead-camera fix corrects onboard dead reckoning.
  * Heading fix. The camera cannot see which way a 10 cm airframe points,
    but it sees where it went. Dead reckoning rotates every displacement by
    the heading error, so the angle between the displacement the estimator
    reports and the one the camera saw, over a leg between two fixes, is the
    heading error.
"""
from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

from backend.core.logger import get_logger
from backend.vision.calibration import calibrate_overhead
from backend.vision.spatial_grounding import SpatialGrounding

logger = get_logger("SceneService")

_CAL_FILE = Path(__file__).resolve().parents[2] / "cache" / "homography.json"
_FIX_MAX_SPEED = 0.15        # only correct while near-stationary: frames lag the vehicle
_FIX_MIN_POINTS = 10
_FIX_DZ = 0.06               # height band around the estimated altitude
_FIX_GATE = 0.12             # largest correction accepted right after a fix
_FIX_GATE_PER_M = 0.08       # dead reckoning drifts with distance: widen the gate by this per metre flown
_FIX_GATE_MAX = 0.30
_YAW_MIN_LEG = 0.8           # shortest leg (m) a heading error is measured over
_YAW_DEADBAND = 1.0          # deg: below this the measurement is noise
_YAW_MAX_STEP = 8.0          # deg: largest heading correction from one leg
_YAW_APPLY_S = 3.0           # seconds the corrected heading is fed back for


class SceneService:
    def __init__(self, drone: Any, grounding: SpatialGrounding, external_fix: bool = True):
        self.drone = drone
        self.grounding = grounding
        self.external_fix = external_fix
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._was_connected = False
        self.calibrated_at: Optional[float] = None
        self.last_fix: Optional[dict] = None
        self.fix_count = 0
        self.fix_rejected = 0
        self.yaw_fix_count = 0
        self.last_yaw_fix_deg: Optional[float] = None
        self._anchor: Optional[tuple] = None        # (est_xy, cam_xy) at the last converged fix
        self._last_fix_xy: Optional[tuple] = None
        self._yaw_target: Optional[float] = None
        self._yaw_until = 0.0

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="scene-service")
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def status(self) -> dict:
        cal = self.grounding.calibration
        return {
            "calibrated": cal is not None,
            "depth": bool(self.grounding.has_depth),
            "markers": list(cal.marker_ids) if cal else [],
            "reproj_rms_px": round(cal.reproj_rms_px, 3) if cal else None,
            "floor_rms_cm": round(cal.floor_rms_m * 100, 2) if cal else None,
            "camera_position": [round(float(v), 3) for v in cal.camera_position] if cal else None,
            "external_fix": self.last_fix,
            "external_fix_count": self.fix_count,
            "external_fix_rejected": self.fix_rejected,
            "heading_fix_count": self.yaw_fix_count,
            "heading_fix_deg": self.last_yaw_fix_deg,
        }

    # ------------------------------------------------------------------ loop
    def _loop(self) -> None:
        last_cal_try = 0.0
        while self._running:
            try:
                connected = bool(getattr(self.drone, "is_connected", False))
                if connected and not self._was_connected:
                    # New simulation run or reconnect: the camera may have moved.
                    self.grounding.clear_calibration()
                    self.calibrated_at = None
                self._was_connected = connected
                if connected and not self.grounding.calibrated and time.time() - last_cal_try > 1.5:
                    last_cal_try = time.time()
                    self._calibrate()
                if connected and self.external_fix and self.grounding.calibrated:
                    self._external_fix()
            except Exception as exc:
                logger.warning("Scene service error: %s", exc)
            time.sleep(0.2)

    def _calibrate(self) -> None:
        frame = self.drone.get_frame()
        if frame is None:
            return
        cal = calibrate_overhead(frame, self.drone.markers(), self.drone.camera_info("overhead"))
        if cal is None:
            return
        room = (getattr(self.drone, "runtime", {}) or {}).get("room", {}).get("size_m")
        bounds = None
        if room:
            bounds = {"x": [-room[0] / 2 + 0.06, room[0] / 2 - 0.06], "y": [-room[1] / 2 + 0.06, room[1] / 2 - 0.06]}
        self.grounding.set_calibration(cal, bounds)
        if hasattr(self.drone, "get_depth_frame"):
            self.grounding.depth_source = self.drone.get_depth_frame
        self.grounding.drone_pose_source = self._drone_pose
        self.calibrated_at = time.time()
        try:
            self.grounding.save(_CAL_FILE)
        except OSError:
            pass

    def _drone_pose(self):
        st = self.drone.get_state()
        return st.x, st.y, st.z

    def _external_fix(self) -> None:
        st = self.drone.get_state()
        if not st.is_armed or st.z < 0.2:
            self._anchor = self._last_fix_xy = self._yaw_target = None     # landed: the next flight starts fresh
            return
        if math.hypot(st.vx, st.vy) > _FIX_MAX_SPEED:
            return
        depth = self.drone.get_depth_frame() if hasattr(self.drone, "get_depth_frame") else None
        cal = self.grounding.calibration
        if depth is None or cal is None:
            return
        # The further the vehicle has flown on dead reckoning, the further
        # from the estimate it may really be.
        flown = math.hypot(st.x - self._last_fix_xy[0], st.y - self._last_fix_xy[1]) if self._last_fix_xy else 0.0
        gate = min(_FIX_GATE + _FIX_GATE_PER_M * flown, _FIX_GATE_MAX)
        radius = gate + 0.08
        u, v = cal.world_to_pixel(st.x, st.y, st.z)
        h, w = depth.shape[:2]
        win = int(radius * cal.K[0, 0] / max(0.3, float(cal.camera_position[2]) - st.z)) + 6
        u0, u1 = int(max(0, u - win)), int(min(w, u + win))
        v0, v1 = int(max(0, v - win)), int(min(h, v + win))
        if u1 - u0 < 8 or v1 - v0 < 8:
            return
        uu, vv = np.meshgrid(np.arange(u0, u1, dtype=np.float32), np.arange(v0, v1, dtype=np.float32))
        d = depth[v0:v1, u0:u1]
        rays = np.stack([(uu - cal.K[0, 2]) / cal.K[0, 0], (vv - cal.K[1, 2]) / cal.K[1, 1], np.ones_like(uu)], axis=-1)
        pts = cal.camera_position + (rays @ cal.R) * d[..., None]
        near = (
            (np.abs(pts[..., 2] - st.z) < _FIX_DZ)
            & ((pts[..., 0] - st.x) ** 2 + (pts[..., 1] - st.y) ** 2 < radius ** 2)
            & (d > 0.05)
        )
        n = int(near.sum())
        if n < _FIX_MIN_POINTS:
            return
        xs, ys = pts[..., 0][near], pts[..., 1][near]
        fx, fy = float(xs.mean()), float(ys.mean())
        # Accept only something the size of the airframe, where the airframe
        # is expected. Anything else at that height (a bottle top, a shelf
        # edge) would drag the estimate onto itself.
        span = (getattr(self.drone, "runtime", {}) or {}).get("airframe", {}).get("arm_m", 0.05) * 2 + 0.10
        if max(float(np.ptp(xs)), float(np.ptp(ys))) > span or math.hypot(fx - st.x, fy - st.y) > gate:
            self.fix_rejected += 1
            return
        yaw = self._heading_fix(st, fx, fy)
        if self.drone.send_external_pose(fx, fy, yaw):
            self.fix_count += 1
            self._last_fix_xy = (fx, fy)
            self.last_fix = {"x": round(fx, 3), "y": round(fy, 3), "dx": round(fx - st.x, 3), "dy": round(fy - st.y, 3)}

    def _heading_fix(self, st, fx: float, fy: float) -> Optional[float]:
        """Heading to feed back with this fix, or None for a position-only fix."""
        now = time.time()
        if self._anchor is not None:
            (ex, ey), (cx, cy) = self._anchor
            d_est, d_cam = (st.x - ex, st.y - ey), (fx - cx, fy - cy)
            if math.hypot(*d_cam) >= _YAW_MIN_LEG and math.hypot(*d_est) >= _YAW_MIN_LEG:
                err = math.degrees(math.atan2(d_est[1], d_est[0]) - math.atan2(d_cam[1], d_cam[0]))
                err = (err + 180.0) % 360.0 - 180.0
                self._anchor = None
                if _YAW_DEADBAND < abs(err) <= 3 * _YAW_MAX_STEP:
                    err = max(-_YAW_MAX_STEP, min(_YAW_MAX_STEP, err))
                    self._yaw_target = st.yaw - err
                    self._yaw_until = now + _YAW_APPLY_S
                    self.yaw_fix_count += 1
                    self.last_yaw_fix_deg = round(err, 2)
                    logger.info("[Scene] Heading error %.1f deg measured over a %.2f m leg; correcting.", err, math.hypot(*d_cam))
        # A new leg is measured from a fix where estimate and camera agree.
        if math.hypot(fx - st.x, fy - st.y) < 0.02 and (self._anchor is None or math.hypot(fx - self._anchor[1][0], fy - self._anchor[1][1]) < 0.1):
            self._anchor = ((st.x, st.y), (fx, fy))
        if self._yaw_target is not None and now < self._yaw_until:
            return self._yaw_target
        self._yaw_target = None
        return None
