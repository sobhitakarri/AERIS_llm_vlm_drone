"""
Mission Executor Runtime — Perception-Action Loop.
"""
import math
import threading
import time
from typing import Any, Callable, List, Optional
from backend.drone.base_interface import DroneInterface
from backend.schemas.mission import MissionPlan
from backend.runtime.uav_monitor import UavMonitor
from backend.runtime.state_manager import StateManager
from backend.runtime.event_bus import (
    event_bus as default_event_bus,
    EVENT_OBJECT_DETECTED,
    EVENT_STEP_STARTED,
    EVENT_STEP_COMPLETED,
    EVENT_MISSION_FINISHED,
    EVENT_ABORT,
)
from backend.planning.obstacle_planner import plan_path
from backend.vision.spatial_grounding import SpatialGrounding
from backend.core.logger import get_logger

logger = get_logger("MissionExecutor")

_FIND_TIMEOUT_S = 25.0       # how long FIND waits for the perception stack to lock on
_DEFAULT_SPEED = 0.4


class MissionExecutor:
    """
    Executes a validated mission plan sequentially by calling DroneInterface skills.
    FIND and INSPECT are resolved against the live perception state, and every
    translation is routed around what the depth camera sees.
    """

    def __init__(
        self,
        drone: DroneInterface,
        monitor: Optional[UavMonitor] = None,
        vlm: Optional[Any] = None,
        grounding: Optional[SpatialGrounding] = None,
        state_manager: Optional[StateManager] = None,
        camera_manager: Optional[Any] = None,
        event_bus: Optional[Any] = None,
        perception: Optional[Any] = None,
        capabilities: Optional[Any] = None,
    ):
        self.drone = drone
        self.monitor = monitor or UavMonitor()
        self.vlm = vlm
        self.grounding = grounding or SpatialGrounding()
        self.state_manager = state_manager
        self.camera_manager = camera_manager
        self.event_bus = event_bus or default_event_bus
        self.perception = perception
        # True when a background loop is driving `perception`; otherwise FIND
        # runs one detection itself.
        self.perception_live = perception is not None
        self.capabilities = capabilities
        self.last_error: Optional[str] = None
        self.last_route: List[tuple] = []
        self._lock = threading.Lock()
        self._pending_replan: Optional[List] = None
        self._busy = False

    def inject_replan(self, skills: List) -> None:
        with self._lock:
            self._pending_replan = list(skills)
        logger.info("Queued mid-mission replan with %s extra skills.", len(skills))

    def clear_replan(self) -> None:
        """Drop any replan queued by a previous mission."""
        with self._lock:
            self._pending_replan = None

    def is_busy(self) -> bool:
        with self._lock:
            return self._busy

    def _safe_abort_flight(self) -> None:
        """Bring the drone down on a step failure. A cut-motor e-stop would drop
        it from altitude, so try a controlled land first and only kill power if
        that fails."""
        try:
            if self.drone.get_state().z > 0.05 and self.drone.land():
                return
        except Exception as exc:
            logger.warning("Controlled land after failure did not work: %s", exc)
        self.drone.emergency_stop()

    def execute_plan(self, plan: MissionPlan, on_step_complete: Optional[Callable[[str], None]] = None) -> bool:
        # One mission at a time: two threads driving the same airframe would
        # interleave setpoints and clobber the monitor's abort flag.
        with self._lock:
            if self._busy:
                logger.error("Rejected plan: a mission is already executing.")
                return False
            self._busy = True
            self._pending_replan = None
        self.last_error = None
        try:
            return self._run(plan, on_step_complete)
        finally:
            with self._lock:
                self._busy = False

    def return_home(self) -> bool:
        """Fly back to the home pad along a planned route and land. Runs as
        its own short mission so it cannot overlap another one."""
        with self._lock:
            if self._busy:
                return False
            self._busy = True
        self.last_error = None
        try:
            st = self.drone.get_state()
            if not st.is_armed:
                return True
            self.monitor.arm_mission(2)
            self.monitor.note_step(1, "RETURN")
            hx, hy = self.drone.home_position()
            ok = self._fly_to(hx, hy, max(st.z, self._z_limits()[0] + 0.3))
            self.monitor.note_step(2, "LAND")
            ok = ok and self.drone.land()
            if not ok:
                self._safe_abort_flight()
            self.monitor.finish(ok)
            return ok
        finally:
            with self._lock:
                self._busy = False

    # ------------------------------------------------------------------ motion
    def _z_limits(self) -> tuple:
        ws = getattr(self.capabilities, "workspace", None)
        if ws is not None:
            return float(ws.z_min), float(ws.z_max)
        return 0.2, 1.5

    def _fly_to(self, x: float, y: float, z: float, speed: float = _DEFAULT_SPEED) -> bool:
        """MOVE_TO with obstacle-aware routing when a depth map is available."""
        st = self.drone.get_state()
        start, goal = (st.x, st.y, st.z), (float(x), float(y), float(z))
        grid = None
        if getattr(self.grounding, "has_depth", False):
            try:
                grid = self.grounding.occupancy_grid()
            except Exception as exc:
                logger.warning("Obstacle map unavailable (%s); flying direct.", exc)
        waypoints, how = plan_path(start, goal, grid, self._z_limits())
        if waypoints is None:
            self.last_error = f"No safe route to ({x:.2f}, {y:.2f}, {z:.2f}): {how}."
            logger.error(self.last_error)
            return False
        if len(waypoints) > 1 or how != "direct":
            logger.info("Route to (%.2f, %.2f, %.2f): %s, %d leg(s).", x, y, z, how, len(waypoints))
        self.last_route = [start] + list(waypoints)
        for wp in waypoints:
            if self.monitor.aborted():
                return False
            if not self.drone.move_to(wp[0], wp[1], wp[2], speed):
                self.last_error = f"Could not reach waypoint ({wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f})."
                return False
        return True

    def _land(self) -> bool:
        """Land where the vehicle is if the floor below is clear, otherwise
        go back to the home pad first."""
        st = self.drone.get_state()
        if getattr(self.grounding, "has_depth", False) and st.z > 0.15:
            below = self.grounding.obstacle_height(st.x, st.y, radius=0.18)
            if below > 0.05:
                hx, hy = self.drone.home_position()
                logger.info("Landing zone not clear (%.2f m surface below); returning to the home pad.", below)
                if not self._fly_to(hx, hy, st.z):
                    return False
        return self.drone.land()

    def _rotate(self, params: dict) -> bool:
        if "yaw" in params:
            # Absolute heading: turn by the shortest difference from where we are.
            delta = (float(params["yaw"]) - self.drone.get_state().yaw + 180.0) % 360.0 - 180.0
            return True if abs(delta) < 2.0 else self.drone.rotate(delta)
        return self.drone.rotate(float(params.get("angle", 90.0)))

    def _circle(self, params: dict) -> bool:
        radius = float(params.get("radius", 0.5))
        st = self.drone.get_state()
        if all(k in params for k in ("x", "y")):
            cx, cy = float(params["x"]), float(params["y"])
            start = math.atan2(st.y - cy, st.x - cx)
        else:
            # No centre given: orbit about a point one radius ahead of the nose.
            heading = math.radians(st.yaw)
            cx, cy = st.x + radius * math.cos(heading), st.y + radius * math.sin(heading)
            start = heading + math.pi
        cz = st.z
        logger.info(f"Orbit around ({cx:.2f}, {cy:.2f}) radius={radius} m at {cz:.2f} m")
        n_points = 12
        for k in range(1, n_points + 1):
            ang = start + 2.0 * math.pi * k / n_points
            if not self._fly_to(cx + radius * math.cos(ang), cy + radius * math.sin(ang), cz):
                return False
        return True

    # -------------------------------------------------------------- perception
    def _find(self, params: dict) -> bool:
        target = params.get("target", "object")
        hover_t = float(params.get("t", 2.0))
        if all(k in params for k in ("x", "y", "z")):
            return self._fly_to(params["x"], params["y"], params["z"]) and self.drone.hover(hover_t)

        logger.info(f"Perception-Action Loop: locating '{target}'")
        pos = self._resolve_visual_target(target)
        if pos is None:
            # Do not fly anywhere on a target that was not seen. Hold, then
            # report the step as failed so the mission ends under control.
            self.last_error = f"Target '{target}' was not found by the perception stack."
            logger.warning(self.last_error)
            self.drone.hover(1.0)
            return False
        gx, gy, gz = pos
        z_min, z_max = self._z_limits()
        gz = max(z_min, min(z_max, gz))
        logger.info(f"Perception-Action Loop: flying to '{target}' at ({gx:.2f}, {gy:.2f}, {gz:.2f}) m")
        return self._fly_to(gx, gy, gz) and self.drone.hover(hover_t)

    def _resolve_visual_target(self, target_label: str) -> Optional[tuple]:
        """Position of a target from the live perception state.

        The background loop does the detecting and tracking; this waits for it
        to report an actionable lock. A label explicitly marked lost is not
        waited for and never falls back to its last known pose.
        """
        if self.state_manager is not None and not self._searchable(target_label):
            return None
        if not self.perception_live:
            return self._detect_once(target_label)
        if hasattr(self.perception, "set_targets"):
            if target_label not in getattr(self.perception, "active_targets", []):
                self.perception.set_targets(list(getattr(self.perception, "active_targets", [])) + [target_label])
        clock = self.drone.sim_time if hasattr(self.drone, "sim_time") else time.time
        deadline = clock() + _FIND_TIMEOUT_S
        wall_deadline = time.time() + 6 * _FIND_TIMEOUT_S
        while clock() < deadline and time.time() < wall_deadline:
            if self.monitor.aborted():
                return None
            live = self._lookup_state_target(target_label)
            if live:
                return live
            if self.state_manager is None or self.perception is None:
                break
            time.sleep(0.2)
        return self._lookup_state_target(target_label)

    def _searchable(self, target_label: str) -> bool:
        """False for a label the perception stack has explicitly marked lost:
        its last pose must not be flown to, and it is not re-searched here."""
        needle = (target_label or "").lower()
        for obj in self.state_manager.get_all_objects():
            label = (obj.label or "").lower()
            if needle in label or label in needle:
                return bool(obj.actionable and obj.world_x is not None)
        return True

    def _detect_once(self, target_label: str) -> Optional[tuple]:
        """Single detect-and-ground for use without a background perception loop."""
        live = self._lookup_state_target(target_label)
        if live:
            return live
        if self.vlm is None:
            return None
        if self.perception is None:
            from backend.vision.fast_cv import FastPerception
            self.perception = FastPerception(grounding=self.grounding, vlm=self.vlm, state_manager=self.state_manager)
        frame = self.drone.get_frame() if hasattr(self.drone, "get_frame") else None
        if frame is None and self.camera_manager:
            frame = self.camera_manager.read_frame()
        det = self.perception.locate(frame, target_label, z_altitude=max(self.drone.get_state().z, 0.4))
        if det is None or det.world_x is None:
            return None
        if self.event_bus:
            self.event_bus.publish(EVENT_OBJECT_DETECTED, det.model_dump())
        return det.world_x, det.world_y, det.world_z or self.drone.get_state().z

    def _lookup_state_target(self, target_label: str) -> Optional[tuple]:
        if not self.state_manager:
            return None
        needle = (target_label or "").lower()
        for obj in self.state_manager.get_actionable_objects():
            label = (obj.label or "").lower()
            if obj.world_x is None:
                continue
            if needle in label or label in needle or needle.replace(" ", "_") in label:
                return obj.world_x, obj.world_y, obj.world_z or self.drone.get_state().z
        return None

    # --------------------------------------------------------------- main loop
    def _run(self, plan: MissionPlan, on_step_complete: Optional[Callable[[str], None]] = None) -> bool:
        logger.info(f"Executing mission plan with {len(plan.skills)} steps...")

        queue = list(plan.skills)
        self.monitor.arm_mission(len(queue))
        i = 0
        while i < len(queue):
            if self.monitor.aborted():
                logger.warning("UAV monitor abort — emergency stop.")
                self.drone.emergency_stop()
                self.monitor.finish(False)
                if self.event_bus:
                    self.event_bus.publish(EVENT_ABORT, {"step": i + 1})
                return False

            item = queue[i]
            skill = item.skill.upper()
            params = item.params or {}
            self.monitor.note_step(i + 1, skill)
            logger.info(f"Executing step {i+1}/{len(queue)}: {skill} with params={params}")

            if self.event_bus:
                self.event_bus.publish(EVENT_STEP_STARTED, {"step": i + 1, "skill": skill, "params": params})

            speed = float(params.get("speed", _DEFAULT_SPEED))
            success = False
            if skill == "TAKEOFF":
                success = self.drone.takeoff(altitude=params.get("z", 1.0))
            elif skill == "LAND":
                success = self._land()
            elif skill == "HOVER":
                success = self.drone.hover(duration=params.get("t", 2.0))
            elif skill == "MOVE_TO":
                st = self.drone.get_state()
                success = self._fly_to(params.get("x", st.x), params.get("y", st.y), params.get("z", st.z), speed)
            elif skill == "ROTATE":
                success = self._rotate(params)
            elif skill == "CIRCLE":
                success = self._circle(params)
            elif skill in ("FIND", "INSPECT"):
                success = self._find(params)
            elif skill == "RETURN":
                hx, hy = self.drone.home_position()
                success = self._fly_to(hx, hy, max(self.drone.get_state().z, self._z_limits()[0] + 0.3), speed)
            else:
                logger.error(f"Unhandled skill '{skill}' — not in the executor dispatch table.")

            if not success:
                if not self.last_error:
                    self.last_error = f"{skill} failed at step {i + 1}."
                logger.error(f"Execution failed at step {i+1}: {skill} ({self.last_error})")
                self._safe_abort_flight()
                self.monitor.finish(False)
                if self.event_bus:
                    self.event_bus.publish(EVENT_MISSION_FINISHED, {
                        "success": False, "step": i + 1, "skill": skill, "error": self.last_error,
                    })
                return False

            if self.event_bus:
                self.event_bus.publish(EVENT_STEP_COMPLETED, {"step": i + 1, "skill": skill})

            if on_step_complete:
                on_step_complete(skill)

            with self._lock:
                extra = self._pending_replan
                self._pending_replan = None
            if extra and skill == "LAND":
                logger.info("Discarding replan queued during LAND; mission is ending.")
            elif extra:
                # Insert after the current step; never drop the remaining plan
                # (a dropped tail would strip the final LAND).
                queue = queue[: i + 1] + list(extra) + queue[i + 1:]
                self.monitor.total_steps = len(queue)
                logger.info("Applied mid-mission replan; queue now %s steps.", len(queue))
            i += 1

        logger.info("Mission plan execution completed successfully.")
        self.monitor.finish(True)
        if self.event_bus:
            self.event_bus.publish(EVENT_MISSION_FINISHED, {"success": True})
        return True
