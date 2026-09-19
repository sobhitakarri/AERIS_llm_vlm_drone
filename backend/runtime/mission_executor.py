"""
Mission Executor Runtime — Perception-Action Loop.
"""
import math
import threading
from typing import Any, Callable, List, Optional
from backend.drone.base_interface import DroneInterface
from backend.schemas.mission import MissionPlan
from backend.schemas.perception import DetectedObject
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
from backend.vision.spatial_grounding import SpatialGrounding
from backend.core.logger import get_logger

logger = get_logger("MissionExecutor")


class MissionExecutor:
    """
    Executes a validated mission plan sequentially by calling DroneInterface hardware primitives.
    Implements a closed-loop Perception-Action loop for FIND and INSPECT skills using VLM visual grounding.
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
    ):
        self.drone = drone
        self.monitor = monitor or UavMonitor()
        self.vlm = vlm
        self.grounding = grounding or SpatialGrounding()
        self.state_manager = state_manager
        self.camera_manager = camera_manager
        self.event_bus = event_bus or default_event_bus
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
        try:
            return self._run(plan, on_step_complete)
        finally:
            with self._lock:
                self._busy = False

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
            params = item.params
            self.monitor.note_step(i + 1, skill)
            logger.info(f"Executing step {i+1}/{len(queue)}: {skill} with params={params}")

            if self.event_bus:
                self.event_bus.publish(EVENT_STEP_STARTED, {"step": i + 1, "skill": skill, "params": params})

            success = False
            if skill == "TAKEOFF":
                z = params.get("z", 1.0)
                success = self.drone.takeoff(altitude=z)
            elif skill == "LAND":
                success = self.drone.land()
            elif skill == "HOVER":
                t = params.get("t", 2.0)
                success = self.drone.hover(duration=t)
            elif skill == "MOVE_TO":
                x = params.get("x", 0.0)
                y = params.get("y", 0.0)
                z = params.get("z", 1.0)
                success = self.drone.move_to(x, y, z)
            elif skill == "ROTATE":
                angle = params.get("angle", 90.0)
                success = self.drone.rotate(angle)
            elif skill == "CIRCLE":
                radius = float(params.get("radius", 0.5))
                state = self.drone.get_state()
                cx, cy, cz = state.x, state.y, max(state.z, 1.0)
                logger.info(f"Flying orbital trajectory around ({cx:.2f}, {cy:.2f}) radius={radius}m...")
                success = True
                n_points = 8
                for k in range(1, n_points + 1):
                    ang = 2.0 * math.pi * k / n_points
                    ok = self.drone.move_to(
                        cx + radius * math.cos(ang),
                        cy + radius * math.sin(ang),
                        cz,
                    )
                    if not ok:
                        success = False
                        break
            elif skill in ["FIND", "INSPECT"]:
                target = params.get("target", "object")
                logger.info(f"Perception-Action Loop: Visual tracking active for target '{target}'")
                if self.state_manager and not self._target_actionable(target):
                    # TARGET_LOST: do not fly to the last known world pose.
                    logger.warning(
                        "TARGET_LOST for '%s' — holding position; stale coordinates "
                        "cannot generate a movement command.",
                        target,
                    )
                    success = self.drone.hover(duration=2.0)
                else:
                    grounded_pos = self._resolve_visual_target(target)
                    if grounded_pos:
                        gx, gy, gz = grounded_pos
                        logger.info(
                            f"Perception-Action Loop: Moving drone to detected target '{target}' "
                            f"at ({gx:.2f}, {gy:.2f}, {gz:.2f})m"
                        )
                        success = self.drone.move_to(gx, gy, gz)
                        if success:
                            hover_dur = float(params.get("t", 2.0))
                            self.drone.hover(duration=hover_dur)
                    elif all(k in params for k in ("x", "y", "z")):
                        success = self.drone.move_to(params["x"], params["y"], params["z"])
                    else:
                        logger.warning(f"Target '{target}' could not be localized in current view; hovering for observation.")
                        success = self.drone.hover(duration=2.0)

            elif skill == "RETURN":
                success = self.drone.move_to(0.0, 0.0, 1.0)
            else:
                logger.error(f"Unhandled skill '{skill}' — not in the executor dispatch table.")
                success = False

            if not success:
                logger.error(f"Execution failed at step {i+1}: {skill}")
                self._safe_abort_flight()
                self.monitor.finish(False)
                if self.event_bus:
                    self.event_bus.publish(EVENT_MISSION_FINISHED, {"success": False, "step": i + 1, "skill": skill})
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

    def _resolve_visual_target(self, target_label: str) -> Optional[tuple]:
        """
        Closed-loop visual grounding:
        Captures camera frame -> calls VLM -> computes 3D coordinates via homography -> updates StateManager.
        """
        live = self._lookup_state_target(target_label)
        if live:
            return live

        # 1. Capture current camera frame (drone camera or external camera manager)
        frame = None
        if hasattr(self.drone, "get_frame"):
            frame = self.drone.get_frame()
        if frame is None and self.camera_manager:
            frame = self.camera_manager.read_frame()

        # 2. VLM visual grounding
        if frame is not None and self.vlm and hasattr(self.vlm, "resolve_target"):
            try:
                import cv2
                _, enc = cv2.imencode(".jpg", frame)
                img_h, img_w = frame.shape[:2]
                det = self.vlm.resolve_target(
                    enc.tobytes(),
                    target_label,
                    image_width=img_w,
                    image_height=img_h,
                )
                if det and det.bbox:
                    u, v = det.bbox.center_pixel
                    state = self.drone.get_state()
                    z_alt = max(state.z, 1.0)
                    wx, wy, wz = self.grounding.pixel_to_world(
                        u, v, z_altitude=z_alt, img_w=img_w, img_h=img_h
                    )
                    det.world_x = wx
                    det.world_y = wy
                    det.world_z = wz

                    # Cache detection in global synchronized state store
                    if self.state_manager:
                        self.state_manager.update_object(det)

                    # Notify event subscribers
                    if self.event_bus:
                        self.event_bus.publish(EVENT_OBJECT_DETECTED, det.model_dump())

                    logger.info(
                        f"[Perception-Action Loop] VLM detected '{target_label}' at pixel ({u}, {v}) "
                        f"-> World ({wx:.2f}, {wy:.2f}, {wz:.2f})m conf={det.confidence}"
                    )
                    return wx, wy, wz
            except Exception as e:
                logger.error(f"[Perception-Action Loop] VLM grounding error: {e}")

        return self._lookup_state_target(target_label)

    def _target_actionable(self, target_label: str) -> bool:
        """True only when a live detection currently authorises motion.

        A missing label is treated as unknown-but-searchable (FIND at the start
        of a mission). An explicit TARGET_LOST mark is not."""
        if not self.state_manager:
            return True
        needle = (target_label or "").lower()
        for obj in self.state_manager.get_all_objects():
            label = (obj.label or "").lower()
            if needle in label or label in needle or needle.replace(" ", "_") in label:
                return bool(obj.actionable and obj.world_x is not None)
        return True

    def _lookup_state_target(self, target_label: str) -> Optional[tuple]:
        if not self.state_manager:
            return None
        needle = (target_label or "").lower()
        for obj in self.state_manager.get_actionable_objects():
            label = (obj.label or "").lower()
            if obj.world_x is None:
                continue
            if needle in label or label in needle or needle.replace(" ", "_") in label:
                logger.info(
                    f"[Perception-Action Loop] Found live state for '{target_label}': "
                    f"({obj.world_x}, {obj.world_y})"
                )
                return obj.world_x, obj.world_y, obj.world_z or 1.0
        return None
