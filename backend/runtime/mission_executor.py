"""
Mission Executor Runtime.
"""
import math
from typing import Callable, Optional
from backend.drone.base_interface import DroneInterface
from backend.schemas.mission import MissionPlan
from backend.runtime.uav_monitor import UavMonitor
from backend.core.logger import get_logger

logger = get_logger("MissionExecutor")


class MissionExecutor:
    """
    Executes a validated mission plan sequentially by calling DroneInterface hardware primitives.
    """

    def __init__(self, drone: DroneInterface, monitor: Optional[UavMonitor] = None):
        self.drone = drone
        self.monitor = monitor or UavMonitor()

    def execute_plan(self, plan: MissionPlan, on_step_complete: Optional[Callable[[str], None]] = None) -> bool:
        logger.info(f"Executing mission plan with {len(plan.skills)} steps...")

        self.monitor.arm_mission(len(plan.skills))
        for i, item in enumerate(plan.skills):
            if self.monitor.aborted():
                logger.warning("UAV monitor abort — emergency stop.")
                self.drone.emergency_stop()
                self.monitor.finish(False)
                return False
            skill = item.skill.upper()
            params = item.params
            self.monitor.note_step(i + 1, skill)
            logger.info(f"Executing step {i+1}/{len(plan.skills)}: {skill} with params={params}")

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
                logger.info(f"Visual perception tracking active for target: '{target}'")
                if all(k in params for k in ("x", "y", "z")):
                    success = self.drone.move_to(params["x"], params["y"], params["z"])
                else:
                    success = self.drone.hover(duration=1.0)
            elif skill == "RETURN":
                success = self.drone.move_to(0.0, 0.0, 1.0)

            if not success:
                logger.error(f"Execution failed at step {i+1}: {skill}")
                self.drone.emergency_stop()
                self.monitor.finish(False)
                return False

            if on_step_complete:
                on_step_complete(skill)

        logger.info("Mission plan execution completed successfully.")
        self.monitor.finish(True)
        return True
