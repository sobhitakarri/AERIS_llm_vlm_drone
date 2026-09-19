"""
PySimverse 3D Simulator Interface.
Bridges high-level skill primitives and camera perception with the PySimverse Unity simulation.
"""
import math
import time
from typing import Optional, Tuple
import numpy as np

from backend.drone.base_interface import DroneInterface
from backend.schemas.telemetry import DroneState, FlightMode
from backend.core.logger import get_logger

logger = get_logger("PySimverseInterface")


class PySimverseInterface(DroneInterface):
    """
    Hardware-abstracted interface for PySimverse 3D Simulator.
    Communicates via ZeroMQ:
    - Port 5550: RC / flight commands
    - Port 5556: Telemetry & state
    - Port 5557: Observer / FPV video stream
    """

    def __init__(self, host: str = "localhost"):
        self.host = host
        self.drone = None
        self.state = DroneState()
        self.is_connected = False
        self._last_frame = None
        self._stream_active = False

    def connect(self) -> bool:
        logger.info("[PySimverse] Initializing connection to 3D simulator...")
        try:
            from pysimverse import Drone
            self.drone = Drone()
            self.drone.connect()
            self.is_connected = True
            logger.info("[PySimverse] Successfully connected to simulator ZeroMQ sockets.")

            # Automatically start video stream
            try:
                self.drone.streamon()
                self._stream_active = True
                logger.info("[PySimverse] Camera video streaming enabled.")
            except Exception as e:
                logger.warning(f"[PySimverse] Video stream init warning: {e}")

            return True
        except Exception as e:
            logger.error(f"[PySimverse] Failed to connect to simulator: {e}")
            self.is_connected = False
            return False

    def disconnect(self) -> None:
        if self.drone:
            try:
                if self.state.is_armed:
                    self.land()
                self.drone.shutdown()
            except Exception as e:
                logger.warning(f"[PySimverse] Error during shutdown: {e}")
        self.is_connected = False
        logger.info("[PySimverse] Disconnected from simulator.")

    def takeoff(self, altitude: float = 1.0) -> bool:
        if not self.is_connected or not self.drone:
            logger.error("[PySimverse] Cannot takeoff: not connected.")
            return False

        logger.info(f"[PySimverse] Commanding takeoff to ~{altitude:.2f}m...")
        try:
            self.state.flight_mode = FlightMode.TAKING_OFF
            self.state.is_armed = True

            # In PySimverse physics engine, 100 units = 0.5m in the HUD.
            # Scale by 200.0 so 1.0m commands 200 units -> exact 1.0m altitude.
            takeoff_units = max(50, int(altitude * 200))
            self.drone.take_off(takeoff_height=takeoff_units)

            self.state.z = float(altitude)
            self.state.flight_mode = FlightMode.HOVERING
            logger.info(f"[PySimverse] Takeoff complete. Hovering at Z={self.state.z:.2f}m.")
            return True
        except Exception as e:
            logger.error(f"[PySimverse] Takeoff failed: {e}")
            return False

    def land(self) -> bool:
        if not self.is_connected or not self.drone:
            return False

        logger.info("[PySimverse] Initiating landing sequence...")
        try:
            self.state.flight_mode = FlightMode.LANDING
            self.drone.land()
            self.state.z = 0.0
            self.state.vx = 0.0
            self.state.vy = 0.0
            self.state.vz = 0.0
            self.state.is_armed = False
            self.state.flight_mode = FlightMode.IDLE
            logger.info("[PySimverse] Landing complete.")
            return True
        except Exception as e:
            logger.error(f"[PySimverse] Landing error: {e}")
            return False

    def hover(self, duration: float = 2.0) -> bool:
        if not self.is_connected or not self.drone:
            return False

        logger.info(f"[PySimverse] Hovering for {duration:.1f}s...")
        self.state.flight_mode = FlightMode.HOVERING
        time.sleep(duration)
        return True

    def move_to(self, x: float, y: float, z: float, velocity: float = 0.5) -> bool:
        """
        Moves drone to target world coordinate (x, y, z) in meters relative to origin.
        Decomposes delta (dx, dy, dz) into simulator movement primitives.
        """
        if not self.is_connected or not self.drone:
            return False

        logger.info(f"[PySimverse] Moving from ({self.state.x:.2f}, {self.state.y:.2f}, {self.state.z:.2f}) to target ({x:.2f}, {y:.2f}, {z:.2f})...")
        self.state.flight_mode = FlightMode.NAVIGATING

        dx = x - self.state.x
        dy = y - self.state.y
        dz = z - self.state.z

        # Move Z first if altitude delta is significant
        if abs(dz) > 0.05:
            dz_units = abs(dz) * 200.0
            if dz > 0:
                self.drone.move_up(dz_units)
            else:
                self.drone.move_down(dz_units)
            self.state.z = z

        # Move X (Left/Right)
        if abs(dx) > 0.05:
            dx_cm = abs(dx) * 100.0
            if dx > 0:
                self.drone.move_right(dx_cm)
            else:
                self.drone.move_left(dx_cm)
            self.state.x = x

        # Move Y (Forward/Backward)
        if abs(dy) > 0.05:
            dy_cm = abs(dy) * 100.0
            if dy > 0:
                self.drone.move_forward(dy_cm)
            else:
                self.drone.move_backward(dy_cm)
            self.state.y = y

        self.state.flight_mode = FlightMode.HOVERING
        logger.info(f"[PySimverse] Arrived at ({self.state.x:.2f}, {self.state.y:.2f}, {self.state.z:.2f})m.")
        return True

    def rotate(self, angle_degrees: float) -> bool:
        if not self.is_connected or not self.drone:
            return False

        logger.info(f"[PySimverse] Rotating by {angle_degrees:+.1f}°...")
        try:
            self.drone.rotate(angle_degrees)
            self.state.yaw = (self.state.yaw + angle_degrees) % 360.0
            return True
        except Exception as e:
            logger.error(f"[PySimverse] Rotation error: {e}")
            return False

    def emergency_stop(self) -> None:
        logger.warning("[PySimverse] EMERGENCY STOP TRIGGERED!")
        if self.drone:
            try:
                self.drone.send_rc_control(0, 0, 0, 0)
                self.drone.land()
            except Exception as e:
                logger.error(f"[PySimverse] Emergency stop error: {e}")
        self.state.flight_mode = FlightMode.EMERGENCY
        self.state.is_armed = False

    def get_state(self) -> DroneState:
        return self.state

    def get_frame(self) -> Optional[np.ndarray]:
        """Captures real-time OpenCV camera frame from PySimverse."""
        if not self.is_connected or not self.drone:
            return None

        try:
            res = self.drone.get_frame()
            if isinstance(res, tuple) and len(res) >= 2:
                frame, ok = res[0], res[1]
                if ok and frame is not None:
                    self._last_frame = frame
                    return frame
            elif isinstance(res, np.ndarray):
                self._last_frame = res
                return res
        except Exception as e:
            logger.debug(f"[PySimverse] Frame capture error: {e}")

        return self._last_frame

    def rotate_camera(self, angle: float) -> bool:
        """Tilts the camera angle in degrees (e.g. -45° to -90° for downward view)."""
        if not self.is_connected or not self.drone:
            return False
        try:
            self.drone.rotate_camera(angle)
            logger.info(f"[PySimverse] Camera tilted to {angle}°.")
            return True
        except Exception as e:
            logger.error(f"[PySimverse] Camera tilt error: {e}")
            return False
