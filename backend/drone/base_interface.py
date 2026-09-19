"""
Abstract Drone Interface — Sim-to-Real Contract.
"""
from abc import ABC, abstractmethod
from typing import Optional
import numpy as np

from backend.schemas.telemetry import DroneState


class DroneInterface(ABC):
    """
    Abstract hardware interface for controlling the UAV.
    Decouples AI high-level mission planning from specific physical flight software.
    """

    @abstractmethod
    def connect(self) -> bool:
        """Establish connection to the drone or simulator."""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Safely disconnect from the drone or simulator."""
        pass

    @abstractmethod
    def takeoff(self, altitude: float = 1.0) -> bool:
        """Command drone to take off to target altitude."""
        pass

    @abstractmethod
    def land(self) -> bool:
        """Command drone to land safely."""
        pass

    @abstractmethod
    def hover(self, duration: float = 2.0) -> bool:
        """Hover at current location for duration (seconds)."""
        pass

    @abstractmethod
    def move_to(self, x: float, y: float, z: float, velocity: float = 0.5) -> bool:
        """Move drone to target physical position setpoint."""
        pass

    @abstractmethod
    def rotate(self, angle_degrees: float) -> bool:
        """Rotate drone heading by angle_degrees."""
        pass

    @abstractmethod
    def emergency_stop(self) -> None:
        """Immediate motor stop / failsafe trigger."""
        pass

    @abstractmethod
    def get_state(self) -> DroneState:
        """Get current 6-DOF telemetry state."""
        pass

    def get_frame(self) -> Optional[np.ndarray]:
        """Capture a camera frame. Returns None if no camera is available."""
        return None

    def clear_path(self) -> bool:
        """Clear visualization trails. No-op on backends without a path display."""
        return True

    def reset_home(self) -> bool:
        """Snap/return to origin (0, 0, 0) and idle."""
        return self.land()

