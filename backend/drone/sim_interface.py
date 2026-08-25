"""
Desktop Simulator Mock Drone Interface.
"""
import time
import math
from backend.drone.base_interface import DroneInterface
from backend.schemas.telemetry import DroneState, FlightMode
from backend.core.logger import get_logger

logger = get_logger("SimInterface")


class SimInterface(DroneInterface):
    """
    Virtual drone interface implementing realistic kinematic motion simulation.
    Used for desktop debugging, software-in-the-loop (SIL) testing, and AI evaluation.
    """

    def __init__(self):
        self.state = DroneState()
        self.is_connected = False

    def connect(self) -> bool:
        self.is_connected = True
        logger.info("[SIM] Connected to virtual LiteWing simulator.")
        return True

    def disconnect(self) -> None:
        self.is_connected = False
        logger.info("[SIM] Disconnected from virtual simulator.")

    def takeoff(self, altitude: float = 1.0) -> bool:
        logger.info(f"[SIM] Taking off to {altitude}m...")
        self.state.flight_mode = FlightMode.TAKING_OFF
        self.state.is_armed = True

        # Simulate gradual climb
        steps = 10
        for i in range(steps):
            self.state.z = (i + 1) * (altitude / steps)
            time.sleep(0.05)

        self.state.z = altitude
        self.state.flight_mode = FlightMode.HOVERING
        logger.info(f"[SIM] Takeoff complete. Hovering at Z={self.state.z:.2f}m.")
        return True

    def land(self) -> bool:
        logger.info("[SIM] Initiating landing sequence...")
        self.state.flight_mode = FlightMode.LANDING

        steps = 10
        initial_z = self.state.z
        for i in range(steps):
            self.state.z = initial_z * (steps - i - 1) / steps
            time.sleep(0.05)

        self.state.z = 0.0
        self.state.is_armed = False
        self.state.flight_mode = FlightMode.IDLE
        logger.info("[SIM] Landed and disarmed.")
        return True

    def hover(self, duration: float = 2.0) -> bool:
        logger.info(f"[SIM] Hovering for {duration}s...")
        self.state.flight_mode = FlightMode.HOVERING
        time.sleep(min(duration, 1.0))  # Accelerated in simulation
        return True

    def move_to(self, x: float, y: float, z: float, velocity: float = 0.5) -> bool:
        logger.info(f"[SIM] Navigating to waypoint (X={x:.2f}, Y={y:.2f}, Z={z:.2f})...")
        self.state.flight_mode = FlightMode.NAVIGATING

        start_x, start_y, start_z = self.state.x, self.state.y, self.state.z
        dist = math.sqrt((x - start_x)**2 + (y - start_y)**2 + (z - start_z)**2)
        steps = max(5, int(dist * 10))

        for i in range(steps):
            alpha = (i + 1) / steps
            self.state.x = start_x + alpha * (x - start_x)
            self.state.y = start_y + alpha * (y - start_y)
            self.state.z = start_z + alpha * (z - start_z)
            time.sleep(0.04)

        self.state.flight_mode = FlightMode.HOVERING
        logger.info(f"[SIM] Reached waypoint (X={self.state.x:.2f}, Y={self.state.y:.2f}, Z={self.state.z:.2f}).")
        return True

    def rotate(self, angle_degrees: float) -> bool:
        logger.info(f"[SIM] Rotating by {angle_degrees}°...")
        self.state.yaw = (self.state.yaw + angle_degrees) % 360
        return True

    def emergency_stop(self) -> None:
        logger.warning("[SIM] EMERGENCY STOP TRIGGERED!")
        self.state.flight_mode = FlightMode.EMERGENCY
        self.state.is_armed = False
        self.state.z = 0.0

    def get_state(self) -> DroneState:
        return self.state

    def reset_home(self) -> bool:
        logger.info("[SIM] Reset to home (0, 0, 0).")
        self.state = DroneState()
        return True
