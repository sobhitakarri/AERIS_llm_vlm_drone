"""
Real LiteWing ESP32-S3 Hardware Interface (cflib CRTP over UDP).
"""
import time
from backend.drone.base_interface import DroneInterface
from backend.schemas.telemetry import DroneState, FlightMode
from backend.core.config import settings
from backend.core.logger import get_logger

logger = get_logger("LiteWingInterface")

try:
    import cflib.crtp
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    CFLIB_AVAILABLE = True
except ImportError:
    CFLIB_AVAILABLE = False


class LiteWingInterface(DroneInterface):
    """
    Real physical LiteWing drone interface using cflib over UDP (`udp://192.168.43.42`).
    Communicates with the ESP-IDF Crazyflie-compatible firmware running on the ESP32-S3.
    """

    def __init__(self, uri: str = None):
        self.uri = uri or settings.drone_uri
        self.state = DroneState()
        self.scf = None
        self.is_connected = False

    def connect(self) -> bool:
        if not CFLIB_AVAILABLE:
            logger.error("cflib is not installed! Run `pip install cflib` to interface with real LiteWing hardware.")
            return False

        logger.info(f"[REAL] Initializing cflib drivers and connecting to {self.uri}...")
        try:
            cflib.crtp.init_drivers()
            self.cf = Crazyflie(rw_cache='./cache')
            self.scf = SyncCrazyflie(self.uri, cf=self.cf)
            self.scf.open_link()
            self.is_connected = True
            logger.info("[REAL] Connected to LiteWing ESP32-S3!")
            return True
        except Exception as e:
            logger.error(f"[REAL] Connection failed to {self.uri}: {e}")
            self.is_connected = False
            return False

    def disconnect(self) -> None:
        if self.scf and self.is_connected:
            self.scf.close_link()
            self.is_connected = False
            logger.info("[REAL] Disconnected from LiteWing.")

    def takeoff(self, altitude: float = 1.0) -> bool:
        logger.info(f"[REAL] Taking off to {altitude}m...")
        if not self.is_connected:
            logger.warning("[REAL] Drone not connected!")
            return False

        try:
            self.cf.platform.send_arming_request(True)
            time.sleep(0.5)

            steps = 5
            for i in range(steps):
                h = (i + 1) * altitude / steps
                self.cf.commander.send_hover_setpoint(0, 0, 0, h)
                time.sleep(0.1)

            self.state.z = altitude
            self.state.is_armed = True
            self.state.flight_mode = FlightMode.HOVERING
            return True
        except Exception as e:
            logger.error(f"[REAL] Takeoff error: {e}")
            return False

    def land(self) -> bool:
        logger.info("[REAL] Landing sequence initiated...")
        if not self.is_connected:
            return False

        try:
            steps = 5
            initial_z = self.state.z
            for i in range(steps):
                h = initial_z * (steps - i - 1) / steps
                self.cf.commander.send_hover_setpoint(0, 0, 0, h)
                time.sleep(0.1)

            self.cf.commander.send_stop_setpoint()
            self.cf.platform.send_arming_request(False)
            self.state.z = 0.0
            self.state.is_armed = False
            self.state.flight_mode = FlightMode.IDLE
            return True
        except Exception as e:
            logger.error(f"[REAL] Landing error: {e}")
            return False

    def hover(self, duration: float = 2.0) -> bool:
        if not self.is_connected:
            return False
        logger.info(f"[REAL] Hovering at Z={self.state.z}m for {duration}s...")
        end_time = time.time() + duration
        while time.time() < end_time:
            self.cf.commander.send_hover_setpoint(0, 0, 0, self.state.z)
            time.sleep(0.1)
        return True

    def move_to(self, x: float, y: float, z: float, velocity: float = 0.5) -> bool:
        logger.info(f"[REAL] Moving setpoint to (X={x:.2f}, Y={y:.2f}, Z={z:.2f})...")
        if not self.is_connected:
            return False

        # Calculate proportional velocity towards target
        dx = x - self.state.x
        dy = y - self.state.y
        vx = max(min(dx * 0.8, velocity), -velocity)
        vy = max(min(dy * 0.8, velocity), -velocity)

        self.cf.commander.send_hover_setpoint(vx, vy, 0, z)
        self.state.x, self.state.y, self.state.z = x, y, z
        return True

    def rotate(self, angle_degrees: float) -> bool:
        if not self.is_connected:
            return False
        logger.info(f"[REAL] Yaw setpoint rotation {angle_degrees}°...")
        yaw_rate = max(min(angle_degrees, 30.0), -30.0)
        self.cf.commander.send_hover_setpoint(0, 0, yaw_rate, self.state.z)
        self.state.yaw = (self.state.yaw + angle_degrees) % 360
        return True

    def emergency_stop(self) -> None:
        logger.warning("[REAL] EMERGENCY STOP ACTIVATED!")
        if self.is_connected:
            try:
                self.cf.commander.send_stop_setpoint()
                self.cf.platform.send_arming_request(False)
            except Exception as e:
                logger.error(f"[REAL] E-Stop transmission error: {e}")
        self.state.flight_mode = FlightMode.EMERGENCY
        self.state.is_armed = False

    def get_state(self) -> DroneState:
        return self.state
