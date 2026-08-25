"""
MATLAB / Simulink UDP Socket Bridge Drone Interface.
"""
import math
import socket
import json
import threading
import time
from backend.drone.base_interface import DroneInterface
from backend.schemas.telemetry import DroneState, FlightMode
from backend.core.logger import get_logger

logger = get_logger("MatlabInterface")


class MatlabInterface(DroneInterface):
    """
    Real-time UDP Socket Bridge connecting Python AI mission planner with MATLAB/Simulink.
    - Sends command setpoints to MATLAB over UDP Port 5005
    - Receives 6-DOF telemetry from MATLAB over UDP Port 5006
    """

    def __init__(self, send_port: int = 5005, recv_port: int = 5006, host: str = "127.0.0.1"):
        self.host = host
        self.send_port = send_port
        self.recv_port = recv_port
        self.state = DroneState()
        self.is_connected = False
        self.send_sock = None
        self.recv_sock = None
        self.listen_thread = None
        self._telem_ok = False

    def connect(self) -> bool:
        logger.info(f"[MATLAB] Initializing UDP Socket Bridge (Send Port: {self.send_port}, Recv Port: {self.recv_port})...")
        try:
            self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

            self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.recv_sock.bind((self.host, self.recv_port))
            self.recv_sock.settimeout(0.5)

            self.is_connected = True

            # Start background thread to listen for MATLAB 6-DOF telemetry
            self.listen_thread = threading.Thread(target=self._telemetry_listener, daemon=True)
            self.listen_thread.start()

            logger.info("[MATLAB] UDP Socket Bridge active and connected to MATLAB/Simulink.")
            return True
        except Exception as e:
            logger.error(f"[MATLAB] UDP socket initialization error: {e}")
            self.is_connected = False
            return False

    def disconnect(self) -> None:
        self.is_connected = False
        if self.send_sock:
            self.send_sock.close()
        if self.recv_sock:
            self.recv_sock.close()
        logger.info("[MATLAB] UDP Socket Bridge closed.")

    def _send_udp_cmd(self, command: str, **kwargs) -> bool:
        if not self.is_connected or not self.send_sock:
            return False

        packet = {
            "cmd": command,
            "timestamp": time.time(),
            **kwargs
        }
        try:
            payload = json.dumps(packet).encode("utf-8")
            self.send_sock.sendto(payload, (self.host, self.send_port))
            return True
        except Exception as e:
            logger.error(f"[MATLAB] UDP send error: {e}")
            return False

    def _telemetry_listener(self) -> None:
        """Background thread receiving MATLAB simulation 6-DOF telemetry."""
        while self.is_connected:
            try:
                data, _ = self.recv_sock.recvfrom(2048)
                packet = json.loads(data.decode("utf-8"))

                # Parse MATLAB 6-DOF telemetry
                self.state.x = float(packet.get("x", self.state.x))
                self.state.y = float(packet.get("y", self.state.y))
                self.state.z = float(packet.get("z", self.state.z))
                self.state.roll = float(packet.get("roll", self.state.roll))
                self.state.pitch = float(packet.get("pitch", self.state.pitch))
                self.state.yaw = float(packet.get("yaw", self.state.yaw))
                self.state.vx = float(packet.get("vx", self.state.vx))
                self.state.vy = float(packet.get("vy", self.state.vy))
                self.state.vz = float(packet.get("vz", self.state.vz))
                self.state.battery_v = float(packet.get("battery_v", 4.1))
                self._telem_ok = True
            except socket.timeout:
                continue
            except Exception:
                continue

    def _wait_until_near(
        self,
        x: float,
        y: float,
        z: float,
        timeout: float = 30.0,
        tol: float = 0.08,
        settle: float = 0.45,
    ) -> bool:
        """Wait until MATLAB is at the waypoint AND has stopped, so corners stay sharp."""
        t0 = time.time()
        inside_since = None
        while time.time() - t0 < timeout:
            if self._telem_ok:
                dist = math.sqrt(
                    (self.state.x - x) ** 2 + (self.state.y - y) ** 2 + (self.state.z - z) ** 2
                )
                speed = math.sqrt(self.state.vx ** 2 + self.state.vy ** 2 + self.state.vz ** 2)
                if dist <= tol and speed <= 0.08:
                    if inside_since is None:
                        inside_since = time.time()
                    elif time.time() - inside_since >= settle:
                        logger.info(
                            f"[MATLAB] Arrived at ({x:.2f}, {y:.2f}, {z:.2f})"
                        )
                        return True
                else:
                    inside_since = None
            elif time.time() - t0 > 0.35:
                self.state.x, self.state.y, self.state.z = x, y, z
                return True
            time.sleep(0.05)
        logger.warning(
            f"[MATLAB] Timeout waiting to reach ({x:.2f}, {y:.2f}, {z:.2f}). Continuing."
        )
        return True

    def takeoff(self, altitude: float = 1.0) -> bool:
        logger.info(f"[MATLAB] Sending TAKEOFF (Z={altitude}m) setpoint to MATLAB/Simulink...")
        self.state.flight_mode = FlightMode.TAKING_OFF
        self.state.is_armed = True
        hold_x, hold_y = self.state.x, self.state.y
        success = self._send_udp_cmd("TAKEOFF", z=altitude)
        if success:
            self._wait_until_near(hold_x, hold_y, altitude)
        self.state.flight_mode = FlightMode.HOVERING
        return success

    def land(self) -> bool:
        logger.info("[MATLAB] Sending LAND setpoint to MATLAB/Simulink...")
        self.state.flight_mode = FlightMode.LANDING
        success = self._send_udp_cmd("LAND")
        if success:
            self._wait_until_near(0.0, 0.0, 0.0)
        self.state.z = 0.0
        self.state.is_armed = False
        self.state.flight_mode = FlightMode.IDLE
        return success

    def hover(self, duration: float = 2.0) -> bool:
        logger.info(f"[MATLAB] Sending HOVER setpoint ({duration}s) to MATLAB/Simulink...")
        self.state.flight_mode = FlightMode.HOVERING
        success = self._send_udp_cmd("HOVER", duration=duration)
        time.sleep(duration if self._telem_ok else min(duration, 1.0))
        return success

    def move_to(self, x: float, y: float, z: float, velocity: float = 0.5) -> bool:
        logger.info(f"[MATLAB] Sending MOVE_TO (X={x:.2f}, Y={y:.2f}, Z={z:.2f}) setpoint to MATLAB/Simulink...")
        self.state.flight_mode = FlightMode.NAVIGATING
        success = self._send_udp_cmd("MOVE_TO", x=x, y=y, z=z, velocity=velocity)
        if success:
            self._wait_until_near(x, y, z)
        self.state.flight_mode = FlightMode.HOVERING
        return success

    def rotate(self, angle_degrees: float) -> bool:
        logger.info(f"[MATLAB] Sending ROTATE ({angle_degrees}°) setpoint to MATLAB/Simulink...")
        success = self._send_udp_cmd("ROTATE", angle=angle_degrees)
        self.state.yaw = (self.state.yaw + angle_degrees) % 360
        return success

    def emergency_stop(self) -> None:
        logger.warning("[MATLAB] Sending EMERGENCY_STOP to MATLAB/Simulink!")
        self._send_udp_cmd("EMERGENCY")
        self.state.flight_mode = FlightMode.EMERGENCY
        self.state.is_armed = False

    def get_state(self) -> DroneState:
        return self.state

    def clear_path(self) -> bool:
        logger.info("[MATLAB] Clearing live flight trails.")
        return self._send_udp_cmd("CLEAR_PATH")

    def reset_home(self) -> bool:
        logger.info("[MATLAB] Resetting twin to home (0, 0, 0).")
        self.state = DroneState()
        return self._send_udp_cmd("HOME")
