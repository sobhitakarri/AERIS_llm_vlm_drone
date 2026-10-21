"""
Digital-twin drone interface.

Talks to the Simulink / Simscape Multibody / Unreal twin in matlab/twin:
  UDP  commands out, telemetry in (JSON)
  TCP  rendered camera frames in (overhead, drone FPV, chase)

The twin publishes cache/twin_runtime.json at start-up with its ports, camera
intrinsics, workspace limits and the surveyed calibration-marker positions.
Object poses in that file sit under "ground_truth" and are exposed here only
through `ground_truth()`, for scoring. Nothing in the planning or perception
path reads them.
"""
import json
import math
import queue
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path
from typing import Dict, Optional

import numpy as np

from backend.drone.base_interface import DroneInterface
from backend.schemas.telemetry import DroneState, FlightMode
from backend.core.logger import get_logger

logger = get_logger("TwinInterface")

REPO_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_FILE = REPO_ROOT / "cache" / "twin_runtime.json"
TWIN_DIR = REPO_ROOT / "matlab" / "twin"

_FRAME_MAGIC = b"AERS"
_FRAME_HEADER = struct.Struct("<4sBB2xHHf")
_FMT_RGB, _FMT_DEPTH_MM = 0, 1
_DEFAULT_NET = {"host": "127.0.0.1", "cmd_port": 5005, "telem_port": 5006, "cam_port": 5007}


def load_runtime() -> dict:
    """Runtime description written by the twin; empty when it has never run."""
    try:
        return json.loads(RUNTIME_FILE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


class TwinInterface(DroneInterface):
    """Skill-level interface to the MATLAB digital twin."""

    ARRIVE_TOL_M = 0.10
    ARRIVE_SPEED_M_S = 0.12
    SETTLE_S = 0.4

    def __init__(self, launch: bool = False):
        self.runtime = load_runtime()
        net = {**_DEFAULT_NET, **(self.runtime.get("net") or {})}
        self.host = net["host"]
        self.cmd_port = int(net["cmd_port"])
        self.telem_port = int(net["telem_port"])
        self.cam_port = int(net["cam_port"])
        self.launch_requested = launch

        self.state = DroneState()
        self.truth: Dict[str, float] = {}
        self.telemetry: dict = {}
        self.is_connected = False
        self._running = False
        self._last_telem = 0.0
        self._frames: Dict[int, np.ndarray] = {}
        self._frame_time: Dict[int, float] = {}
        self._frame_sim_t: Dict[int, float] = {}
        self._frame_lock = threading.Lock()
        self._cam_ids = self._camera_ids()
        self._send_sock: Optional[socket.socket] = None
        self._recv_sock: Optional[socket.socket] = None
        self._proc: Optional[subprocess.Popen] = None
        self._flight_mode = FlightMode.IDLE
        # Injected command-link latency (communication-delay disturbance).
        # Flight commands are held this long before they are sent, in order.
        self.cmd_delay_s = 0.0
        self._delayed: "queue.Queue" = queue.Queue()
        self._delay_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ setup
    def _camera_ids(self) -> Dict[str, int]:
        streams = self.runtime.get("streams") or {}
        ids = {name: int(sid) for name, sid in streams.items()}
        return ids or {"overhead": 1, "overhead_depth": 2, "fpv": 3, "chase": 4}

    def connect(self) -> bool:
        try:
            self._send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._recv_sock.bind((self.host, self.telem_port))
            self._recv_sock.settimeout(0.5)
        except OSError as exc:
            logger.error(f"[Twin] Could not open telemetry port {self.telem_port}: {exc}")
            return False

        self._running = True
        threading.Thread(target=self._telemetry_loop, daemon=True, name="twin-telemetry").start()
        threading.Thread(target=self._camera_loop, daemon=True, name="twin-camera").start()

        if self.wait_for_telemetry(1.5):
            logger.info("[Twin] Connected to a running digital twin.")
            return True
        if self.launch_requested:
            self.launch()
            logger.info("[Twin] Waiting for the twin to come up (MATLAB + Unreal can take a few minutes)...")
        else:
            logger.warning(
                "[Twin] No telemetry yet. Start the twin in MATLAB with "
                "`run_aeris_twin` (matlab/twin), or pass --launch-twin."
            )
        return True

    def launch(self) -> None:
        """Start MATLAB and the twin as a child process."""
        if self._proc and self._proc.poll() is None:
            return
        cmd = ["matlab", "-nosplash", "-nodesktop", "-r",
               f"cd('{TWIN_DIR.as_posix()}'); run_aeris_twin"]
        try:
            self._proc = subprocess.Popen(cmd, cwd=str(TWIN_DIR))
            logger.info("[Twin] Launched MATLAB twin process.")
        except OSError as exc:
            logger.error(f"[Twin] Could not launch MATLAB ({exc}). Is `matlab` on PATH?")

    def disconnect(self) -> None:
        self._running = False
        self.is_connected = False
        for sock in (self._send_sock, self._recv_sock):
            try:
                if sock:
                    sock.close()
            except OSError:
                pass

    def wait_for_telemetry(self, timeout: float) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.is_connected:
                return True
            time.sleep(0.05)
        return self.is_connected

    # -------------------------------------------------------------- telemetry
    def _telemetry_loop(self) -> None:
        while self._running:
            try:
                data, _ = self._recv_sock.recvfrom(8192)
            except socket.timeout:
                if time.time() - self._last_telem > 1.5:
                    self.is_connected = False
                continue
            except OSError:
                break
            try:
                m = json.loads(data.decode("utf-8"))
            except ValueError:
                continue
            self.telemetry = m
            self._last_telem = time.time()
            if not self.is_connected:
                # The twin rewrites the runtime file each time it starts.
                self.runtime = load_runtime() or self.runtime
                self._cam_ids = self._camera_ids()
                self.is_connected = True
            self.truth = m.get("truth") or {}
            armed = bool(m.get("armed", False))
            s = self.state
            s.x, s.y, s.z = float(m.get("x", 0.0)), float(m.get("y", 0.0)), float(m.get("z", 0.0))
            s.vx, s.vy, s.vz = float(m.get("vx", 0.0)), float(m.get("vy", 0.0)), float(m.get("vz", 0.0))
            s.roll, s.pitch = float(m.get("roll", 0.0)), float(m.get("pitch", 0.0))
            s.yaw = float(m.get("yaw", 0.0))
            s.battery_v = float(m.get("battery_v", s.battery_v))
            s.battery_percentage = int(max(0, min(100, round(float(m.get("battery_pct", 100.0))))))
            s.is_armed = armed
            if not armed and self._flight_mode not in (FlightMode.EMERGENCY, FlightMode.TAKING_OFF):
                self._flight_mode = FlightMode.IDLE
            elif armed and self._flight_mode == FlightMode.IDLE:
                self._flight_mode = FlightMode.HOVERING
            s.flight_mode = self._flight_mode

    def _send(self, command: str, **kwargs) -> bool:
        if not self._send_sock:
            return False
        try:
            payload = json.dumps({"cmd": command, **kwargs}).encode("utf-8")
            # Test apparatus (moving objects, restarting) is not on the drone's link.
            if self.cmd_delay_s > 0 and command not in ("SET_OBJECT", "RESTART", "STOP"):
                self._delayed.put((time.time() + self.cmd_delay_s, payload))
                if self._delay_thread is None or not self._delay_thread.is_alive():
                    self._delay_thread = threading.Thread(target=self._delay_loop, daemon=True, name="twin-link-delay")
                    self._delay_thread.start()
                return True
            self._send_sock.sendto(payload, (self.host, self.cmd_port))
            return True
        except OSError as exc:
            logger.error(f"[Twin] Command send failed: {exc}")
            return False

    def set_link_delay(self, ms: float) -> None:
        self.cmd_delay_s = max(0.0, float(ms) / 1000.0)
        logger.info(f"[Twin] Command-link delay set to {ms:.0f} ms")

    def _delay_loop(self) -> None:
        # FIFO with one fixed delay keeps commands in the order they were sent.
        while self._running or not self._delayed.empty():
            try:
                due, payload = self._delayed.get(timeout=0.5)
            except queue.Empty:
                continue
            wait = due - time.time()
            if wait > 0:
                time.sleep(wait)
            try:
                self._send_sock.sendto(payload, (self.host, self.cmd_port))
            except (OSError, AttributeError) as exc:
                logger.error(f"[Twin] Delayed command send failed: {exc}")

    # ---------------------------------------------------------------- cameras
    def _camera_loop(self) -> None:
        while self._running:
            try:
                with socket.create_connection((self.host, self.cam_port), timeout=2.0) as sock:
                    sock.settimeout(5.0)
                    logger.info("[Twin] Camera stream connected.")
                    while self._running:
                        header = self._recv_exact(sock, _FRAME_HEADER.size)
                        magic, cam_id, fmt, width, height, sim_t = _FRAME_HEADER.unpack(header)
                        if magic != _FRAME_MAGIC:
                            raise ConnectionError("camera stream out of sync")
                        if fmt == _FMT_DEPTH_MM:
                            raw = self._recv_exact(sock, width * height * 2)
                            frame = np.frombuffer(raw, np.uint16).reshape(height, width).astype(np.float32) / 1000.0
                        else:
                            raw = self._recv_exact(sock, width * height * 3)
                            frame = np.frombuffer(raw, np.uint8).reshape(height, width, 3)[:, :, ::-1].copy()  # BGR
                        with self._frame_lock:
                            self._frames[cam_id] = frame
                            self._frame_time[cam_id] = time.time()
                            self._frame_sim_t[cam_id] = float(sim_t)
            except (OSError, ConnectionError, struct.error):
                time.sleep(1.0)

    @staticmethod
    def _recv_exact(sock: socket.socket, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("camera stream closed")
            buf.extend(chunk)
        return bytes(buf)

    def get_camera_frame(self, name: str, max_age: float = 2.0) -> Optional[np.ndarray]:
        cam_id = self._cam_ids.get(name)
        if cam_id is None:
            return None
        with self._frame_lock:
            frame = self._frames.get(cam_id)
            if frame is None or time.time() - self._frame_time.get(cam_id, 0.0) > max_age:
                return None
            return frame.copy()

    def get_frame(self) -> Optional[np.ndarray]:
        """Scene camera used for grounding: the fixed overhead view."""
        return self.get_camera_frame("overhead")

    def get_depth_frame(self, max_age: float = 2.0) -> Optional[np.ndarray]:
        """Overhead depth image in metres (distance along the optical axis)."""
        return self.get_camera_frame("overhead_depth", max_age)

    def camera_names(self):
        return [n for n in self._cam_ids if not n.endswith("_depth")]

    def camera_info(self, name: str) -> dict:
        return (self.runtime.get("cameras") or {}).get(name, {})

    # ------------------------------------------------------------ world model
    def home_position(self) -> tuple:
        home = (self.runtime.get("home") or {}).get("pos") or [0.0, 0.0]
        return (float(home[0]), float(home[1]))

    def workspace(self) -> dict:
        return self.runtime.get("workspace") or {}

    def markers(self) -> dict:
        return self.runtime.get("markers") or {}

    def ground_truth(self) -> dict:
        """True object poses, for scoring only. Never feed this to the planner."""
        gt = json.loads(json.dumps((self.runtime.get("ground_truth") or {}).get("objects") or {}))
        for name, pos in (self.telemetry.get("objects_truth") or {}).items():
            if name in gt and isinstance(pos, list) and len(pos) == 3:
                gt[name]["pos"] = pos
        return gt

    def set_object(self, name: str, x: float, y: float, z: Optional[float] = None) -> bool:
        """Move a movable scene object (disturbance injection for replanning tests)."""
        args = {"name": name, "x": float(x), "y": float(y)}
        if z is not None:
            args["z"] = float(z)
        return self._send("SET_OBJECT", **args)

    def send_external_pose(self, x: float, y: float, yaw_deg: Optional[float] = None) -> bool:
        """Feed an external fix (e.g. from the overhead camera) to the estimator.
        Without `yaw_deg` the fix corrects position only."""
        if yaw_deg is None:
            return self._send("EXT_POSE", x=float(x), y=float(y))
        return self._send("EXT_POSE", x=float(x), y=float(y), yaw=float(yaw_deg))

    # ----------------------------------------------------------------- skills
    def sim_time(self) -> float:
        """Simulation clock. Every wait below is measured on it, so a twin
        that runs slower than real time still gets the durations it was asked
        for and is not timed out for being slow."""
        t = self.telemetry.get("t")
        return float(t) if t is not None else time.time()

    def _speed(self) -> float:
        s = self.state
        return math.sqrt(s.vx ** 2 + s.vy ** 2 + s.vz ** 2)

    def _wait_until_near(self, x: float, y: float, z: float, velocity: float = 0.5) -> bool:
        """Block until the estimated pose has reached and settled at the setpoint."""
        s = self.state
        dist0 = math.sqrt((s.x - x) ** 2 + (s.y - y) ** 2 + (s.z - z) ** 2)
        timeout = 8.0 + 3.0 * dist0 / max(velocity, 0.15)
        t0 = self.sim_time()
        inside_since = None
        dist = dist0
        while self.sim_time() - t0 < timeout:
            if not self.is_connected:
                logger.error("[Twin] Telemetry lost while waiting for a setpoint.")
                return False
            arming = self._flight_mode == FlightMode.TAKING_OFF and self.sim_time() - t0 < 2.0
            if not self.state.is_armed and not arming:
                logger.error("[Twin] Vehicle disarmed before reaching the setpoint "
                             "(emergency stop, or tumble protection after a collision).")
                return False
            s = self.state
            dist = math.sqrt((s.x - x) ** 2 + (s.y - y) ** 2 + (s.z - z) ** 2)
            if dist <= self.ARRIVE_TOL_M and self._speed() <= self.ARRIVE_SPEED_M_S:
                inside_since = inside_since if inside_since is not None else self.sim_time()
                if self.sim_time() - inside_since >= self.SETTLE_S:
                    return True
            else:
                inside_since = None
            time.sleep(0.02)
        logger.error(f"[Twin] Timed out reaching ({x:.2f}, {y:.2f}, {z:.2f}); last distance {dist:.2f} m.")
        return False

    def takeoff(self, altitude: float = 1.0) -> bool:
        if not self.is_connected:
            logger.error("[Twin] Takeoff rejected: the twin is not running.")
            return False
        logger.info(f"[Twin] TAKEOFF to {altitude:.2f} m")
        self._flight_mode = FlightMode.TAKING_OFF
        x, y = self.state.x, self.state.y
        if not self._send("TAKEOFF", z=float(altitude)):
            return False
        target = self._target_after_command(default=(x, y, altitude))
        ok = self._wait_until_near(*target, velocity=0.5)
        self._flight_mode = FlightMode.HOVERING if ok else self._flight_mode
        return ok

    def _target_after_command(self, default):
        """The twin clamps setpoints to its workspace; wait for the clamped value."""
        seq0 = self.telemetry.get("seq")
        t0 = time.time()
        while time.time() - t0 < 1.5:
            if self.telemetry.get("seq") != seq0:
                tgt = self.telemetry.get("target")
                if isinstance(tgt, list) and len(tgt) >= 3:
                    return float(tgt[0]), float(tgt[1]), float(tgt[2])
            time.sleep(0.02)
        tgt = self.telemetry.get("target")
        if isinstance(tgt, list) and len(tgt) >= 3:
            return float(tgt[0]), float(tgt[1]), float(tgt[2])
        return default

    def move_to(self, x: float, y: float, z: float, velocity: float = 0.5) -> bool:
        if not self.state.is_armed:
            logger.error("[Twin] MOVE_TO rejected: vehicle is not airborne.")
            return False
        logger.info(f"[Twin] MOVE_TO ({x:.2f}, {y:.2f}, {z:.2f}) at {velocity:.2f} m/s")
        self._flight_mode = FlightMode.NAVIGATING
        if not self._send("MOVE_TO", x=float(x), y=float(y), z=float(z), velocity=float(velocity)):
            return False
        target = self._target_after_command(default=(x, y, z))
        ok = self._wait_until_near(*target, velocity=velocity)
        self._flight_mode = FlightMode.HOVERING
        return ok

    def hover(self, duration: float = 2.0) -> bool:
        self._flight_mode = FlightMode.HOVERING
        self._send("HOVER")
        t0 = self.sim_time()
        while self.sim_time() - t0 < duration:
            if not self.is_connected:
                return False
            time.sleep(0.05)
        return True

    def rotate(self, angle_degrees: float) -> bool:
        logger.info(f"[Twin] ROTATE {angle_degrees:+.0f} deg")
        if not self._send("ROTATE", angle=float(angle_degrees)):
            return False
        target = (self.state.yaw + angle_degrees + 180.0) % 360.0 - 180.0
        t0 = self.sim_time()
        timeout = 4.0 + abs(angle_degrees) / 30.0
        err = angle_degrees
        while self.sim_time() - t0 < timeout:
            if not self.is_connected:
                return False
            err = (self.state.yaw - target + 180.0) % 360.0 - 180.0
            if abs(err) < 4.0:
                return True
            time.sleep(0.05)
        logger.error(f"[Twin] Yaw did not converge (error {err:.1f} deg).")
        return False

    def land(self) -> bool:
        if not self.state.is_armed:
            self._flight_mode = FlightMode.IDLE
            return True
        logger.info("[Twin] LAND")
        self._flight_mode = FlightMode.LANDING
        if not self._send("LAND"):
            return False
        t0 = self.sim_time()
        timeout = 10.0 + self.state.z / 0.15
        while self.sim_time() - t0 < timeout:
            if not self.is_connected:
                return False
            if not self.state.is_armed:
                self._flight_mode = FlightMode.IDLE
                return True
            time.sleep(0.05)
        logger.error("[Twin] Landing did not complete.")
        return False

    def emergency_stop(self) -> None:
        logger.warning("[Twin] EMERGENCY motor kill")
        self._send("EMERGENCY")
        self._flight_mode = FlightMode.EMERGENCY

    def reset_home(self) -> bool:
        """Return to the home pad and land."""
        if not self.state.is_armed:
            return True
        home = (self.runtime.get("home") or {}).get("pos") or [0.0, 0.0, 0.0]
        self._flight_mode = FlightMode.NAVIGATING
        self._send("HOME")
        target = self._target_after_command(default=(home[0], home[1], self.state.z))
        if not self._wait_until_near(*target):
            return False
        return self.land()

    def restart(self) -> bool:
        """Restart the simulation from its initial state."""
        self._flight_mode = FlightMode.IDLE
        return self._send("RESTART")

    def get_state(self) -> DroneState:
        return self.state
