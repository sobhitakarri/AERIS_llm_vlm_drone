"""
Records a twin test case to video.

Each frame is a mosaic of the overhead camera (with the perception overlays),
the room camera, the onboard camera and a telemetry panel that shows the
estimated pose next to the simulator's true pose. Frames are taken on the
simulation clock, so the video plays at a fixed multiple of simulated time
however fast the twin happens to run.

    rec = Recorder(path, "Case 2", HttpSource("http://127.0.0.1:8000"))
    rec.start(); ...; rec.stop()
"""
import json
import math
import shutil
import subprocess
import threading
import time
import urllib.request
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np

W, H, HEAD = 1280, 720, 40
SIDE = 374
FONT = cv2.FONT_HERSHEY_SIMPLEX


class HttpSource:
    """Frames and telemetry from a running backend."""

    def __init__(self, base: str = "http://127.0.0.1:8000"):
        self.base = base

    def _get(self, path: str, timeout: float = 4.0) -> bytes:
        with urllib.request.urlopen(self.base + path, timeout=timeout) as r:
            return r.read()

    def frame(self, cam: str) -> Optional[np.ndarray]:
        try:
            data = self._get(f"/api/snapshot?cam={cam}&overlay=1")
        except Exception:
            return None
        return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

    def info(self) -> dict:
        try:
            m = json.loads(self._get("/api/telemetry"))
        except Exception:
            return {}
        mon, twin, st = m.get("monitor") or {}, m.get("twin") or {}, m.get("state") or {}
        return {
            "sim_time": twin.get("sim_time"),
            "est": (st.get("x"), st.get("y"), st.get("z"), st.get("yaw")),
            "truth": twin.get("truth"),
            "battery_v": st.get("battery_v"),
            "skill": mon.get("current_skill"),
            "step": (mon.get("current_step"), mon.get("total_steps")),
            "status": mon.get("status"),
            "replans": m.get("replan_count", 0),
            "targets": [o for o in m.get("objects", []) if o.get("world_x") is not None],
        }


class TwinSource:
    """Frames and telemetry straight from a TwinInterface (no backend running)."""

    def __init__(self, twin):
        self.twin = twin

    def frame(self, cam: str) -> Optional[np.ndarray]:
        return self.twin.get_camera_frame(cam)

    def info(self) -> dict:
        st = self.twin.state
        return {
            "sim_time": self.twin.sim_time(),
            "est": (st.x, st.y, st.z, st.yaw),
            "truth": self.twin.truth or None,
            "battery_v": st.battery_v,
            "skill": st.flight_mode.value,
            "step": (None, None),
            "status": None,
            "replans": 0,
            "targets": [],
        }


def _fit(img: Optional[np.ndarray], w: int, h: int, label: str) -> np.ndarray:
    tile = np.full((h, w, 3), 18, np.uint8)
    if img is not None:
        s = min(w / img.shape[1], h / img.shape[0])
        r = cv2.resize(img, (max(1, int(img.shape[1] * s)), max(1, int(img.shape[0] * s))), interpolation=cv2.INTER_AREA)
        y0, x0 = (h - r.shape[0]) // 2, (w - r.shape[1]) // 2
        tile[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
    cv2.rectangle(tile, (0, 0), (len(label) * 9 + 12, 20), (0, 0, 0), -1)
    cv2.putText(tile, label, (6, 15), FONT, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
    return tile


def _fmt(v, spec: str = "+.2f") -> str:
    return format(v, spec) if isinstance(v, (int, float)) else "-"


class Recorder:
    def __init__(self, path, title: str, source, caption: str = "", fps: int = 10, sim_dt: float = 0.2):
        self.path = Path(path)
        self.title, self.caption, self.note = title, caption, ""
        self.source, self.fps, self.sim_dt = source, fps, sim_dt
        self.last_frame: Optional[np.ndarray] = None
        self.frames = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._raw = self.path.with_suffix(".raw.mp4")
        self._writer = None

    # ------------------------------------------------------------- lifecycle
    def start(self) -> "Recorder":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._writer = cv2.VideoWriter(str(self._raw), cv2.VideoWriter_fourcc(*"mp4v"), self.fps, (W, H))
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        return self

    def stop(self) -> Path:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=10)
        if self._writer is not None:
            self._writer.release()
        ffmpeg = shutil.which("ffmpeg")
        done = False
        if ffmpeg and self.frames:
            cmd = [ffmpeg, "-y", "-loglevel", "error", "-i", str(self._raw), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                   "-crf", "27", "-preset", "medium", "-movflags", "+faststart", str(self.path)]
            done = subprocess.run(cmd).returncode == 0
        if done:
            self._raw.unlink(missing_ok=True)
        else:
            self._raw.replace(self.path)       # mp4v: plays in VLC, not in every browser
        return self.path

    def save_still(self, path) -> bool:
        if self.last_frame is None:
            return False
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        return bool(cv2.imwrite(str(path), self.last_frame, [cv2.IMWRITE_JPEG_QUALITY, 90]))

    # ----------------------------------------------------------------- frames
    def _loop(self) -> None:
        last_sim, last_wall = None, 0.0
        while not self._stop.is_set():
            info = self.source.info()
            sim_t = info.get("sim_time")
            if isinstance(sim_t, (int, float)):
                due = last_sim is None or sim_t - last_sim >= self.sim_dt or sim_t < last_sim
            else:
                due = time.time() - last_wall >= self.sim_dt
            if not due:
                time.sleep(0.03)
                continue
            last_sim, last_wall = sim_t, time.time()
            frame = self._compose(info)
            self._writer.write(frame)
            self.last_frame = frame
            self.frames += 1

    def _compose(self, info: dict) -> np.ndarray:
        canvas = np.full((H, W, 3), 18, np.uint8)
        cv2.rectangle(canvas, (0, 0), (W, HEAD), (40, 32, 24), -1)
        cv2.putText(canvas, self.title, (12, 27), FONT, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        (tw, _), _ = cv2.getTextSize(self.title, FONT, 0.7, 2)
        cv2.putText(canvas, self.caption[:95], (tw + 36, 26), FONT, 0.55, (255, 200, 120), 1, cv2.LINE_AA)

        body = H - HEAD
        canvas[HEAD:, :W - SIDE] = _fit(self.source.frame("overhead"), W - SIDE, body, "overhead RGB-D camera + perception")
        canvas[HEAD:HEAD + 210, W - SIDE:] = _fit(self.source.frame("chase"), SIDE, 210, "room camera")
        canvas[HEAD + 210:HEAD + 490, W - SIDE:] = _fit(self.source.frame("fpv"), SIDE, 280, "onboard camera")

        x0, y = W - SIDE + 12, HEAD + 490 + 22
        est, tr = info.get("est") or (None,) * 4, info.get("truth") or {}
        err = None
        if tr and all(isinstance(v, (int, float)) for v in est[:3]):
            err = 100 * math.dist(est[:3], (tr.get("x", 0), tr.get("y", 0), tr.get("z", 0)))
        step, total = info.get("step") or (None, None)
        phase = info.get("skill") or "-"
        if step is not None and total:
            phase = f"{phase}  ({min(step + 1, total)}/{total})"
        rows = [
            (f"t = {_fmt(info.get('sim_time'), '.1f')} s sim   x{self.fps * self.sim_dt:.0f} playback", (200, 200, 200)),
            (f"phase   {phase}", (120, 220, 255)),
            (f"status  {info.get('status') or '-'}   replans {info.get('replans', 0)}", (200, 200, 200)),
            (f"est   {_fmt(est[0])} {_fmt(est[1])} {_fmt(est[2])} m", (255, 194, 76)),
            (f"true  {_fmt(tr.get('x'))} {_fmt(tr.get('y'))} {_fmt(tr.get('z'))} m", (255, 140, 180)),
            (f"est. error {_fmt(err, '.1f')} cm   battery {_fmt(info.get('battery_v'), '.2f')} V", (200, 200, 200)),
        ]
        for o in (info.get("targets") or [])[:1]:
            rows.append((f"target {o['label'][:14]} at {o['world_x']:+.2f} {o['world_y']:+.2f}", (138, 208, 63)))
        if self.note:
            rows.append((self.note[:44], (93, 93, 255)))
        for text, colour in rows[:8]:
            cv2.putText(canvas, text, (x0, y), FONT, 0.47, colour, 1, cv2.LINE_AA)
            y += 22
        return canvas
