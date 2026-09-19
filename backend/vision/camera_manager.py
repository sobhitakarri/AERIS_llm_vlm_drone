"""
Camera Stream Manager — latest-frame grab thread for overhead / webcam.
"""
from __future__ import annotations

import threading
import time
from typing import Optional

import numpy as np

from backend.core.config import settings
from backend.core.logger import get_logger

logger = get_logger("CameraManager")


class CameraManager:
    """OpenCV capture with a background thread that keeps the newest frame."""

    def __init__(self, camera_index: int = None):
        self.camera_index = camera_index if camera_index is not None else settings.camera_index
        self.cap = None
        self._lock = threading.Lock()
        self._latest: Optional[np.ndarray] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        if self._running:
            # Already grabbing; re-opening here would fight the grab thread.
            return self.cap is not None and self.cap.isOpened()
        opened = self._open()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="camera-grab")
        self._thread.start()
        logger.info("Camera grab thread started.")
        return opened

    def _open(self) -> bool:
        try:
            import cv2
            if self.cap is not None:
                try:
                    self.cap.release()
                except Exception:
                    pass
            cap = cv2.VideoCapture(self.camera_index)
            if cap.isOpened():
                self.cap = cap
                logger.info(f"Opened camera device index {self.camera_index}.")
                return True
            cap.release()
        except ImportError:
            logger.warning("OpenCV not installed. Using synthetic frame mode.")
        # Leave cap as None so the grab thread keeps retrying the device.
        self.cap = None
        return False

    def _loop(self) -> None:
        while self._running:
            frame = None
            if self.cap is None and not self._open():
                time.sleep(2.0)  # device absent: back off instead of hammering it
                continue
            if self.cap and self.cap.isOpened():
                ret, grabbed = self.cap.read()
                if ret and grabbed is not None:
                    frame = grabbed
                else:
                    self.cap.release()
                    self.cap = None  # dropped device: reopen on next pass
            if frame is not None:
                with self._lock:
                    self._latest = frame
            time.sleep(0.066)  # ~15 Hz

    def read_frame(self, timeout_s: float = 1.0) -> Optional[np.ndarray]:
        """Returns the newest grabbed frame. Never touches the device directly —
        concurrent reads with the grab thread corrupt the capture backend.
        Pass timeout_s=0 from hot loops that must not block."""
        if not self._running:
            self.start()
        deadline = time.time() + timeout_s
        while True:
            with self._lock:
                if self._latest is not None:
                    return self._latest.copy()
            if time.time() >= deadline:
                return None
            time.sleep(0.02)  # first call: let the grab thread produce a frame

    def read_jpeg(self) -> Optional[bytes]:
        frame = self.read_frame()
        if frame is None:
            return None
        try:
            import cv2
            ret, buf = cv2.imencode(".jpg", frame)
            if ret:
                return buf.tobytes()
        except Exception as e:
            logger.warning(f"Error encoding JPEG frame: {e}")
        return None

    def stop(self) -> None:
        self._running = False
        # Join before release: the grab thread must not be inside cap.read()
        # when the device goes away.
        thread, self._thread = self._thread, None
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.5)
        if self.cap:
            self.cap.release()
            self.cap = None
            logger.info("Released camera device.")
        with self._lock:
            self._latest = None
