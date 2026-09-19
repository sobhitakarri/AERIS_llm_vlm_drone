"""
Camera Stream Manager.
"""
from typing import Optional
import numpy as np

from backend.core.config import settings
from backend.core.logger import get_logger

logger = get_logger("CameraManager")


class CameraManager:
    """Manages OpenCV camera video capture and streaming MJPEG frames."""

    def __init__(self, camera_index: int = None):
        self.camera_index = camera_index if camera_index is not None else settings.camera_index
        self.cap = None

    def start(self) -> bool:
        try:
            import cv2
            self.cap = cv2.VideoCapture(self.camera_index)
            if self.cap.isOpened():
                logger.info(f"Opened camera device index {self.camera_index}.")
                return True
        except ImportError:
            logger.warning("OpenCV not installed or camera device unavailable. Using synthetic frame mode.")
        return False

    def read_frame(self) -> Optional[np.ndarray]:
        """Capture a single frame from the camera as a numpy BGR array."""
        if self.cap is None:
            self.start()
        if self.cap and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret and frame is not None:
                return frame
        return None

    def read_jpeg(self) -> Optional[bytes]:
        """Capture a single frame encoded as JPEG bytes."""
        frame = self.read_frame()
        if frame is not None:
            try:
                import cv2
                ret, buf = cv2.imencode(".jpg", frame)
                if ret:
                    return buf.tobytes()
            except Exception as e:
                logger.warning(f"Error encoding JPEG frame: {e}")
        return None

    def stop(self) -> None:
        if self.cap:
            self.cap.release()
            self.cap = None
            logger.info("Released camera device.")
