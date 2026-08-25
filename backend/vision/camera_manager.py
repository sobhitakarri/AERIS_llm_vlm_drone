"""
Camera Stream Manager.
"""
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

    def stop(self) -> None:
        if self.cap:
            self.cap.release()
            self.cap = None
            logger.info("Released camera device.")
