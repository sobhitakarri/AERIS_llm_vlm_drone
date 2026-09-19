"""
Fast Perception Stack (OpenCV + Object Bounding Box Tracking).
"""
import time
from typing import Any, List, Optional
import numpy as np

from backend.schemas.perception import BoundingBox, DetectedObject, PerceptionResult
from backend.vision.spatial_grounding import SpatialGrounding
from backend.runtime.state_manager import StateManager
from backend.core.logger import get_logger

logger = get_logger("FastPerception")


class FastPerception:
    """
    Visual tracking loop running at high frequency.
    Detects bounding boxes via VLM / CV and grounds pixel centers to real-world coordinates.
    Synchronizes detections with StateManager.
    """

    def __init__(
        self,
        grounding: Optional[SpatialGrounding] = None,
        vlm: Optional[Any] = None,
        state_manager: Optional[StateManager] = None,
    ):
        self.grounding = grounding or SpatialGrounding()
        self.vlm = vlm
        self.state_manager = state_manager

    def process_frame(
        self,
        frame: Optional[np.ndarray] = None,
        targets: Optional[List[str]] = None,
        z_altitude: float = 1.0,
    ) -> PerceptionResult:
        """
        Processes camera frame to produce detected objects with grounded world coordinates.
        Uses real VLM visual grounding when available, otherwise reads active StateManager objects.
        """
        objects: List[DetectedObject] = []

        # 1. If VLM and frame are provided, run visual grounding on requested targets
        if frame is not None and self.vlm and hasattr(self.vlm, "resolve_target") and targets:
            try:
                import cv2
                _, enc = cv2.imencode(".jpg", frame)
                img_bytes = enc.tobytes()
                h, w = frame.shape[:2]

                for target in targets:
                    det = self.vlm.resolve_target(img_bytes, target, image_width=w, image_height=h)
                    if det and det.bbox:
                        u, v = det.bbox.center_pixel
                        wx, wy, wz = self.grounding.pixel_to_world(
                            u, v, z_altitude=z_altitude, img_w=w, img_h=h
                        )
                        det.world_x = wx
                        det.world_y = wy
                        det.world_z = wz
                        objects.append(det)
                        if self.state_manager:
                            self.state_manager.update_object(det)
            except Exception as e:
                logger.warning(f"Error during VLM perception loop: {e}")

        # 2. If no objects resolved from frame, read live objects from synchronized StateManager
        if not objects and self.state_manager:
            objects = self.state_manager.get_all_objects()

        # 3. Apply spatial grounding to any ungrounded objects
        for obj in objects:
            if obj.bbox and (obj.world_x is None or obj.world_y is None):
                u_center, v_center = obj.bbox.center_pixel
                world_x, world_y, world_z = self.grounding.pixel_to_world(u_center, v_center, z_altitude=z_altitude)
                obj.world_x = world_x
                obj.world_y = world_y
                obj.world_z = world_z

        return PerceptionResult(
            timestamp=time.time(),
            objects=objects,
            drone_pixel=(320, 240),
            drone_world=(0.0, 0.0),
        )
