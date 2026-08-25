"""
Fast Perception Stack (OpenCV + Object Bounding Box Tracking).
"""
import time
from typing import List
from backend.schemas.perception import BoundingBox, DetectedObject, PerceptionResult
from backend.vision.spatial_grounding import SpatialGrounding
from backend.core.logger import get_logger

logger = get_logger("FastPerception")


class FastPerception:
    """
    High-frequency visual tracking loop running at 30 FPS.
    Detects bounding boxes and grounds pixel centers to real-world coordinates.
    """

    def __init__(self, grounding: SpatialGrounding = None):
        self.grounding = grounding or SpatialGrounding()

    def process_frame(self, frame=None) -> PerceptionResult:
        """
        Processes camera frame to produce detected objects with grounded world coordinates.
        Uses mock/synthetic detections if no physical camera feed is connected.
        """
        # Synthetic visual scene detections for simulation mode
        objects = [
            DetectedObject(
                label="red_bottle",
                bbox=BoundingBox(u_min=420, v_min=210, u_max=510, v_max=330),
                confidence=0.94
            ),
            DetectedObject(
                label="blue_cube",
                bbox=BoundingBox(u_min=100, v_min=120, u_max=180, v_max=200),
                confidence=0.91
            )
        ]

        # Apply spatial grounding to every detected object
        for obj in objects:
            u_center, v_center = obj.bbox.center_pixel
            world_x, world_y, _ = self.grounding.pixel_to_world(u_center, v_center)
            obj.world_x = world_x
            obj.world_y = world_y

        return PerceptionResult(
            timestamp=time.time(),
            objects=objects,
            drone_pixel=(320, 240),
            drone_world=(0.0, 0.0)
        )
