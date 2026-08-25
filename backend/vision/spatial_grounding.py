"""
Spatial Grounding — Homography Matrix Transformation (Pixel -> Physical World).
"""
import numpy as np
from typing import Tuple


class SpatialGrounding:
    """
    Transforms 2D overhead camera pixel coordinates (u, v) into physical workspace
    world coordinates (X, Y) in meters using a calibrated 3x3 Homography matrix H.
    """

    def __init__(self, homography_matrix: np.ndarray = None):
        if homography_matrix is not None:
            self.H = homography_matrix
        else:
            # Default linear scale mapping for 640x480 camera covering [-1.5, 1.5]m workspace
            # u=0 -> X=-1.5, u=640 -> X=1.5
            # v=0 -> Y=1.5,  v=480 -> Y=-1.5
            src_pts = np.float32([[0, 0], [640, 0], [640, 480], [0, 480]])
            dst_pts = np.float32([[-1.5, 1.5], [1.5, 1.5], [1.5, -1.5], [-1.5, -1.5]])
            self.H, _ = cv2_find_homography(src_pts, dst_pts)

    def pixel_to_world(self, u: int, v: int, z_altitude: float = 1.0) -> Tuple[float, float, float]:
        """
        Maps pixel (u, v) to physical world position (X, Y, Z) in meters.
        """
        pixel_vec = np.array([u, v, 1.0], dtype=np.float32)
        world_vec = np.dot(self.H, pixel_vec)
        world_x = world_vec[0] / world_vec[2]
        world_y = world_vec[1] / world_vec[2]
        return round(float(world_x), 3), round(float(world_y), 3), round(float(z_altitude), 3)


def cv2_find_homography(src_pts, dst_pts):
    try:
        import cv2
        H, mask = cv2.findHomography(src_pts, dst_pts)
        return H, mask
    except ImportError:
        # Fallback linear transformation matrix if OpenCV not installed
        H = np.array([
            [3.0/640.0, 0.0, -1.5],
            [0.0, -3.0/480.0, 1.5],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        return H, None
