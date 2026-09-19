"""
Spatial Grounding — Homography Matrix Transformation (Pixel -> Physical World).
"""
import numpy as np
from typing import Tuple


class SpatialGrounding:
    """
    Transforms 2D overhead camera pixel coordinates (u, v) into physical workspace
    world coordinates (X, Y) in meters using a calibrated 3x3 Homography matrix H.
    Supports dynamic image dimensions and workspace extents.
    """

    def __init__(self, homography_matrix: np.ndarray = None, img_w: int = 640, img_h: int = 480):
        self.img_w = img_w
        self.img_h = img_h
        if homography_matrix is not None:
            self.H = homography_matrix
        else:
            self.update_resolution(img_w, img_h)

    def update_resolution(self, img_w: int, img_h: int, workspace_extent: float = 1.5):
        """Recomputes homography for given camera resolution and metric bounds."""
        self.img_w = max(1, int(img_w))
        self.img_h = max(1, int(img_h))
        w, h = float(self.img_w), float(self.img_h)
        ext = float(workspace_extent)
        src_pts = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        dst_pts = np.float32([[-ext, ext], [ext, ext], [ext, -ext], [-ext, -ext]])
        self.H, _ = cv2_find_homography(src_pts, dst_pts, w, h, ext)

    def pixel_to_world(self, u: float, v: float, z_altitude: float = 1.0,
                       img_w: int = None, img_h: int = None) -> Tuple[float, float, float]:
        """
        Maps pixel (u, v) to physical world position (X, Y, Z) in meters.
        If img_w and img_h are provided and differ from initialized resolution,
        coordinates are dynamically scaled.
        """
        if img_w is not None and img_h is not None and img_w > 0 and img_h > 0:
            if img_w != self.img_w or img_h != self.img_h:
                u = u * (self.img_w / img_w)
                v = v * (self.img_h / img_h)

        pixel_vec = np.array([u, v, 1.0], dtype=np.float32)
        world_vec = np.dot(self.H, pixel_vec)
        denom = world_vec[2] if abs(world_vec[2]) > 1e-6 else 1.0
        world_x = world_vec[0] / denom
        world_y = world_vec[1] / denom
        return round(float(world_x), 3), round(float(world_y), 3), round(float(z_altitude), 3)


def cv2_find_homography(src_pts, dst_pts, w: float = 640.0, h: float = 480.0, ext: float = 1.5):
    try:
        import cv2
        H, mask = cv2.findHomography(src_pts, dst_pts)
        if H is not None:
            return H, mask
    except Exception:
        pass
    # Fallback linear transformation matrix if OpenCV is unavailable or homography fails
    H = np.array([
        [2.0 * ext / w, 0.0, -ext],
        [0.0, -2.0 * ext / h, ext],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)
    return H, None
