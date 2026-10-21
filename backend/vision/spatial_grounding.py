"""
Spatial Grounding — pixel coordinates to physical world coordinates.

Three levels, used in this order of preference:
  1. Calibrated RGB-D camera: a detection is matched to a physical object
     found in the depth image and takes that object's 3D position. A box that
     matches no object is rejected.
  2. Calibrated RGB camera: the pixel is projected onto the floor plane.
  3. Uncalibrated camera: a nominal homography that maps the image onto the
     workspace. Good enough for a webcam demo, not for measurement.
"""
from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Callable, List, Optional, Tuple

import numpy as np

from backend.core.logger import get_logger
from backend.schemas.perception import BoundingBox

logger = get_logger("SpatialGrounding")


class SpatialGrounding:
    """
    Transforms overhead camera pixel coordinates (u, v) into workspace
    coordinates in metres.
    """

    def __init__(self, homography_matrix: np.ndarray = None, img_w: int = 640, img_h: int = 480):
        self.img_w = img_w
        self.img_h = img_h
        self.calibration = None             # OverheadCalibration
        self.geometry = None                # SceneGeometry
        self.bounds: Optional[dict] = None  # {"x": [lo, hi], "y": [lo, hi]}
        self.depth_source: Optional[Callable[[], Optional[np.ndarray]]] = None
        self.drone_pose_source: Optional[Callable[[], Tuple[float, float, float]]] = None
        self._lock = threading.Lock()
        self._proposals: list = []
        self._proposals_t = 0.0
        self._height_map: Optional[np.ndarray] = None
        self._points: Optional[np.ndarray] = None
        if homography_matrix is not None:
            self.H = homography_matrix
        else:
            self.update_resolution(img_w, img_h)

    # ------------------------------------------------------------ calibration
    @property
    def calibrated(self) -> bool:
        return self.calibration is not None

    @property
    def has_depth(self) -> bool:
        return self.calibration is not None and self.depth_source is not None

    def set_calibration(self, calibration, bounds: Optional[dict] = None) -> None:
        from backend.vision.scene_proposals import SceneGeometry
        with self._lock:
            self.calibration = calibration
            self.geometry = SceneGeometry(calibration)
            self.H = np.array(calibration.H, dtype=np.float64)
            self.img_w, self.img_h = calibration.image_size
            if bounds:
                self.bounds = bounds
            self._proposals_t = 0.0

    def clear_calibration(self) -> None:
        with self._lock:
            self.calibration = None
            self.geometry = None
            self._proposals = []
            self._height_map = None
            self._points = None

    def update_resolution(self, img_w: int, img_h: int, workspace_extent: float = 1.5):
        """Nominal homography for an uncalibrated camera of this resolution."""
        self.img_w = max(1, int(img_w))
        self.img_h = max(1, int(img_h))
        w, h = float(self.img_w), float(self.img_h)
        ext = float(workspace_extent)
        src_pts = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
        dst_pts = np.float32([[-ext, ext], [ext, ext], [ext, -ext], [-ext, -ext]])
        self.H, _ = cv2_find_homography(src_pts, dst_pts, w, h, ext)

    # ------------------------------------------------------- scene from depth
    def refresh_scene(self, max_age_s: float = 0.25) -> list:
        """Object proposals from the latest depth frame (cached briefly)."""
        if not self.has_depth:
            return []
        now = time.time()
        with self._lock:
            if now - self._proposals_t < max_age_s:
                return list(self._proposals)
        depth = self.depth_source()
        if depth is None:
            return list(self._proposals)
        from backend.vision.scene_proposals import find_proposals
        exclude = []
        if self.drone_pose_source:
            try:
                x, y, z = self.drone_pose_source()
                exclude.append((x, y, z, 0.22))
            except Exception:
                pass
        points = self.geometry.world_points(depth)
        proposals = find_proposals(depth, self.geometry, bounds=self.bounds, exclude=exclude, points=points)
        with self._lock:
            self._proposals = proposals
            self._proposals_t = now
            self._points = points
            self._height_map = points[..., 2]
        return list(proposals)

    def ground_bbox(self, bbox: BoundingBox, img_w: int = None, img_h: int = None, verify: bool = True):
        """Ground a detection box.

        Returns (x, y, top_z, proposal). With a depth camera the proposal is
        the physical object the box refers to, and None for all four means
        the box matched nothing real. Without depth, or with verify=False
        (the no_depth_check ablation), proposal is None and the position is
        the floor projection of the box centre.
        """
        box = self._to_native(bbox, img_w, img_h)
        if self.has_depth and verify:
            from backend.vision.scene_proposals import match_proposal
            match = match_proposal(box, self.refresh_scene())
            if match is None:
                return None, None, None, None
            return match.x, match.y, match.top_z, match
        u, v = box.center_pixel
        x, y, _ = self.pixel_to_world(u, v, 0.0)
        return x, y, 0.0, None

    def _to_native(self, bbox: BoundingBox, img_w: Optional[int], img_h: Optional[int]) -> BoundingBox:
        if not img_w or not img_h or (img_w == self.img_w and img_h == self.img_h):
            return bbox
        sx, sy = self.img_w / img_w, self.img_h / img_h
        return BoundingBox(u_min=int(bbox.u_min * sx), v_min=int(bbox.v_min * sy),
                           u_max=int(bbox.u_max * sx), v_max=int(bbox.v_max * sy))

    def obstacle_height(self, x: float, y: float, radius: float = 0.2) -> float:
        """Tallest surface within `radius` of a world point (0 without depth)."""
        self.refresh_scene()
        with self._lock:
            pts = self._points
        if pts is None:
            return 0.0
        near = (pts[..., 0] - x) ** 2 + (pts[..., 1] - y) ** 2 < radius ** 2
        if self.drone_pose_source:
            try:
                dx, dy, dz = self.drone_pose_source()
                near &= (pts[..., 0] - dx) ** 2 + (pts[..., 1] - dy) ** 2 + (pts[..., 2] - dz) ** 2 > 0.22 ** 2
            except Exception:
                pass
        if not near.any():
            return 0.0
        return float(max(0.0, np.percentile(pts[..., 2][near], 99)))

    def occupancy_grid(self, cell: float = 0.1):
        """Top-down grid of the tallest surface per cell.

        Returns (heights, origin_xy, cell) or None without depth. Cells the
        camera cannot see keep height 0; the planner adds its own margin.
        """
        if not self.has_depth or not self.bounds:
            return None
        self.refresh_scene()
        with self._lock:
            pts = self._points
        if pts is None:
            return None
        x0, x1 = self.bounds["x"]
        y0, y1 = self.bounds["y"]
        nx = int(np.ceil((x1 - x0) / cell))
        ny = int(np.ceil((y1 - y0) / cell))
        ix = np.floor((pts[..., 0] - x0) / cell).astype(int).ravel()
        iy = np.floor((pts[..., 1] - y0) / cell).astype(int).ravel()
        z = pts[..., 2].ravel().copy()
        ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny) & (z > 0.03) & (z < 2.6)
        if self.drone_pose_source:
            try:
                dx, dy, dz = self.drone_pose_source()
                d2 = (pts[..., 0].ravel() - dx) ** 2 + (pts[..., 1].ravel() - dy) ** 2 + (z - dz) ** 2
                ok &= d2 > 0.22 ** 2
            except Exception:
                pass
        grid = np.zeros((nx, ny), np.float32)
        np.maximum.at(grid, (ix[ok], iy[ok]), z[ok].astype(np.float32))
        return grid, (x0, y0), cell

    # ------------------------------------------------------------- projection
    def pixel_to_world(self, u: float, v: float, z_altitude: float = 1.0,
                       img_w: int = None, img_h: int = None) -> Tuple[float, float, float]:
        """
        Maps pixel (u, v) to the floor-plane position (X, Y) in metres and
        passes z_altitude through. If img_w and img_h differ from the
        calibrated resolution, the pixel is rescaled first.
        """
        if img_w is not None and img_h is not None and img_w > 0 and img_h > 0:
            if img_w != self.img_w or img_h != self.img_h:
                u = u * (self.img_w / img_w)
                v = v * (self.img_h / img_h)

        pixel_vec = np.array([u, v, 1.0], dtype=np.float64)
        world_vec = np.dot(self.H, pixel_vec)
        denom = world_vec[2] if abs(world_vec[2]) > 1e-6 else 1.0
        world_x = world_vec[0] / denom
        world_y = world_vec[1] / denom
        return round(float(world_x), 3), round(float(world_y), 3), round(float(z_altitude), 3)

    def world_to_pixel(self, x: float, y: float, z: float = 0.0) -> Optional[Tuple[int, int]]:
        if self.calibration is None:
            return None
        u, v = self.calibration.world_to_pixel(x, y, z)
        return int(round(u)), int(round(v))

    # ------------------------------------------------------------ persistence
    def load(self, path: Path | str) -> None:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        self.img_w = int(data.get("img_w", self.img_w))
        self.img_h = int(data.get("img_h", self.img_h))
        if data.get("H"):
            self.H = np.array(data["H"], dtype=np.float64)
            return
        src = np.float32(data["src_px"])
        dst = np.float32(data["dst_m"])
        self.H, _ = cv2_find_homography(src, dst, float(self.img_w), float(self.img_h))

    def save(self, path: Path | str, src_px: Optional[List] = None, dst_m: Optional[List] = None) -> None:
        payload = {
            "img_w": self.img_w,
            "img_h": self.img_h,
            "H": np.asarray(self.H).tolist(),
            "src_px": src_px or [],
            "dst_m": dst_m or [],
        }
        if self.calibration is not None:
            payload.update(self.calibration.to_dict())
        dest = Path(path)
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(json.dumps(payload, indent=2), encoding="utf-8")


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
