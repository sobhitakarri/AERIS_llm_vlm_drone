"""
Overhead camera calibration from surveyed ArUco floor markers.

The scene (or the real lab) has fiducials taped to the floor at measured
positions. Detecting them in the overhead image gives:
  * the camera pose in the world frame (solvePnP), and
  * a pixel -> floor homography.
With a depth image the pose turns any pixel into a full 3D world point, so an
object standing on a table is localised at its true position instead of where
its ray would hit the floor.

Marker convention: `pos` is the marker centre on the floor (z = 0); the
marker's top edge faces +y and its right edge faces +x.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from backend.core.logger import get_logger

logger = get_logger("Calibration")

_MIN_MARKERS = 4


@dataclass
class OverheadCalibration:
    K: np.ndarray                      # 3x3 intrinsics
    R: np.ndarray                      # world -> camera rotation
    t: np.ndarray                      # world -> camera translation (3,)
    H: np.ndarray                      # pixel -> floor (x, y) homography
    image_size: Tuple[int, int]        # (width, height)
    marker_ids: List[int] = field(default_factory=list)
    reproj_rms_px: float = 0.0
    floor_rms_m: float = 0.0

    @property
    def camera_position(self) -> np.ndarray:
        return -self.R.T @ self.t

    def pixel_to_floor(self, u: float, v: float) -> Tuple[float, float]:
        w = self.H @ np.array([u, v, 1.0])
        return float(w[0] / w[2]), float(w[1] / w[2])

    def pixel_to_world(self, u: float, v: float, depth_m: float) -> Tuple[float, float, float]:
        """3D world point for a pixel with planar depth (distance along the optical axis)."""
        ray = np.array([(u - self.K[0, 2]) / self.K[0, 0], (v - self.K[1, 2]) / self.K[1, 1], 1.0])
        world = self.R.T @ (ray * depth_m - self.t)
        return float(world[0]), float(world[1]), float(world[2])

    def pixel_at_height(self, u: float, v: float, height_m: float) -> Tuple[float, float]:
        """World (x, y) where the pixel's ray crosses the plane z = height_m."""
        ray_w = self.R.T @ np.array([(u - self.K[0, 2]) / self.K[0, 0], (v - self.K[1, 2]) / self.K[1, 1], 1.0])
        c = self.camera_position
        if abs(ray_w[2]) < 1e-9:
            return self.pixel_to_floor(u, v)
        s = (height_m - c[2]) / ray_w[2]
        p = c + s * ray_w
        return float(p[0]), float(p[1])

    def world_to_pixel(self, x: float, y: float, z: float = 0.0) -> Tuple[float, float]:
        p = self.K @ (self.R @ np.array([x, y, z]) + self.t)
        return float(p[0] / p[2]), float(p[1] / p[2])

    def to_dict(self) -> dict:
        return {
            "img_w": int(self.image_size[0]),
            "img_h": int(self.image_size[1]),
            "K": self.K.tolist(),
            "R": self.R.tolist(),
            "t": self.t.tolist(),
            "H": self.H.tolist(),
            "camera_position": self.camera_position.tolist(),
            "marker_ids": list(self.marker_ids),
            "reproj_rms_px": self.reproj_rms_px,
            "floor_rms_m": self.floor_rms_m,
        }

    def save(self, path: Path | str) -> None:
        dest = Path(path)
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(json.dumps(self.to_dict(), indent=2), encoding="utf-8")

    @classmethod
    def load(cls, path: Path | str) -> Optional["OverheadCalibration"]:
        try:
            d = json.loads(Path(path).read_text(encoding="utf-8"))
            return cls(
                K=np.array(d["K"], float), R=np.array(d["R"], float), t=np.array(d["t"], float),
                H=np.array(d["H"], float), image_size=(int(d["img_w"]), int(d["img_h"])),
                marker_ids=list(d.get("marker_ids", [])),
                reproj_rms_px=float(d.get("reproj_rms_px", 0.0)),
                floor_rms_m=float(d.get("floor_rms_m", 0.0)),
            )
        except (OSError, ValueError, KeyError):
            return None


def intrinsics_from_info(info: dict, image_size: Tuple[int, int]) -> np.ndarray:
    """Pinhole K from a camera description; falls back to the horizontal FOV."""
    w, h = image_size
    if info.get("fx"):
        sx = w / float(info.get("width", w))
        sy = h / float(info.get("height", h))
        return np.array([[info["fx"] * sx, 0, info.get("cx", w / 2) * sx],
                         [0, info.get("fy", info["fx"]) * sy, info.get("cy", h / 2) * sy],
                         [0, 0, 1]], float)
    hfov = float(info.get("hfov_deg", 90.0))
    f = (w / 2) / np.tan(np.radians(hfov / 2))
    return np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], float)


def detect_markers(frame: np.ndarray, dictionary_name: str = "DICT_4X4_50") -> Dict[int, np.ndarray]:
    """Detected marker corners keyed by id: 4x2 arrays, ordered TL, TR, BR, BL."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2).astype(float) for c, i in zip(corners, ids.ravel())}


def _marker_corners_world(centre, size: float) -> np.ndarray:
    cx, cy = float(centre[0]), float(centre[1])
    h = size / 2.0
    return np.array([[cx - h, cy + h, 0], [cx + h, cy + h, 0], [cx + h, cy - h, 0], [cx - h, cy - h, 0]], float)


def calibrate_overhead(frame: np.ndarray, markers: dict, camera_info: dict) -> Optional[OverheadCalibration]:
    """Estimate the overhead camera pose from one frame. None if too few markers are visible."""
    if frame is None or not markers or not markers.get("items"):
        return None
    h, w = frame.shape[:2]
    K = intrinsics_from_info(camera_info or {}, (w, h))
    size = float(markers.get("size_m", 0.3))
    surveyed = {int(m["id"]): m["pos"] for m in markers["items"]}
    found = detect_markers(frame, markers.get("dictionary", "DICT_4X4_50"))
    ids = sorted(i for i in found if i in surveyed)
    if len(ids) < _MIN_MARKERS:
        logger.warning(f"[Calibration] Only {len(ids)} of {len(surveyed)} markers visible; need {_MIN_MARKERS}.")
        return None

    centres_w = np.array([[surveyed[i][0], surveyed[i][1], 0.0] for i in ids], float)
    centres_px = np.array([found[i].mean(axis=0) for i in ids], float)
    ok, rvec, tvec = cv2.solvePnP(centres_w, centres_px, K, None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None

    # Use all four corners per marker when they agree with the centre-only
    # pose; that also confirms the surveyed orientation convention.
    corners_w = np.vstack([_marker_corners_world(surveyed[i], size) for i in ids])
    corners_px = np.vstack([found[i] for i in ids])
    proj, _ = cv2.projectPoints(corners_w, rvec, tvec, K, None)
    if float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corners_px) ** 2, axis=1)))) < 6.0:
        ok2, rvec2, tvec2 = cv2.solvePnP(corners_w, corners_px, K, None, rvec, tvec, True, cv2.SOLVEPNP_ITERATIVE)
        if ok2:
            rvec, tvec = rvec2, tvec2
            pts_w, pts_px = corners_w, corners_px
        else:
            pts_w, pts_px = centres_w, centres_px
    else:
        pts_w, pts_px = centres_w, centres_px

    proj, _ = cv2.projectPoints(pts_w, rvec, tvec, K, None)
    reproj = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - pts_px) ** 2, axis=1))))
    R, _ = cv2.Rodrigues(rvec)
    Hm, _ = cv2.findHomography(pts_px, pts_w[:, :2])
    if Hm is None:
        return None
    cal = OverheadCalibration(K=K, R=R, t=tvec.reshape(3), H=Hm, image_size=(w, h), marker_ids=ids, reproj_rms_px=reproj)
    floor = np.array([cal.pixel_to_floor(u, v) for u, v in centres_px])
    cal.floor_rms_m = float(np.sqrt(np.mean(np.sum((floor - centres_w[:, :2]) ** 2, axis=1))))
    logger.info(
        f"[Calibration] Overhead camera calibrated from markers {ids}: "
        f"reprojection {reproj:.2f} px, floor RMS {cal.floor_rms_m * 100:.1f} cm, "
        f"camera at {np.round(cal.camera_position, 2).tolist()}"
    )
    return cal
