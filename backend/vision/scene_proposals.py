"""
Class-agnostic object proposals from the overhead RGB-D camera.

The calibrated camera pose turns the depth image into a height-above-floor
map. Anything that stands out of the floor is a physical object, whatever it
is, so segmenting that map gives object candidates with metric position and
height before any model has been asked what they are.

This is what makes target grounding verifiable: a vision-language model may
only name one of these candidates. A box it draws on bare floor matches no
candidate and is rejected, instead of being flown to.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np

from backend.schemas.perception import BoundingBox
from backend.vision.calibration import OverheadCalibration

_MIN_HEIGHT_M = 0.04          # below this it is floor, tape or a marker
_STEP_M = 0.045               # height step between neighbouring cells that separates two surfaces
_CELL_M = 0.02                # bird's-eye grid resolution
_MIN_CELLS = 5                # smallest surface kept (about 20 cm^2)


@dataclass
class Proposal:
    index: int
    bbox: BoundingBox
    area_px: int
    x: float                  # world centre of the top surface (m)
    y: float
    top_z: float              # height of the top surface above the floor (m)
    size_m: Tuple[float, float]   # top-surface extent along world x, y
    support_z: float = 0.0    # height of the surface it stands on

    @property
    def stands_on_floor(self) -> bool:
        return self.support_z < _MIN_HEIGHT_M

    def as_dict(self) -> dict:
        return {
            "index": self.index, "bbox": self.bbox.model_dump(), "area_px": self.area_px,
            "x": round(self.x, 3), "y": round(self.y, 3), "top_z": round(self.top_z, 3),
            "size_m": [round(self.size_m[0], 3), round(self.size_m[1], 3)],
            "support_z": round(self.support_z, 3),
        }


class SceneGeometry:
    """Per-pixel world coordinates for a calibrated overhead depth camera."""

    def __init__(self, cal: OverheadCalibration):
        self.cal = cal
        w, h = cal.image_size
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        rays_cam = np.stack([(u - cal.K[0, 2]) / cal.K[0, 0], (v - cal.K[1, 2]) / cal.K[1, 1], np.ones_like(u)], axis=-1)
        self._rays_world = (rays_cam @ cal.R).astype(np.float32)   # row-vector form of R^T @ ray
        self._centre = cal.camera_position.astype(np.float32)

    def world_points(self, depth: np.ndarray) -> np.ndarray:
        """H x W x 3 world coordinates for a planar depth image in metres."""
        if depth.shape[:2] != self._rays_world.shape[:2]:
            depth = cv2.resize(depth, (self._rays_world.shape[1], self._rays_world.shape[0]), interpolation=cv2.INTER_NEAREST)
        return self._centre + self._rays_world * depth[..., None].astype(np.float32)

    def height_map(self, depth: np.ndarray) -> np.ndarray:
        return self.world_points(depth)[..., 2]


def find_proposals(
    depth: np.ndarray,
    geometry: SceneGeometry,
    bounds: Optional[dict] = None,
    exclude: Optional[List[Tuple[float, float, float, float]]] = None,
    max_height: float = 2.4,
    points: Optional[np.ndarray] = None,
) -> List[Proposal]:
    """Segment the scene into object surfaces.

    The depth image is re-projected into a bird's-eye grid holding the
    tallest point per cell. Seen from the side an object blends into what it
    stands on through its visible flank; seen from above it is a clean step,
    so neighbouring cells are joined only when their heights agree.

    bounds  : {"x": [lo, hi], "y": [lo, hi]} region of interest in world metres
    exclude : (x, y, z, radius) spheres to ignore, e.g. the drone itself
    """
    pts = geometry.world_points(depth) if points is None else points
    px, py, pz = pts[..., 0], pts[..., 1], pts[..., 2]
    if bounds is None:
        bounds = {"x": [float(np.nanmin(px)), float(np.nanmax(px))], "y": [float(np.nanmin(py)), float(np.nanmax(py))]}
    x0, x1 = bounds["x"]
    y0, y1 = bounds["y"]
    valid = (depth > 0.05) & (pz > -0.05) & (pz < max_height) & (px > x0) & (px < x1) & (py > y0) & (py < y1)
    for ex, ey, ez, er in exclude or []:
        valid &= (px - ex) ** 2 + (py - ey) ** 2 + (pz - ez) ** 2 > er ** 2

    nx = int(np.ceil((x1 - x0) / _CELL_M))
    ny = int(np.ceil((y1 - y0) / _CELL_M))
    ix = np.clip(((px - x0) / _CELL_M).astype(np.int32), 0, nx - 1)
    iy = np.clip(((py - y0) / _CELL_M).astype(np.int32), 0, ny - 1)
    top = np.full((nx, ny), -1.0, np.float32)
    np.maximum.at(top, (ix[valid], iy[valid]), pz[valid].astype(np.float32))
    seen = top > -0.5
    # Close pin-holes left where the pixel footprint is wider than a cell.
    filled = cv2.dilate(top, np.ones((3, 3), np.uint8))
    top = np.where(seen, top, np.where(cv2.morphologyEx(seen.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)) > 0, filled, -1.0))
    seen = top > -0.5
    obj = seen & (top > _MIN_HEIGHT_M)

    # Union neighbouring object cells whose heights agree. Cells sit on the
    # even coordinates of a lattice and the links between them on the odd
    # ones, so ordinary connected-component labelling does the union-find.
    lattice = np.zeros((2 * nx - 1, 2 * ny - 1), np.uint8)
    lattice[::2, ::2] = obj
    lattice[1::2, ::2] = obj[:-1, :] & obj[1:, :] & (np.abs(top[:-1, :] - top[1:, :]) < _STEP_M)
    lattice[::2, 1::2] = obj[:, :-1] & obj[:, 1:] & (np.abs(top[:, :-1] - top[:, 1:]) < _STEP_M)
    count, lat_labels = cv2.connectedComponents(lattice, connectivity=4)
    labels = lat_labels[::2, ::2]

    cells = np.bincount(labels.ravel(), minlength=count)
    keep = [i for i in range(1, count) if cells[i] >= _MIN_CELLS]
    if not keep:
        return []

    # Image-space box of each surface: every pixel that lands in its cells
    # and is not far below its top (so the visible flank is included).
    pix_label = np.where(valid, labels[ix, iy], 0)
    pix_label = np.where(pz > top[ix, iy] - 0.5, pix_label, 0)
    vv, uu = np.nonzero(pix_label)
    ll = pix_label[vv, uu]
    u_min = np.full(count, 10 ** 6); v_min = np.full(count, 10 ** 6)
    u_max = np.full(count, -1); v_max = np.full(count, -1)
    np.minimum.at(u_min, ll, uu); np.minimum.at(v_min, ll, vv)
    np.maximum.at(u_max, ll, uu); np.maximum.at(v_max, ll, vv)
    area_px = np.bincount(ll, minlength=count)

    gx, gy = np.meshgrid(np.arange(nx), np.arange(ny), indexing="ij")
    kernel = np.ones((7, 7), np.uint8)
    proposals: List[Proposal] = []
    for i in keep:
        if u_max[i] < 0:
            continue
        mask = labels == i
        heights = top[mask]
        top_z = float(np.percentile(heights, 92))
        cx = x0 + (float(gx[mask].mean()) + 0.5) * _CELL_M
        cy = y0 + (float(gy[mask].mean()) + 0.5) * _CELL_M
        sx = (int(gx[mask].max()) - int(gx[mask].min()) + 1) * _CELL_M
        sy = (int(gy[mask].max()) - int(gy[mask].min()) + 1) * _CELL_M
        # What it stands on: lower surfaces seen in a ring around it.
        ring = (cv2.dilate(mask.astype(np.uint8), kernel) > 0) & ~mask & seen & (top < top_z - _STEP_M)
        support = float(np.percentile(top[ring], 75)) if int(ring.sum()) >= 6 else 0.0
        proposals.append(Proposal(
            index=len(proposals),
            bbox=BoundingBox(u_min=int(u_min[i]), v_min=int(v_min[i]), u_max=int(u_max[i]) + 1, v_max=int(v_max[i]) + 1),
            area_px=int(area_px[i]),
            x=cx, y=cy, top_z=top_z, size_m=(sx, sy),
            support_z=support if support > _MIN_HEIGHT_M else 0.0,
        ))
    return proposals


def _iou(a: BoundingBox, b: BoundingBox) -> float:
    iw = min(a.u_max, b.u_max) - max(a.u_min, b.u_min)
    ih = min(a.v_max, b.v_max) - max(a.v_min, b.v_min)
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    union = (a.u_max - a.u_min) * (a.v_max - a.v_min) + (b.u_max - b.u_min) * (b.v_max - b.v_min) - inter
    return inter / max(union, 1)


def match_proposal(bbox: BoundingBox, proposals: List[Proposal], min_score: float = 0.10) -> Optional[Proposal]:
    """The proposal a detector's box refers to, or None.

    Scored by box similarity (IoU) and how close the centres are, so a box
    drawn around a bottle selects the bottle and not the table whose much
    larger box merely contains it. A box over bare floor overlaps nothing
    and returns None.
    """
    best, best_score = None, 0.0
    cu, cv_ = bbox.center_pixel
    diag = float(np.hypot(bbox.u_max - bbox.u_min, bbox.v_max - bbox.v_min)) + 1.0
    for p in proposals:
        iou = _iou(bbox, p.bbox)
        if iou <= 0.0:
            continue
        pu, pv = p.bbox.center_pixel
        closeness = max(0.0, 1.0 - float(np.hypot(pu - cu, pv - cv_)) / diag)
        score = 0.7 * iou + 0.3 * closeness * min(1.0, iou * 8)
        if score > best_score:
            best, best_score = p, score
    return best if best_score >= min_score else None


def nearest_proposal(x: float, y: float, top_z: float, proposals: List[Proposal],
                     max_dist: float = 0.50, max_dz: float = 0.08) -> Optional[Proposal]:
    """Frame-to-frame association: the proposal that continues a known object."""
    best, best_d = None, max_dist
    for p in proposals:
        if abs(p.top_z - top_z) > max_dz:
            continue
        d = float(np.hypot(p.x - x, p.y - y))
        if d < best_d:
            best, best_d = p, d
    return best


def annotate(frame: np.ndarray, proposals: List[Proposal]) -> np.ndarray:
    """Numbered proposal boxes on a copy of the frame (set-of-mark prompt image)."""
    out = frame.copy()
    for p in proposals:
        b = p.bbox
        cv2.rectangle(out, (b.u_min, b.v_min), (b.u_max, b.v_max), (0, 255, 255), 1)
        tag = str(p.index)
        (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(out, (b.u_min, b.v_min - th - 5), (b.u_min + tw + 4, b.v_min), (0, 255, 255), -1)
        cv2.putText(out, tag, (b.u_min + 2, b.v_min - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
    return out
