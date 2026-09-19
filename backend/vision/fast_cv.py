"""
Fast Perception Stack (OpenCV + Object Bounding Box Tracking).
"""
import time
from typing import Any, List, Optional
import numpy as np

from backend.schemas.perception import BoundingBox, DetectedObject, PerceptionResult
from backend.vision.object_tracker import TargetLock, TrackerManager
from backend.vision.spatial_grounding import SpatialGrounding
from backend.runtime.state_manager import StateManager
from backend.core.logger import get_logger

logger = get_logger("FastPerception")

# ~25x25 px. The old 80 px gate latched onto specular noise and produced
# phantom targets that jittered enough to trigger replans.
_MIN_BLOB_AREA = 600


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
        trackers: Optional[TrackerManager] = None,
    ):
        self.grounding = grounding or SpatialGrounding()
        self.vlm = vlm
        self.state_manager = state_manager
        self.trackers = trackers or TrackerManager()
        # Labels the current mission cares about. Set from the command router so
        # the loop can track arbitrary objects rather than a hardcoded list.
        self.active_targets: List[str] = []

    def set_targets(self, labels: Optional[List[str]]) -> None:
        """Points the loop at a new set of objects and drops stale trackers."""
        new = [l for l in (labels or []) if l]
        if new == self.active_targets:
            return
        for gone in set(self.active_targets) - set(new):
            self.trackers.drop(gone)
        self.active_targets = new
        logger.info("Perception targets set to %s", new or "[]")

    def _ground(self, det: DetectedObject, img_w: int, img_h: int, z_altitude: float) -> DetectedObject:
        u, v = det.bbox.center_pixel
        wx, wy, wz = self.grounding.pixel_to_world(
            u, v, z_altitude=z_altitude, img_w=img_w, img_h=img_h
        )
        det.world_x, det.world_y, det.world_z = wx, wy, wz
        if self.state_manager:
            self.state_manager.update_object(det)
        return det

    def _track_and_detect(
        self,
        frame: np.ndarray,
        targets: List[str],
        z_altitude: float,
    ) -> tuple:
        """VLM identifies once, tracker follows. The VLM is consulted only when a
        label has no live track, or when a live track has gone stale.

        Returns (live_objects, lost_labels). Lost labels must not be filled in
        from last-known world coordinates."""
        h, w = frame.shape[:2]
        out: List[DetectedObject] = []
        lost: List[str] = []

        for label in targets:
            tracker = self.trackers.get(label)

            if tracker.alive and not tracker.stale(self.trackers.redetect_interval_s):
                upd = tracker.update(frame)
                if upd.ok and upd.bbox:
                    self.trackers.declare_tracking(label)
                    out.append(self._ground(
                        DetectedObject(label=label, bbox=upd.bbox, confidence=upd.confidence),
                        w, h, z_altitude,
                    ))
                    continue
                # Confirmed CSRT failure: last world pose is immediately stale.
                # SEEKING until the VLM answers; TARGET_LOST if it also fails.
                self.trackers.declare_seeking(label)
                if self.state_manager:
                    self.state_manager.mark_lost(label)

            if not self._can_detect() or not self.trackers.needs_detection(label):
                if self.trackers.lock_state(label) in (TargetLock.SEEKING, TargetLock.TARGET_LOST):
                    if label not in lost:
                        lost.append(label)
                continue

            self.trackers.note_detection_attempt(label)
            det = self._vlm_detect(frame, label, w, h)
            if det is None:
                self.trackers.note_detection_missed(label)
                if self.state_manager:
                    self.state_manager.mark_lost(label)
                lost.append(label)
                continue
            if self.trackers.seed(label, frame, det.bbox):
                out.append(self._ground(det, w, h, z_altitude))

        return out, lost

    def _can_detect(self) -> bool:
        return bool(self.vlm) and hasattr(self.vlm, "resolve_target")

    def _vlm_detect(
        self, frame: np.ndarray, label: str, w: int, h: int
    ) -> Optional[DetectedObject]:
        try:
            import cv2
            ok, enc = cv2.imencode(".jpg", frame)
            if not ok:
                return None
            det = self.vlm.resolve_target(
                enc.tobytes(), label, image_width=w, image_height=h
            )
        except Exception as exc:
            logger.warning("VLM detection failed for '%s': %s", label, exc)
            return None
        if det is None or not det.bbox:
            return None
        det.label = label
        return det

    def process_frame(
        self,
        frame: Optional[np.ndarray] = None,
        targets: Optional[List[str]] = None,
        z_altitude: float = 1.0,
    ) -> PerceptionResult:
        """
        Produces detected objects with grounded world coordinates.

        Primary path is VLM-identify-once then CSRT-track-continuously, which
        works for arbitrary objects of any colour. The HSV colour tracker is a
        fallback for when no VLM is configured, and only knows red/blue.
        """
        objects: List[DetectedObject] = []
        lost: List[str] = []
        wanted = [t for t in (targets or self.active_targets) if t]

        # 1. Track what we already found; ask the VLM only for what we have lost.
        if frame is not None and wanted and self._can_detect():
            live, lost = self._track_and_detect(frame, wanted, z_altitude)
            objects.extend(live)

        # 2. No VLM configured: fall back to the colour heuristic.
        if not objects and frame is not None and not self._can_detect():
            objects.extend(self._color_track(frame, targets, z_altitude))

        # 3. Only actionable objects. Last-known coordinates after TARGET_LOST
        # must not re-enter the perception result and generate motion.
        if not objects and self.state_manager:
            objects = [
                obj for obj in self.state_manager.get_actionable_objects()
                if obj.label not in lost
            ]

        for obj in objects:
            if obj.bbox and obj.actionable and (obj.world_x is None or obj.world_y is None):
                u_center, v_center = obj.bbox.center_pixel
                world_x, world_y, world_z = self.grounding.pixel_to_world(u_center, v_center, z_altitude=z_altitude)
                obj.world_x = world_x
                obj.world_y = world_y
                obj.world_z = world_z

        return PerceptionResult(
            timestamp=time.time(),
            objects=objects,
            lost_labels=lost,
            drone_pixel=(320, 240),
            drone_world=(0.0, 0.0),
        )

    def _color_track(
        self,
        frame: np.ndarray,
        targets: Optional[List[str]],
        z_altitude: float,
    ) -> List[DetectedObject]:
        try:
            import cv2
        except ImportError:
            return []

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, w = frame.shape[:2]
        wanted = [t.lower() for t in (targets or ["red_bottle", "blue_cube", "bottle", "cube"])]
        found: List[DetectedObject] = []

        ranges = []
        if any("red" in t or "bottle" in t for t in wanted):
            ranges.append(("red_bottle", (np.array([0, 90, 70]), np.array([10, 255, 255]))))
            ranges.append(("red_bottle", (np.array([170, 90, 70]), np.array([180, 255, 255]))))
        if any("blue" in t or "cube" in t for t in wanted):
            ranges.append(("blue_cube", (np.array([95, 80, 50]), np.array([130, 255, 255]))))

        best: dict = {}
        for label, (lo, hi) in ranges:
            mask = cv2.inRange(hsv, lo, hi)
            mask = cv2.medianBlur(mask, 5)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area < _MIN_BLOB_AREA:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            prev = best.get(label)
            if prev and prev[0] >= area:
                continue
            best[label] = (area, x, y, bw, bh)

        for label, (area, x, y, bw, bh) in best.items():
            bbox = BoundingBox(u_min=int(x), v_min=int(y), u_max=int(x + bw), v_max=int(y + bh))
            u, v = bbox.center_pixel
            wx, wy, wz = self.grounding.pixel_to_world(u, v, z_altitude=z_altitude, img_w=w, img_h=h)
            det = DetectedObject(
                label=label,
                bbox=bbox,
                confidence=min(0.95, 0.4 + area / 8000.0),
                world_x=wx,
                world_y=wy,
                world_z=wz,
            )
            found.append(det)
            if self.state_manager:
                self.state_manager.update_object(det)
        return found
