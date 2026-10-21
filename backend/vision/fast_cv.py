"""
Fast Perception Stack (OpenCV + Object Bounding Box Tracking).
"""
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from typing import Any, Dict, List, Optional
import numpy as np

from backend.schemas.perception import BoundingBox, DetectedObject, PerceptionResult
from backend.vision.object_tracker import TargetLock, TrackerManager, TrackStatus
from backend.vision.spatial_grounding import SpatialGrounding
from backend.runtime.state_manager import StateManager
from backend.core import run_mode, trace
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
        async_detect: bool = False,
        clearance_m: float = 0.35,
        z_max: float = 1.5,
    ):
        # async_detect: VLM calls run on a single worker thread and the loop
        # picks the answer up on a later frame. Inline calls stall tracking
        # for seconds per target, and with several targets every track goes
        # stale before the loop comes round again.
        self.async_detect = async_detect
        self.clearance_m = clearance_m
        self.z_max = z_max
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vlm-detect") if async_detect else None
        self._pending: Dict[str, Future] = {}
        # label -> (x, y, top_z) of objects being followed in the depth map.
        self._geo: Dict[str, tuple] = {}
        self._lock = threading.RLock()
        self.rejected_ungrounded = 0
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
        # Under the lock: a frame being processed must not write a dropped
        # target back into the state store after it has been removed.
        with self._lock:
            for gone in set(self.active_targets) - set(new):
                self.trackers.drop(gone)
                self._pending.pop(gone, None)
                self._geo.pop(gone, None)
                if self.state_manager:
                    self.state_manager.remove_object(gone)
            self.active_targets = new
        logger.info("Perception targets set to %s", new or "[]")

    def _ground(self, det: DetectedObject, img_w: int, img_h: int, z_altitude: float) -> Optional[DetectedObject]:
        """Fill in world coordinates. Returns None when a depth camera shows
        there is no physical object under the box."""
        if getattr(self.grounding, "calibrated", False):
            x, y, top_z, match = self.grounding.ground_bbox(det.bbox, img_w, img_h, verify=self._depth_verified())
            if x is None:
                self.rejected_ungrounded += 1
                return None
            det.world_x, det.world_y = round(x, 3), round(y, 3)
            det.top_z = round(float(top_z), 3)
            # Fly-to altitude: clear of the object, inside the envelope.
            det.world_z = round(min(self.z_max, max(z_altitude, top_z + self.clearance_m)), 3)
            if match is not None:
                det.bbox = match.bbox
                det.support_z = round(match.support_z, 3)
                det.size_m = [round(match.size_m[0], 3), round(match.size_m[1], 3)]
                det.grounding = "depth"
            else:
                det.grounding = "floor"
        else:
            u, v = det.bbox.center_pixel
            wx, wy, wz = self.grounding.pixel_to_world(
                u, v, z_altitude=z_altitude, img_w=img_w, img_h=img_h
            )
            det.world_x, det.world_y, det.world_z = wx, wy, wz
            det.grounding = "nominal"
        if self.state_manager:
            self.state_manager.update_object(det)
        return det

    def _detect(self, frame: np.ndarray, label: str, w: int, h: int):
        """Returns (ready, detection). In async mode the first call submits
        the job and later calls collect it; `ready` is False while it runs."""
        if not self.async_detect:
            return True, self._vlm_detect(frame, label, w, h)
        fut = self._pending.get(label)
        if fut is None:
            if any(not f.done() for f in self._pending.values()):
                return False, None                 # one detection in flight at a time
            self.trackers.note_detection_attempt(label)
            self._pending[label] = self._worker.submit(self._vlm_detect, frame.copy(), label, w, h)
            return False, None
        if not fut.done():
            return False, None
        self._pending.pop(label, None)
        try:
            return True, fut.result()
        except Exception as exc:
            logger.warning("VLM detection failed for '%s': %s", label, exc)
            return True, None

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

        depth_mode = self._depth_verified()
        for label in targets:
            tracker = self.trackers.get(label)

            if depth_mode and label in self._geo:
                # With a calibrated depth camera the object is followed as a
                # physical thing in the height map, not as a patch of pixels:
                # it cannot drift onto background texture.
                followed = self._follow_in_depth(label, z_altitude)
                if followed is not None:
                    self.trackers.declare_tracking(label)
                    out.append(followed)
                    continue
                self._geo.pop(label, None)
                self.trackers.drop(label)
                self.trackers.declare_seeking(label)
                if self.state_manager:
                    self.state_manager.mark_lost(label)
                tracker = self.trackers.get(label)

            elif tracker.alive and not tracker.stale(self.trackers.redetect_interval_s):
                upd = tracker.update(frame)
                if upd.ok and upd.bbox:
                    grounded = None
                    if upd.status is TrackStatus.TRACKING:
                        grounded = self._ground(
                            DetectedObject(label=label, bbox=upd.bbox, confidence=upd.confidence),
                            w, h, z_altitude,
                        )
                    if grounded is not None:
                        self.trackers.declare_tracking(label)
                        out.append(grounded)
                    else:
                        # TEMPORARY_ANOMALY: the box is reported for the UI but is
                        # not grounded and not written to StateManager, so a frame
                        # we already flagged as suspect cannot generate motion.
                        # Only TRACKING may drive a target-directed command.
                        out.append(DetectedObject(
                            label=label, bbox=upd.bbox,
                            confidence=upd.confidence, actionable=False,
                        ))
                    continue
                # Confirmed CSRT failure: last world pose is immediately stale.
                # SEEKING until the VLM answers; TARGET_LOST if it also fails.
                self.trackers.declare_seeking(label)
                if self.state_manager:
                    self.state_manager.mark_lost(label)

            waiting = label in self._pending
            if not self._can_detect() or not (waiting or self.trackers.needs_detection(label)):
                if self.trackers.lock_state(label) in (TargetLock.SEEKING, TargetLock.TARGET_LOST):
                    if label not in lost:
                        lost.append(label)
                continue

            if not self.async_detect:
                self.trackers.note_detection_attempt(label)
            ready, det = self._detect(frame, label, w, h)
            if not ready:
                if self.trackers.lock_state(label) in (TargetLock.SEEKING, TargetLock.TARGET_LOST):
                    if label not in lost:
                        lost.append(label)
                continue
            # A box with nothing physical under it counts as a miss, which is
            # what lets TARGET_LOST fire against a model that never refuses.
            grounded = self._ground(det, w, h, z_altitude) if det is not None else None
            if grounded is None:
                self.trackers.note_detection_missed(label)
                if self.state_manager:
                    self.state_manager.mark_lost(label)
                lost.append(label)
                continue
            if depth_mode:
                self._geo[label] = (grounded.world_x, grounded.world_y, grounded.top_z or 0.0)
                self.trackers.declare_tracking(label)
                out.append(grounded)
            elif self.trackers.seed(label, frame, det.bbox):
                out.append(grounded)

        return out, lost

    def _follow_in_depth(self, label: str, z_altitude: float) -> Optional[DetectedObject]:
        from backend.vision.scene_proposals import nearest_proposal
        x, y, top = self._geo[label]
        match = nearest_proposal(x, y, top, self.grounding.refresh_scene())
        if match is None:
            return None
        self._geo[label] = (match.x, match.y, match.top_z)
        det = DetectedObject(
            label=label, bbox=match.bbox, confidence=0.9,
            world_x=round(match.x, 3), world_y=round(match.y, 3), top_z=round(match.top_z, 3),
            world_z=round(min(self.z_max, max(z_altitude, match.top_z + self.clearance_m)), 3),
            support_z=round(match.support_z, 3), size_m=[round(match.size_m[0], 3), round(match.size_m[1], 3)],
            grounding="depth",
        )
        if self.state_manager:
            self.state_manager.update_object(det)
        return det

    def locate(self, frame: Optional[np.ndarray], label: str, z_altitude: float = 0.6) -> Optional[DetectedObject]:
        """One-shot detect + ground for planning. Blocks for one VLM call.

        On success the tracker for `label` is seeded, so the background loop
        keeps following the object without asking the model again."""
        if frame is None or not label or not self._can_detect():
            return None
        h, w = frame.shape[:2]
        det = self._vlm_detect(frame, label, w, h)
        if det is None:
            return None
        with self._lock:
            grounded = self._ground(det, w, h, z_altitude)
            if grounded is None:
                self.trackers.note_detection_missed(label)
                return None
            self._pending.pop(label, None)
            if self._depth_verified():
                self._geo[label] = (grounded.world_x, grounded.world_y, grounded.top_z or 0.0)
                self.trackers.declare_tracking(label)
            else:
                self.trackers.seed(label, frame, det.bbox)
        self._save_snapshot(frame, det, grounded)
        return grounded

    def _save_snapshot(self, frame: np.ndarray, det: DetectedObject, grounded: DetectedObject) -> None:
        """Audit image for each planning-time detection: the model's box and
        where it was grounded."""
        try:
            import cv2
            from pathlib import Path
            out_dir = Path(__file__).resolve().parents[2] / "cache" / "identified"
            out_dir.mkdir(parents=True, exist_ok=True)
            img = frame.copy()
            b = det.bbox
            cv2.rectangle(img, (b.u_min, b.v_min), (b.u_max, b.v_max), (0, 200, 255), 2)
            tag = f"{grounded.label}: ({grounded.world_x:+.2f}, {grounded.world_y:+.2f}) m [{grounded.grounding}]"
            cv2.putText(img, tag, (b.u_min, max(14, b.v_min - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1, cv2.LINE_AA)
            name = "".join(c if c.isalnum() else "_" for c in grounded.label)[:40]
            cv2.imwrite(str(out_dir / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.jpg"), img)
        except Exception as exc:
            logger.debug("Snapshot not saved: %s", exc)

    def adopt(self, det: DetectedObject) -> None:
        """Make an already-located object the loop's only target, keeping its
        track so no new detection is requested."""
        with self._lock:
            keep_geo = self._geo.get(det.label)
            if self.state_manager:
                for old in self.active_targets:
                    if old != det.label:
                        self.state_manager.remove_object(old)
            self.active_targets = [det.label]
            self._geo = {det.label: keep_geo} if keep_geo else {}
            self._pending.clear()
            if self.state_manager:
                self.state_manager.update_object(det)
        logger.info("Perception targets set to %s", self.active_targets)

    def _can_detect(self) -> bool:
        return bool(self.vlm) and hasattr(self.vlm, "resolve_target") and run_mode.current().use_vlm

    def _depth_verified(self) -> bool:
        """Boxes are checked against the depth map unless the run mode turns it off."""
        return bool(getattr(self.grounding, "has_depth", False)) and run_mode.current().depth_check

    def _vlm_detect(
        self, frame: np.ndarray, label: str, w: int, h: int
    ) -> Optional[DetectedObject]:
        t0 = time.perf_counter()
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
            trace.model_call("vlm", self.vlm, time.perf_counter() - t0, False, target=label, error=str(exc)[:300])
            return None
        trace.model_call("vlm", self.vlm, time.perf_counter() - t0, det is not None and bool(det.bbox), target=label,
                         bbox=det.bbox.model_dump() if det is not None and det.bbox else None)
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
            with self._lock:
                live, lost = self._track_and_detect(frame, wanted, z_altitude)
            objects.extend(live)

        # 2. No VLM configured: fall back to the colour heuristic. Not when a
        # run mode switched the VLM off: that baseline has no perception at all.
        if not objects and frame is not None and not self._can_detect() and run_mode.current().use_vlm:
            objects.extend(self._color_track(frame, targets, z_altitude))

        # 3. Only actionable objects. Last-known coordinates after TARGET_LOST
        # must not re-enter the perception result and generate motion.
        if not objects and self.state_manager:
            objects = [
                obj for obj in self.state_manager.get_actionable_objects()
                if obj.label not in lost
            ]

        return PerceptionResult(timestamp=time.time(), objects=objects, lost_labels=lost)

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
