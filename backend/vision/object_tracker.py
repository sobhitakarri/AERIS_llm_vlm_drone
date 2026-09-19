"""Class-agnostic bbox tracking between VLM detections.

The VLM identifies an object once and hands over a box; a correlation-filter
tracker then follows it frame to frame at camera rate. CSRT is used because it
learns the target's appearance from the init box alone, so it works for any
object and any colour, unlike the HSV thresholding it replaces.

CSRT cannot re-acquire a target it has lost, so every loss must be escalated to
a fresh VLM detection. Its own `ok` flag is necessary but not sufficient: it
also returns True while drifting onto background texture. The health checks here
exist to catch that silent-drift case.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np

from backend.core.logger import get_logger
from backend.schemas.perception import BoundingBox

logger = get_logger("ObjectTracker")

# Tracker choice is measured, not assumed: on a 640x480 fixed overhead feed CSRT
# holds ~0.89 IoU at ~57 FPS, KCF ~0.80 at ~210 FPS, and MIL never reports loss
# at all (which makes escalation impossible). Prefer CSRT, fall back to KCF.
_TRACKER_PREFERENCE = ("CSRT", "KCF")

_MAX_SCALE_DRIFT = 3.0      # box may not grow/shrink beyond this factor
_MAX_ASPECT_DRIFT = 2.5     # nor change shape beyond this
_MAX_STEP_FRAC = 0.35       # centre may not jump >35% of frame diagonal per update
_REDETECT_INTERVAL_S = 8.0  # periodic re-detection bounds slow drift

# Appearance agreement (NCC) against the VLM-confirmed crop is reported as the
# track's confidence, but it is a noisy per-frame signal: a couple of pixels of
# box drift on a finely textured object tanks it for one frame. So it only kills
# a track when it stays low for several consecutive frames.
_MIN_TEMPLATE_SCORE = 0.10
_MAX_LOW_APPEARANCE_FRAMES = 5


class TrackStatus(str, Enum):
    """CSRT-layer health for one frame."""
    TRACKING = "TRACKING"
    TEMPORARY_ANOMALY = "TEMPORARY_ANOMALY"
    CONFIRMED_FAILURE = "CONFIRMED_FAILURE"


class TargetLock(str, Enum):
    """Pipeline lock. Only TRACKING may generate target-directed motion."""
    TRACKING = "TRACKING"
    SEEKING = "SEEKING"
    TARGET_LOST = "TARGET_LOST"


@dataclass
class TrackUpdate:
    """Outcome of one tracker step."""
    ok: bool
    bbox: Optional[BoundingBox] = None
    confidence: float = 0.0
    reason: str = ""
    status: TrackStatus = TrackStatus.TRACKING


def _make_tracker():
    """Returns a fresh tracker instance, or None if OpenCV has no usable one."""
    try:
        import cv2
    except ImportError:
        return None
    for name in _TRACKER_PREFERENCE:
        for factory in (
            getattr(cv2, f"Tracker{name}_create", None),
            getattr(getattr(cv2, "legacy", None), f"Tracker{name}_create", None),
        ):
            if factory is None:
                continue
            try:
                return factory()
            except Exception:
                continue
    return None


def _to_xywh(bbox: BoundingBox) -> Tuple[int, int, int, int]:
    return bbox.u_min, bbox.v_min, bbox.u_max - bbox.u_min, bbox.v_max - bbox.v_min


@dataclass
class ObjectTracker:
    """Tracks one object, seeded from a VLM detection."""

    label: str
    _tracker: object = None
    _template: Optional[np.ndarray] = None
    _init_area: float = 0.0
    _init_aspect: float = 1.0
    _last_center: Optional[Tuple[float, float]] = None
    _started_at: float = 0.0
    _frames: int = 0
    _low_appearance: int = 0
    alive: bool = False
    last_reason: str = ""

    def start(self, frame: np.ndarray, bbox: BoundingBox) -> bool:
        """Seeds the tracker from a confirmed detection."""
        tracker = _make_tracker()
        if tracker is None or frame is None:
            self.alive = False
            self.last_reason = "no tracker available"
            return False
        x, y, w, h = _to_xywh(bbox)
        if w <= 1 or h <= 1:
            self.alive = False
            self.last_reason = "degenerate seed box"
            return False
        try:
            tracker.init(frame, (int(x), int(y), int(w), int(h)))
        except Exception as exc:
            self.alive = False
            self.last_reason = f"init failed: {exc}"
            return False

        self._tracker = tracker
        self._template = self._crop(frame, bbox)
        self._init_area = float(w * h)
        self._init_aspect = w / float(max(1, h))
        self._last_center = ((x + w / 2.0), (y + h / 2.0))
        self._started_at = time.time()
        self._frames = 0
        self._low_appearance = 0
        self.alive = True
        self.last_reason = "seeded"
        logger.info("[%s] Tracker seeded at px(%d,%d)-(%d,%d).",
                    self.label, bbox.u_min, bbox.v_min, bbox.u_max, bbox.v_max)
        return True

    def update(self, frame: np.ndarray) -> TrackUpdate:
        if not self.alive or self._tracker is None or frame is None:
            return TrackUpdate(
                ok=False,
                reason=self.last_reason or "not started",
                status=TrackStatus.CONFIRMED_FAILURE,
            )

        try:
            ok, raw = self._tracker.update(frame)
        except Exception as exc:
            return self._fail(f"update raised: {exc}")
        if not ok:
            return self._fail("tracker reported loss")

        x, y, w, h = (float(v) for v in raw)
        if w <= 1 or h <= 1:
            return self._fail("collapsed box")

        fh, fw = frame.shape[:2]
        bbox = BoundingBox(
            u_min=max(0, int(x)), v_min=max(0, int(y)),
            u_max=min(fw, int(x + w)), v_max=min(fh, int(y + h)),
        )
        if bbox.u_max <= bbox.u_min or bbox.v_max <= bbox.v_min:
            return self._fail("box left the frame")

        # Scale and shape sanity: CSRT drifting onto background usually shows up
        # as the box ballooning or going needle-thin before `ok` ever flips.
        area = float(w * h)
        if self._init_area > 0:
            ratio = area / self._init_area
            if ratio > _MAX_SCALE_DRIFT or ratio < 1.0 / _MAX_SCALE_DRIFT:
                return self._fail(f"scale drift x{ratio:.1f}")
        aspect = w / max(1.0, h)
        if self._init_aspect > 0:
            a_ratio = aspect / self._init_aspect
            if a_ratio > _MAX_ASPECT_DRIFT or a_ratio < 1.0 / _MAX_ASPECT_DRIFT:
                return self._fail(f"aspect drift x{a_ratio:.1f}")

        center = (x + w / 2.0, y + h / 2.0)
        if self._last_center is not None:
            step = ((center[0] - self._last_center[0]) ** 2
                    + (center[1] - self._last_center[1]) ** 2) ** 0.5
            if step > _MAX_STEP_FRAC * ((fw ** 2 + fh ** 2) ** 0.5):
                return self._fail(f"implausible jump {step:.0f}px")
        self._last_center = center

        score = self._template_score(frame, bbox)
        if score is not None and score < _MIN_TEMPLATE_SCORE:
            self._low_appearance += 1
            if self._low_appearance >= _MAX_LOW_APPEARANCE_FRAMES:
                return self._fail(
                    f"appearance mismatch {score:.2f} for "
                    f"{self._low_appearance} frames"
                )
            # Temporary anomaly: keep the box, do not kill the track, do not
            # escalate to the VLM. One noisy frame is not a confirmed loss.
            self._frames += 1
            self.last_reason = f"temporary anomaly {score:.2f}"
            return TrackUpdate(
                ok=True,
                bbox=bbox,
                confidence=round(float(score), 2),
                reason=self.last_reason,
                status=TrackStatus.TEMPORARY_ANOMALY,
            )
        else:
            self._low_appearance = 0

        self._frames += 1
        self.last_reason = "tracking"
        return TrackUpdate(
            ok=True,
            bbox=bbox,
            confidence=round(float(score) if score is not None else 0.6, 2),
            reason="tracking",
            status=TrackStatus.TRACKING,
        )

    def stale(self, interval_s: float = _REDETECT_INTERVAL_S) -> bool:
        """True when it is time to re-confirm with the VLM regardless of health."""
        return self.alive and (time.time() - self._started_at) >= interval_s

    def _fail(self, reason: str) -> TrackUpdate:
        if self.alive:
            logger.info("[%s] Track lost after %d frames: %s", self.label, self._frames, reason)
        self.alive = False
        self._tracker = None
        self.last_reason = reason
        return TrackUpdate(ok=False, reason=reason, status=TrackStatus.CONFIRMED_FAILURE)

    @staticmethod
    def _crop(frame: np.ndarray, bbox: BoundingBox) -> Optional[np.ndarray]:
        patch = frame[bbox.v_min:bbox.v_max, bbox.u_min:bbox.u_max]
        if patch.size == 0:
            return None
        try:
            import cv2
            return cv2.resize(patch, (32, 32))
        except Exception:
            return None

    def _template_score(self, frame: np.ndarray, bbox: BoundingBox) -> Optional[float]:
        """Normalised correlation of the current crop against the VLM-confirmed
        one. CSRT does not expose its filter response, so this stands in for the
        confidence the tracker will not give us."""
        if self._template is None:
            return None
        patch = self._crop(frame, bbox)
        if patch is None:
            return None
        try:
            import cv2
            res = cv2.matchTemplate(patch, self._template, cv2.TM_CCOEFF_NORMED)
            return float(res.max())
        except Exception:
            return None


class TrackerManager:
    """Per-label trackers plus the decision of when to spend a VLM call.

    VLM inference is orders of magnitude slower than a tracker step, so
    re-detection is rate limited. Without that limit a target that is genuinely
    absent would trigger a VLM call on every frame of the perception loop.
    """

    def __init__(
        self,
        redetect_cooldown_s: float = 3.0,
        redetect_interval_s: float = _REDETECT_INTERVAL_S,
        max_cooldown_s: float = 60.0,
    ):
        self.redetect_cooldown_s = redetect_cooldown_s
        self.redetect_interval_s = redetect_interval_s
        self.max_cooldown_s = max_cooldown_s
        self._trackers: Dict[str, ObjectTracker] = {}
        self._last_vlm_attempt: Dict[str, float] = {}
        self._misses: Dict[str, int] = {}
        self._lock: Dict[str, TargetLock] = {}

    def get(self, label: str) -> ObjectTracker:
        if label not in self._trackers:
            self._trackers[label] = ObjectTracker(label=label)
        return self._trackers[label]

    def seed(self, label: str, frame: np.ndarray, bbox: BoundingBox) -> bool:
        ok = self.get(label).start(frame, bbox)
        if ok:
            self._misses.pop(label, None)
            self._lock[label] = TargetLock.TRACKING
        return ok

    def note_detection_missed(self, label: str) -> None:
        """A VLM call came back empty. Back off so an object that simply is not
        in the room cannot spend a VLM call every few seconds forever."""
        self._misses[label] = self._misses.get(label, 0) + 1
        self._lock[label] = TargetLock.TARGET_LOST

    def declare_seeking(self, label: str) -> None:
        """CSRT confirmed failure: last pose is stale, ask the VLM next."""
        self._lock[label] = TargetLock.SEEKING

    def declare_tracking(self, label: str) -> None:
        self._lock[label] = TargetLock.TRACKING

    def lock_state(self, label: str) -> TargetLock:
        return self._lock.get(label, TargetLock.SEEKING)

    def is_actionable(self, label: str) -> bool:
        return self.lock_state(label) == TargetLock.TRACKING

    def track(self, label: str, frame: np.ndarray) -> TrackUpdate:
        return self.get(label).update(frame)

    def needs_detection(self, label: str) -> bool:
        """True when the VLM should be asked for this label right now."""
        tracker = self.get(label)
        if tracker.alive and not tracker.stale(self.redetect_interval_s):
            return False
        last = self._last_vlm_attempt.get(label, 0.0)
        misses = self._misses.get(label, 0)
        cooldown = min(self.redetect_cooldown_s * (2 ** misses), self.max_cooldown_s)
        return (time.time() - last) >= cooldown

    def note_detection_attempt(self, label: str) -> None:
        self._last_vlm_attempt[label] = time.time()

    def active_labels(self) -> List[str]:
        return [lbl for lbl, t in self._trackers.items() if t.alive]

    def drop(self, label: str) -> None:
        self._trackers.pop(label, None)
        self._last_vlm_attempt.pop(label, None)
        self._misses.pop(label, None)
        self._lock.pop(label, None)

    def reset(self) -> None:
        self._trackers.clear()
        self._last_vlm_attempt.clear()
        self._misses.clear()
        self._lock.clear()

    def snapshot(self) -> dict:
        return {
            lbl: {
                "alive": t.alive,
                "reason": t.last_reason,
                "lock": self._lock.get(lbl, TargetLock.SEEKING).value,
            }
            for lbl, t in self._trackers.items()
        }
