"""
ReplanManager — Dynamic Mission Replanning via EventBus Subscriptions.

Re-runs UAV-VLA style object-search + action update when a target drifts.
"""
from __future__ import annotations

import time
from typing import Any, Callable, Dict, Optional

from backend.runtime.event_bus import EventBus, EVENT_OBJECT_DETECTED
from backend.core.logger import get_logger

logger = get_logger("ReplanManager")

_REPLAN_DISTANCE_THRESHOLD = 0.3
_COOLDOWN_S = 2.0
_MAX_REPLANS = 3
_CONFIRM_FRAMES = 3


class ReplanManager:
    def __init__(
        self,
        event_bus: EventBus,
        replan_callback: Optional[Callable[[str, Dict], None]] = None,
        distance_threshold: float = _REPLAN_DISTANCE_THRESHOLD,
        cooldown_s: float = _COOLDOWN_S,
        max_replans: int = _MAX_REPLANS,
        mission_active: Optional[Callable[[], bool]] = None,
        confirm_frames: int = _CONFIRM_FRAMES,
    ):
        self.event_bus = event_bus
        self.replan_callback = replan_callback
        self.distance_threshold = distance_threshold
        self.cooldown_s = cooldown_s
        self.max_replans = max_replans
        self.mission_active = mission_active
        self.confirm_frames = confirm_frames
        # A detector that jitters between two blobs would otherwise look like a
        # target repeatedly teleporting; require the drift to persist.
        self._confirm: Dict[str, int] = {}

        # Anchor = position the current plan was built around, per label.
        # Drift is measured against the anchor (not the previous frame), so a
        # slow hand-move still accumulates past the threshold.
        self._anchors: Dict[str, tuple] = {}
        self._active = False
        self._last_replan_t = 0.0
        self.replan_count = 0

    def start(self) -> None:
        if self._active:
            return
        self.event_bus.subscribe(EVENT_OBJECT_DETECTED, self._on_object_detected)
        self._active = True
        logger.info("ReplanManager started — monitoring for significant target drift.")

    def stop(self) -> None:
        if not self._active:
            return
        self.event_bus.unsubscribe(EVENT_OBJECT_DETECTED, self._on_object_detected)
        self._active = False
        self._anchors.clear()
        logger.info("ReplanManager stopped.")

    def reset_mission(self) -> None:
        self._anchors.clear()
        self._confirm.clear()
        self.replan_count = 0
        self._last_replan_t = 0.0

    def snapshot(self) -> dict:
        return {
            "active": self._active,
            "count": self.replan_count,
            "threshold_m": self.distance_threshold,
        }

    def _on_object_detected(self, data: Any) -> None:
        if not isinstance(data, dict):
            return
        if data.get("actionable") is False:
            return
        label = data.get("label", "unknown")
        wx = data.get("world_x")
        wy = data.get("world_y")
        if wx is None or wy is None:
            return
        wx, wy = float(wx), float(wy)

        # Idle: keep re-anchoring so pre-mission movement never counts as drift.
        if self.mission_active and not self.mission_active():
            self._anchors[label] = (wx, wy)
            return

        anchor = self._anchors.get(label)
        if anchor is None:
            self._anchors[label] = (wx, wy)
            return

        dist = ((wx - anchor[0]) ** 2 + (wy - anchor[1]) ** 2) ** 0.5
        if dist < self.distance_threshold:
            self._confirm[label] = 0
            return

        self._confirm[label] = self._confirm.get(label, 0) + 1
        if self._confirm[label] < self.confirm_frames:
            return
        self._confirm[label] = 0
        if self.replan_count >= self.max_replans:
            # Re-anchor so this does not log once per perception frame.
            self._anchors[label] = (wx, wy)
            logger.warning("[ReplanManager] Max replans (%s) reached; ignoring drift.", self.max_replans)
            return
        now = time.time()
        if now - self._last_replan_t < self.cooldown_s:
            return

        logger.warning(
            f"[ReplanManager] Target '{label}' drifted {dist:.2f}m "
            f"(from ({anchor[0]:.2f}, {anchor[1]:.2f}) to ({wx:.2f}, {wy:.2f})). "
            f"Triggering replan."
        )
        self._last_replan_t = now
        self.replan_count += 1
        self._anchors[label] = (wx, wy)
        if self.replan_callback:
            try:
                self.replan_callback(label, {
                    "label": label,
                    "old_x": anchor[0], "old_y": anchor[1],
                    "new_x": wx, "new_y": wy,
                    "drift": dist,
                    "replan_count": self.replan_count,
                })
            except Exception as e:
                logger.error(f"[ReplanManager] Replan callback error: {e}")
