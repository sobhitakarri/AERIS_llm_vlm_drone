"""
ReplanManager — Dynamic Mission Replanning via EventBus Subscriptions.

Monitors object detection events during execution and triggers LLM
replanning when unexpected obstacles are detected or a target has
moved significantly from its last known position.
"""
from typing import Any, Callable, Dict, Optional
from backend.runtime.event_bus import EventBus, EVENT_OBJECT_DETECTED
from backend.core.logger import get_logger

logger = get_logger("ReplanManager")

# Minimum distance (meters) a target must shift to trigger a replan
_REPLAN_DISTANCE_THRESHOLD = 0.3


class ReplanManager:
    """
    Watches for perception events published on the EventBus and decides
    whether the current mission plan needs dynamic replanning.

    Usage:
        replan_mgr = ReplanManager(event_bus=event_bus, replan_callback=my_replan_fn)
        replan_mgr.start()
    """

    def __init__(
        self,
        event_bus: EventBus,
        replan_callback: Optional[Callable[[str, Dict], None]] = None,
        distance_threshold: float = _REPLAN_DISTANCE_THRESHOLD,
    ):
        self.event_bus = event_bus
        self.replan_callback = replan_callback
        self.distance_threshold = distance_threshold

        # Track last-known positions per label
        self._last_positions: Dict[str, tuple] = {}
        self._active = False

    def start(self) -> None:
        """Subscribe to detection events and begin monitoring."""
        if self._active:
            return
        self.event_bus.subscribe(EVENT_OBJECT_DETECTED, self._on_object_detected)
        self._active = True
        logger.info("ReplanManager started — monitoring for significant target drift.")

    def stop(self) -> None:
        """Unsubscribe and stop monitoring."""
        if not self._active:
            return
        self.event_bus.unsubscribe(EVENT_OBJECT_DETECTED, self._on_object_detected)
        self._active = False
        self._last_positions.clear()
        logger.info("ReplanManager stopped.")

    def _on_object_detected(self, data: Any) -> None:
        """Called whenever a detection event fires during execution."""
        if not isinstance(data, dict):
            return

        label = data.get("label", "unknown")
        wx = data.get("world_x")
        wy = data.get("world_y")

        if wx is None or wy is None:
            return

        prev = self._last_positions.get(label)
        if prev is not None:
            dx = wx - prev[0]
            dy = wy - prev[1]
            dist = (dx * dx + dy * dy) ** 0.5

            if dist >= self.distance_threshold:
                logger.warning(
                    f"[ReplanManager] Target '{label}' drifted {dist:.2f}m "
                    f"(from ({prev[0]:.2f}, {prev[1]:.2f}) to ({wx:.2f}, {wy:.2f})). "
                    f"Triggering replan."
                )
                if self.replan_callback:
                    try:
                        self.replan_callback(label, {
                            "label": label,
                            "old_x": prev[0], "old_y": prev[1],
                            "new_x": wx, "new_y": wy,
                            "drift": dist,
                        })
                    except Exception as e:
                        logger.error(f"[ReplanManager] Replan callback error: {e}")

        self._last_positions[label] = (wx, wy)
""", "Description": "New ReplanManager module that monitors EventBus object_detected events and triggers replanning when a target drifts beyond a threshold distance."
