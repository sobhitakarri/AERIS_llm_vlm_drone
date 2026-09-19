"""Background FastPerception loop (AutoFly-style observation, no VLM every frame)."""
from __future__ import annotations

import threading
import time
from typing import Any, Optional

from backend.core.logger import get_logger
from backend.runtime.event_bus import EVENT_OBJECT_DETECTED, EVENT_TARGET_LOST, EventBus
from backend.vision.fast_cv import FastPerception

logger = get_logger("PerceptionLoop")


class PerceptionLoop:
    def __init__(
        self,
        perception: FastPerception,
        camera_manager: Any = None,
        drone: Any = None,
        event_bus: Optional[EventBus] = None,
        period_s: float = 0.1,
    ):
        self.perception = perception
        self.camera_manager = camera_manager
        self.drone = drone
        self.event_bus = event_bus
        self.period_s = period_s
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_pub: dict = {}
        self._errors = 0

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="perception-loop")
        self._thread.start()
        logger.info("Perception loop started (~%.0f Hz).", 1.0 / self.period_s)

    def stop(self) -> None:
        self._running = False
        thread, self._thread = self._thread, None
        if thread and thread.is_alive():
            thread.join(timeout=1.0)

    def _grab(self):
        frame = None
        if self.camera_manager and hasattr(self.camera_manager, "read_frame"):
            try:
                frame = self.camera_manager.read_frame(timeout_s=0.0)
            except TypeError:  # camera stub without the timeout arg
                frame = self.camera_manager.read_frame()
        if frame is None and self.drone and hasattr(self.drone, "get_frame"):
            frame = self.drone.get_frame()
        return frame

    def _loop(self) -> None:
        while self._running:
            try:
                frame = self._grab()
                z = 1.0
                if self.drone and hasattr(self.drone, "get_state"):
                    z = max(getattr(self.drone.get_state(), "z", 1.0) or 1.0, 0.4)
                result = self.perception.process_frame(frame, z_altitude=z)
                if self.event_bus:
                    for label in getattr(result, "lost_labels", []) or []:
                        self.event_bus.publish(
                            EVENT_TARGET_LOST,
                            {"label": label, "actionable": False},
                        )
                        self._last_pub.pop(label, None)
                    for obj in result.objects:
                        if not getattr(obj, "actionable", True):
                            continue
                        if obj.world_x is None or obj.world_y is None:
                            continue
                        key = obj.label
                        prev = self._last_pub.get(key)
                        now = (obj.world_x, obj.world_y)
                        if prev is None or abs(prev[0] - now[0]) > 0.02 or abs(prev[1] - now[1]) > 0.02:
                            self.event_bus.publish(EVENT_OBJECT_DETECTED, obj.model_dump())
                            self._last_pub[key] = now
                self._errors = 0
            except Exception as exc:
                self._errors += 1
                # Throttle: a persistent fault here would otherwise log at 10 Hz.
                if self._errors == 1 or self._errors % 100 == 0:
                    logger.warning("Perception loop error (x%s): %s", self._errors, exc)
            time.sleep(self.period_s)
