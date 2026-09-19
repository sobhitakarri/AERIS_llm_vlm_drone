"""
Simple In-Memory Pub/Sub Event Bus.
"""
from typing import Callable, Dict, List, Any
from backend.core.logger import get_logger

logger = get_logger("EventBus")

# Standard event names
EVENT_DRONE_TELEMETRY = "drone_telemetry"
EVENT_OBJECT_DETECTED = "object_detected"
EVENT_TARGET_LOST = "target_lost"
EVENT_STEP_STARTED = "step_started"
EVENT_STEP_COMPLETED = "step_completed"
EVENT_MISSION_FINISHED = "mission_finished"
EVENT_ABORT = "mission_aborted"


class EventBus:
    """Decoupled in-memory event publisher/subscriber for inter-module notifications."""

    def __init__(self):
        self._listeners: Dict[str, List[Callable]] = {}

    def subscribe(self, event_type: str, callback: Callable[[Any], None]) -> None:
        if event_type not in self._listeners:
            self._listeners[event_type] = []
        self._listeners[event_type].append(callback)

    def unsubscribe(self, event_type: str, callback: Callable[[Any], None]) -> None:
        if event_type in self._listeners:
            self._listeners[event_type] = [cb for cb in self._listeners[event_type] if cb != callback]

    def publish(self, event_type: str, data: Any = None) -> None:
        if event_type in self._listeners:
            for callback in self._listeners[event_type]:
                try:
                    callback(data)
                except Exception as e:
                    logger.error(f"Error in subscriber for event '{event_type}': {e}")


# Global shared instance
event_bus = EventBus()
