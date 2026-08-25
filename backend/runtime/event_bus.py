"""
Simple In-Memory Pub/Sub Event Bus.
"""
from typing import Callable, Dict, List, Any


class EventBus:
    """Decoupled in-memory event publisher/subscriber for inter-module notifications."""

    def __init__(self):
        self._listeners: Dict[str, List[Callable]] = {}

    def subscribe(self, event_type: str, callback: Callable[[Any], None]) -> None:
        if event_type not in self._listeners:
            self._listeners[event_type] = []
        self._listeners[event_type].append(callback)

    def publish(self, event_type: str, data: Any) -> None:
        if event_type in self._listeners:
            for callback in self._listeners[event_type]:
                callback(data)
