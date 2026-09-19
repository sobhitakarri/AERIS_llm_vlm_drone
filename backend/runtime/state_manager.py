"""
Global Runtime State Store.
"""
import threading
from typing import Dict, List, Optional
from backend.schemas.telemetry import DroneState
from backend.schemas.perception import DetectedObject


class StateManager:
    """Central synchronized thread-safe state store for drone telemetry and perceived objects."""

    def __init__(self):
        self._lock = threading.Lock()
        self.drone_state = DroneState()
        self.perceived_objects: Dict[str, DetectedObject] = {}

    def update_drone_state(self, new_state: DroneState) -> None:
        with self._lock:
            self.drone_state = new_state

    def get_drone_state(self) -> DroneState:
        with self._lock:
            return self.drone_state.model_copy()

    def update_object(self, obj: DetectedObject) -> None:
        with self._lock:
            self.perceived_objects[obj.label] = obj

    def get_object(self, label: str) -> Optional[DetectedObject]:
        with self._lock:
            return self.perceived_objects.get(label)

    def get_all_objects(self) -> List[DetectedObject]:
        with self._lock:
            return list(self.perceived_objects.values())

    def clear_objects(self) -> None:
        with self._lock:
            self.perceived_objects.clear()

    def objects_as_context(self) -> list:
        """Return perceived objects as a list of dicts suitable for LLM context."""
        with self._lock:
            return [
                {
                    "label": obj.label,
                    "x": obj.world_x,
                    "y": obj.world_y,
                    "confidence": obj.confidence,
                }
                for obj in self.perceived_objects.values()
                if obj.world_x is not None
            ]
