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
            obj.actionable = True
            self.perceived_objects[obj.label] = obj

    def mark_lost(self, label: str) -> Optional[DetectedObject]:
        """TARGET_LOST: keep the last pose for the UI, but strip the coordinates
        that would otherwise generate a new movement command."""
        with self._lock:
            obj = self.perceived_objects.get(label)
            if obj is None:
                return None
            obj.actionable = False
            obj.world_x = None
            obj.world_y = None
            return obj

    def is_actionable(self, label: str) -> bool:
        with self._lock:
            obj = self.perceived_objects.get(label)
            return bool(obj and obj.actionable and obj.world_x is not None)

    def get_object(self, label: str) -> Optional[DetectedObject]:
        with self._lock:
            return self.perceived_objects.get(label)

    def get_all_objects(self) -> List[DetectedObject]:
        with self._lock:
            return list(self.perceived_objects.values())

    def get_actionable_objects(self) -> List[DetectedObject]:
        with self._lock:
            return [
                obj for obj in self.perceived_objects.values()
                if obj.actionable and obj.world_x is not None
            ]

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
                if obj.actionable and obj.world_x is not None
            ]
