"""
Global Runtime State Store.
"""
from typing import Dict
from backend.schemas.telemetry import DroneState
from backend.schemas.perception import DetectedObject


class StateManager:
    """Central synchronized thread-safe state store for drone telemetry and perceived objects."""

    def __init__(self):
        self.drone_state = DroneState()
        self.perceived_objects: Dict[str, DetectedObject] = {}

    def update_drone_state(self, new_state: DroneState) -> None:
        self.drone_state = new_state

    def update_object(self, obj: DetectedObject) -> None:
        self.perceived_objects[obj.label] = obj

    def get_object(self, label: str) -> DetectedObject:
        return self.perceived_objects.get(label)
