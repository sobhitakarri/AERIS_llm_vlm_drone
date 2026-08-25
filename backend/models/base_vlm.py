"""
Abstract VLM Provider Base Class.
"""
from abc import ABC, abstractmethod
from typing import Optional
from backend.schemas.perception import DetectedObject


class VLMProvider(ABC):
    """
    Abstract interface for Vision-Language Models.
    Resolves natural language queries to visual object targets in scene frames.
    """

    @abstractmethod
    def resolve_target(self, image_bytes: bytes, target_description: str) -> Optional[DetectedObject]:
        """Resolves target object bounding box given visual image input and natural language query."""
        pass
