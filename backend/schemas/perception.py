"""
Perception & Visual Grounding Schemas.
"""
from typing import List, Tuple, Optional
from pydantic import BaseModel, Field


class BoundingBox(BaseModel):
    u_min: int
    v_min: int
    u_max: int
    v_max: int

    @property
    def center_pixel(self) -> Tuple[int, int]:
        return (self.u_min + self.u_max) // 2, (self.v_min + self.v_max) // 2


class DetectedObject(BaseModel):
    label: str = Field(..., description="Semantic object label, e.g., 'red_bottle'")
    bbox: BoundingBox
    confidence: float = Field(default=1.0, ge=0.0, le=1.0)
    world_x: Optional[float] = Field(default=None, description="Grounded physical X position in meters")
    world_y: Optional[float] = Field(default=None, description="Grounded physical Y position in meters")
    world_z: Optional[float] = Field(default=None, description="Grounded physical Z position (altitude) in meters")
    actionable: bool = Field(
        default=True,
        description="False when the target is lost; stale coordinates must not drive motion.",
    )


class PerceptionResult(BaseModel):
    timestamp: float
    objects: List[DetectedObject] = Field(default_factory=list)
    lost_labels: List[str] = Field(default_factory=list)
    drone_pixel: Optional[Tuple[int, int]] = Field(default=None)
    drone_world: Optional[Tuple[float, float]] = Field(default=None)
