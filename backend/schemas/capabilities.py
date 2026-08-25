"""
Robot Capabilities & Workspace Constraints Schema.
"""
from typing import List, Tuple
from pydantic import BaseModel, Field


class WorkspaceBounds(BaseModel):
    x_min: float = Field(default=-1.5, description="Minimum X coordinate in meters")
    x_max: float = Field(default=1.5, description="Maximum X coordinate in meters")
    y_min: float = Field(default=-1.5, description="Minimum Y coordinate in meters")
    y_max: float = Field(default=1.5, description="Maximum Y coordinate in meters")
    z_min: float = Field(default=0.2, description="Minimum Z coordinate (altitude) in meters")
    z_max: float = Field(default=1.5, description="Maximum Z coordinate (altitude) in meters")
    max_velocity: float = Field(default=0.8, description="Maximum linear velocity in m/s")


class RobotCapabilities(BaseModel):
    robot_name: str = Field(default="LiteWing ESP32-S3", description="Identifier for the robot platform")
    supported_skills: List[str] = Field(
        default_factory=lambda: [
            "TAKEOFF", "LAND", "HOVER", "MOVE_TO", "ROTATE", "CIRCLE", "FIND", "INSPECT", "RETURN"
        ],
        description="Supported Skill Primitives"
    )
    workspace: WorkspaceBounds = Field(default_factory=WorkspaceBounds)
    has_camera: bool = Field(default=True, description="Whether overhead visual feedback is active")
