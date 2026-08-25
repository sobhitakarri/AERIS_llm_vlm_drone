"""
Telemetry & Drone State Schemas.
"""
from enum import Enum
from pydantic import BaseModel, Field


class FlightMode(str, Enum):
    IDLE = "IDLE"
    ARMED = "ARMED"
    TAKING_OFF = "TAKING_OFF"
    HOVERING = "HOVERING"
    NAVIGATING = "NAVIGATING"
    LANDING = "LANDING"
    EMERGENCY = "EMERGENCY"


class DroneState(BaseModel):
    x: float = Field(default=0.0, description="X position in meters")
    y: float = Field(default=0.0, description="Y position in meters")
    z: float = Field(default=0.0, description="Z position (altitude) in meters")
    roll: float = Field(default=0.0, description="Roll angle in degrees")
    pitch: float = Field(default=0.0, description="Pitch angle in degrees")
    yaw: float = Field(default=0.0, description="Yaw angle in degrees")
    vx: float = Field(default=0.0, description="Velocity X in m/s")
    vy: float = Field(default=0.0, description="Velocity Y in m/s")
    vz: float = Field(default=0.0, description="Velocity Z in m/s")
    battery_v: float = Field(default=4.1, description="Battery voltage in Volts")
    battery_percentage: int = Field(default=100, ge=0, le=100)
    is_armed: bool = Field(default=False)
    flight_mode: FlightMode = Field(default=FlightMode.IDLE)


class TelemetryPacket(BaseModel):
    timestamp: float
    state: DroneState
    active_skill: str = Field(default="NONE")
    planner_status: str = Field(default="IDLE")
