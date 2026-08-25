"""
Framework Configuration Settings.
"""
import os
from pathlib import Path
from pydantic import BaseModel, Field

_PROJECT_ROOT = Path(__file__).resolve().parents[2]


def _clean_key(value: str) -> str:
    return (value or "").strip().strip('"').strip("'")


def _load_env_files() -> None:
    try:
        from dotenv import load_dotenv
    except ImportError:
        return
    env_file = _PROJECT_ROOT / ".env"
    backend_env = Path(__file__).resolve().parents[1] / ".env"
    example_file = _PROJECT_ROOT / ".env.example"
    if env_file.is_file():
        load_dotenv(env_file, override=False)
    if backend_env.is_file():
        load_dotenv(backend_env, override=False)
    key = _clean_key(os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY") or "")
    if not key and example_file.is_file():
        load_dotenv(example_file, override=False)


_load_env_files()


class Settings(BaseModel):
    # App Information
    app_name: str = "LLM-VLM Adaptive Drone Framework"
    version: str = "1.0.0"
    debug: bool = True

    # Backend / Model Choices
    llm_provider: str = Field(default="gemini", description="Options: gemini, qwen")
    vlm_provider: str = Field(default="gemini", description="Options: gemini, qwen")
    drone_backend: str = Field(default="sim", description="Options: sim, real, matlab")

    # API Keys
    gemini_api_key: str = Field(
        default_factory=lambda: _clean_key(
            os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY") or ""
        )
    )
    gemini_model: str = Field(default_factory=lambda: os.getenv("GEMINI_MODEL", "gemini-3.7-flash"))

    # Drone Connection
    drone_uri: str = "udp://192.168.43.42"
    telemetry_rate_hz: int = 20

    # Vision / Camera
    camera_index: int = 0
    frame_width: int = 640
    frame_height: int = 480

    # Geofence / Workspace
    workspace_x_min: float = -1.5
    workspace_x_max: float = 1.5
    workspace_y_min: float = -1.5
    workspace_y_max: float = 1.5
    workspace_z_min: float = 0.2
    workspace_z_max: float = 1.5
    max_step_velocity: float = 0.8


settings = Settings()
