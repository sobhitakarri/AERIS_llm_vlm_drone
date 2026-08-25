"""
Core Package for Configuration and Logging.
"""
from backend.core.config import settings
from backend.core.logger import get_logger

__all__ = ["settings", "get_logger"]
