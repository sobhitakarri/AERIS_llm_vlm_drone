"""
FastAPI Server Setup & Lifecycle Factory.
"""
import os
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from backend.api.routes import router, app_context
from backend.planning.planner import MissionPlanner
from backend.runtime.mission_executor import MissionExecutor
from backend.runtime.uav_monitor import UavMonitor
from backend.drone.sim_interface import SimInterface
from backend.drone.litewing_interface import LiteWingInterface
from backend.drone.matlab_interface import MatlabInterface
from backend.models.gemini_provider import GeminiProvider
from backend.models.qwen_provider import QwenProvider
from backend.core.config import settings
from backend.core.logger import get_logger

logger = get_logger("Server")


def create_app(drone_mode: str = None, model_mode: str = None) -> FastAPI:
    app = FastAPI(title=settings.app_name, version=settings.version)

    # Enable CORS for frontend connection
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    drone_mode = drone_mode or settings.drone_backend
    model_mode = model_mode or settings.llm_provider

    # Initialize Drone Interface (sim | real | matlab)
    if drone_mode.lower() == "real":
        drone = LiteWingInterface()
    elif drone_mode.lower() == "matlab":
        drone = MatlabInterface()
    else:
        drone = SimInterface()

    drone.connect()

    # Initialize LLM Provider
    if model_mode.lower() == "qwen":
        llm = QwenProvider()
    else:
        llm = GeminiProvider()

    planner = MissionPlanner()
    monitor = UavMonitor()
    executor = MissionExecutor(drone=drone, monitor=monitor)

    # Inject global application context
    app_context["drone"] = drone
    app_context["llm"] = llm
    app_context["planner"] = planner
    app_context["executor"] = executor
    app_context["monitor"] = monitor

    app.include_router(router)

    # Serve Operator Dashboard Frontend Static Files
    frontend_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "frontend"))
    if os.path.exists(frontend_dir):
        app.mount("/", StaticFiles(directory=frontend_dir, html=True), name="frontend")
        logger.info(f"Mounted frontend dashboard static files from {frontend_dir}")

    logger.info(f"FastAPI initialized with Drone Mode: '{drone_mode}', Model Mode: '{model_mode}'.")
    return app
