"""
FastAPI Server Setup & Lifecycle Factory.
"""
import os
from fastapi import FastAPI, Request
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
from backend.models.qwen25_llm_provider import Qwen25LLMProvider
from backend.runtime.state_manager import StateManager
from backend.runtime.event_bus import event_bus
from backend.vision.camera_manager import CameraManager
from backend.vision.spatial_grounding import SpatialGrounding
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

    # Disable caching for frontend static files so dashboard updates immediately
    @app.middleware("http")
    async def add_no_cache_headers(request: Request, call_next):
        response = await call_next(request)
        path = request.url.path
        if path == "/" or path.startswith(("/css", "/js", "/favicon.ico")):
            response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate, max-age=0"
            response.headers["Pragma"] = "no-cache"
            response.headers["Expires"] = "0"
        return response

    drone_mode = drone_mode or settings.drone_backend
    model_mode = model_mode or settings.llm_provider

    # Initialize Drone Interface (sim | real | matlab | pysimverse)
    if drone_mode.lower() == "real":
        drone = LiteWingInterface()
    elif drone_mode.lower() == "matlab":
        drone = MatlabInterface()
    elif drone_mode.lower() in ("pysimverse", "simverse"):
        from backend.drone.pysimverse_interface import PySimverseInterface
        drone = PySimverseInterface()
    else:
        drone = SimInterface()

    drone.connect()

    # Initialize Vision, State & Event Subsystems
    camera_manager = CameraManager()
    grounding = SpatialGrounding()
    state_manager = StateManager()

    # Initialize LLM/VLM Providers
    # Split architecture: Qwen2-VL-2B = VLM (bounding boxes only)
    #                     Qwen2.5-3B  = LLM (text planning, reasoning)
    from backend.planning.vla_pipeline import UAVVLAPipeline
    if model_mode.lower() in ("qwen", "qwen25", "local"):
        vlm = QwenProvider()          # Qwen2-VL-2B: visual grounding / bounding boxes
        llm = Qwen25LLMProvider()     # Qwen2.5-3B:  LLM planning, high-level reasoning
        if not llm.is_live:
            # Qwen2.5-3B not yet pulled; fall back to VLM model for planning too
            logger.warning("[Server] Qwen2.5-3B not available. Falling back to Qwen2-VL-2B for planning.")
            llm = vlm
        else:
            logger.info("[Server] Split mode: VLM=Qwen2-VL-2B | LLM=Qwen2.5-3B")
        vla = UAVVLAPipeline(vlm_provider=vlm, llm_provider=llm)
    else:
        llm = GeminiProvider()
        vlm = llm
        vla = UAVVLAPipeline(vlm_provider=llm, llm_provider=llm)

    planner = MissionPlanner()
    monitor = UavMonitor()
    executor = MissionExecutor(
        drone=drone,
        monitor=monitor,
        vlm=vlm,
        grounding=grounding,
        state_manager=state_manager,
        camera_manager=camera_manager,
        event_bus=event_bus,
    )

    # Inject global application context
    app_context["drone"] = drone
    app_context["llm"] = llm
    app_context["vlm"] = vlm
    app_context["vla"] = vla
    app_context["planner"] = planner
    app_context["executor"] = executor
    app_context["monitor"] = monitor
    app_context["state_manager"] = state_manager
    app_context["camera_manager"] = camera_manager
    app_context["grounding"] = grounding
    app_context["event_bus"] = event_bus
    app_context["model_mode"] = model_mode or "gemini"
    app_context["stream_source"] = "webcam"
    app_context["drone_backend"] = drone_mode or "sim"
    app_context["drone_uri"] = settings.drone_uri
    app_context["refresh_viewer"] = False

    app.include_router(router)

    # Serve Operator Dashboard Frontend Static Files
    frontend_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "frontend"))
    if os.path.exists(frontend_dir):
        app.mount("/", StaticFiles(directory=frontend_dir, html=True), name="frontend")
        logger.info(f"Mounted frontend dashboard static files from {frontend_dir}")

    def start_dashboard_viewer():
        import threading
        import cv2
        import time

        def viewer_loop():
            # Wait for backend to fully start
            time.sleep(2.0)
            active_source = app_context.get("stream_source", "webcam")
            active_window = f"AERIS Tracking [{active_source.upper()}]"
            cv2.namedWindow(active_window, cv2.WINDOW_NORMAL)

            while True:
                time.sleep(0.033)  # ~30fps

                # Detect dynamic stream change or explicit refresh request
                current_source = app_context.get("stream_source", "webcam")
                if current_source != active_source or app_context.get("refresh_viewer", False):
                    app_context["refresh_viewer"] = False
                    try:
                        cv2.destroyWindow(active_window)
                    except Exception:
                        pass
                    active_source = current_source
                    active_window = f"AERIS Tracking [{active_source.upper()}]"
                    cv2.namedWindow(active_window, cv2.WINDOW_NORMAL)
                    logger.info(f"[AERIS Viewer] Refreshed OpenCV window for stream: '{active_source}'")

                frame = None
                if active_source == "webcam":
                    cam_ref = app_context.get("camera_manager")
                    if cam_ref and hasattr(cam_ref, "read_frame"):
                        frame = cam_ref.read_frame()
                elif active_source in ("pysimverse", "simverse"):
                    drone_ref = app_context.get("drone")
                    if drone_ref and hasattr(drone_ref, "get_frame"):
                        frame = drone_ref.get_frame()
                else:  # fallback
                    drone_ref = app_context.get("drone")
                    if drone_ref and hasattr(drone_ref, "get_frame"):
                        frame = drone_ref.get_frame()
                    if frame is None:
                        cam_ref = app_context.get("camera_manager")
                        if cam_ref and hasattr(cam_ref, "read_frame"):
                            frame = cam_ref.read_frame()

                if frame is not None:
                    display_frame = frame.copy()
                    vla_ref = app_context.get("vla")
                    state_mgr = app_context.get("state_manager")

                    if vla_ref and vla_ref.object_search.last_bbox:
                        bbox = vla_ref.object_search.last_bbox
                        label = vla_ref.object_search.last_label or "target"
                        cv2.rectangle(display_frame, (bbox.u_min, bbox.v_min), (bbox.u_max, bbox.v_max), (0, 240, 255), 2)
                        cv2.putText(display_frame, f"AERIS: {label}", (bbox.u_min, max(0, bbox.v_min - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 240, 255), 2)
                    elif state_mgr:
                        for obj in state_mgr.get_all_objects():
                            if obj.bbox:
                                b = obj.bbox
                                cv2.rectangle(display_frame, (b.u_min, b.v_min), (b.u_max, b.v_max), (0, 215, 255), 2)
                                cv2.putText(display_frame, f"{obj.label} ({obj.confidence:.2f})", (b.u_min, max(0, b.v_min - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 215, 255), 2)

                    cv2.imshow(active_window, display_frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    pass
        
        t = threading.Thread(target=viewer_loop, daemon=True)
        t.start()
        logger.info("Started local OpenCV tracking viewer thread with dynamic source refresh.")

    start_dashboard_viewer()

    logger.info(f"FastAPI initialized with Drone Mode: '{drone_mode}', Model Mode: '{model_mode}'.")
    return app
