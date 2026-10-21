"""
FastAPI Server Setup & Lifecycle Factory.
"""
import os
from pathlib import Path
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from backend.api.routes import router, app_context
from backend.planning.planner import MissionPlanner
from backend.planning.safety_validator import SafetyValidator
from backend.runtime.mission_executor import MissionExecutor
from backend.runtime.uav_monitor import UavMonitor
from backend.drone.sim_interface import SimInterface
from backend.models.gemini_provider import GeminiProvider
from backend.models.qwen_provider import QwenProvider
from backend.models.qwen25_llm_provider import Qwen25LLMProvider
from backend.runtime.state_manager import StateManager
from backend.runtime.event_bus import (
    EVENT_MISSION_FINISHED, EVENT_STEP_COMPLETED, EVENT_STEP_STARTED, EVENT_TARGET_LOST, event_bus,
)
from backend.core import run_mode, trace
from backend.vision.camera_manager import CameraManager
from backend.vision.spatial_grounding import SpatialGrounding
from backend.vision.fast_cv import FastPerception
from backend.vision.perception_loop import PerceptionLoop
from backend.runtime.replan_manager import ReplanManager
from backend.schemas.capabilities import RobotCapabilities, WorkspaceBounds
from backend.schemas.mission import MissionPlan, SkillPrimitive
from backend.core.config import settings
from backend.core.logger import get_logger

logger = get_logger("Server")

TWIN_MODES = ("twin", "digital_twin", "simulink")
# Largest move of a tracked target that is replanned to. Beyond it the
# re-detection is taken to be a different object (see mission_executor).
_MAX_TARGET_JUMP_M = 1.0


def make_drone(mode: str, uri: str = None, launch_twin: bool = False):
    """Drone backend for a mode name. Returns (drone, canonical_mode)."""
    mode = (mode or "sim").lower().strip()
    if mode in TWIN_MODES:
        from backend.drone.twin_interface import TwinInterface
        return TwinInterface(launch=launch_twin), "twin"
    if mode in ("real", "litewing", "esp32"):
        from backend.drone.litewing_interface import LiteWingInterface
        return (LiteWingInterface(uri=uri) if uri else LiteWingInterface()), "litewing"
    return SimInterface(), "sim"


def capabilities_for(drone) -> RobotCapabilities:
    """Flight envelope from the backend when it publishes one."""
    ws = drone.workspace() if hasattr(drone, "workspace") else None
    if not ws:
        return RobotCapabilities()
    airframe = (getattr(drone, "runtime", {}) or {}).get("airframe", {})
    v_max = (airframe.get("v_max") or [0.8])[0]
    return RobotCapabilities(
        robot_name=airframe.get("name", "Digital twin"),
        workspace=WorkspaceBounds(
            x_min=ws["x"][0], x_max=ws["x"][1], y_min=ws["y"][0], y_max=ws["y"][1],
            z_min=ws["z"][0], z_max=ws["z"][1], max_velocity=float(v_max),
        ),
    )


def make_models(model_mode: str):
    """(llm, vlm, canonical_mode) for a model backend name."""
    if (model_mode or "").lower() in ("qwen", "qwen25", "local", "local_split", "hybrid"):
        vlm = QwenProvider()          # Qwen2-VL-2B: visual grounding / bounding boxes
        llm = Qwen25LLMProvider()     # Qwen2.5-3B:  LLM planning, high-level reasoning
        if not llm.is_live:
            logger.warning("[Server] Qwen2.5-3B not available. Falling back to Qwen2-VL-2B for planning.")
            llm = vlm
        return llm, vlm, "local_split"
    llm = GeminiProvider()
    return llm, llm, "gemini"


def _trace_step_started(d: dict) -> None:
    trace.event("step_started", step=d.get("step"), skill=d.get("skill"), params=d.get("params"))


def _trace_step_completed(d: dict) -> None:
    trace.event("step_completed", step=d.get("step"), skill=d.get("skill"))


def create_app(drone_mode: str = None, model_mode: str = None, launch_twin: bool = False) -> FastAPI:
    app = FastAPI(title=settings.app_name, version=settings.version)

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

    drone, drone_mode = make_drone(drone_mode, launch_twin=launch_twin)
    drone.connect()
    is_twin = drone_mode == "twin"
    capabilities = capabilities_for(drone)

    # A backend with its own scene cameras must not also grab the laptop webcam.
    camera_manager = CameraManager(autostart=not is_twin)
    if not is_twin:
        camera_manager.start()

    grounding = SpatialGrounding()
    if not is_twin:
        _h_path = Path(__file__).resolve().parents[2] / "cache" / "homography.json"
        if _h_path.is_file():
            try:
                grounding.load(_h_path)
                logger.info("Loaded homography from %s", _h_path)
            except Exception as exc:
                logger.warning("Homography load failed (%s); using default H.", exc)
    state_manager = StateManager()

    llm, vlm, model_mode = make_models(model_mode)

    planner = MissionPlanner(validator=SafetyValidator(capabilities))
    monitor = UavMonitor()

    # VLM calls run on a worker thread; the loop only tracks and grounds.
    fast_cv = FastPerception(
        grounding=grounding, vlm=vlm, state_manager=state_manager,
        async_detect=True, z_max=capabilities.workspace.z_max,
    )
    from backend.planning.vla_pipeline import UAVVLAPipeline
    vla = UAVVLAPipeline(vlm_provider=vlm, llm_provider=llm, perception=fast_cv, grounding=grounding)

    executor = MissionExecutor(
        drone=drone,
        monitor=monitor,
        vlm=vlm,
        grounding=grounding,
        state_manager=state_manager,
        camera_manager=camera_manager,
        event_bus=event_bus,
        perception=fast_cv,
        capabilities=capabilities,
    )

    perception_loop = PerceptionLoop(
        perception=fast_cv,
        camera_manager=camera_manager,
        drone=drone,
        event_bus=event_bus,
    )
    perception_loop.start()

    scene_service = None
    if is_twin:
        from backend.vision.scene_service import SceneService
        scene_service = SceneService(drone, grounding)
        scene_service.start()

    def _mission_active() -> bool:
        return monitor.status == "running" and not monitor.aborted()

    def _on_replan(label: str, info: dict) -> None:
        # Never resurrect a drone that is on the ground: a replan arriving as the
        # plan's final LAND runs would otherwise fly it again after landing.
        active_drone = app_context.get("drone") or drone
        st = active_drone.get_state()
        if st.z <= 0.1 or not st.is_armed:
            logger.info("[Replan] Ignoring drift for '%s': drone is not airborne.", label)
            return
        sm = app_context.get("state_manager") or state_manager
        if sm and not sm.is_actionable(label):
            logger.info("[Replan] Ignoring drift for '%s': target is not actionable.", label)
            return
        drift = {k: round(float(info[k]), 3) for k in ("old_x", "old_y", "new_x", "new_y", "drift") if k in info}
        active_executor = app_context.get("executor") or executor
        if float(info.get("drift", 0.0)) > _MAX_TARGET_JUMP_M:
            # Too far for the same object to have moved: a different object
            # answered the re-detection. Treat the target as lost instead.
            trace.event("replan_rejected_jump", label=label, **drift)
            logger.warning("[Replan] '%s' reappeared %.2f m away: not the same object; treating it as lost.",
                           label, float(info.get("drift", 0.0)))
            active_executor.note_target_lost(label)
            return
        if not run_mode.current().replan:
            # Ablation: the drift was seen, but the plan is not changed.
            trace.event("replan_suppressed", label=label, **drift)
            logger.info("[Replan] Drift of '%s' not acted on: replanning is off in this run mode.", label)
            return
        trace.event("replan", label=label, **drift)
        planner.trigger_replan(f"{label} drifted {info.get('drift', 0):.2f}m")
        ws = (app_context.get("capabilities") or capabilities).workspace
        nx = max(ws.x_min, min(float(info["new_x"]), ws.x_max))
        ny = max(ws.y_min, min(float(info["new_y"]), ws.y_max))
        obj = sm.get_object(label) if sm else None
        z = obj.world_z if obj and obj.world_z is not None else st.z
        z = max(ws.z_min, min(ws.z_max, max(z, st.z)))
        skills = [
            SkillPrimitive(skill="MOVE_TO", params={"x": round(nx, 3), "y": round(ny, 3), "z": round(z, 3), "speed": 0.4}),
            SkillPrimitive(skill="HOVER", params={"t": 2.0}),
        ]
        patch = MissionPlan(raw_command=f"replan {label}", reasoning="target drift", skills=skills, source="replan")
        validation = planner.validator.validate_plan(patch)
        from backend.planning.planner import PlannerState
        if not validation.is_valid and not run_mode.current().enforce_safety:
            trace.event("validator_overridden", source="replan", errors=validation.errors)
            validation = validation.model_copy(update={"is_valid": True, "validated_plan": patch})
        if not validation.is_valid:
            trace.event("replan_rejected", label=label, errors=validation.errors)
            logger.warning("[Replan] Rejected unsafe replan: %s", validation.errors)
            if planner.state == PlannerState.REPLANNING:
                planner.state = PlannerState.EXECUTING
            return
        active_executor.inject_replan((validation.validated_plan or patch).skills)
        active_executor.update_target_position(nx, ny)
        if planner.state == PlannerState.REPLANNING:
            planner.state = PlannerState.EXECUTING

    replan = ReplanManager(
        event_bus=event_bus,
        replan_callback=_on_replan,
        mission_active=_mission_active,
    )
    replan.start()

    def _on_mission_finished(_data) -> None:
        # Stop hunting for mission targets once it is over, otherwise the loop
        # keeps spending VLM calls on objects nobody is asking about any more.
        fast_cv.set_targets([])
        fast_cv.trackers.reset()
        state_manager.clear_objects()
        app_context["last_mission"] = _data if isinstance(_data, dict) else {}

    def _on_target_lost(data) -> None:
        label = (data or {}).get("label", "")
        trace.event("target_lost", label=label)
        logger.warning("[Safety] TARGET_LOST '%s' — dropping pending target-directed replan.", label)
        active_executor = app_context.get("executor") or executor
        active_executor.clear_replan()
        # The plan's coordinates came from this target: stop flying to them.
        active_executor.note_target_lost(label)
        if state_manager and label:
            state_manager.mark_lost(label)

    event_bus.subscribe(EVENT_MISSION_FINISHED, _on_mission_finished)
    event_bus.subscribe(EVENT_TARGET_LOST, _on_target_lost)
    # Step boundaries on the mission trace, for per-skill timing. The bus is
    # global: unsubscribe first so a rebuilt app does not record steps twice.
    for evt, fn in ((EVENT_STEP_STARTED, _trace_step_started), (EVENT_STEP_COMPLETED, _trace_step_completed)):
        event_bus.unsubscribe(evt, fn)
        event_bus.subscribe(evt, fn)

    app_context.update({
        "drone": drone,
        "llm": llm,
        "vlm": vlm,
        "vla": vla,
        "planner": planner,
        "executor": executor,
        "monitor": monitor,
        "state_manager": state_manager,
        "camera_manager": camera_manager,
        "grounding": grounding,
        "event_bus": event_bus,
        "replan": replan,
        "perception_loop": perception_loop,
        "perception": fast_cv,
        "scene_service": scene_service,
        "model_mode": model_mode,
        "stream_source": "twin" if is_twin else "webcam",
        "camera_view": "overhead" if is_twin else "pip",
        "drone_backend": drone_mode,
        "drone_uri": settings.drone_uri,
        "capabilities": capabilities,
        "last_mission": {},
    })

    app.include_router(router)

    # Serve Operator Dashboard Frontend Static Files
    frontend_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "frontend"))
    if os.path.exists(frontend_dir):
        app.mount("/", StaticFiles(directory=frontend_dir, html=True), name="frontend")
        logger.info(f"Mounted frontend dashboard static files from {frontend_dir}")

    logger.info(f"FastAPI initialized with Drone Mode: '{drone_mode}', Model Mode: '{model_mode}'.")
    return app
