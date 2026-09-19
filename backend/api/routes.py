"""
FastAPI REST & WebSocket Route Handlers (Pydantic V2 Compatible).
"""
import asyncio
import time
from typing import Optional
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import StreamingResponse
from pydantic import BaseModel
from backend.schemas.capabilities import RobotCapabilities
from backend.schemas.telemetry import TelemetryPacket, DroneState
from backend.planning.planner import PlannerState
from backend.core.logger import get_logger

logger = get_logger("APIRoutes")

router = APIRouter()

# Shared Global App Context References (injected by server initialization)
app_context = {
    "planner": None,
    "executor": None,
    "drone": None,
    "llm": None,
    "vlm": None,
    "vla": None,
    "monitor": None,
    "state_manager": None,
    "camera_manager": None,
    "grounding": None,
    "event_bus": None,
    "model_mode": "gemini",
    "stream_source": "webcam",
    "drone_backend": "sim",
    "drone_uri": "udp://192.168.43.42:1988",
    "refresh_viewer": False,
    "capabilities": RobotCapabilities(),
}


class CommandRequest(BaseModel):
    command: str


class ModelConfigRequest(BaseModel):
    model_mode: str  # "gemini" | "local_split" | "qwen"


class StreamConfigRequest(BaseModel):
    stream_source: str  # "webcam" | "pysimverse" | "auto"
    camera_index: Optional[int] = None


class DroneConfigRequest(BaseModel):
    drone_backend: str  # "litewing" | "sim" | "pysimverse" | "matlab"
    drone_uri: Optional[str] = None  # e.g. "udp://192.168.43.42:1988"


@router.get("/api/health")
async def health_check():
    drone = app_context["drone"]
    llm = app_context["llm"]
    return {
        "status": "online",
        "framework": "AERIS",
        "drone_backend": type(drone).__name__ if drone else "none",
        "model_backend": type(llm).__name__ if llm else "none",
        "llm_live": bool(getattr(llm, "is_live", False)),
        "llm_model": getattr(llm, "model_name", None),
        "stream_source": app_context.get("stream_source", "webcam"),
    }


@router.get("/api/config")
async def get_config():
    drone = app_context.get("drone")
    llm = app_context.get("llm")
    vlm = app_context.get("vlm")
    cam = app_context.get("camera_manager")
    caps = app_context.get("capabilities") or RobotCapabilities()

    is_drone_connected = False
    if drone:
        is_drone_connected = getattr(drone, "is_connected", True)

    return {
        "framework": "AERIS",
        "version": "2.0",
        "model_mode": app_context.get("model_mode", "gemini"),
        "model_name": getattr(llm, "model_name", type(llm).__name__ if llm else "None"),
        "vlm_name": getattr(vlm, "model_name", type(vlm).__name__ if vlm else "None"),
        "stream_source": app_context.get("stream_source", "webcam"),
        "camera_index": getattr(cam, "camera_index", 0) if cam else 0,
        "drone_backend": app_context.get("drone_backend", "sim"),
        "drone_uri": app_context.get("drone_uri", "udp://192.168.43.42:1988"),
        "drone_connected": is_drone_connected,
        "drone_class": type(drone).__name__ if drone else "None",
        "geofence": caps.workspace.model_dump() if caps else {},
        "supported_skills": caps.supported_skills if caps else [],
    }


@router.post("/api/config/model")
async def set_model_config(req: ModelConfigRequest):
    mode = req.model_mode.lower().strip()
    logger.info(f"[AERIS Config] Switching reasoning model mode to: '{mode}'")

    from backend.planning.vla_pipeline import UAVVLAPipeline
    from backend.models.gemini_provider import GeminiProvider
    from backend.models.qwen_provider import QwenProvider
    from backend.models.qwen25_llm_provider import Qwen25LLMProvider

    if mode in ("local", "local_split", "qwen", "hybrid"):
        vlm = QwenProvider()
        llm = Qwen25LLMProvider()
        if not llm.is_live:
            logger.warning("[AERIS Config] Qwen2.5-3B not active. Using Qwen2-VL-2B for both VLM & LLM.")
            llm = vlm
        vla = UAVVLAPipeline(vlm_provider=vlm, llm_provider=llm)
        app_context["model_mode"] = "local_split"
    else:
        llm = GeminiProvider()
        vlm = llm
        vla = UAVVLAPipeline(vlm_provider=llm, llm_provider=llm)
        app_context["model_mode"] = "gemini"

    app_context["llm"] = llm
    app_context["vlm"] = vlm
    app_context["vla"] = vla

    executor = app_context.get("executor")
    if executor:
        executor.vlm = vlm

    return {
        "success": True,
        "model_mode": app_context["model_mode"],
        "llm_model": getattr(llm, "model_name", type(llm).__name__),
        "vlm_model": getattr(vlm, "model_name", type(vlm).__name__),
    }


@router.post("/api/config/stream")
async def set_stream_config(req: StreamConfigRequest):
    source = req.stream_source.lower().strip()
    logger.info(f"[AERIS Config] Switching stream source to: '{source}'")
    app_context["stream_source"] = source
    app_context["refresh_viewer"] = True

    cam = app_context.get("camera_manager")
    if req.camera_index is not None and cam:
        cam.camera_index = req.camera_index
        if cam.cap:
            cam.stop()
            cam.start()

    return {
        "success": True,
        "stream_source": source,
        "message": f"Stream source switched to {source}. OpenCV tracking window refreshed.",
    }


@router.post("/api/config/drone")
async def set_drone_config(req: DroneConfigRequest):
    backend = req.drone_backend.lower().strip()
    uri = req.drone_uri or app_context.get("drone_uri", "udp://192.168.43.42:1988")
    logger.info(f"[AERIS Config] Switching drone backend to '{backend}' (uri: '{uri}')")

    old_drone = app_context.get("drone")
    if old_drone:
        try:
            old_drone.emergency_stop()
            old_drone.disconnect()
        except Exception as e:
            logger.warning(f"Error disconnecting previous drone: {e}")

    new_drone = None
    connected = False
    error_msg = None

    if backend in ("litewing", "real", "esp32"):
        from backend.drone.litewing_interface import LiteWingInterface
        new_drone = LiteWingInterface(uri=uri)
        connected = new_drone.connect()
        backend = "litewing"
        if not connected:
            error_msg = f"Failed to connect to LiteWing ESP32 at {uri}. Please check drone WiFi AP and power."
    elif backend in ("pysimverse", "simverse"):
        from backend.drone.pysimverse_interface import PySimverseInterface
        new_drone = PySimverseInterface()
        connected = new_drone.connect()
        backend = "pysimverse"
        if not connected:
            error_msg = "Failed to connect to PySimverse Unity simulator."
    elif backend == "matlab":
        from backend.drone.matlab_interface import MatlabInterface
        new_drone = MatlabInterface()
        connected = new_drone.connect()
        backend = "matlab"
    else:
        from backend.drone.sim_interface import SimInterface
        new_drone = SimInterface()
        connected = new_drone.connect()
        backend = "sim"

    app_context["drone"] = new_drone
    app_context["drone_backend"] = backend
    app_context["drone_uri"] = uri

    executor = app_context.get("executor")
    if executor:
        executor.drone = new_drone

    return {
        "success": connected,
        "drone_backend": backend,
        "drone_uri": uri,
        "connected": connected,
        "error": error_msg,
    }


@router.get("/api/video_feed")
async def video_feed():
    def frame_generator():
        import cv2
        while True:
            source = app_context.get("stream_source", "webcam")
            frame = None
            if source == "webcam":
                cam = app_context.get("camera_manager")
                if cam and hasattr(cam, "read_frame"):
                    frame = cam.read_frame()
            elif source in ("pysimverse", "simverse"):
                drone = app_context.get("drone")
                if drone and hasattr(drone, "get_frame"):
                    frame = drone.get_frame()
            else:
                drone = app_context.get("drone")
                if drone and hasattr(drone, "get_frame"):
                    frame = drone.get_frame()
                if frame is None:
                    cam = app_context.get("camera_manager")
                    if cam and hasattr(cam, "read_frame"):
                        frame = cam.read_frame()

            if frame is not None:
                # Overlay detected objects
                state_mgr = app_context.get("state_manager")
                if state_mgr:
                    for obj in state_mgr.get_all_objects():
                        if obj.bbox:
                            b = obj.bbox
                            cv2.rectangle(frame, (b.u_min, b.v_min), (b.u_max, b.v_max), (0, 240, 255), 2)
                            cv2.putText(
                                frame,
                                f"{obj.label} ({obj.confidence:.2f})",
                                (b.u_min, max(0, b.v_min - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.45,
                                (0, 240, 255),
                                1,
                            )

                ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ret:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n")
            time.sleep(0.06)

    return StreamingResponse(frame_generator(), media_type="multipart/x-mixed-replace; boundary=frame")


@router.get("/api/capabilities")
async def get_capabilities():
    return app_context["capabilities"].model_dump()


@router.post("/api/command")
async def submit_command(req: CommandRequest):
    logger.info(f"Received API command request: '{req.command}'")
    planner = app_context["planner"]
    llm = app_context["llm"]
    executor = app_context["executor"]
    capabilities = app_context["capabilities"]

    if not planner or not llm or not executor:
        return {"success": False, "error": "System not initialized."}

    planner.start_mission(req.command)

    drone = app_context["drone"]
    pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    if drone:
        st = drone.get_state()
        pose = {"x": st.x, "y": st.y, "z": st.z, "yaw": st.yaw}

    state_mgr = app_context.get("state_manager")
    objects = state_mgr.objects_as_context() if state_mgr else []
    context = {
        "pose": pose,
        "objects": objects,
    }

    vla = app_context.get("vla")
    frame = None
    if drone and hasattr(drone, "get_frame"):
        frame = drone.get_frame()
    if frame is None:
        cam_ref = app_context.get("camera_manager")
        if cam_ref and hasattr(cam_ref, "read_frame"):
            frame = cam_ref.read_frame()

    # Use intent router to determine if this requires visual grounding / VLA pipeline
    from backend.planning.intent_router import parse_intent
    intent = parse_intent(req.command, context)
    is_visual_target = intent.intent in ("find", "inspect") or (intent.target is not None and intent.intent != "shape")

    if vla and is_visual_target:
        logger.info("[APIRoutes] Dispatching visual command to UAV-VLA pipeline...")
        plan = vla.process(req.command, frame, capabilities)
    else:
        plan = llm.plan(req.command, capabilities, context)

    # 2. Safety Validation
    validation = planner.process_generated_plan(plan)
    if not validation.is_valid:
        return {
            "success": False,
            "errors": validation.errors,
            "raw_command": req.command
        }

    # 3. Asynchronous Mission Execution (with error propagation)
    async def _run_mission_async(plan):
        try:
            ok = await asyncio.to_thread(executor.execute_plan, plan)
            if ok:
                planner.finish_mission()
            else:
                planner.state = PlannerState.ERROR
                logger.error("[AERIS] Mission execution returned failure.")
        except Exception as exc:
            import traceback
            planner.state = PlannerState.ERROR
            logger.error(f"[AERIS] Mission execution crashed: {exc}\n{traceback.format_exc()}")

    asyncio.create_task(_run_mission_async(validation.validated_plan))

    return {
        "success": True,
        "raw_command": req.command,
        "reasoning": plan.reasoning,
        "skills": [s.model_dump() for s in plan.skills],
        "source": plan.source,
        "pipeline": plan.pipeline,
        "spec": plan.spec.model_dump() if plan.spec else None,
    }


@router.post("/api/clear-path")
async def clear_path():
    drone = app_context["drone"]
    if not drone:
        return {"success": False, "error": "Drone not initialized."}
    ok = drone.clear_path()
    return {"success": bool(ok)}


@router.post("/api/abort")
async def abort_mission():
    monitor = app_context.get("monitor")
    drone = app_context["drone"]
    planner = app_context["planner"]
    if monitor:
        monitor.request_abort()
    if drone:
        drone.emergency_stop()
    if planner:
        planner.state = PlannerState.ERROR
    return {"success": True, "monitor": monitor.snapshot() if monitor else None}


@router.get("/api/mission")
async def mission_status():
    monitor = app_context.get("monitor")
    planner = app_context["planner"]
    return {
        "planner": planner.state.value if planner else "IDLE",
        "monitor": monitor.snapshot() if monitor else None,
    }


@router.post("/api/home")
async def return_home():
    drone = app_context["drone"]
    planner = app_context["planner"]
    monitor = app_context.get("monitor")
    if not drone:
        return {"success": False, "error": "Drone not initialized."}
    if monitor:
        monitor.request_abort()
    ok = drone.reset_home()
    if planner:
        planner.reset()
    return {"success": bool(ok)}


@router.websocket("/ws/telemetry")
async def telemetry_websocket(websocket: WebSocket):
    await websocket.accept()
    logger.info("WebSocket client connected to /ws/telemetry")
    drone = app_context["drone"]
    planner = app_context["planner"]

    try:
        while True:
            state = drone.get_state() if drone else DroneState()
            packet = TelemetryPacket(
                timestamp=time.time(),
                state=state,
                active_skill=state.flight_mode.value,
                planner_status=planner.state.value if planner else "IDLE"
            )
            payload = packet.model_dump()
            monitor = app_context.get("monitor")
            if monitor:
                payload["monitor"] = monitor.snapshot()

            state_mgr = app_context.get("state_manager")
            if state_mgr:
                payload["objects"] = [
                    {
                        "label": obj.label,
                        "confidence": obj.confidence,
                        "world_x": obj.world_x,
                        "world_y": obj.world_y,
                        "world_z": obj.world_z,
                        "bbox": obj.bbox.model_dump() if obj.bbox else None,
                    }
                    for obj in state_mgr.get_all_objects()
                ]

            await websocket.send_json(payload)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected.")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
