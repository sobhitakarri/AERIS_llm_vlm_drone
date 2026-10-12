"""
FastAPI REST & WebSocket Route Handlers (Pydantic V2 Compatible).
"""
import asyncio
import math
import time
from collections import deque
from pathlib import Path
from typing import Optional

import numpy as np
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from fastapi.responses import Response, StreamingResponse
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
    "replan": None,
    "perception_loop": None,
    "perception": None,
    "scene_service": None,
    "model_mode": "gemini",
    "stream_source": "webcam",
    "camera_view": "overhead",
    "drone_backend": "sim",
    "drone_uri": "udp://192.168.43.42",
    "capabilities": RobotCapabilities(),
    "last_mission": {},
}

# Rolling event log shown on the dashboard.
_events: deque = deque(maxlen=200)


def log_event(message: str, level: str = "info") -> None:
    _events.append({"t": time.time(), "level": level, "msg": message})


class CommandRequest(BaseModel):
    command: str


class ModelConfigRequest(BaseModel):
    model_mode: str  # "gemini" | "local_split" | "qwen"


class StreamConfigRequest(BaseModel):
    stream_source: str  # "twin" | "webcam" | "auto"
    camera_index: Optional[int] = None


class DroneConfigRequest(BaseModel):
    drone_backend: str  # "twin" | "litewing" | "sim"
    drone_uri: Optional[str] = None  # e.g. "udp://192.168.43.42"


class CameraViewRequest(BaseModel):
    camera_view: str


class ObjectMoveRequest(BaseModel):
    name: str
    x: float
    y: float
    z: Optional[float] = None


def _is_twin(drone) -> bool:
    return drone is not None and hasattr(drone, "get_camera_frame")


@router.get("/api/health")
async def health_check():
    drone = app_context["drone"]
    llm = app_context["llm"]
    return {
        "status": "online",
        "framework": "AERIS",
        "drone_backend": type(drone).__name__ if drone else "none",
        "drone_connected": bool(getattr(drone, "is_connected", False)) if drone else False,
        "model_backend": type(llm).__name__ if llm else "none",
        "llm_live": bool(getattr(llm, "is_live", False)),
        "llm_model": getattr(llm, "model_name", None),
        "stream_source": app_context.get("stream_source", "webcam"),
        "control_authority": "flight controller (skills are setpoints; no motor commands leave Python)",
    }


@router.get("/api/config")
async def get_config():
    drone = app_context.get("drone")
    llm = app_context.get("llm")
    vlm = app_context.get("vlm")
    cam = app_context.get("camera_manager")
    caps = app_context.get("capabilities") or RobotCapabilities()
    runtime = getattr(drone, "runtime", {}) or {}

    return {
        "framework": "AERIS",
        "version": "3.0",
        "model_mode": app_context.get("model_mode", "gemini"),
        "model_name": getattr(llm, "model_name", type(llm).__name__ if llm else "None"),
        "vlm_name": getattr(vlm, "model_name", type(vlm).__name__ if vlm else "None"),
        "llm_live": bool(getattr(llm, "is_live", False)),
        "vlm_live": bool(getattr(vlm, "is_live", False)),
        "stream_source": app_context.get("stream_source", "webcam"),
        "camera_view": app_context.get("camera_view", "overhead"),
        "cameras": drone.camera_names() if _is_twin(drone) else ["auto"],
        "camera_index": getattr(cam, "camera_index", 0) if cam else 0,
        "drone_backend": app_context.get("drone_backend", "sim"),
        "drone_uri": app_context.get("drone_uri", "udp://192.168.43.42"),
        "drone_connected": bool(getattr(drone, "is_connected", True)) if drone else False,
        "drone_class": type(drone).__name__ if drone else "None",
        "robot_name": caps.robot_name,
        "airframe": runtime.get("airframe"),
        "scene": runtime.get("scene"),
        "room": runtime.get("room"),
        "home": list(drone.home_position()) if drone else [0.0, 0.0],
        "movable_objects": [n for n, o in ((runtime.get("ground_truth") or {}).get("objects") or {}).items() if o.get("movable")],
        "geofence": caps.workspace.model_dump() if caps else {},
        "supported_skills": caps.supported_skills if caps else [],
    }


@router.post("/api/config/model")
async def set_model_config(req: ModelConfigRequest):
    mode = req.model_mode.lower().strip()
    logger.info(f"[AERIS Config] Switching reasoning model mode to: '{mode}'")
    from backend.api.server import make_models

    llm, vlm, canonical = make_models(mode)
    app_context["llm"] = llm
    app_context["vlm"] = vlm
    app_context["model_mode"] = canonical

    # Everything that holds a model reference must follow the switch.
    vla = app_context.get("vla")
    if vla:
        vla.goal_extractor.llm = llm
        vla.object_search.vlm = vlm
    executor = app_context.get("executor")
    if executor:
        executor.vlm = vlm
    perception = app_context.get("perception")
    if perception:
        perception.vlm = vlm
        perception.trackers.reset()

    log_event(f"Model backend switched to {canonical}")
    return {
        "success": True,
        "model_mode": canonical,
        "llm_model": getattr(llm, "model_name", type(llm).__name__),
        "vlm_model": getattr(vlm, "model_name", type(vlm).__name__),
        "llm_live": bool(getattr(llm, "is_live", False)),
    }


@router.post("/api/config/stream")
async def set_stream_config(req: StreamConfigRequest):
    source = req.stream_source.lower().strip()
    logger.info(f"[AERIS Config] Switching stream source to: '{source}'")
    app_context["stream_source"] = source

    cam = app_context.get("camera_manager")
    if cam and source == "webcam":
        if req.camera_index is not None and int(req.camera_index) != int(getattr(cam, "camera_index", -1)):
            cam.camera_index = req.camera_index
            cam.stop()
        cam.start()
    return {"success": True, "stream_source": source}


@router.post("/api/config/drone")
async def set_drone_config(req: DroneConfigRequest):
    from backend.api.server import make_drone, capabilities_for

    uri = req.drone_uri or app_context.get("drone_uri", "udp://192.168.43.42")
    executor = app_context.get("executor")
    if executor and executor.is_busy():
        return {"success": False, "error": "A mission is in flight. Abort it before switching backends."}
    logger.info(f"[AERIS Config] Switching drone backend to '{req.drone_backend}' (uri: '{uri}')")

    old_drone = app_context.get("drone")
    if old_drone:
        try:
            if old_drone.get_state().is_armed:
                old_drone.land()
            old_drone.disconnect()
        except Exception as e:
            logger.warning(f"Error disconnecting previous drone: {e}")

    new_drone, backend = make_drone(req.drone_backend, uri=uri)
    connected = bool(new_drone.connect())
    error_msg = None
    if not connected:
        error_msg = f"Could not connect to the '{backend}' backend."

    app_context["drone"] = new_drone
    app_context["drone_backend"] = backend
    app_context["drone_uri"] = uri
    app_context["stream_source"] = "twin" if backend == "twin" else app_context.get("stream_source", "webcam")

    caps = capabilities_for(new_drone)
    app_context["capabilities"] = caps
    planner = app_context.get("planner")
    if planner:
        planner.validator.capabilities = caps
    if executor:
        executor.drone = new_drone
        executor.capabilities = caps

    # The perception loop reads frames from the drone, so it must not keep
    # pointing at the backend we just disconnected.
    loop = app_context.get("perception_loop")
    if loop:
        loop.drone = new_drone
    grounding = app_context.get("grounding")
    if grounding:
        grounding.clear_calibration()
        grounding.depth_source = None
    old_scene = app_context.get("scene_service")
    if old_scene:
        old_scene.stop()
        app_context["scene_service"] = None
    if backend == "twin" and grounding:
        from backend.vision.scene_service import SceneService
        scene = SceneService(new_drone, grounding)
        scene.start()
        app_context["scene_service"] = scene

    replan = app_context.get("replan")
    if replan:
        replan.reset_mission()

    log_event(f"Drone backend switched to {backend}" + ("" if connected else " (not connected)"), "info" if connected else "error")
    return {
        "success": connected,
        "drone_backend": backend,
        "drone_uri": uri,
        "connected": connected,
        "error": error_msg,
    }


@router.post("/api/config/camera_view")
async def set_camera_view(req: CameraViewRequest):
    view = req.camera_view.lower().strip()
    app_context["camera_view"] = view
    return {"success": True, "camera_view": view}


# ---------------------------------------------------------------------- video
def _depth_to_colour(depth: np.ndarray) -> np.ndarray:
    import cv2
    near, far = 0.5, 3.2
    norm = np.clip((far - depth) / (far - near), 0, 1)
    return cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO)


def _draw_overhead_overlays(frame: np.ndarray, show_proposals: bool) -> np.ndarray:
    """Perception state drawn on the overhead view: what the system believes."""
    import cv2
    grounding = app_context.get("grounding")
    state_mgr = app_context.get("state_manager")
    drone = app_context.get("drone")
    executor = app_context.get("executor")
    caps = app_context.get("capabilities")
    cal = getattr(grounding, "calibration", None)
    h, w = frame.shape[:2]

    def px(x, y, z=0.0):
        u, v = cal.world_to_pixel(x, y, z)
        return int(round(u * w / cal.image_size[0])), int(round(v * h / cal.image_size[1]))

    if cal is not None:
        ws = caps.workspace
        corners = [px(ws.x_min, ws.y_min), px(ws.x_max, ws.y_min), px(ws.x_max, ws.y_max), px(ws.x_min, ws.y_max)]
        cv2.polylines(frame, [np.array(corners, np.int32)], True, (90, 200, 90), 1, cv2.LINE_AA)
        if show_proposals:
            for p in grounding.refresh_scene(max_age_s=1.0):
                b = p.bbox
                cv2.rectangle(frame, (b.u_min, b.v_min), (b.u_max, b.v_max), (200, 170, 60), 1)
        route = list(getattr(executor, "last_route", []) or [])
        if executor is not None and executor.is_busy() and len(route) > 1:
            pts = np.array([px(*p) for p in route], np.int32)
            cv2.polylines(frame, [pts], False, (60, 190, 255), 2, cv2.LINE_AA)       # amber (BGR)
            for p in pts[1:]:
                cv2.circle(frame, tuple(int(c) for c in p), 4, (60, 190, 255), -1, cv2.LINE_AA)
        if drone is not None:
            st = drone.get_state()
            u, v = px(st.x, st.y, st.z)
            cv2.circle(frame, (u, v), 14, (255, 200, 60), 2, cv2.LINE_AA)            # estimated pose
            nose = px(st.x + 0.2 * math.cos(math.radians(st.yaw)), st.y + 0.2 * math.sin(math.radians(st.yaw)), st.z)
            cv2.line(frame, (u, v), nose, (255, 200, 60), 2, cv2.LINE_AA)

    if state_mgr:
        for obj in state_mgr.get_all_objects():
            if not obj.bbox:
                continue
            b = obj.bbox
            colour = (80, 230, 120) if obj.actionable else (60, 60, 230)
            cv2.rectangle(frame, (b.u_min, b.v_min), (b.u_max, b.v_max), colour, 2)
            tag = obj.label
            if obj.actionable and obj.world_x is not None:
                tag += f"  ({obj.world_x:+.2f}, {obj.world_y:+.2f})"
                if obj.top_z is not None:
                    tag += f" h={obj.top_z:.2f}"
            else:
                tag += "  LOST"
            (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            y0 = max(th + 6, b.v_min)
            cv2.rectangle(frame, (b.u_min, y0 - th - 6), (b.u_min + tw + 6, y0), colour, -1)
            cv2.putText(frame, tag, (b.u_min + 3, y0 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (10, 10, 10), 1, cv2.LINE_AA)
    return frame


def _grab_frame(cam: str, overlay: bool, proposals: bool) -> Optional[np.ndarray]:
    drone = app_context.get("drone")
    source = app_context.get("stream_source", "webcam")
    frame = None
    if _is_twin(drone):
        if cam == "depth":
            depth = drone.get_depth_frame()
            return _depth_to_colour(depth) if depth is not None else None
        frame = drone.get_camera_frame(cam if cam in drone.camera_names() else "overhead")
        if frame is not None and overlay and cam in ("overhead", "auto", ""):
            frame = _draw_overhead_overlays(frame, proposals)
        return frame
    if source == "webcam":
        cam_mgr = app_context.get("camera_manager")
        if cam_mgr and hasattr(cam_mgr, "read_frame"):
            frame = cam_mgr.read_frame(timeout_s=0.0)
    if frame is None and drone is not None and hasattr(drone, "get_frame"):
        frame = drone.get_frame()
    if frame is not None and overlay:
        frame = _draw_overhead_overlays(frame.copy(), False)
    return frame


def _placeholder(text: str) -> np.ndarray:
    import cv2
    img = np.full((360, 640, 3), (24, 22, 20), np.uint8)
    cv2.putText(img, text, (24, 186), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1, cv2.LINE_AA)
    return img


@router.get("/api/video_feed")
async def video_feed(cam: Optional[str] = None, overlay: int = 1, proposals: int = 0):
    def frame_generator():
        import cv2
        while True:
            view = (cam or app_context.get("camera_view", "overhead")).lower()
            try:
                frame = _grab_frame(view, bool(overlay), bool(proposals))
            except Exception as exc:
                logger.warning(f"video feed error: {exc}")
                frame = None
            if frame is None:
                drone = app_context.get("drone")
                waiting = "Waiting for the digital twin (run_aeris_twin in MATLAB)" if _is_twin(drone) else "No camera frame"
                frame = _placeholder(waiting)
            ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
            if ret:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg.tobytes() + b"\r\n")
            time.sleep(0.08)

    return StreamingResponse(frame_generator(), media_type="multipart/x-mixed-replace; boundary=frame")


@router.get("/api/snapshot")
async def snapshot(cam: Optional[str] = None, overlay: int = 1, proposals: int = 0):
    """One JPEG from a camera: for recordings, reports and tests."""
    import cv2
    view = (cam or app_context.get("camera_view", "overhead")).lower()
    frame = await asyncio.to_thread(_grab_frame, view, bool(overlay), bool(proposals))
    if frame is None:
        return Response(status_code=503)
    ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 88])
    return Response(content=jpeg.tobytes(), media_type="image/jpeg") if ok else Response(status_code=500)


@router.get("/api/capabilities")
async def get_capabilities():
    return app_context["capabilities"].model_dump()


# -------------------------------------------------------------------- mission
def _pose_context() -> dict:
    drone = app_context["drone"]
    pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
    if drone:
        st = drone.get_state()
        pose = {"x": st.x, "y": st.y, "z": st.z, "yaw": st.yaw}
    state_mgr = app_context.get("state_manager")
    return {"pose": pose, "objects": state_mgr.objects_as_context() if state_mgr else []}


def _scene_frame():
    drone = app_context["drone"]
    frame = drone.get_frame() if drone and hasattr(drone, "get_frame") else None
    if frame is None:
        cam_ref = app_context.get("camera_manager")
        if cam_ref and hasattr(cam_ref, "read_frame"):
            frame = cam_ref.read_frame()
    return frame


@router.post("/api/command")
async def submit_command(req: CommandRequest):
    logger.info(f"Received API command request: '{req.command}'")
    planner = app_context["planner"]
    llm = app_context["llm"]
    executor = app_context["executor"]
    capabilities = app_context["capabilities"]
    drone = app_context["drone"]

    if not planner or not llm or not executor:
        return {"success": False, "error": "System not initialized."}
    if executor.is_busy():
        return {
            "success": False,
            "error": "A mission is already in flight. Abort it before sending a new command.",
            "raw_command": req.command,
        }
    if drone is not None and getattr(drone, "is_connected", True) is False:
        return {"success": False, "error": "The drone backend is not connected.", "raw_command": req.command}

    log_event(f'Command: "{req.command}"')
    planner.start_mission(req.command)
    context = _pose_context()

    from backend.planning.intent_router import parse_intent
    intent = parse_intent(req.command, context)
    vla = app_context.get("vla")
    perception = app_context.get("perception")
    replan = app_context.get("replan")
    if replan:
        replan.reset_mission()
    # A new mission starts from a clean perception state.
    if perception and hasattr(perception, "set_targets"):
        perception.set_targets([])
    state_mgr = app_context.get("state_manager")
    if state_mgr:
        state_mgr.clear_objects()

    if vla and intent.intent == "vision":
        goal = vla.goal_extractor.extract_goal(req.command)
        if perception and hasattr(perception, "set_targets"):
            perception.set_targets([])
        frame = await asyncio.to_thread(_scene_frame)
        plan = await asyncio.to_thread(vla.process, req.command, frame, capabilities, context["pose"], goal)
        # Hand the located object to the perception loop, which keeps it
        # tracked for the rest of the mission (and for replanning) without
        # another model call.
        if perception and plan.skills and vla.object_search.last_detection is not None:
            perception.adopt(vla.object_search.last_detection)
    else:
        if perception and hasattr(perception, "set_targets"):
            perception.set_targets([])
        plan = await asyncio.to_thread(llm.plan, req.command, capabilities, context)

    if not plan.skills:
        planner.state = PlannerState.ERROR
        log_event(f"No plan: {plan.reasoning}", "error")
        return {
            "success": False,
            "error": plan.reasoning or "No plan could be generated for this command.",
            "raw_command": req.command,
            "source": plan.source,
            "pipeline": plan.pipeline,
        }

    try:
        from backend.planning import plan_cache
        # Plans built from a live detection depend on where the object was;
        # only cache the ones that are a pure function of the command text.
        if plan.source != "uav-vla":
            plan_cache.store_entry(
                command=req.command,
                request_payload={"command": req.command, "context": context},
                response_payload={"reasoning": plan.reasoning, "source": plan.source, "pipeline": plan.pipeline},
                skills=plan.skills,
                allow_template=True,
            )
    except Exception as e:
        logger.warning(f"[APIRoutes] Failed to save mission plan to cache: {e}")

    validation = planner.process_generated_plan(plan)
    if not validation.is_valid:
        log_event("Plan rejected by the safety validator: " + "; ".join(validation.errors), "error")
        return {"success": False, "errors": validation.errors, "raw_command": req.command}

    async def _run_mission_async(valid_plan):
        try:
            ok = await asyncio.to_thread(executor.execute_plan, valid_plan)
            if ok:
                planner.finish_mission()
                log_event("Mission complete.")
            else:
                planner.state = PlannerState.ERROR
                log_event(f"Mission failed: {executor.last_error or 'see backend log'}", "error")
        except Exception as exc:
            import traceback
            planner.state = PlannerState.ERROR
            log_event(f"Mission crashed: {exc}", "error")
            logger.error(f"[AERIS] Mission execution crashed: {exc}\n{traceback.format_exc()}")

    asyncio.create_task(_run_mission_async(validation.validated_plan))
    log_event(f"Dispatched {len(plan.skills)} skills [{plan.source or 'planner'}]")
    # Published on the telemetry socket so every open dashboard shows the
    # mission, whichever client started it.
    app_context["last_plan"] = {
        "id": time.time(), "command": req.command, "reasoning": plan.reasoning,
        "source": plan.source, "skills": [s.model_dump() for s in plan.skills],
    }

    return {
        "success": True,
        "raw_command": req.command,
        "reasoning": plan.reasoning,
        "skills": [s.model_dump() for s in plan.skills],
        "source": plan.source,
        "pipeline": plan.pipeline,
        "spec": plan.spec.model_dump() if plan.spec else None,
    }


@router.get("/api/cache/status")
async def get_cache_status():
    """Status of the on-disk caches (plans, calibration, detection snapshots)."""
    from backend.planning import plan_cache
    store = plan_cache.load_store()
    exact_keys = list(store.get("exact", {}).keys())
    template_keys = list(store.get("templates", {}).keys())

    cache_root = Path(__file__).resolve().parents[2] / "cache"
    identified_dir = cache_root / "identified"
    snapshots = sorted(f.name for f in identified_dir.glob("*.jpg")) if identified_dir.exists() else []
    return {
        "cache_ok": True,
        "mission_plans": {
            "file_exists": (cache_root / "mission_plans.json").exists(),
            "exact_cached_count": len(exact_keys),
            "template_cached_count": len(template_keys),
            "recent_keys": exact_keys[-10:],
        },
        "identified_targets": {
            "directory_exists": identified_dir.exists(),
            "snapshot_count": len(snapshots),
            "recent_snapshots": snapshots[-10:],
        },
        "homography": {"file_exists": (cache_root / "homography.json").exists()},
    }


@router.post("/api/clear-path")
async def clear_path():
    drone = app_context["drone"]
    if not drone:
        return {"success": False, "error": "Drone not initialized."}
    return {"success": bool(drone.clear_path())}


@router.post("/api/abort")
async def abort_mission():
    """Emergency stop: ends the mission and cuts the motors."""
    monitor = app_context.get("monitor")
    drone = app_context["drone"]
    planner = app_context["planner"]
    if monitor:
        monitor.request_abort()
    if drone:
        drone.emergency_stop()
    if planner:
        planner.state = PlannerState.ERROR
    log_event("EMERGENCY STOP: motors cut.", "error")
    return {"success": True, "monitor": monitor.snapshot() if monitor else None}


@router.post("/api/land")
async def land_now():
    drone = app_context["drone"]
    if not drone:
        return {"success": False, "error": "Drone not initialized."}
    executor = app_context.get("executor")
    if executor and executor.is_busy():
        return {"success": False, "error": "A mission is running. Use Stop first, or wait for it to finish."}
    ok = await asyncio.to_thread(drone.land)
    log_event("Landed." if ok else "Landing failed.", "info" if ok else "error")
    return {"success": bool(ok)}


@router.get("/api/mission")
async def mission_status():
    monitor = app_context.get("monitor")
    planner = app_context["planner"]
    executor = app_context.get("executor")
    return {
        "planner": planner.state.value if planner else "IDLE",
        "monitor": monitor.snapshot() if monitor else None,
        "last_error": getattr(executor, "last_error", None),
    }


@router.post("/api/home")
async def return_home():
    drone = app_context["drone"]
    planner = app_context["planner"]
    executor = app_context.get("executor")
    if not drone:
        return {"success": False, "error": "Drone not initialized."}
    # A mission reports "done" a moment before its worker thread is free.
    for _ in range(30):
        if not (executor and executor.is_busy()):
            break
        await asyncio.sleep(0.1)
    if executor and executor.is_busy():
        return {"success": False, "error": "A mission is running. Use Stop first, or wait for it to finish."}
    # Through the executor, so the way home is routed around obstacles.
    ok = await asyncio.to_thread(executor.return_home) if executor else await asyncio.to_thread(drone.reset_home)
    if planner:
        planner.reset()
    why = getattr(executor, "last_error", None)
    log_event("Returned home." if ok else f"Return home failed: {why or 'see backend log'}", "info" if ok else "error")
    return {"success": bool(ok), "error": None if ok else why}


# ----------------------------------------------------------------------- twin
@router.post("/api/twin/restart")
async def twin_restart():
    drone = app_context["drone"]
    executor = app_context.get("executor")
    if not _is_twin(drone):
        return {"success": False, "error": "The active backend is not the digital twin."}
    if executor and executor.is_busy():
        return {"success": False, "error": "A mission is running."}
    ok = drone.restart()
    planner = app_context.get("planner")
    if planner:
        planner.reset()
    log_event("Digital twin restarting from its initial state.")
    return {"success": bool(ok)}


@router.post("/api/twin/object")
async def twin_move_object(req: ObjectMoveRequest):
    """Move a movable scene object: a controlled disturbance for replanning tests."""
    drone = app_context["drone"]
    if not _is_twin(drone):
        return {"success": False, "error": "The active backend is not the digital twin."}
    ok = drone.set_object(req.name, req.x, req.y, req.z)
    log_event(f"Disturbance: moved '{req.name}' to ({req.x:.2f}, {req.y:.2f})")
    return {"success": bool(ok)}


@router.get("/api/scene")
async def scene_status():
    """What the perception stack currently knows about the scene."""
    grounding = app_context.get("grounding")
    scene = app_context.get("scene_service")
    perception = app_context.get("perception")
    proposals = []
    if grounding is not None and grounding.has_depth:
        proposals = [p.as_dict() for p in await asyncio.to_thread(grounding.refresh_scene, 1.0)]
    return {
        "calibration": scene.status() if scene else {"calibrated": bool(getattr(grounding, "calibrated", False))},
        "proposals": proposals,
        "targets": list(getattr(perception, "active_targets", [])),
        "rejected_ungrounded": getattr(perception, "rejected_ungrounded", 0),
        "trackers": perception.trackers.snapshot() if perception else {},
    }


@router.get("/api/eval")
async def evaluation():
    """Scoring against simulator ground truth. Evaluation only: none of this
    is available to the planner or the perception stack."""
    drone = app_context["drone"]
    state_mgr = app_context.get("state_manager")
    if not _is_twin(drone):
        return {"available": False}
    gt = drone.ground_truth()
    truth = drone.truth or {}
    st = drone.get_state()
    rows = []
    for obj in (state_mgr.get_all_objects() if state_mgr else []):
        if obj.world_x is None:
            continue
        name, best = None, None
        for gname, g in gt.items():
            d = math.hypot(g["pos"][0] - obj.world_x, g["pos"][1] - obj.world_y)
            if best is None or d < best:
                name, best = gname, d
        g = gt.get(name, {})
        rows.append({
            "label": obj.label, "nearest_truth": name,
            "error_cm": round(best * 100, 1) if best is not None else None,
            "top_error_cm": round((obj.top_z - (g["pos"][2] + g["size_m"][2])) * 100, 1) if obj.top_z is not None and g else None,
        })
    return {
        "available": True,
        "estimator_error_cm": round(100 * math.dist((st.x, st.y, st.z), (truth.get("x", st.x), truth.get("y", st.y), truth.get("z", st.z))), 1) if truth else None,
        "yaw_error_deg": round(((st.yaw - truth.get("yaw", st.yaw) + 180) % 360) - 180, 1) if truth else None,
        "objects": rows,
    }


# ------------------------------------------------------------------ telemetry
def _telemetry_payload() -> dict:
    # Read the live references each tick: the backend can be switched.
    drone = app_context["drone"]
    planner = app_context["planner"]
    state = drone.get_state() if drone else DroneState()
    packet = TelemetryPacket(
        timestamp=time.time(),
        state=state,
        active_skill=state.flight_mode.value,
        planner_status=planner.state.value if planner else "IDLE",
    )
    payload = packet.model_dump()
    payload["connected"] = bool(getattr(drone, "is_connected", True)) if drone else False
    monitor = app_context.get("monitor")
    if monitor:
        payload["monitor"] = monitor.snapshot()
    replan = app_context.get("replan")
    if replan:
        payload["replan"] = replan.snapshot()
    if planner:
        payload["replan_count"] = getattr(planner, "replan_count", 0)
    executor = app_context.get("executor")
    if executor:
        payload["last_error"] = executor.last_error
        payload["route"] = [[round(c, 3) for c in p] for p in (executor.last_route if executor.is_busy() else [])]

    if _is_twin(drone):
        tel = drone.telemetry or {}
        payload["twin"] = {
            "truth": drone.truth or None,
            "motors": tel.get("motors"),
            "current_a": tel.get("current_a"),
            "range": tel.get("range"),
            "sim_time": tel.get("t"),
            "saturated": tel.get("saturated"),
            "target": tel.get("target"),
        }
    scene = app_context.get("scene_service")
    if scene:
        payload["scene"] = scene.status()

    perception = app_context.get("perception")
    state_mgr = app_context.get("state_manager")
    if state_mgr:
        payload["objects"] = [
            {
                "label": obj.label,
                "confidence": obj.confidence,
                "world_x": obj.world_x,
                "world_y": obj.world_y,
                "world_z": obj.world_z,
                "top_z": obj.top_z,
                "support_z": obj.support_z,
                "grounding": obj.grounding,
                "actionable": obj.actionable,
                "bbox": obj.bbox.model_dump() if obj.bbox else None,
            }
            for obj in state_mgr.get_all_objects()
        ]
    if perception:
        payload["targets"] = list(getattr(perception, "active_targets", []))
    return payload


@router.get("/api/telemetry")
async def telemetry_once():
    """The telemetry packet the websocket streams, for clients that poll."""
    return _telemetry_payload()


@router.websocket("/ws/telemetry")
async def telemetry_websocket(websocket: WebSocket):
    await websocket.accept()
    logger.info("WebSocket client connected to /ws/telemetry")
    last_event_t = 0.0
    last_plan_id = None

    try:
        while True:
            payload = _telemetry_payload()

            plan = app_context.get("last_plan")
            if plan and plan["id"] != last_plan_id:
                payload["plan"] = plan
                last_plan_id = plan["id"]

            fresh = [e for e in _events if e["t"] > last_event_t]
            if fresh:
                payload["events"] = fresh
                last_event_t = fresh[-1]["t"]

            await websocket.send_json(payload)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected.")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
