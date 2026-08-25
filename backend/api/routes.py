"""
FastAPI REST & WebSocket Route Handlers (Pydantic V2 Compatible).
"""
import asyncio
import time
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
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
    "monitor": None,
    "capabilities": RobotCapabilities()
}


class CommandRequest(BaseModel):
    command: str


@router.get("/api/health")
async def health_check():
    drone = app_context["drone"]
    llm = app_context["llm"]
    return {
        "status": "online",
        "drone_backend": type(drone).__name__ if drone else "none",
        "model_backend": type(llm).__name__ if llm else "none",
        "llm_live": bool(getattr(llm, "is_live", False)),
        "llm_model": getattr(llm, "model_name", None),
    }


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
    context = {
        "pose": pose,
        "objects": [{"label": "red_bottle", "x": 0.5, "y": 0.5, "z": 0.0}],
    }

    plan = llm.plan(req.command, capabilities, context)

    # 2. Safety Validation
    validation = planner.process_generated_plan(plan)
    if not validation.is_valid:
        return {
            "success": False,
            "errors": validation.errors,
            "raw_command": req.command
        }

    # 3. Asynchronous Mission Execution
    def _run_and_finish(plan):
        ok = executor.execute_plan(plan)
        if ok:
            planner.finish_mission()
        else:
            planner.state = PlannerState.ERROR

    asyncio.create_task(asyncio.to_thread(_run_and_finish, validation.validated_plan))

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
            await websocket.send_json(payload)
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        logger.info("WebSocket client disconnected.")
    except Exception as e:
        logger.error(f"WebSocket error: {e}")
