"""
LiteWing flight console API.

The console (frontend/litewing.html) is the Circuit-Digest positioning-module
GUI rebuilt on the web: live values, four plots, PID tuning, LEDs, joystick,
sensor test and an output log. The one difference is who gives the orders: a
sentence goes through the usual /api/command (LLM -> skills -> validator ->
executor), and the buttons here build the same skills by rule.
"""
from __future__ import annotations

import asyncio
import time
from typing import Any, Dict, Optional

from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

from backend.api.routes import DroneConfigRequest, _events, app_context, log_event, set_drone_config
from backend.core.logger import get_logger
from backend.drone.litewing_interface import LiteWingInterface
from backend.planning.planner import PlannerState
from backend.schemas.mission import MissionPlan, SkillPrimitive

logger = get_logger("LiteWingAPI")
router = APIRouter(prefix="/api/litewing")

_DIRECTIONS = {"forward": (1.0, 0.0), "backward": (-1.0, 0.0), "left": (0.0, 1.0), "right": (0.0, -1.0)}


class ConnectRequest(BaseModel):
    uri: Optional[str] = None
    debug: Optional[bool] = None


class ParamsRequest(BaseModel):
    values: Dict[str, Any]


class FlagRequest(BaseModel):
    enabled: bool


class SensorTestRequest(BaseModel):
    action: str = "start"             # start | stop
    log_csv: bool = False


class LedRequest(BaseModel):
    action: str                       # set | blink | stop_blink | clear
    r: int = 255
    g: int = 255
    b: int = 255


class FlightRequest(BaseModel):
    kind: str                         # start_flight | square | forward | backward | left | right
    distance: Optional[float] = None
    hover_s: Optional[float] = None


class JoystickRequest(BaseModel):
    action: str                       # start | stop | key | hold
    key: Optional[str] = None
    down: bool = True
    hold_origin: Optional[bool] = None


def _lw() -> LiteWingInterface:
    drone = app_context.get("drone")
    if not isinstance(drone, LiteWingInterface):
        raise HTTPException(409, "The active vehicle is not a LiteWing. Press Connect first.")
    return drone


def _err(msg: Optional[str]) -> dict:
    return {"success": msg is None, "error": msg}


# ----------------------------------------------------------------- connection
def _is_own_address(uri: str) -> bool:
    """True when the URI's host is this PC (e.g. its 192.168.43.43 on the drone's Wi-Fi)."""
    import socket
    from urllib.parse import urlparse
    host = urlparse(uri if "://" in uri else "udp://" + uri).hostname
    if not host or uri.startswith("sim://"):
        return False
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect((host, 9))
        local = s.getsockname()[0]
        s.close()
        return local == host or host in ("127.0.0.1", "localhost")
    except OSError:
        return False


@router.post("/connect")
async def connect(req: ConnectRequest):
    if req.uri and _is_own_address(req.uri):
        # Checked first: a reconnect tears the working link down before it tries the new one.
        return {"success": False, "connected": False,
                "error": f"{req.uri} is this PC's own address, not the drone's. The LiteWing is at 192.168.43.42."}
    res = await set_drone_config(DroneConfigRequest(drone_backend="litewing", drone_uri=req.uri))
    drone = app_context.get("drone")
    if isinstance(drone, LiteWingInterface) and req.debug is not None and not drone.state.is_armed:
        drone.set_debug(req.debug)
    return res


@router.post("/disconnect")
async def disconnect():
    drone = _lw()
    if drone.state.is_armed:
        return {"success": False, "error": "Land first."}
    await asyncio.to_thread(drone.disconnect)
    return {"success": True}


@router.get("/status")
async def status():
    drone = app_context.get("drone")
    if not isinstance(drone, LiteWingInterface):
        return {"active": False, "backend": app_context.get("drone_backend")}
    return {"active": True, **_snapshot(drone)}


# ---------------------------------------------------------------- parameters
@router.post("/params")
async def set_params(req: ParamsRequest):
    return {"success": True, **_lw().apply_params(req.values)}


@router.post("/params/reset")
async def reset_params():
    return {"success": True, **_lw().reset_params()}


@router.post("/debug")
async def set_debug(req: FlagRequest):
    return _err(_lw().set_debug(req.enabled))


@router.post("/battery/reset")
async def reset_battery():
    _lw().reset_battery()
    return {"success": True}


@router.post("/csv")
async def set_csv(req: FlagRequest):
    path = _lw().set_csv(req.enabled)
    return {"success": True, "path": path}


@router.post("/sensor_test")
async def sensor_test(req: SensorTestRequest):
    drone = _lw()
    if req.action == "stop":
        drone.stop_sensor_test()
        return {"success": True}
    return _err(drone.start_sensor_test(req.log_csv))


class MotorTestRequest(BaseModel):
    percent: float = 8.0
    seconds: float = 1.5
    confirmed: bool = False           # the operator must say the drone is on a flat surface, clear of everything
    props_fitted: bool = False        # recorded in the log; capped thrust cannot lift the drone either way


def _motor_gate(req_confirmed: bool):
    if not req_confirmed:
        return {"success": False, "error": "Confirm the drone is on a flat surface with the area clear (confirmed)."}
    executor = app_context.get("executor")
    if executor and executor.is_busy():
        return {"success": False, "error": "A mission is running."}
    return None


@router.post("/prop_test")
async def prop_test(req: MotorTestRequest):
    """The firmware's own per-motor propeller self-test (vibration + battery), ground only."""
    drone = _lw()
    refused = _motor_gate(req.confirmed)
    if refused:
        return refused
    res = await asyncio.to_thread(drone.prop_test)
    log_event("Prop test: " + ("all motors vibrated" if res.get("success") else f"{res.get('error')}"),
              "info" if res.get("success") else "error")
    return res


@router.post("/motor_test")
async def motor_test(req: MotorTestRequest):
    """Ground test of the motors: low raw thrust, a few seconds, never a flight."""
    drone = _lw()
    refused = _motor_gate(req.confirmed)
    if refused:
        return refused
    drone.log(f"Motor test requested; props {'FITTED' if req.props_fitted else 'removed'}")
    res = await asyncio.to_thread(drone.motor_test, req.percent, req.seconds)
    log_event("Motor test: " + ("done" if res.get("success") else f"not run or stopped: {res.get('error')}"),
              "info" if res.get("success") else "error")
    return res


class CalibrateRequest(BaseModel):
    distance: float


@router.post("/calibrate")
async def calibrate(req: CalibrateRequest):
    return _lw().calibrate_flow(req.distance)


@router.post("/led")
async def led(req: LedRequest):
    drone = _lw()
    fn = {
        "set": lambda: drone.led_color(req.r, req.g, req.b),
        "blink": lambda: drone.led_blink(True),
        "stop_blink": lambda: drone.led_blink(False),
        "clear": drone.led_clear,
    }.get(req.action)
    if fn is None:
        raise HTTPException(400, f"unknown LED action '{req.action}'")
    return _err(await asyncio.to_thread(fn))


# -------------------------------------------------------------------- joystick
@router.post("/joystick")
async def joystick(req: JoystickRequest):
    drone = _lw()
    executor = app_context.get("executor")
    if req.action == "start":
        if executor and executor.is_busy():
            return {"success": False, "error": "A mission is running."}
        return _err(await asyncio.to_thread(drone.manual_start))
    if req.action == "stop":
        await asyncio.to_thread(drone.manual_stop)
        return {"success": True}
    if req.action == "hold":
        drone.hold_origin = bool(req.hold_origin)
        return {"success": True, "hold_origin": drone.hold_origin}
    if req.action == "key":
        drone.manual_key(req.key or "", req.down)
        return {"success": True}
    raise HTTPException(400, f"unknown joystick action '{req.action}'")


# --------------------------------------------------------------------- flights
def _build_skills(drone: LiteWingInterface, req: FlightRequest):
    cfg = drone.cfg
    d = max(0.1, min(float(req.distance if req.distance else cfg.maneuver_distance), 1.0))
    z = cfg.target_height
    here = drone.get_state()
    airborne = drone.state.is_armed
    x0, y0 = (here.x, here.y) if airborne else (0.0, 0.0)

    def mv(x, y):
        return SkillPrimitive(skill="MOVE_TO", params={"x": round(x, 3), "y": round(y, 3), "z": z, "speed": 0.4})

    if req.kind == "start_flight":
        label = f"Start flight: hover {req.hover_s or cfg.hover_duration:.0f} s"
        body = [SkillPrimitive(skill="HOVER", params={"t": float(req.hover_s or cfg.hover_duration)})]
    elif req.kind == "square":
        # the official order: right, forward, left, back home (+X is left there)
        label = f"Square {d:.2f} m"
        body = [mv(x0, y0 - d), mv(x0 + d, y0 - d), mv(x0 + d, y0), mv(x0, y0)]
    elif req.kind in _DIRECTIONS:
        fx, fy = _DIRECTIONS[req.kind]
        label = f"{req.kind.capitalize()} {d:.2f} m"
        body = [mv(x0 + fx * d, y0 + fy * d)]
    else:
        raise HTTPException(400, f"unknown flight '{req.kind}'")
    skills = []
    if not airborne:
        skills.append(SkillPrimitive(skill="TAKEOFF", params={"z": z, "speed": 0.4}))
    skills += body
    if not airborne:
        skills += [SkillPrimitive(skill="HOVER", params={"t": 1.0}), SkillPrimitive(skill="LAND", params={})]
    return label, skills


@router.post("/flight")
async def flight(req: FlightRequest):
    drone = _lw()
    planner, executor = app_context["planner"], app_context["executor"]
    if executor.is_busy():
        return {"success": False, "error": "A mission is already in flight."}
    if drone.manual_active:
        return {"success": False, "error": "Stop the joystick session first."}
    label, skills = _build_skills(drone, req)
    plan = MissionPlan(raw_command=label, reasoning="Console button (rule-built skills, no LLM).",
                       skills=skills, source="manual")
    planner.start_mission(label)
    validation = planner.process_generated_plan(plan)
    if not validation.is_valid:
        log_event("Plan rejected by the safety validator: " + "; ".join(validation.errors), "error")
        return {"success": False, "errors": validation.errors}

    async def _run():
        try:
            ok = await asyncio.to_thread(executor.execute_plan, validation.validated_plan)
            if ok:
                planner.finish_mission()
                log_event(f"{label}: complete.")
            else:
                planner.state = PlannerState.ERROR
                log_event(f"{label}: failed ({executor.last_error or 'see Output Log'}).", "error")
        except Exception as exc:                    # noqa: BLE001
            planner.state = PlannerState.ERROR
            log_event(f"{label}: crashed: {exc}", "error")

    asyncio.create_task(_run())
    app_context["last_plan"] = {"id": time.time(), "command": label, "reasoning": plan.reasoning,
                                "source": "manual", "skills": [s.model_dump() for s in skills], "clips": []}
    log_event(f"{label}: dispatched {len(skills)} skills")
    return {"success": True, "label": label, "skills": [s.model_dump() for s in skills]}


@router.post("/stop")
async def stop():
    """The console STOP: end the mission or session and bring the drone down under control."""
    drone = _lw()
    planner = app_context.get("planner")
    executor = app_context.get("executor")
    # Not monitor.request_abort(): the executor answers that with a motor cut.
    # The drone's own flag ends its loops; the failed step then lands it.
    drone._abort.set()
    was_manual = drone.manual_active
    if was_manual:
        await asyncio.to_thread(drone.manual_stop)
    else:
        for _ in range(40):                 # the mission thread winds down, then we land
            if not (executor and executor.is_busy()):
                break
            await asyncio.sleep(0.05)
        if drone.state.is_armed:
            await asyncio.to_thread(drone.land)
    if planner:
        planner.state = PlannerState.IDLE
    log_event("STOP: landed under control.", "error")
    return {"success": True}


# ---------------------------------------------------------------- dashboard feed
def _snapshot(drone: LiteWingInterface) -> dict:
    state = drone.dashboard_state()
    monitor, planner, executor = app_context.get("monitor"), app_context.get("planner"), app_context.get("executor")
    state["mission"] = {
        "planner": planner.state.value if planner else "IDLE",
        "monitor": monitor.snapshot() if monitor else None,
        "busy": bool(executor and executor.is_busy()),
        "error": getattr(executor, "last_error", None),
        "route": [[round(p[0], 3), round(p[1], 3)] for p in (getattr(executor, "last_route", None) or [])],
    }
    state["model"] = app_context.get("model_mode")
    return state


@router.websocket("/ws")
async def ws(websocket: WebSocket):
    await websocket.accept()
    seq, epoch, log_seq = 0, -1, 0
    last_event_t, last_plan_id = 0.0, None
    try:
        while True:
            drone = app_context.get("drone")
            if not isinstance(drone, LiteWingInterface):
                await websocket.send_json({"active": False, "backend": app_context.get("drone_backend")})
                await asyncio.sleep(0.5)
                continue
            try:
                payload = {"active": True, **_snapshot(drone)}
            except Exception as exc:                # noqa: BLE001 - tell the page, keep the socket
                logger.error("LiteWing snapshot failed: %s", exc, exc_info=True)
                await websocket.send_json({"active": True, "error": f"{type(exc).__name__}: {exc}"})
                await asyncio.sleep(1.0)
                continue
            hist = drone.rec.since(seq, epoch)
            seq, epoch = hist["seq"], hist["epoch"]
            payload["history"] = hist
            lg = drone.output.since(log_seq)
            log_seq = lg["seq"]
            payload["log"] = lg["lines"]
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
        pass
    except Exception as exc:                        # noqa: BLE001
        logger.error("LiteWing websocket error: %s", exc)
