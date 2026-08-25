# LLM–VLM Drone Framework — Project Scaffold Implementation Plan

## Goal
Build the complete, modular project scaffold for the **Model-Agnostic Closed-Loop Autonomy Framework with a Foundation-Model Layer** using:
1. **Backend**: Python AI/mission layer (`schemas`, `core`, `models`, `planning`, `runtime`, `vision`, `drone`, `api`, `tests`)
2. **Frontend**: Lightweight Web-based operator dashboard (HTML/CSS/JS)
3. **MATLAB**: Simulink/Simscape digital twin & control design environment (`init_params`, controllers, simulation runners, homography utils)

All code lives under `d:\UG\B.TECH\7th\Project_phase1\`.

---

## Key Research Architecture & Design Principles

> **Core Research Claim:** A resource-constrained closed-loop embodied-AI framework that converts natural-language goals into validated drone skills, grounds visually perceived targets into real-world coordinates, executes skills on a LiteWing ESP32-S3 UAV (or simulator), and adaptively replans using visual feedback.

### 1. Two-Level Safety Architecture
- **AI-side Safety Validator (Pre-Execution):** Checks LLM/DSL outputs against workspace geofence (`x, y, z`), maximum displacement, action validity, capability schema, target existence, and battery state before the plan hits the executor.
- **Firmware-side Safety (Embedded):** Watchdog, communication timeouts, emergency stop, and physical limits. Never let the LLM be a safety authority.

### 2. High-Level Skill DSL & Robot Capabilities Schema
- **Skill DSL as Central Interface:** Standardized skill primitives (`TAKEOFF`, `LAND`, `HOVER`, `MOVE_TO`, `ROTATE`, `CIRCLE`, `FIND`, `INSPECT`, `RETURN`).
- **Capability Generalization:** The LLM is provided explicit `RobotCapabilities` schema so it only selects valid skills for the target hardware.

### 3. Model-Agnostic Provider Architecture
- `LLMProvider` & `VLMProvider` abstractions.
- Initial backend: **Gemini 2.5 Flash** using the modern `google-genai` Python SDK.
- Future local backend: **Qwen3 / Qwen3-VL** via OpenAI-compatible endpoints.

### 4. Hybrid Perception Stack & Spatial Grounding
- **Fast CV (OpenCV/YOLO):** High-frequency object tracking & bounding box extraction.
- **VLM:** Semantic target resolution (maps user natural language to detected visual object tags on trigger).
- **Spatial Grounding:** Homography matrix transforms pixel center $(u, v) \rightarrow (X, Y, Z)$ physical workspace coordinates.

### 5. Dual-Target Drone Interface (Sim-to-Real Contract)
- `DroneInterface` abstraction with:
  - `LiteWingInterface`: Real ESP32-S3 UAV via `cflib` (Crazyflie CRTP protocol over UDP `udp://192.168.43.42`).
  - `SimInterface`: Desktop simulator mock following the exact same contract.

---

## User Review Required

> [!IMPORTANT]
> - `google-generativeai` has been replaced with the modern `google-genai` SDK in `requirements.txt`.
> - LiteWing communication is standardized on `cflib` (CRTP over UDP), matching the official LiteWing ESP-IDF firmware.
> - Simulink models (`.slx`) will be built interactively in later steps, while MATLAB parameter initialization (`init_params.m`) and standalone controller scripts (`.m`) are scaffolded now.

---

## Proposed Folder & File Structure

```text
Project_phase1/
├── backend/
│   ├── __init__.py
│   ├── main.py                        # Entry point (python -m backend.main --backend gemini --drone sim)
│   ├── requirements.txt               # Core runtime dependencies (fastapi, pydantic, google-genai, opencv, cflib)
│   ├── requirements-dev.txt           # Test/Dev tools (pytest, httpx)
│   │
│   ├── core/
│   │   ├── __init__.py
│   │   ├── config.py                  # Central configuration dataclass & env settings
│   │   └── logger.py                  # Structured JSON logger
│   │
│   ├── schemas/
│   │   ├── __init__.py
│   │   ├── capabilities.py            # RobotCapabilities & Workspace boundaries
│   │   ├── mission.py                 # SkillPrimitive, MissionPlan, PlanValidationResult
│   │   ├── perception.py              # BoundingBox, DetectedObject, PerceptionResult
│   │   └── telemetry.py               # DroneState, TelemetryPacket, BatteryStatus
│   │
│   ├── models/
│   │   ├── __init__.py
│   │   ├── base_llm.py                # Abstract LLMProvider base class
│   │   ├── base_vlm.py                # Abstract VLMProvider base class
│   │   ├── gemini_provider.py         # Gemini 2.5 Flash implementation (using google-genai)
│   │   └── qwen_provider.py           # Local Qwen3-VL provider stub
│   │
│   ├── planning/
│   │   ├── __init__.py
│   │   ├── skill_dsl.py               # Skill primitive definitions & JSON schemas
│   │   ├── safety_validator.py        # Pre-execution geofence, velocity & capability validator
│   │   └── planner.py                 # Mission Planner FSM (IDLE -> PLANNING -> EXECUTING -> REPLANNING -> DONE)
│   │
│   ├── runtime/
│   │   ├── __init__.py
│   │   ├── mission_executor.py        # Sequentially dispatches validated skills to DroneInterface
│   │   ├── state_manager.py           # Central state store for drone telemetry & world objects
│   │   └── event_bus.py               # In-memory pub/sub for decoupled events
│   │
│   ├── vision/
│   │   ├── __init__.py
│   │   ├── camera_manager.py          # Frame capture & MJPEG web stream generator
│   │   ├── fast_cv.py                 # High-frequency OpenCV/YOLO target tracking
│   │   └── spatial_grounding.py       # Homography transformation (pixel u,v -> world X,Y,Z)
│   │
│   ├── drone/
│   │   ├── __init__.py
│   │   ├── base_interface.py          # Abstract DroneInterface class
│   │   ├── litewing_interface.py      # Real LiteWing ESP32 interface via cflib CRTP/UDP
│   │   └── sim_interface.py           # Desktop virtual drone mock interface
│   │
│   ├── api/
│   │   ├── __init__.py
│   │   ├── server.py                  # FastAPI application setup
│   │   └── routes.py                  # REST endpoints & WebSocket live telemetry connection
│   │
│   └── tests/
│       ├── __init__.py
│       ├── test_skill_dsl.py
│       ├── test_safety_validator.py
│       ├── test_planner.py
│       ├── test_grounding.py
│       └── test_sim_interface.py
│
├── frontend/
│   ├── index.html                     # Responsive operator dashboard
│   ├── css/
│   │   └── style.css                  # Clean dark mode layout
│   └── js/
│       ├── app.js                     # Main UI controller & status wiring
│       ├── websocket.js               # Backend WebSocket telemetry consumer
│       ├── telemetry.js               # Real-time state & metric gauges
│       └── mission.js                 # Natural language input & skill execution display
│
└── matlab/
    ├── README.md                      # Guide for running scripts & building Simulink twin
    ├── scripts/
    │   ├── init_params.m              # LiteWing mass, inertia, motor constant definitions
    │   ├── run_simulation.m           # Scripted trajectory evaluation runner
    │   └── plot_results.m             # Standardized plotting utility
    ├── controllers/
    │   ├── attitude_pid.m             # Roll/Pitch/Yaw cascaded PID logic
    │   ├── altitude_controller.m      # ToF / Baro altitude PID controller
    │   └── position_controller.m      # Outer-loop XY position controller
    ├── utils/
    │   ├── homography_utils.m         # MATLAB-side camera calibration & homography tools
    │   └── trajectory_gen.m           # Waypoint trajectory generation helper
    └── models/
        └── README.md                  # Instructions for `.slx` Digital Twin creation
```

---

## Detailed Components to Implement

### Component 1: Core & Schemas
- `backend/schemas/capabilities.py`: `RobotCapabilities` with allowed actions list and workspace bounding boxes.
- `backend/schemas/mission.py`: `SkillPrimitive`, `MissionPlan`, and validation feedback formats.
- `backend/schemas/perception.py`: Bounding boxes, pixel center, confidence score, and world coordinates.
- `backend/schemas/telemetry.py`: 6-DOF position, orientation, velocity, and battery status.
- `backend/core/config.py`: Environment variables, default workspace coordinates, camera parameters, and default model backend.
- `backend/core/logger.py`: JSON-formatted logger.

### Component 2: Planning & Safety
- `backend/planning/skill_dsl.py`: Defines skill definitions (`TAKEOFF`, `LAND`, `HOVER`, `MOVE_TO`, `ROTATE`, `CIRCLE`, `FIND`, `INSPECT`, `RETURN`) with JSON schema export.
- `backend/planning/safety_validator.py`: Evaluates every step of a `MissionPlan`. Rejects waypoints outside workspace $[-1.5, 1.5]\text{ m}$, $z > 1.5\text{ m}$, invalid skills, or excessive step velocities.
- `backend/planning/planner.py`: High-level FSM that orchestrates task decomposition, validation, execution triggering, and closed-loop replanning on visual disturbance.

### Component 3: Drone Interface & Runtime
- `backend/drone/base_interface.py`: Defines methods `takeoff()`, `land()`, `hover()`, `move_to()`, `rotate()`, `emergency_stop()`, and `get_state()`.
- `backend/drone/sim_interface.py`: Implements simulated physics motion for desktop testing.
- `backend/drone/litewing_interface.py`: Connects via `cflib` (`udp://192.168.43.42`), sends `send_hover_setpoint(vx, vy, yawrate, z)` and logs telemetry.
- `backend/runtime/mission_executor.py`: Takes validated skill lists and executes them sequentially via `DroneInterface`.
- `backend/runtime/state_manager.py` & `event_bus.py`: Maintains live telemetry and target objects.

### Component 4: Vision & Spatial Grounding
- `backend/vision/spatial_grounding.py`: Calibrated $3 \times 3$ Homography matrix to project camera pixel $(u, v) \rightarrow (X, Y)$ workspace coordinates.
- `backend/vision/fast_cv.py`: OpenCV color/contour/YOLO tracking loop.
- `backend/vision/camera_manager.py`: Video capture stream handler.

### Component 5: AI Models (Provider Abstraction)
- `backend/models/base_llm.py` & `base_vlm.py`: Abstract provider contracts.
- `backend/models/gemini_provider.py`: Uses `google-genai` SDK to call Gemini 2.5 Flash with structured output schemas for planning and visual target grounding.
- `backend/models/qwen_provider.py`: Stub provider for offline local LLM/VLM research comparison.

### Component 6: API & Web Operator Dashboard
- `backend/api/server.py` & `routes.py`: FastAPI server serving REST endpoints (`/api/command`, `/api/capabilities`, `/api/state`) and WebSocket `/ws/telemetry`.
- `frontend/`: Clean, dark dashboard showing live camera feed, real-time telemetry coordinates, natural language command form, and skill sequence breakdown.

### Component 7: MATLAB / Simulink Foundation
- `matlab/scripts/init_params.m`: Hardware physical constants (Mass $m = 33\text{ g}$, $I_{xx}, I_{yy}, I_{zz}$, motor constants).
- `matlab/controllers/`: Cascaded PID controllers (`attitude_pid.m`, `altitude_controller.m`, `position_controller.m`).
- `matlab/utils/`: Homography and trajectory generation scripts.

---

## Verification Plan

### Automated Tests
Execute backend pytest test suite from project root:
```bash
python -m pytest backend/tests
```
Tests check:
1. `test_skill_dsl.py`: Serialization and schema validity of skill primitives.
2. `test_safety_validator.py`: Accepts safe waypoints, rejects out-of-bound `(x=10, y=10)` commands, rejects unknown skills.
3. `test_sim_interface.py`: Takeoff, move_to, and hover state transitions.
4. `test_grounding.py`: Accurate pixel to world coordinate homography conversion.

### System Verification
Start the complete stack in simulation mode:
```bash
python -m backend.main --backend gemini --drone sim
```
- Verify API docs at `http://localhost:8000/docs`.
- Open `frontend/index.html` in a web browser, submit command "Take off and hover at 1 meter", observe skill generation in logs and telemetry updates on dashboard.

### MATLAB Verification
Run parameter initialization and control response in MATLAB:
```matlab
run('matlab/scripts/init_params.m');
run('matlab/scripts/run_simulation.m');
```
Verify step response plots generated by `plot_results.m`.
