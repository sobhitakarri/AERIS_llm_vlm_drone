# Revised Implementation Plan — Final

## LiteWing Comms Protocol (from repo)
- **ESP-IDF firmware (main firmware)**: cflib/CRTP over UDP (`udp://192.168.43.42`)
- **Python API**: `cflib` — `cf.commander.send_hover_setpoint(vx,vy,yaw_rate,height)` 
- **Telemetry**: `LogConfig` subscriptions (roll, pitch, yaw, battery, motor PWM)
- **Arduino firmware**: ESP-NOW only (peer-to-peer, not usable from Python TCP/IP)
- **Conclusion**: Use `cflib` as the LiteWing communication layer

## Final Folder Structure

```
Project_phase1/
├── backend/
│   ├── __init__.py
│   ├── main.py                        # CLI: --backend gemini|qwen --drone real|sim
│   ├── requirements.txt
│   ├── requirements-dev.txt
│   │
│   ├── core/
│   │   ├── __init__.py
│   │   ├── config.py                  # Central config dataclass
│   │   └── logger.py                  # Structured JSON logger
│   │
│   ├── schemas/
│   │   ├── __init__.py
│   │   ├── capabilities.py            # RobotCapabilities, Workspace
│   │   ├── mission.py                 # Skill, Mission, PlanResult
│   │   ├── perception.py              # WorldObject, PerceptionResult, BBox
│   │   └── telemetry.py               # DroneState, TelemetryPacket
│   │
│   ├── models/
│   │   ├── __init__.py
│   │   ├── base_llm.py                # Abstract LLMProvider
│   │   ├── base_vlm.py                # Abstract VLMProvider
│   │   ├── gemini_provider.py         # Gemini 2.5 Flash (LLM + VLM)
│   │   └── qwen_provider.py           # Qwen3-VL local stub
│   │
│   ├── planning/
│   │   ├── __init__.py
│   │   ├── skill_dsl.py               # Skill primitives + JSON schema
│   │   ├── safety_validator.py        # Pre-execution constraint checker
│   │   └── planner.py                 # FSM: IDLE→PLANNING→EXECUTING→REPLANNING→DONE
│   │
│   ├── runtime/
│   │   ├── __init__.py
│   │   ├── mission_executor.py        # Executes skill list via DroneInterface
│   │   ├── state_manager.py           # Shared drone+world state store
│   │   └── event_bus.py               # Simple pub/sub for inter-module events
│   │
│   ├── vision/
│   │   ├── __init__.py
│   │   ├── camera_manager.py          # Camera capture + MJPEG streaming
│   │   ├── fast_cv.py                 # OpenCV + YOLO fast perception
│   │   └── spatial_grounding.py       # Homography: pixel → world coordinates
│   │
│   ├── drone/
│   │   ├── __init__.py
│   │   ├── base_interface.py          # Abstract DroneInterface
│   │   ├── litewing_interface.py      # cflib/CRTP real implementation
│   │   └── sim_interface.py           # Mock implementation (same API)
│   │
│   ├── api/
│   │   ├── __init__.py
│   │   ├── server.py                  # FastAPI app + WebSocket
│   │   └── routes.py                  # REST + WS route handlers
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
│   ├── index.html                     # Operator dashboard
│   ├── css/
│   │   └── style.css
│   └── js/
│       ├── app.js
│       ├── websocket.js
│       ├── telemetry.js
│       └── mission.js
│
└── matlab/
    ├── README.md
    ├── scripts/
    │   ├── init_params.m              # LiteWing physical parameters
    │   ├── run_simulation.m           # Top-level simulation runner
    │   └── plot_results.m             # Standard plots
    ├── controllers/
    │   ├── attitude_pid.m
    │   ├── altitude_controller.m
    │   └── position_controller.m
    ├── utils/
    │   ├── homography_utils.m
    │   └── trajectory_gen.m
    └── models/
        └── README.md                  # Instructions for building .slx models
```

## Build Order
1. schemas/ (foundation types)
2. core/ (config + logger)  
3. planning/skill_dsl.py
4. planning/safety_validator.py
5. drone/base_interface.py + sim_interface.py
6. planning/planner.py
7. runtime/
8. vision/
9. models/ (Gemini provider)
10. api/ (FastAPI server)
11. main.py + requirements
12. tests/
13. frontend/
14. matlab/
