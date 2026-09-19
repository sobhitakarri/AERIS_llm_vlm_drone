# Task List — Framework Audit Fixes

## Phase 1: Wire the Perception-Action Loop
- [ ] 1a. `mission_executor.py` — FIND/INSPECT triggers VLM, grabs frame, gets world coords, injects MOVE_TO
- [ ] 1b. `base_interface.py` — Add `get_frame()` to abstract base
- [ ] 1c. `camera_manager.py` — Add `read_frame()` method
- [ ] 1d. `state_manager.py` — Add threading.Lock for thread safety
- [ ] 1e. `server.py` — Store `vlm` separately in app_context, pass VLM + grounding to executor
- [ ] 1f. `routes.py` — Replace hardcoded objects with live StateManager data

## Phase 2: Fix Active Bugs
- [ ] 2a. `pysimverse_interface.py` — `FlightMode.ERROR` → `FlightMode.EMERGENCY`
- [ ] 2b. `spatial_grounding.py` — Accept dynamic image resolution
- [ ] 2c. `safety_validator.py` — Validate CIRCLE center + radius vs geofence
- [ ] 2d. `routes.py` — Remove hardcoded VLA keyword check, use intent router

## Phase 3: Wire Event Bus + Generalize
- [ ] 3a. `event_bus.py` — Create global instance, wire executor events
- [ ] 3b. `routes.py` — Forward event_bus events to WebSocket

## Phase 4: Clean up Dead Code
- [ ] 4a. `qwen_provider.py` — Remove `_pattern_fallback()` (duplicates shape_paths)
- [ ] 4b. `fast_cv.py` — Replace hardcoded mocks with real VLM-backed detection
