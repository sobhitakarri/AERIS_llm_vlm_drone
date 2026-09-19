# Remaining Design — 10 Architecture Objectives

This document closes the gap between `Natural_Language_Adaptive_Drone_Architecture.md` §1 and the current AERIS codebase. Objectives 1–2 and 6 are mostly done. Objectives 3–5 and 7–10 need the work below. Do **not** change the Skill DSL or let the LLM emit motor PWM.

**Research bar from the architecture doc:** closed-loop adaptive execution  
`NL → plan → skills → fly → see → replan`

**External references (adapt, do not clone):**

| Paper / repo | What they do | What we take | What we refuse |
|--------------|--------------|--------------|----------------|
| [UAV-VLA (HRI 2025)](https://github.com/Sautenich/UAV-VLA) | NL → extract object types → Gemini Vision on **satellite** image (% coords) → **lat/lon** → ArduPilot mission text (`takeoff`, `guided lat lon`, `circle`, `rtl`). Open-loop **mission generation**. | 3-stage VLA: Goal Extractor → Object Search VLM → Actions Generator. Image-normalized coords then metric convert. Annotated detection images. Localization-error metric. | GIS / lat-lon / Mission Planner strings. 34 m outdoor error scale. VLM on every satellite tile as the only perception. |
| [AutoFly-VLA (ICLR 2026)](https://github.com/xiaolousun/AutoFly-VLA) | `RGB + language → UAV velocity` end-to-end; pseudo-depth; trained VLA. Closed-loop **in-the-wild** flight. | Continuous visual observation during flight. Re-query vision when the scene changes. Separate fast spatial cue vs slow language. | Direct velocity / PWM from the VLM. Training 2.5M triplets. TensorRT onboard policy. |

Indoor mapping of UAV-VLA’s `generate_plans.py` (their 3 steps):

```text
UAV-VLA outdoor                         AERIS indoor (LiteWing, meters)
─────────────────────────               ────────────────────────────────
1. LLM: object_types from command   →   GoalExtractor / intent_router
2. VLM: % on image → lat/lon        →   ObjectSearchVLM: bbox → homography → (X,Y) m
3. LLM: ArduPilot mission text      →   ActionsGenerator: Skill DSL JSON + SafetyValidator
   (open loop, fly later)               + executor + FastCV + ReplanManager (closed loop)
```

AutoFly supplies the **missing loop** UAV-VLA does not have (see + replan while flying). We implement that loop with **skills + PID**, not with a velocity VLA.

---

## Status of the 10 objectives

| # | Objective | Now | Remaining |
|---|-----------|-----|-----------|
| 1 | Understand NL command | Code-first + cache + Gemini/Qwen | Keep as-is; only tighten VLA vs LLM routing |
| 2 | Convert to structured mission | Skill DSL + NeLV finalize | Keep as-is |
| 3 | Observe with overhead camera | Webcam / PySimverse on demand | Calibrated overhead stream + 10–15 Hz loop |
| 4 | Identify objects and locations | FIND/INSPECT + mock/known labels | Continuous FastCV + VLM on trigger |
| 5 | Pixel → world | Synthetic homography | Calibrated `H` + cm-level check |
| 6 | Safe skills / waypoints | SafetyValidator + geofence | Keep as-is |
| 7 | Send commands to LiteWing | `LiteWingInterface` exists | Wait-for-pose + real telemetry |
| 8 | Low-level ESP32 control | Firmware/cflib assumed | Skill API only; no PWM from Python |
| 9 | Continuous state/visual feedback | WS telemetry; FIND is one-shot | Perception thread + overlay |
| 10 | Replan on change | Wired: drift ≥ 0.3 m injects MOVE_TO | Calibrated H + real LiteWing demo |

---

## Shared design (do not break)

```text
User NL
  → Intent / VLA / LLM  (Obj 1–2)
  → SafetyValidator     (Obj 6)
  → MissionExecutor
       ├─ FastPerception 10–15 Hz   (Obj 3–4, 9)
       ├─ SpatialGrounding (calib H) (Obj 5)
       └─ DroneInterface
            sim | matlab | litewing | pysimverse  (Obj 7–8)
  → EventBus OBJECT_DETECTED
  → ReplanManager (0.3 m drift)     (Obj 10)
  → new MissionPlan → same executor
```

**Hard rules**

- LLM/VLM output only Skill JSON (`TAKEOFF`, `MOVE_TO`, `FIND`, …) — UAV-VLA style **actions**, not AutoFly velocities.
- Fast loop never calls Gemini/Qwen every frame (AutoFly-style always-on **observation**, not always-on VLM).
- Replan must pass `SafetyValidator` again.
- Same Skill API for sim, MATLAB, and real LiteWing.
- Grounding is **meters in the room**, never lat/lon.

---

## Objective 1 — Understand NL

**Ref:** UAV-VLA Step 1 (`step_1_template`) extracts `object_types` from the command **before** looking at the image.

**Already done:** `intent_router.py`, `local_planner.py`, `plan_cache.py`, `gemini_provider.py` / Qwen, `vla_pipeline.GoalExtractor`.

**Remaining**

- Treat `intent.intent == "vision"` as visual (routes currently check `"find"` / `"inspect"`, which never appear).
- If `vision` and a frame exists → `UAVVLAPipeline.process` (GoalExtractor first, like UAV-VLA); else → `llm.plan()`.
- GoalExtractor should emit **object types list** (e.g. `["red bottle", "blue cube"]`), not only one string — matches UAV-VLA `object_types`.
- Prefer LLM `extract_goal` when `llm_provider` is live; keep regex fallback.
- Log `source`: `code | cache | gemini | qwen | vla`.

**Accept:** `Find the red bottle` → VLA; `fly in a square` → code, no VLM; `inspect the bottle and the cube` → two object types.

---

## Objective 2 — Structured mission

**Ref:** UAV-VLA Step 3 writes ArduPilot text (`takeoff 100`, `mode guided lat lon`, `rtl`). We keep the **same stage** but emit indoor Skill JSON.

**Already done:** `SkillPrimitive`, `MissionSpec`, NeLV `finalize_skills`, `ActionsGenerator`.

**Remaining**

- Run VLA skills through `finalize_skills` (takeoff sandwich + hop densify).
- Map UAV-VLA verbs: `takeoff` → `TAKEOFF`, `guided` → `MOVE_TO`, `circle` → `CIRCLE`, `rtl` → `RETURN`/`LAND`. No lat/lon fields.
- Do not invent new skill names. Do not emit AutoFly `vx,vy,vz`.

**Accept:** Every `/api/command` response has `skills[]` + `pipeline[]` including `uav-vla-*` stages when visual.

---

## Objective 3 — Overhead camera

**Ref:** UAV-VLA uses one **nadir map image** per mission. AutoFly uses **continuous RGB** in flight. Indoor: one fixed overhead camera = their satellite image, streamed like AutoFly.

**Gap:** `CameraManager` is a laptop webcam; no calibration UI; no always-on grab.

**Design**

| Item | Choice |
|------|--------|
| Camera | Fixed overhead USB, index in `/api/config/stream` |
| Rate | 10–15 Hz grab thread (not only on FIND) |
| Frame bus | Latest frame in `CameraManager.latest` for executor + MJPEG |
| Fallback | Webcam or `drone.get_frame()` if overhead fails |

**Files**

- `backend/vision/camera_manager.py` — background capture thread
- `backend/api/server.py` — `camera_manager.start()` on app startup
- `frontend` — stream source `overhead`

**Accept:** Dashboard video stays live with no command submitted.

---

## Objective 4 — Identify objects

**Ref:** UAV-VLA Step 2 `detect_objects_with_gemini` returns **image-percentage** points + draws annotated images (`identified_new_data/`). AutoFly keeps recognizing the target **while flying**.

**Gap:** Detection runs only inside FIND/INSPECT. `FastPerception` is unused on the server path. VLA `ObjectSearch` still falls back to **preset mock coords**.

**Design — hybrid (architecture §33.4 + UAV-VLA Step 2)**

```text
Every 66–100 ms:  FastPerception
  - color / contour tracker for known classes (red bottle, blue cube)
  - publish EVENT_OBJECT_DETECTED
  - update StateManager

On semantic trigger only (FIND / unknown label / replan):
  - one VLM resolve_target(frame, phrase)
```

**Known indoor set (Phase 1):** `red_bottle`, `blue_cube`, plus open-vocab via VLM.

**UAV-VLA-style VLM output (then we convert):**

```json
{ "red_bottle": { "x_pct": 0.62, "y_pct": 0.41 } }
```

`x_pct, y_pct` → pixel `(u,v)` → `SpatialGrounding.pixel_to_world` (our substitute for `recalculate_to_latlon.py`).

Also save one annotated JPEG per VLM call under `cache/identified/` (their `draw_circles.py`).

**Files**

- Wire `FastPerception` in `server.py`
- Background `PerceptionLoop` thread (new, short)
- Executor FIND uses `state_manager` first, VLM if missing
- Remove silent mock coords as “success” unless `ALLOW_MOCK_GROUNDING=1`

**Accept:** Bottle visible → `StateManager` has world x,y before any FIND skill. Mock coords are not counted as a VLA hit.

---

## Objective 5 — Pixel → world

**Ref:** UAV-VLA `parsed_coordinates.csv` stores **image-corner lat/lon** and `recalculate_coordinates` maps % → geo. We store **image-corner meters** and map %/pixel → `(X,Y)`.

**Gap:** Default `H` maps image corners to ±1.5 m. Not a real calibration. Outdoor UAV-VLA KNN error is **~34 m**; our accept is **±8 cm**.

**Design**

1. Print 4 workspace corners (or chessboard) at known meters.
2. Click 4 pixel points in UI or load `cache/homography.json`.
3. `SpatialGrounding.load(path)` / `save(path)`.
4. Keep OpenCV `findHomography`; linear fallback only if OpenCV missing.

**JSON**

```json
{
  "img_w": 1280,
  "img_h": 720,
  "src_px": [[u,v], [u,v], [u,v], [u,v]],
  "dst_m": [[-1.5, 1.5], [1.5, 1.5], [1.5, -1.5], [-1.5, -1.5]],
  "H": [[...], [...], [...]]
}
```

**Accept:** Object at a taped (0.50, 0.50) m mark reports within **±8 cm**.

---

## Objective 6 — Safe skills

**Already done:** geofence, allowed skills, hop clip.

**Remaining (small)**

- Replanned skills must call `SafetyValidator` again (Obj 10).
- FIND grounded points clipped to workspace before `MOVE_TO`.

**Accept:** Command or replan outside `[-1.5, 1.5] × [0.2, 1.5]` is rejected.

---

## Objective 7 — Commands to LiteWing

**Ref:** UAV-VLA hands a **finished plan** to Mission Planner (open loop). AutoFly **streams velocities**. We stream **skill setpoints** and wait on pose (middle path).

**Gap:** `move_to` sends one hover setpoint and **writes local state as if arrived**. No estimator wait.

**Design (Skill API unchanged)**

| Skill | CRTP / cflib |
|-------|----------------|
| TAKEOFF | arm + hover setpoint climb |
| MOVE_TO | stream hover or position setpoints until error &lt; 0.10 m |
| HOVER | hold setpoint for `t` |
| LAND | descend + disarm |
| ROTATE | yaw rate or yaw setpoint |
| EMERGENCY | stop + disarm |

**Must add**

- Subscribe `cflib` pose/log (`stateEstimate.x/y/z/yaw`) into `get_state()`
- `_wait_until_near` like MATLAB (0.10 m, timeout 15 s)
- If link drop → `emergency_stop` + monitor abort

**Accept:** `take off to 0.5m` on `--drone real` — telemetry Z rises; `move_to` does not finish until pose is near.

---

## Objective 8 — ESP32 low-level control

**Rule from the doc + AutoFly contrast:** Python never sends PWM or body velocity from the VLM. AutoFly’s action de-tokenizer is **out of scope**.

**Rule from the doc:** Python never sends PWM.

**Remaining**

- Keep mixer/PID on firmware (or MATLAB twin only).
- Document in dashboard/health: `control_authority = firmware`.
- Optional: read battery / armed from CRTP for safety interlock (no takeoff if voltage low).

**Accept:** No Python path writes motor PWM. Skills are setpoints only.

---

## Objective 9 — Continuous feedback

**Ref:** AutoFly’s contribution we **do** copy: keep RGB + object state live. UAV-VLA we **do not** copy: one image, then fly blind.

**Gap:** Telemetry WS exists; vision is not continuous; events are not shown as a live object list.

**Design**

| Stream | Rate | Content |
|--------|------|---------|
| `/ws/telemetry` | 10–20 Hz | pose, battery, monitor step |
| Perception | 10–15 Hz | objects + world x,y + bbox overlay |
| `/api/video_feed` | ~15 Hz | JPEG + boxes (already sketched) |

**Add to WS payload**

```json
{
  "pose": {"x": 0.1, "y": 0.0, "z": 0.5, "yaw": 0},
  "objects": [{"label": "red_bottle", "x": 0.48, "y": 0.51, "conf": 0.8}],
  "monitor": {"step": 2, "skill": "MOVE_TO"},
  "replan": {"active": false, "count": 0}
}
```

**Accept:** Move the bottle by hand; dashboard object x,y updates without a new NL command.

---

## Objective 10 — Replan

**Ref:** UAV-VLA has **no** mid-mission replan (plan is generated once). AutoFly adapts every step via a new velocity. We add **UAV-VLA Step 2+3 again** when FastCV says the target moved ≥ 0.3 m.

**Gap:** `ReplanManager` is never constructed in `create_app`. FIND does not refresh target mid-skill.

**Design**

```text
ReplanManager.start()
  subscribe OBJECT_DETECTED
  if same label moved ≥ 0.3 m:
      monitor.request_abort()  OR  inject remaining skills
      llm.plan("update: target {label} now at {x,y}; finish original command")
      SafetyValidator
      executor continues from current pose (no extra takeoff if already airborne)
```

**Wire in `server.py`**

```python
replan = ReplanManager(event_bus, replan_callback=on_replan)
replan.start()
app_context["replan"] = replan
```

**Callback rules**

1. Ignore drift if no mission armed.
2. Cooldown 2 s (avoid replan storm).
3. Cap 3 replans per mission.
4. Always validate.
5. Increment `replan_count` on monitor snapshot.

**Demo (must pass)**

1. `Find the red bottle and hover`
2. After first MOVE_TO starts, shift bottle ≥ 0.3 m
3. Log: `Triggering replan`
4. Drone flies to new world point
5. UI shows `replan_count ≥ 1`

---

## Implementation order

| Step | Objectives | Work | Done when |
|------|------------|------|-----------|
| A | 1, 2, 6 | Fix vision intent routing; VLA → finalize + safety | Visual vs shape commands route correctly |
| B | 3, 9 | Camera thread + WS objects | Live video + object list without a command |
| C | 5 | Load/save calibrated `H` | ±8 cm at a known mark |
| D | 4 | Start FastPerception loop | Known objects in StateManager continuously |
| E | 10 | Construct ReplanManager; callback | Bottle-move demo |
| F | 7, 8 | LiteWing pose wait + no PWM | Real takeoff/hover/land with live pose |

Do A–E on `--drone sim` or `matlab`. Do F last on hardware.

---

## Files to touch (only remaining work)

| File | Change |
|------|--------|
| `backend/api/server.py` | Start camera thread, FastPerception, ReplanManager |
| `backend/api/routes.py` | Fix `is_visual_target`; WS objects + replan_count |
| `backend/vision/camera_manager.py` | Latest-frame thread |
| `backend/vision/spatial_grounding.py` | `load` / `save` `cache/homography.json` |
| `backend/vision/fast_cv.py` | Color tracker for bottle/cube (VLM fallback) |
| `backend/runtime/replan_manager.py` | Cooldown + max replans (already almost done) |
| `backend/runtime/mission_executor.py` | FIND prefers live StateManager; allow mid-mission plan swap |
| `backend/drone/litewing_interface.py` | Pose logs + wait-until-near |
| `frontend/js/telemetry.js` | Show objects + replan badge |
| `cache/homography.json` | Calibrated H (new) |

---

## Indoor metrics (from UAV-VLA experiments, scaled)

UAV-VLA reports trajectory-length delta and localization RMSE (KNN / DTW) in **meters outdoor**. Log the same names indoors:

| Metric | UAV-VLA outdoor | AERIS indoor target |
|--------|-----------------|---------------------|
| Localization error | ~34 m KNN | ≤ 0.08 m vs taped mark |
| Trajectory length vs scripted | 22% delta | log `path_len_vla / path_len_code` |
| Mission gen time | minutes (30 images) | < 5 s per FIND |
| Replan count | n/a (open loop) | ≥ 1 on bottle-move demo |

Do **not** run their 30-image satellite benchmark.

---

## Out of scope (do not do to “finish” the 10)

- Satellite / lat-lon / ArduPilot mission files ([UAV-VLA](https://github.com/Sautenich/UAV-VLA))
- End-to-end velocity VLA, pseudo-depth training, TensorRT ([AutoFly](https://github.com/xiaolousun/AutoFly-VLA))
- New foundation models
- VLM on every frame
- LLM motor mixing
- Full B0–B4 paper table (after the bottle-move demo)
- Obstacle occupancy grid (optional after Obj 10 works)

---

## One-line per objective (for reviews)

1. **NL** — UAV-VLA Step 1 object types; fix vision routing.  
2. **Mission** — UAV-VLA Step 3 as Skill JSON, not ArduPilot.  
3. **Camera** — their nadir image, streamed (AutoFly-style).  
4. **Detect** — UAV-VLA Step 2 + FastCV; no mock-as-success.  
5. **Ground** — their corner-CSV idea in meters; ±8 cm.  
6. **Safety** — already works; apply to replans.  
7. **LiteWing** — execute the plan with pose wait (not fly-blind).  
8. **ESP32** — firmware PID; refuse AutoFly velocities.  
9. **Feedback** — AutoFly continuous RGB; UAV-VLA annotated overlay.  
10. **Replan** — re-run VLA Steps 2–3 when target drifts 0.3 m.  
