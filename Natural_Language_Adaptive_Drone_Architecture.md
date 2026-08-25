# Natural Language Adaptive Drone Framework
## Complete Architecture, Technology Stack, Simulation, and Hardware–Software Integration

> **Project direction:** LLM/VLM-based autonomous indoor drone using a low-cost **LiteWing ESP32-S3** platform, an external camera, a laptop/edge computer, and a custom high-level mission interface.
>
> **Core idea:** Natural language → AI reasoning → visual perception → spatial grounding → safe drone skills → ESP32 flight execution → visual feedback → replanning.

---

# 1. Project Objective

The system should allow a user to issue commands such as:

- “Fly in a square.”
- “Go to the blue cube.”
- “Circle the red bottle.”
- “Inspect the objects.”
- “Find the red object and return.”

The system should:

1. Understand the user's natural-language command.
2. Convert it into a structured mission.
3. Observe the environment using an external overhead camera.
4. Identify relevant objects and their locations.
5. Convert image coordinates into real-world workspace coordinates.
6. Generate safe high-level drone skills/waypoints.
7. Send commands to the LiteWing.
8. Execute low-level stabilization and motor control on the ESP32-S3.
9. Continuously receive state/visual feedback.
10. Replan if the environment or execution state changes.

The important research focus is **closed-loop adaptive execution**, not merely translating English into drone commands.

---

# 2. High-Level Architecture

```text
                         USER
                          |
                   Text / Voice Command
                          |
                          v
                  +----------------+
                  |      LLM       |
                  | Task Reasoning |
                  +-------+--------+
                          |
                  Structured Skill Plan
                          |
                          v
                  +----------------+
                  | Safety /       |
                  | Plan Validator |
                  +-------+--------+
                          |
                          v
                  +----------------+
                  | Mission        |
                  | Planner        |
                  +-------+--------+
                          |
             +------------+-------------+
             |                          |
             v                          v
     +---------------+          +---------------+
     | Vision System |          | Drone State   |
     | CV + VLM      |          | / Telemetry   |
     +-------+-------+          +-------+-------+
             |                          |
             +------------+-------------+
                          |
                          v
                 Spatial Grounding
                 Pixel -> World
                          |
                          v
                 Waypoint / Skill
                          |
                          v
                 Drone Interface
                          |
                    Wi-Fi / CRTP
                          |
                          v
                 +----------------+
                 |   LiteWing     |
                 |   ESP32-S3     |
                 +-------+--------+
                         |
                 Flight Controller
                 IMU / Estimator / PID
                         |
                         v
                       Motors
                         |
                         v
                       DRONE
                         |
                         v
                  Overhead Camera
                         |
                         +--------> Feedback / Replanning
```

---

# 3. Core Design Principle

## Separate intelligence from flight control

The LLM/VLM must **NOT** directly control motor PWM.

Bad:

```text
LLM -> Motor 1/2/3/4 commands
```

Correct:

```text
LLM
  ↓
Mission / Skill
  ↓
Safety Validator
  ↓
Waypoint / Motion Primitive
  ↓
Flight Controller
  ↓
PID / Motor Mixer
  ↓
Motors
```

The AI operates at the **high level**.

The ESP32 flight controller operates at the **low level**.

This separation gives:

- deterministic flight control
- safer AI integration
- lower AI latency requirements
- easier debugging
- cleaner simulation-to-real transfer
- modular research architecture

---

# 4. AI Layer — What We Need to Study

The AI side is the main research component.

## 4.1 LLM — Large Language Model

**Key terms to study:**

**LLM, prompt engineering, structured output, function calling, tool calling, task decomposition, planning, agent, state, memory, reasoning, hallucination, constrained generation, DSL.**

The LLM should convert:

> “Circle the red bottle and then land.”

into a structured plan such as:

```text
TAKEOFF(z=1.0)
FIND(target="red_bottle")
CIRCLE(target="red_bottle", radius=0.5)
LAND()
```

The LLM should not generate arbitrary low-level control code.

### Important study topics

- LLM basics
- tokens
- transformer architecture
- attention
- prompt design
- structured JSON output
- function/tool calling
- task decomposition
- planning
- hallucination
- constrained generation
- local vs cloud LLM
- latency
- model quantization

---

# 5. Drone Skill DSL

A major interface between AI and the drone should be a small **Drone Skill DSL (Domain-Specific Language)**.

Example:

```text
TAKEOFF(z=1.0)
MOVE_TO(x=1.2,y=0.8,z=1.0)
HOVER(t=2)
CIRCLE(x=1.2,y=0.8,r=0.5)
INSPECT(target="red_bottle")
RETURN()
LAND()
```

Pipeline:

```text
Natural Language
      ↓
     LLM
      ↓
Skill DSL
      ↓
Validator
      ↓
Mission Planner
```

## Key AI terms

**DSL, intermediate representation, tool calling, function calling, constrained decoding, schema validation, action space, skill primitive.**

This is important because the LLM should select from a **safe action space** rather than inventing arbitrary commands.

---

# 6. VLM — Vision-Language Model

The VLM provides semantic visual understanding.

Example:

User:

> “Find the red bottle.”

Camera image:

```text
+--------------------------+
|                          |
|       blue box           |
|                          |
|                red       |
|               bottle     |
|                          |
|             drone        |
+--------------------------+
```

The perception system should identify:

```text
Object: red bottle
Bounding box: [x1,y1,x2,y2]
Center: (u,v)
Confidence: 0.94
```

## Key AI terms to study

**VLM, multimodal model, vision encoder, language encoder, cross-modal alignment, visual grounding, object detection, open-vocabulary detection, image-text embedding, CLIP, BLIP, LLaVA, Qwen-VL, visual question answering (VQA).**

---

# 7. Do NOT make the VLM do everything

For efficiency, use a hybrid perception stack.

## Fast perception

Use:

- OpenCV
- object detector
- tracking
- ArUco/AprilTag
- color segmentation
- geometric processing

for fast repetitive tasks.

## VLM

Use the VLM for:

- semantic object identification
- ambiguous object descriptions
- scene interpretation
- natural-language visual queries
- matching user language to visual objects

Architecture:

```text
Camera
  |
  v
Fast CV / Detector
  |
  +----> Bounding boxes / tracking
  |
  v
VLM
  |
  +----> Semantic interpretation
  |
  v
Scene Representation
```

## Key AI terms

**YOLO, tracking, segmentation, open-vocabulary detection, semantic perception, scene graph, visual grounding, multimodal reasoning.**

---

# 8. Spatial Grounding — Critical Project Component

This is one of the most important technical parts.

The camera gives a pixel coordinate:

```text
(u, v)
```

The drone needs a physical coordinate:

```text
(X, Y, Z)
```

Therefore:

```text
Image
  ↓
Object detection
  ↓
Pixel coordinate
  ↓
Camera calibration / Homography
  ↓
World coordinate
  ↓
Drone waypoint
```

For a flat indoor workspace:

```text
pixel (u,v)
      |
      v
Homography H
      |
      v
world (X,Y)
```

Altitude can initially be handled separately using:

- ToF
- barometer
- known flight altitude
- controller state

## Key AI/robotics terms to study

**spatial grounding, visual grounding, camera calibration, intrinsic parameters, extrinsic parameters, homography, perspective transformation, coordinate frames, pixel coordinates, world coordinates, reprojection error.**

This is where your project becomes more than a chatbot controlling a drone.

---

# 9. Mission Planner

The Mission Planner combines:

- LLM-generated task
- VLM observations
- drone state
- safety constraints
- skill completion

Example:

```text
User:
"Circle the red bottle."

LLM:
CIRCLE(target=red_bottle)

VLM:
red_bottle = (1.2, 0.8)

Planner:
CIRCLE(1.2,0.8)

Drone:
Execute

Camera:
Bottle moved to (1.8,1.1)

Planner:
Target state changed

LLM / Planner:
REPLAN

Drone:
Move to new target
```

## Key terms

**task planning, hierarchical planning, behavior tree, finite-state machine, task graph, action sequencing, goal state, precondition, termination condition, replanning, feedback control.**

For the first implementation, a **finite-state machine + skill executor** is safer and easier to debug than a fully autonomous AI agent.

---

# 10. Safety Validator

The LLM-generated plan must be checked before execution.

Example:

```text
LLM:
MOVE_TO(x=8,y=10,z=5)

        ↓

Safety Validator

Workspace:
2 m x 2 m

Maximum altitude:
1.5 m

        ↓

REJECT
```

Checks can include:

- workspace boundary
- altitude
- maximum velocity
- maximum displacement
- battery
- communication status
- invalid skill
- missing target
- emergency-stop state

## Key terms

**constraint checking, safety supervisor, guard condition, action validation, geofencing, fail-safe, watchdog, emergency stop, runtime verification.**

---

# 11. Closed-Loop Adaptive Architecture

This should be the main research contribution.

## Open-loop system

```text
User
 ↓
LLM
 ↓
Plan
 ↓
Drone
 ↓
Done
```

## Closed-loop system

```text
User
 ↓
LLM
 ↓
Plan
 ↓
Drone
 ↓
Camera
 ↓
Perception
 ↓
World State
 ↓
Goal achieved?
 ├── YES -> Finish
 |
 └── NO / Environment changed
          ↓
       Replan
          ↓
         LLM
          ↓
        Drone
```

Example:

```text
"Go to the blue cube."

Initial:
blue cube = (1.0, 0.8)

Drone moves.

Cube moves.

VLM:
blue cube = (1.7, 1.2)

Planner:
Expected position != observed position

REPLAN

Drone:
Move to new position
```

## Key terms

**closed-loop autonomy, feedback loop, adaptive planning, replanning, state estimation, disturbance recovery, online planning, dynamic environment.**

---

# 12. AI Timing Architecture

Do not put the LLM in the fast flight-control loop.

## Fast loop

```text
IMU
 ↓
State estimation
 ↓
PID
 ↓
Motor control
```

This should run deterministically and quickly.

## Slow/high-level loop

```text
Camera
 ↓
VLM
 ↓
LLM
 ↓
Mission planner
 ↓
Waypoint / skill
```

This can run asynchronously.

Conceptually:

```text
FAST LOOP
20–200+ Hz depending on controller

SLOW AI LOOP
event-driven / asynchronous
```

The exact rates must be measured for the chosen hardware and implementation.

## Key terms

**control loop, sampling rate, latency, jitter, asynchronous processing, real-time control, high-level planning, low-level control.**

---

# 13. Hardware Platform — LiteWing

Target platform:

## LiteWing ESP32-S3

Main elements:

- ESP32-S3
- Wi-Fi
- MPU6050 IMU
- four coreless brushed motors
- motor drivers/H-bridge
- small PCB frame
- 1S Li-Po
- optional ToF
- optional optical-flow support depending on configuration

The LiteWing firmware is based on the open ESP-Drone/Crazyflie ecosystem.

The existing flight stack should be reused where practical instead of rewriting stabilization from zero.

---

# 14. Firmware Architecture

Recommended:

```text
LiteWing Firmware
|
+-- Sensors
|    +-- MPU6050
|    +-- ToF
|    +-- Optical Flow
|
+-- State Estimator
|
+-- Flight Controller
|    +-- Attitude PID
|    +-- Altitude Controller
|    +-- Position Controller
|
+-- Motor Driver
|    +-- Motor Mixer
|
+-- Communication
|    +-- Wi-Fi
|    +-- CRTP / command interface
|
+-- Mission / Skill Interface
|
+-- Safety
     +-- Watchdog
     +-- Battery
     +-- Emergency Stop
```

### Important principle

Keep the existing low-level stabilization where possible.

Add your research layer **above** it:

```text
LLM/VLM
   ↓
Mission Planner
   ↓
Skill API
   ↓
Existing Flight Controller
   ↓
Motors
```

---

# 15. Hardware–Software Integration

## AI computer

Laptop / PC:

```text
Python
|
+-- LLM
+-- VLM
+-- OpenCV
+-- Spatial Grounding
+-- Mission Planner
+-- Safety Validator
+-- Logging
```

## Communication

Use a lightweight interface such as:

- Wi-Fi
- UDP
- existing LiteWing CRTP/cflib interface

Example command:

```json
{
  "cmd": "MOVE_TO",
  "x": 1.2,
  "y": 0.8,
  "z": 1.0
}
```

Telemetry:

```json
{
  "state": "MOVING",
  "x": 1.15,
  "y": 0.79,
  "z": 0.98,
  "battery": 82
}
```

The AI layer should communicate using **high-level commands**, not motor PWM.

---

# 16. MATLAB + Simulink + Simscape

MATLAB/Simulink is the primary **control-development and drone-simulation environment**.

It is not the AI middleware.

## MATLAB

Use MATLAB for:

- data analysis
- plotting
- controller parameter analysis
- trajectory generation
- coordinate calculations
- system identification
- simulation scripts
- experiment comparison

## Simulink

Use Simulink for:

- flight-control block diagrams
- PID controllers
- state estimation
- trajectory tracking
- controller tuning
- closed-loop simulation
- subsystem integration

## Simscape / Simscape Multibody

Use it for:

- rigid-body dynamics
- drone body
- forces and torques
- motor/propulsion representation
- 6-DOF physical behavior
- sensor/physical-system modeling

---

# 17. LiteWing Digital Twin

Do not start by trying to make a perfect simulator.

Build progressively.

## Level 1 — Kinematic model

State:

```text
x, y, z
roll, pitch, yaw
```

Test:

- takeoff
- move
- hover
- land

## Level 2 — Dynamic quadrotor

Add:

- mass
- inertia
- gravity
- thrust
- drag
- motor response
- 6-DOF dynamics

## Level 3 — Sensor model

Add:

- IMU
- accelerometer
- gyroscope
- barometer
- ToF
- optical flow

Add:

- noise
- bias
- sampling
- delay

## Level 4 — Control validation

Test:

```text
Reference trajectory
       ↓
Position Controller
       ↓
Attitude Controller
       ↓
Motor Mixer
       ↓
Quadrotor Plant
       ↓
Sensors
       ↓
Estimator
       └──────── feedback
```

## Level 5 — AI-in-the-loop

Connect:

```text
Python AI
   ↓
VLM
   ↓
Spatial Grounding
   ↓
LLM
   ↓
Mission Planner
   ↓
Waypoint
   ↓
Simulink LiteWing
```

---

# 18. Why Simulink Before Hardware?

Simulation lets us find:

- unstable PID
- wrong motor mixing
- poor trajectory tracking
- sensor noise sensitivity
- controller saturation
- waypoint errors
- disturbance response

before flying the real drone.

The goal is:

```text
SIMULATION
    ↓
Controller validated
    ↓
Firmware implementation
    ↓
Bench test
    ↓
Low-altitude flight
    ↓
Full experiment
```

---

# 19. Important Sim-to-Real Principle

Do not create two completely different control systems.

Aim for:

```text
                  CONTROL LOGIC
                       |
              +--------+--------+
              |                 |
              v                 v
          SIMULATION          REAL
              |                 |
        Virtual Sensors      Real Sensors
              |                 |
        Virtual Motors      Real Motors
```

The AI layer should also use the same high-level command interface:

```text
                DroneInterface
                 /          \
                /            \
        SimLiteWing        RealLiteWing
```

So:

```text
LLM/VLM
   ↓
Drone Skill
   ↓
DroneInterface
   ├── Simulator
   └── Real LiteWing
```

This makes the transition from simulation to hardware much cleaner.

---

# 20. Complete Software Stack

```text
+------------------------------------------------------+
|                  USER INTERFACE                      |
|              Text / Voice Command                   |
+--------------------------+---------------------------+
                           |
+--------------------------v---------------------------+
|                    AI LAYER                         |
|                                                      |
| LLM | VLM | OpenCV | Grounding | Mission Planner   |
| Safety Validator | Skill DSL | Logging              |
+--------------------------+---------------------------+
                           |
+--------------------------v---------------------------+
|                 DRONE INTERFACE                     |
|          UDP / Wi-Fi / CRTP / cflib                |
+--------------------------+---------------------------+
                           |
              +------------+------------+
              |                         |
              v                         v
+-------------------------+   +------------------------+
|      SIMULATION         |   |       HARDWARE         |
| MATLAB / Simulink       |   | LiteWing ESP32-S3      |
| Simscape                |   | Sensors                |
| LiteWing Digital Twin   |   | Flight Controller      |
| Virtual Sensors         |   | PID / Motor Mixer      |
+-------------------------+   +------------------------+
```

---

# 21. Recommended Technology Stack

| Layer | Technology | Purpose |
|---|---|---|
| User input | Python/UI | Natural language |
| LLM (V1) | **Gemini 2.5 Flash — API** | Task planning, skill generation, replanning |
| LLM (V2 comparison) | **Qwen3 — local** | Model-agnostic experiment |
| VLM (V1) | **Gemini 2.5 Flash — API (multimodal)** | Semantic visual reasoning |
| VLM (V2 comparison) | **Qwen3-VL — local** | Model-agnostic experiment |
| Fast vision | OpenCV + YOLO / ArUco | High-frequency tracking + localization |
| Spatial grounding | OpenCV + calibration | Pixel → world |
| Model interface | **Pluggable LLMProvider / VLMProvider** | Backend-agnostic AI layer |
| Planner | Python | Mission/skill execution |
| Skill representation | Custom DSL / JSON | Safe action interface |
| Safety | Python + firmware | Constraints/failsafe |
| Communication | Wi-Fi/UDP/CRTP | AI ↔ drone |
| Firmware | ESP-IDF / existing LiteWing stack | Embedded control |
| RTOS | FreeRTOS | Embedded task scheduling |
| MCU | ESP32-S3 | Drone control |
| IMU | MPU6050 | Attitude/state sensing |
| Simulation | MATLAB/Simulink | Control + system simulation |
| Physical modeling | Simscape | Dynamics/plant |
| Hardware | LiteWing | Real UAV |
| Camera | USB overhead camera | Global perception |
| Logging | Python/MATLAB | Evaluation |

> **Key architectural principle:** We own the framework. We do not own the foundation model.
> Gemini today. Qwen tomorrow. Another open model later.
> Same framework. Same skill API. Same robot interface.

---

# 22. What NOT to Make Mandatory

The following are useful but should NOT be project dependencies:

- ROS 2
- Gazebo
- PX4
- ArduPilot
- Nav2
- QGroundControl

They can be learned later for comparison, portability, or future expansion.

The core project can run with:

```text
Python
+
MATLAB/Simulink
+
LiteWing firmware
+
Wi-Fi/CRTP
+
External camera
```

---

# 23. Development Roadmap

## Phase 0 — Platform study

Study:

- LiteWing hardware
- ESP32-S3
- existing firmware
- cflib/CRTP
- IMU
- motors
- FreeRTOS
- safety

Deliverable:

**LiteWing controlled manually through existing interface.**

---

## Phase 1 — Flight controller understanding

Study:

- quadrotor dynamics
- coordinate frames
- PID
- attitude control
- altitude control
- motor mixing
- sensor fusion

Deliverable:

**Stable basic flight and documented control architecture.**

---

## Phase 2 — Simulink Digital Twin

Build:

```text
LiteWing dynamics
+
motor model
+
IMU
+
controller
```

Deliverable:

**Stable simulated hover and trajectory tracking.**

---

## Phase 3 — Spatial perception

Build:

```text
Overhead Camera
 ↓
Drone/object detection
 ↓
Calibration
 ↓
Pixel → World
```

Deliverable:

**Object location in centimetres.**

---

## Phase 4 — LLM skill generation

Build:

```text
Natural language
 ↓
LLM
 ↓
Skill DSL
 ↓
Validator
```

Deliverable:

**Reliable structured drone missions.**

---

## Phase 5 — VLM integration

Build:

```text
Image
 ↓
Detector
 ↓
VLM
 ↓
Semantic target
 ↓
World coordinate
```

Deliverable:

**Natural-language target grounding.**

---

## Phase 6 — AI-in-the-loop simulation

```text
LLM
 ↓
VLM
 ↓
Grounding
 ↓
Planner
 ↓
Simulated LiteWing
 ↓
Camera feedback
 ↓
Replan
```

Deliverable:

**Complete simulation demonstration.**

---

## Phase 7 — Real LiteWing integration

Replace:

```text
SimLiteWing
```

with:

```text
RealLiteWing
```

Keep:

```text
LLM
VLM
Planner
Skill API
Safety
```

unchanged.

Deliverable:

**Real autonomous indoor flight.**

---

# 24. Research Experiments

## Baseline B0 — Scripted

```text
Predefined waypoints
```

No LLM.

No VLM.

---

## Baseline B1 — LLM only

```text
Natural language
 ↓
LLM
 ↓
Known coordinates
 ↓
Drone
```

Tests language planning.

---

## Baseline B2 — Vision only

```text
Camera
 ↓
Object detection
 ↓
Waypoint
 ↓
Drone
```

Tests perception/grounding.

---

## Baseline B3 — LLM + VLM, open loop

```text
LLM + VLM
 ↓
Plan
 ↓
Drone
```

No replanning.

---

## Proposed B4 — Closed-loop system

```text
LLM
 +
VLM
 +
Spatial Grounding
 +
Mission Planner
 +
Feedback
 +
Replanning
```

This is the main system.

---

## Experiment E1 — Cloud Foundation Model

Run the complete B4 system with:

```text
Gemini 2.5 Flash (API)
 ↓
Framework
 ↓
Drone
```

---

## Experiment E2 — Local Foundation Model

Run the exact same framework with:

```text
Qwen3-VL (local)
 ↓
Framework
 ↓
Drone
```

Same tasks. Same drone. Same evaluation protocol.

This makes model-deployment strategy an **explicit research variable**.

---

# 25. Disturbance Tests

Test:

### Target movement

Move the target after planning.

### Obstacle insertion

Introduce an obstacle after planning.

### Waypoint disturbance

Add artificial position error.

### Communication delay

Test delays such as:

```text
100 ms
250 ms
500 ms
```

### Invalid LLM command

Example:

> “Fly outside the workspace.”

Safety validator should reject it.

---

# 26. Evaluation Metrics

## Task Success Rate

```text
successful missions / total missions × 100
```

## Spatial Grounding Error

```text
predicted position vs actual position
```

Measure in cm.

## Position Tracking Error

```text
desired waypoint vs actual drone position
```

## Mission Time

Time from command to successful completion.

## Replanning Latency

```text
environment change
        ↓
change detected
        ↓
new plan issued
```

## Replan Count

Number of replanning events per mission.

## Command Interpretation Accuracy

Percentage of natural-language commands converted into the correct skill sequence.

## Recovery Success Rate

Percentage of disturbed missions that successfully recover.

## Safety Rejection Accuracy

Whether unsafe commands are correctly rejected.

## Communication Latency

Measure:

```text
command sent → drone receives
```

## Model Comparison Metrics (E1 vs E2)

| Metric | Cloud (Gemini) | Local (Qwen) |
|---|---|---|
| Task success rate | | |
| Planning latency | | |
| VLM latency | | |
| Replanning latency | | |
| Internet dependency | Yes | No |
| Cost / mission | API cost | Hardware cost |
| Privacy | Cloud | On-device |
| Offline capability | No | Yes |
| GPU / hardware req. | Laptop only | GPU needed |

---

# 27. Key AI Topics — Study Checklist

## LLM

- Transformer
- Attention
- Tokens
- Prompt engineering
- Structured output
- Function calling
- Tool calling
- Task decomposition
- Planning
- Agents
- Memory/state
- Hallucination
- Constrained generation
- DSL

## VLM

- Vision Transformer
- CLIP
- BLIP
- LLaVA
- Qwen-VL
- Multimodal embeddings
- Visual grounding
- VQA
- Open-vocabulary detection
- Scene understanding

## Computer Vision

- OpenCV
- Camera calibration
- Homography
- Bounding boxes
- Segmentation
- Tracking
- ArUco/AprilTag
- Perspective transform
- Optical flow

## Robotics AI

- Embodied AI
- Vision-Language Navigation (VLN)
- UAV-VLN
- Task planning
- Hierarchical planning
- Behavior trees
- Finite-state machines
- Skill primitives
- Replanning
- Closed-loop autonomy

## AI Safety

- Guardrails
- Constraint checking
- Tool validation
- Geofencing
- Runtime verification
- Fail-safe
- Watchdog
- Emergency stop

---

# 28. Key Embedded/Drone Topics — Study Checklist

- ESP32-S3
- ESP-IDF
- FreeRTOS
- MPU6050
- I2C
- PWM
- motor drivers
- brushed motors
- ESC/H-bridge
- motor mixing
- PID
- attitude control
- altitude control
- state estimation
- complementary filter
- Kalman/EKF basics
- telemetry
- Wi-Fi
- UDP
- CRTP/cflib
- watchdog
- battery monitoring
- failsafe

---

# 29. Key MATLAB/Simulink Topics — Study Checklist

## MATLAB

- matrices
- plotting
- numerical analysis
- system identification
- data logging
- parameter sweeps
- trajectory generation

## Simulink

- blocks
- subsystems
- signals
- scopes
- discrete simulation
- solver selection
- PID Controller
- State-Space
- MATLAB Function blocks
- Stateflow
- model logging
- parameter tuning

## Simscape

- physical networks
- rigid bodies
- forces
- torques
- 6-DOF dynamics
- mechanical joints
- motor/propulsion models
- sensor models

---

# 30. Final System

The final system should look like:

```text
                         NATURAL LANGUAGE
                               |
                               v
                         +-----------+
                         |    LLM    |
                         +-----+-----+
                               |
                         Skill / Plan
                               |
                               v
                     +-------------------+
                     | Safety Validator  |
                     +---------+---------+
                               |
                               v
                     +-------------------+
                     | Mission Planner   |
                     +---------+---------+
                               |
                  +------------+------------+
                  |                         |
                  v                         v
            +-----------+             Drone State
            | VLM / CV  |                 |
            +-----+-----+                 |
                  |                       |
            Scene / Object               |
                  |                       |
                  +----------+------------+
                             |
                             v
                    Spatial Grounding
                     Pixel -> World
                             |
                             v
                       Skill / Waypoint
                             |
                             v
                      Drone Interface
                             |
                       Wi-Fi / CRTP
                             |
                             v
                      +-------------+
                      |  LiteWing   |
                      | ESP32-S3    |
                      +------+------+
                             |
                      Flight Controller
                             |
                        PID / Estimator
                             |
                             v
                           Motors
                             |
                             v
                           DRONE
                             |
                             v
                      OVERHEAD CAMERA
                             |
                             +----------> FEEDBACK
                                            |
                                            v
                                         REPLAN
```

---

# 31. One-Sentence Project Definition

> **A resource-constrained closed-loop embodied-AI framework that converts natural-language goals into validated drone skills, grounds visually perceived targets into real-world coordinates, executes those skills on an ESP32-S3 LiteWing UAV, and adaptively replans using visual feedback.**

That is the architecture we should build toward.

---

# 32. Immediate Next Steps

1. **Acquire/confirm LiteWing hardware.**
2. Clone and understand the existing LiteWing/ESP-Drone firmware.
3. Test existing cflib/CRTP control.
4. Characterize LiteWing mass, motor response and basic flight behavior.
5. Build the **LiteWing Simulink/Simscape digital twin**.
6. Validate takeoff/hover/translation/landing in simulation.
7. Build the overhead-camera calibration system.
8. Implement pixel → world coordinate transformation.
9. Implement the Drone Skill DSL.
10. Add the LLM planner.
11. Add fast visual perception + VLM semantic reasoning.
12. Connect AI to the simulated drone.
13. Test disturbances/replanning.
14. Port the high-level interface to the real LiteWing.
15. Run the baseline/ablation experiments.

## Final principle

**Simulink/Simscape is for understanding and validating the vehicle/control system.**

**Python + LLM/VLM is for intelligence and mission reasoning.**

**LiteWing/ESP32-S3 is for deterministic embedded flight execution.**

**The overhead camera provides global visual feedback and spatial grounding.**

**The Skill API is the bridge between AI and the drone.**

**The feedback loop is the research contribution.**

---

# 33. Foundation Model Strategy

> **Core architectural principle:** We own the framework. We do not own the foundation model.

---

## 33.1 Why Not Local-First

The research question is:

> **Can a general LLM–VLM framework generalize across tasks and robot capabilities?**

It is **not**:

> Can we squeeze an LLM into an ESP32?

If we start with a local 7B/8B model and spend weeks fighting VRAM, quantization, inference speed, and model serving, we lose time on infrastructure rather than the actual research.

For the first milestone, we want to answer:

**Does the framework work?**

API models give us the strongest reasoning and perception quickly.

---

## 33.2 But Don't Make the Framework API-Dependent

Build a pluggable model interface:

```text
             AI MODEL INTERFACE
                   │
          ┌────────┴────────┐
          │                 │
      API MODEL         LOCAL MODEL
          │                 │
   Gemini / etc.       Qwen / etc.
```

The planner should not care which backend is underneath.

```python
llm = LLMProvider(config)
vlm = VLMProvider(config)

# Planner calls:
plan = llm.plan(task, state, capabilities)
objects = vlm.detect_and_ground(image=image, instruction=instruction)
```

Provider hierarchy:

```text
LLMProvider
 ├── GeminiProvider
 ├── OpenAIProvider
 └── LocalQwenProvider

VLMProvider
 ├── GeminiVisionProvider
 └── QwenVLProvider
```

The framework should **never** contain:

```python
# BAD
if backend == "gemini":
    ...
elif backend == "qwen":
    ...
```

everywhere. The backend is hidden behind the provider interface.

---

## 33.3 Version 1 — Get It Working (API)

Use **Gemini 2.5 Flash** as both the LLM and the VLM (it is multimodal).

This makes the first prototype extremely simple:

```text
             Gemini 2.5 Flash
             /              \
            /                \
       LLM role            VLM role
       planning            vision
            \                /
             \              /
              Mission Planner
```

Input to the model:

```text
IMAGE  +  USER COMMAND  +  CURRENT STATE
```

Expected structured output:

```json
{
  "target": "red_bottle",
  "bbox": [420, 210, 510, 330],
  "confidence": 0.93
}
```

Then spatial grounding converts the bounding box to world coordinates.

### Why Gemini 2.5 Flash

- Stable model targeted at speed + intelligence for agentic and multimodal tasks
- Supports structured outputs and function calling — directly fits the Skill DSL architecture
- Multimodal: handles image/video inputs natively
- Does not require any local GPU setup

### For higher-quality research comparison

**Gemini 2.0 Pro** (or latest Pro preview) — use when testing:

> "Does a stronger reasoning model improve planning quality?"

But do **not** make Pro the default. It is slower and more expensive.

---

## 33.4 Hybrid Perception — Don't Send Every Frame to the VLM

```text
             CAMERA
                │
                ▼
          OpenCV / YOLO
                │
         fast perception
         (runs frequently)
                │
                ▼
         ┌─────────────┐
         │    VLM      │  ← runs on semantic trigger only
         │ Gemini/API  │
         └──────┬──────┘
                │
         semantic reasoning
                │
                ▼
               LLM
                │
           task planning
```

**Fast CV** (runs at high frequency):

- drone tracking
- object tracking / bounding boxes
- ArUco / AprilTag markers
- workspace localization

**VLM** (runs on semantic trigger):

- semantic object identification from natural language
- ambiguous descriptions
- scene interpretation

**LLM** (runs on planning/replanning trigger):

- task decomposition
- skill selection
- replanning after environment change

This is far more realistic than a `30 FPS → GPT → motor` loop.

---

## 33.5 Version 2 — Research Comparison (Local)

Once the framework is working, swap the backend:

```text
         MODEL-AGNOSTIC FRAMEWORK
                 │
     ┌───────────┴───────────┐
     │                       │
CLOUD MODE               LOCAL MODE
     │                       │
Gemini 2.5 Flash          Qwen3-VL
     │                       │
     └──────────┬────────────┘
                │
           SAME SKILLS
                │
           SAME DRONE
                │
           SAME TASKS
```

This lets us legitimately claim:

> **The framework is model-agnostic at the foundation-model layer.**

That is much stronger than saying: "We used Gemini to control a drone."

### On Qwen3-VL size

Do not immediately jump to the largest available checkpoint.

Choose the **smallest model that meets the task requirements**, not the largest that fits in memory. Qwen3-VL supports local deployment via vLLM/SGLang with an OpenAI-compatible serving interface, which makes swapping backends trivial.

---

## 33.6 Version 3 — Split LLM / VLM Backends

Once the framework matures, the LLM and VLM can be different models:

```text
             USER
               │
               ▼
          ┌─────────┐
          │   LLM   │  e.g. Gemini 2.5 Flash
          │ Planner │
          └────┬────┘
               │
          Skill Plan
               │
               ▼
         Mission Agent
               ▲
               │
          ┌────┴────┐
          │   VLM   │  e.g. Qwen3-VL / Gemini 2.5 Flash
          │ Vision  │
          └────┬────┘
               ▲
               │
             Camera
```

Benchmark LLM and VLM latency and accuracy independently.

---

## 33.7 What NOT to Do

| Approach | Reason to avoid |
|---|---|
| LLM directly on ESP32 | No processing capacity |
| VLM on ESP32 | No processing capacity |
| LLM → motor commands | Bypasses safety layer |
| VLM on every camera frame | Latency / cost unacceptable |
| One giant model controlling everything | Not the research question |
| Hard-code framework around Gemini | Defeats the general-framework goal |

---

## 33.8 Physical Deployment Architecture

```text
             AI COMPUTER
      Laptop / Jetson / PC
                │
    ┌───────────┴───────────┐
    │                       │
Local LLM/VLM           Cloud API
    │                       │
    └───────────┬───────────┘
                │
            Planner
                │
              Wi-Fi
                │
            ESP32-S3
                │
              Drone
```

The **ESP32 handles**: IMU, PID, motor mixing, telemetry, safety watchdog.

The **laptop/edge computer handles**: LLM, VLM, OpenCV, spatial grounding, mission planning, logging.

---

## 33.9 Decided Model Stack

| Component | Version 1 (working prototype) | Version 2 (research comparison) |
|---|---|---|
| **LLM** | Gemini 2.5 Flash API | Qwen3 local |
| **VLM** | Gemini 2.5 Flash API (multimodal) | Qwen3-VL local |
| **Fast vision** | OpenCV + YOLO / ArUco | same |
| **Planner** | Our Python code | unchanged |
| **Skill interface** | Our DSL / JSON schema | unchanged |
| **Safety** | Our code | unchanged |
| **Simulation** | MATLAB + Simulink + Simscape | unchanged |
| **Flight controller** | LiteWing firmware | unchanged |
| **AI computer** | Laptop / PC | Jetson / edge later |
| **Drone** | LiteWing ESP32-S3 | other robots later |

---

## 33.10 Key Terms to Study — Model Layer

- Model provider / backend abstraction
- Structured output / constrained generation
- Function calling / tool calling
- Gemini API: multimodal inputs, structured outputs
- Qwen3-VL: local VLM deployment, vLLM serving, SGLang
- OpenAI-compatible API interface
- Model quantization (for local deployment)
- Inference latency profiling
- API cost estimation
- Edge vs. cloud tradeoffs
- Foundation model evaluation methodology
