# Simulink & Simscape Digital Twin

Two layers exist. You do **not** need Simscape to fly the live Python bridge.

| Layer | File | What it is |
|---|---|---|
| **Live UDP twin** | `matlab/scripts/matlab_udp_bridge.m` | Newton–Euler + PID, UDP to Python |
| **Simulink only (your license)** | `matlab/models/build_litewing_simulink.m` | Same plant in a Level-2 S-Function + Scopes + 6DoF Animation |
| **Simscape Multibody** | `build_litewing_simscape.m` | Not available — `license('test','Simscape_Multibody') = 0` |

## What “real control/physics” means here

```text
Python waypoint
    -> position PID  (XY error -> desired roll/pitch)
    -> altitude PID  (Z error  -> total thrust T)
    -> attitude/rate PID (angle error -> body torque tau)
    -> motor mixer   (T, tau -> 4 rotor forces / PWM)
    -> Newton-Euler  (m, Ixx/Iyy/Izz, gravity, gyroscopic coupling)
    -> fake IMU noise on roll/pitch
    -> telemetry UDP 5006
```

Plant files:

- `matlab/plant/quadrotor_6dof_step.m` — rigid-body integration
- `matlab/plant/cascaded_controller.m` — SI-unit cascade
- `matlab/plant/motor_mixer.m` — + configuration allocation
- `matlab/plant/litewing_closed_loop_step.m` — one inner tick

## Run the live 6-DOF twin

```matlab
cd('d:/UG/B.TECH/7th/Project_phase1/matlab/scripts');
matlab_udp_bridge
```

## Simulink 3D (Aerospace Blockset)

Uses **Aerospace Blockset** animation only (`6DoF Animation` or `MATLAB Animation`).

```matlab
cd('d:/UG/B.TECH/7th/Project_phase1/matlab/models');
build_litewing_simulink
```

Then press **Run** on `litewing_full_system`. A 3D Aerospace animation window should open.

If the builder prints `add_block failed`, drag the block by hand:

1. Library Browser → **Aerospace Blockset** → **Animation** → **MATLAB-Based Animation**
2. Drop **6DoF Animation** (or **MATLAB Animation**)
3. Wire `[x; y; -z]` into the position port and roll/pitch/yaw (radians) into Euler

`6DoF Animation` uses altitude **positive down**, so the builder already sends `-z`.

## Optional: generate Simscape Multibody `.slx`

Needs **Simulink + Simscape Multibody**. This machine has Simscape foundation (`ver simscape`) and Simulink (`license = 1`), but **`license('test','Simscape_Multibody') = 0`**. Without Multibody you cannot use Mechanics Explorer 3D bodies.

The live UDP dashboard draws a 3D quadrotor mesh (arms + rotors) that tilts with Newton–Euler roll/pitch. That is the 3D body view on this license.

Needs **Simulink + Simscape Multibody** licenses.

```matlab
cd('d:/UG/B.TECH/7th/Project_phase1/matlab/models');
build_litewing_simscape
```

That creates `litewing_plant.slx`:

1. **World Frame** + **Solver Configuration** + **Mechanism Configuration** (gravity −Z).
2. **6-DOF Joint** (free floating).
3. **Brick Solid** fuselage, mass `0.033` kg, about 9×9×2 cm.
4. **External Force and Torque** in the follower (body) frame: thrust along +Z, control torques from `cascaded_controller`.

### If you build it by hand in the Simscape Multibody UI

1. Run `init_params`.
2. New model → add **Multibody** library blocks above.
3. Set fuselage **Mass** = `params.m`.
4. Wrench **Resolution Frame** = `Follower`.
5. Force input `[0; 0; T]`, torque input `tau` (3×1).
6. Sense joint transform → `To Workspace` as `x,y,z,roll,pitch,yaw`.
7. Save as `litewing_plant.slx` then wrap with PID in `litewing_full_system.slx`.

Simscape is the **visual / Multibody** copy of the same equations. The UDP demo is already using those equations in MATLAB.
