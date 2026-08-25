# LiteWing Micro-UAV — MATLAB, Simulink & Simscape Control Suite

This directory contains the physical parameter definitions, cascaded PID controllers, trajectory generation utilities, and simulation evaluation scripts for the **LiteWing ESP32-S3 quadrotor**.

---

## Folder Architecture

```text
matlab/
├── README.md                      # Instructions & workflow guide
├── scripts/
│   ├── init_params.m              # LiteWing physical & motor parameters
│   ├── run_simulation.m           # Top-level trajectory simulation runner
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

## Physical Parameters Summary (`init_params.m`)

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Mass | $m$ | $0.033$ | $\text{kg}$ ($33\text{ g}$) |
| Arm Length | $l$ | $0.045$ | $\text{m}$ |
| Inertia Roll | $I_{xx}$ | $1.65 \times 10^{-5}$ | $\text{kg}\cdot\text{m}^2$ |
| Inertia Pitch | $I_{yy}$ | $1.65 \times 10^{-5}$ | $\text{kg}\cdot\text{m}^2$ |
| Inertia Yaw | $I_{zz}$ | $2.92 \times 10^{-5}$ | $\text{kg}\cdot\text{m}^2$ |
| Thrust Coefficient | $K_t$ | $1.2 \times 10^{-7}$ | $\text{N}/(\text{rad/s})^2$ |
| Drag Coefficient | $K_d$ | $2.5 \times 10^{-9}$ | $\text{N}\cdot\text{m}/(\text{rad/s})^2$ |

---

## How to Run

1. Open MATLAB and navigate to the project directory:
   ```matlab
   cd('d:/UG/B.TECH/7th/Project_phase1/matlab');
   ```

2. Initialize LiteWing physical workspace parameters:
   ```matlab
   run('scripts/init_params.m');
   ```

3. Run closed-loop trajectory simulation:
   ```matlab
   run('scripts/run_simulation.m');
   ```

4. View step response & 3D flight trajectory plots:
   ```matlab
   run('scripts/plot_results.m');
   ```
