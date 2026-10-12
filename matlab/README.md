# MATLAB side of AERIS

Everything MATLAB in this project is the digital twin in [`twin/`](twin/README.md):
a Simulink model with a Simscape Multibody plant, sensor, estimator and
controller blocks, and an Unreal scene with RGB-D cameras, generated from two
JSON configuration files.

![The lab scene seen by the twin's room camera](../reports/twin_results/images/camera_chase.jpg)

```text
matlab/
└── twin/
    ├── run_aeris_twin.m          start the live twin and wait for Python
    ├── validate_aeris_twin.m     scripted flight with metrics and plots, no Python
    ├── preview_scene.m           render the scene cameras without the plant
    ├── build_aeris_twin.m        generates models/aeris_twin.slx from the config
    ├── config/                   airframe_litewing.json, airframe_x500.json, scene_lab.json
    ├── +aeris/                   propulsion, sensors, estimator, controller, scene, link
    └── assets/tex/               textures and ArUco markers
```

## Requirements

MATLAB R2026b with Simulink, Simscape, Simscape Multibody, UAV Toolbox,
Simulink 3D Animation, Instrument Control Toolbox and MATLAB Coder with a C
compiler (MinGW works).

## Run

```matlab
cd matlab/twin
run_aeris_twin                 % LiteWing in the lab scene, waits for the Python backend
validate_aeris_twin            % stand-alone flight test, writes cache/twin_validation
```

The earlier stand-alone scripts (Newton-Euler plant, MATLAB-figure
visualisation, UDP bridge, fixed-waypoint "scenario" players) were removed
when the twin replaced them. Measured results are in
[`reports/twin_results`](../reports/twin_results/README.md).
