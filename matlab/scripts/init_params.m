% =========================================================================
% LiteWing ESP32-S3 Physical Parameters Initialization Script
% =========================================================================
% Do not `clear` here — this script is called from functions (e.g. the
% Simscape builder) whose local variables would be wiped.

disp('Initializing LiteWing Physical Parameters...');

% 1. Physical Constants
params.g = 9.81;                  % Gravity (m/s^2)
params.m = 0.033;                 % Mass = 33 grams (kg)
params.l = 0.045;                 % Arm length = 4.5 cm (m)

% 2. Inertia Tensor (kg*m^2)
params.Ixx = 1.65e-5;             % Roll moment of inertia
params.Iyy = 1.65e-5;             % Pitch moment of inertia
params.Izz = 2.92e-5;             % Yaw moment of inertia
params.I = diag([params.Ixx, params.Iyy, params.Izz]);

% 3. Propulsion & Motor Constants (Brushed Coreless Motors)
params.Kt = 1.2e-7;               % Thrust coefficient (N/(rad/s)^2)
params.Kd = 2.5e-9;               % Drag coefficient (N*m/(rad/s)^2)
params.min_pwm = 0;               % Minimum motor PWM (8-bit)
params.max_pwm = 255;             % Maximum motor PWM (8-bit)
params.hover_thrust = params.m * params.g; % Total hover thrust required (N)

% 4. Cascaded PID Controller Gains
% Angle Controller (Outer Loop)
params.PID.kp_roll_angle  = 5.0;
params.PID.ki_roll_angle  = 0.0;
params.PID.kd_roll_angle  = 0.1;

params.PID.kp_pitch_angle = 5.0;
params.PID.ki_pitch_angle = 0.0;
params.PID.kd_pitch_angle = 0.1;

% Angular Rate Controller (Inner Loop)
params.PID.kp_roll_rate   = 0.8;
params.PID.ki_roll_rate   = 0.0;
params.PID.kd_roll_rate   = 0.1;

params.PID.kp_pitch_rate  = 0.8;
params.PID.ki_pitch_rate  = 0.0;
params.PID.kd_pitch_rate  = 0.1;

params.PID.kp_yaw_rate    = 8.0;
params.PID.ki_yaw_rate    = 0.1;
params.PID.kd_yaw_rate    = 0.01;

% Altitude Controller
params.PID.kp_z           = 6.0;
params.PID.ki_z           = 1.2;
params.PID.kd_z           = 4.0;

% Outer-loop XY (deg / m)
params.PID.kp_xy          = 8.0;
params.PID.kd_xy          = 5.0;

% 5. Workspace Geofence Boundaries (meters)
params.workspace.x_limits = [-1.5, 1.5];
params.workspace.y_limits = [-1.5, 1.5];
params.workspace.z_limits = [0.0, 1.5];

assignin('base', 'params', params);
disp('LiteWing parameters loaded successfully into workspace!');
