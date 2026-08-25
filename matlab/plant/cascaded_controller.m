function [T, tau] = cascaded_controller(target_pos, target_yaw, plant, params, dt)
% CASCADED_CONTROLLER Position -> attitude -> rate, SI units (N, N*m).
% Outer: XY error -> desired tilt. Z error -> thrust.
% Inner: Euler error -> body-rate command -> inertia-scaled torque.

m = params.m;
g = params.g;
I = params.I;

pos_err = target_pos(:) - plant.p;
vel = plant.v;

% --- Altitude (world Z) ---
plant_z_i = plant.integ_z; %#ok<NASGU>
% integral lives on plant; caller updates after this function
ez = pos_err(3);
az_cmd = params.PID.kp_z * ez - params.PID.kd_z * vel(3) + params.PID.ki_z * plant.integ_z;
T = m * g + m * az_cmd;
T = min(max(T, 0.15 * m * g), 2.3 * m * g);

% --- Position XY -> desired roll/pitch (deg then rad) ---
[des_roll_deg, des_pitch_deg] = position_controller(target_pos(1:2), plant.p(1:2), vel(1:2), params);
des_eul = [des_roll_deg; des_pitch_deg; target_yaw * 180/pi] * pi/180;

% --- Attitude: angle error -> desired body rate ---
eul_err = des_eul - plant.eul;
eul_err(3) = atan2(sin(eul_err(3)), cos(eul_err(3)));
wn = 18; zeta = 0.85;
kp_ang = wn;
kd_rate = 2 * zeta * wn;
omega_des = kp_ang * eul_err;
omega_err = omega_des - plant.omega;
tau = I * (kd_rate * omega_err);
tau = min(max(tau, -0.004), 0.004);

% Hold on ground until takeoff thrust
if plant.p(3) <= 1e-3 && T < 1.02 * m * g && target_pos(3) <= 1e-3
    T = 0;
    tau = [0; 0; 0];
end
end
