function plant = litewing_closed_loop_step(plant, target_pos, target_yaw, params, dt)
% LITEWING_CLOSED_LOOP_STEP One inner-loop tick:
% position PID -> attitude PID -> mixer -> Newton-Euler 6-DOF.

% Altitude integrator (anti-windup)
ez = target_pos(3) - plant.p(3);
if abs(ez) < 0.6
    plant.integ_z = plant.integ_z + ez * dt;
    plant.integ_z = min(max(plant.integ_z, -0.4), 0.4);
end

[T, tau] = cascaded_controller(target_pos, target_yaw, plant, params, dt);
[forces, pwm] = motor_mixer(T, tau, params);
plant.motors = pwm;

% Use allocated forces for slightly more realistic T/tau (saturation)
l = params.l;
c = params.Kd / max(params.Kt, 1e-16);
T_act = sum(forces);
tau_act = [
    l * (forces(2) - forces(4));
    l * (forces(1) - forces(3));
    c * (forces(1) - forces(2) + forces(3) - forces(4))
];

plant = quadrotor_6dof_step(plant, T_act, tau_act, params, dt);
end
