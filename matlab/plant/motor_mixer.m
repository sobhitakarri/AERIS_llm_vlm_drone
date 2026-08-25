function [forces, pwm] = motor_mixer(T, tau, params)
% MOTOR_MIXER + configuration allocation: total thrust + body torques -> 4 forces.
% Motors: 1 front, 2 right, 3 rear, 4 left.
l = params.l;
c = params.Kd / max(params.Kt, 1e-16);
A = [
    1,   1,   1,   1;
    0,   l,   0,  -l;
    l,   0,  -l,   0;
    c,  -c,   c,  -c
];
forces = A \ [T; tau(:)];
Fmax = 2.4 * (params.m * params.g) / 4;
forces = min(max(forces, 0), Fmax);

% PWM from force (hover ~140/255 at mg/4)
F_hover = (params.m * params.g) / 4;
pwm = 140 * sqrt(max(forces, 0) / max(F_hover, 1e-9));
pwm = min(max(pwm, 0), params.max_pwm);
end
