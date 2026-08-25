function plant = quadrotor_6dof_step(plant, T, tau, params, dt)
% QUADROTOR_6DOF_STEP One Newton-Euler rigid-body step (world NED-up Z).
% Body +Z is thrust direction. Gravity is -Z in the world frame.

m = params.m;
g = params.g;
I = params.I;

R = eul_to_R(plant.eul);
acc = (1 / m) * (R * [0; 0; T]) + [0; 0; -g];

omega = plant.omega;
omega_dot = I \ (tau(:) - cross(omega, I * omega));

phi = plant.eul(1); th = plant.eul(2);
th = max(min(th, 0.6), -0.6); % keep pitch away from gimbal lock
cphi = cos(phi); sphi = sin(phi);
tth = tan(th);
cth = max(cos(th), 0.3);
W = [
    1, sphi * tth,  cphi * tth;
    0, cphi,       -sphi;
    0, sphi / cth,  cphi / cth
];
eul_dot = W * omega;

plant.p = plant.p + plant.v * dt + 0.5 * acc * dt^2;
plant.v = plant.v + acc * dt;
plant.eul = plant.eul + eul_dot * dt;
plant.omega = plant.omega + omega_dot * dt;
plant.T = T;
plant.tau = tau(:);

% Floor / ground contact
if plant.p(3) < 0
    plant.p(3) = 0;
    if plant.v(3) < 0
        plant.v(3) = 0;
    end
    if T < 0.9 * m * g
        plant.v = [0; 0; 0];
        plant.omega = [0; 0; 0];
        plant.eul(1:2) = 0;
    end
end

% Workspace soft clamp
plant.p(1) = min(max(plant.p(1), -1.7), 1.7);
plant.p(2) = min(max(plant.p(2), -1.7), 1.7);
plant.p(3) = min(max(plant.p(3), 0.0), 1.7);
end
