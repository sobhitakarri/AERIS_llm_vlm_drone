function plant = init_litewing_plant()
% INIT_LITEWING_PLANT Zero 6-DOF rigid-body state for the LiteWing twin.
plant.p = [0; 0; 0];          % world position (m)
plant.v = [0; 0; 0];          % world velocity (m/s)
plant.eul = [0; 0; 0];        % roll, pitch, yaw (rad)  ZYX
plant.omega = [0; 0; 0];      % body rates p,q,r (rad/s)
plant.integ_z = 0;
plant.integ_xy = [0; 0];
plant.motors = [0; 0; 0; 0];  % PWM 0-255
plant.T = 0;                  % total thrust (N)
plant.tau = [0; 0; 0];        % body torques (N*m)
end
