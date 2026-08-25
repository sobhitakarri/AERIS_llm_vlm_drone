% =========================================================================
% LiteWing Closed-Loop Trajectory Simulation Runner
% =========================================================================
init_params;

disp('Starting LiteWing Trajectory Simulation...');

% Simulation Time Parameters
dt = 0.01;                        % 100 Hz simulation step
t_final = 10.0;                   % 10 seconds simulation time
time_vec = 0:dt:t_final;
N = length(time_vec);

% Generate Target Trajectory (Takeoff -> Square -> Land)
waypoints = [
    0.0,  0.0,  1.0;
    0.5,  0.5,  1.0;
    0.5, -0.5,  1.0;
   -0.5, -0.5,  1.0;
   -0.5,  0.5,  1.0;
    0.0,  0.0,  0.0
];

% Initialize State Arrays
x_history = zeros(N, 3);          % Position (X, Y, Z)
v_history = zeros(N, 3);          % Velocity (Vx, Vy, Vz)
angles_history = zeros(N, 3);     % Roll, Pitch, Yaw

current_pos = [0.0; 0.0; 0.0];
current_vel = [0.0; 0.0; 0.0];
current_angles = [0.0; 0.0; 0.0];

target_idx = 1;
wp_target = waypoints(target_idx, :)';

for k = 1:N
    t = time_vec(k);
    
    % Switch waypoints every 1.5 seconds
    if mod(t, 1.5) < dt && target_idx < size(waypoints, 1)
        target_idx = target_idx + 1;
        wp_target = waypoints(target_idx, :)';
    end
    
    % Simple Second-Order Kinematic Dynamics
    error_pos = wp_target - current_pos;
    desired_acc = params.PID.kp_z * error_pos - 1.5 * current_vel;
    
    current_vel = current_vel + desired_acc * dt;
    current_pos = current_pos + current_vel * dt;
    
    x_history(k, :) = current_pos';
    v_history(k, :) = current_vel';
    angles_history(k, :) = (error_pos(1:3)' * 5); % Simulated tilt angles
end

sim_results.time = time_vec;
sim_results.position = x_history;
sim_results.velocity = v_history;
sim_results.angles = angles_history;
sim_results.waypoints = waypoints;

assignin('base', 'sim_results', sim_results);
disp('Simulation finished successfully! Run `plot_results` to view figures.');
