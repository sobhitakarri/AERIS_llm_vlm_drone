function trajectory = trajectory_gen(waypoints, velocity, dt)
% TRAJECTORY_GEN Generates linear interpolated trajectory points between waypoints
%
% Inputs:
%   waypoints : Nx3 matrix of target 3D setpoints [X, Y, Z]
%   velocity  : Target flight velocity (m/s)
%   dt        : Time step (seconds)

if nargin < 2, velocity = 0.5; end
if nargin < 3, dt = 0.01; end

num_wp = size(waypoints, 1);
traj_points = [];

for i = 1:(num_wp - 1)
    p_start = waypoints(i, :);
    p_end = waypoints(i+1, :);
    dist = norm(p_end - p_start);
    
    duration = max(dist / velocity, 0.5);
    steps = round(duration / dt);
    
    t_steps = linspace(0, 1, steps)';
    segment = (1 - t_steps) * p_start + t_steps * p_end;
    
    traj_points = [traj_points; segment];
end

trajectory = traj_points;
end
