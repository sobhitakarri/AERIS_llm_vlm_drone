% =========================================================================
% Standardized Trajectory & Telemetry Plotting Utility
% =========================================================================
if ~exist('sim_results', 'var')
    error('No sim_results found in workspace. Run `run_simulation` first.');
end

figure('Name', 'LiteWing Simulation Evaluation', 'NumberTitle', 'off', 'Position', [100, 100, 1000, 600]);

% 1. 3D Flight Trajectory Plot
subplot(2, 2, [1, 3]);
plot3(sim_results.position(:, 1), sim_results.position(:, 2), sim_results.position(:, 3), 'b-', 'LineWidth', 2);
hold on;
plot3(sim_results.waypoints(:, 1), sim_results.waypoints(:, 2), sim_results.waypoints(:, 3), 'r*', 'MarkerSize', 8);
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('3D Flight Trajectory vs Waypoints');
legend('Simulated Path', 'Waypoints', 'Location', 'northwest');

% 2. Altitude (Z) Step Response
subplot(2, 2, 2);
plot(sim_results.time, sim_results.position(:, 3), 'g-', 'LineWidth', 1.5);
grid on; xlabel('Time (s)'); ylabel('Altitude Z (m)');
title('Altitude Control (Z)');

% 3. Simulated Attitude Angles (Roll, Pitch, Yaw)
subplot(2, 2, 4);
plot(sim_results.time, sim_results.angles(:, 1), 'r-', 'LineWidth', 1.2); hold on;
plot(sim_results.time, sim_results.angles(:, 2), 'b-', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Attitude Response (Roll / Pitch)');
legend('Roll', 'Pitch');
