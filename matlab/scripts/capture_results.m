% =========================================================================
% AERIS results capture — run the LiteWing 6-DOF twin offline and export
% MATLAB figures + numeric metrics for the written report.
%
% Prompt (same string the Python stack would receive):
%   "Take off to 1.0 m, fly a square of side 1 m, return home and land."
%
% Execution path (code-first, no LLM):
%   NL  ->  intent_router.parse_intent  (intent=shape, shape=square, land)
%       ->  local_planner.plan_from_intent
%       ->  TAKEOFF(z=1.0) + closed MOVE_TO corners + LAND
%       ->  SafetyValidator + finalize_skills (takeoff/land sandwich)
%       ->  MATLAB plant (this file): JSON skill semantics applied as
%           setpoints to cascaded PID + Newton-Euler 6-DOF
%
% Usage (from repo root or this folder):
%   matlab -batch "cd('matlab/scripts'); capture_results"
% =========================================================================
this_dir = fileparts(mfilename('fullpath'));
repo_root = fullfile(this_dir, '..', '..');
out_root = fullfile(repo_root, 'reports', 'results_metrics');
fig_dir  = fullfile(out_root, 'matlab', 'figures');
met_dir  = fullfile(out_root, 'matlab', 'metrics');
calc_dir = fullfile(out_root, 'calculations');
if ~exist(fig_dir, 'dir'),  mkdir(fig_dir);  end
if ~exist(met_dir, 'dir'),  mkdir(met_dir);  end
if ~exist(calc_dir, 'dir'), mkdir(calc_dir); end

addpath(this_dir);
addpath(fullfile(this_dir, '..', 'controllers'));
addpath(fullfile(this_dir, '..', 'plant'));
addpath(fullfile(this_dir, '..', 'utils'));
init_params;

prompt = 'Take off to 1.0 m, fly a square of side 1 m, return home and land.';
skills = { ...
    struct('cmd','TAKEOFF','x',0.0,'y',0.0,'z',1.0,'hold',3.5), ...
    struct('cmd','MOVE_TO','x', 0.5,'y', 0.5,'z',1.0,'hold',4.0), ...
    struct('cmd','MOVE_TO','x', 0.5,'y',-0.5,'z',1.0,'hold',4.0), ...
    struct('cmd','MOVE_TO','x',-0.5,'y',-0.5,'z',1.0,'hold',4.0), ...
    struct('cmd','MOVE_TO','x',-0.5,'y', 0.5,'z',1.0,'hold',4.0), ...
    struct('cmd','MOVE_TO','x', 0.0,'y', 0.0,'z',1.0,'hold',3.5), ...
    struct('cmd','LAND',   'x',0.0,'y',0.0,'z',0.0,'hold',4.0) ...
};

dt_inner = 0.002;
dt_log   = 0.02;
n_sub    = round(dt_log / dt_inner);
settle_r = 0.08;   % m

plant = init_litewing_plant();
target_pos = [0; 0; 0];
target_yaw = 0;
last_cmd = 'IDLE';

% Dashboard is the same figure as matlab_udp_bridge.m
viz = create_live_dashboard();
sgtitle(viz.fig, sprintf('AERIS MATLAB twin  |  prompt: %s', prompt), ...
    'Color', 'w', 'FontWeight', 'bold', 'FontSize', 11);

Nmax = 20000;
log_t   = zeros(Nmax, 1);
log_p   = zeros(Nmax, 3);
log_v   = zeros(Nmax, 3);
log_eul = zeros(Nmax, 3);
log_tgt = zeros(Nmax, 3);
log_pwm = zeros(Nmax, 4);
k = 0;
sim_time = 0.0;
snap_names = {'takeoff_hold', 'square_ne', 'square_se', 'square_sw', ...
              'square_nw', 'home_hover', 'landed'};
snap_done = false(1, numel(skills));

for si = 1:numel(skills)
    sk = skills{si};
    last_cmd = sk.cmd;
    if strcmp(sk.cmd, 'TAKEOFF')
        target_pos = [plant.p(1); plant.p(2); sk.z];
    elseif strcmp(sk.cmd, 'LAND')
        target_pos = [0; 0; 0];
    else
        target_pos = [sk.x; sk.y; sk.z];
    end

    t_skill = 0;
    while t_skill < sk.hold
        for s = 1:n_sub
            plant = litewing_closed_loop_step(plant, target_pos, target_yaw, params, dt_inner);
            sim_time = sim_time + dt_inner;
            t_skill  = t_skill + dt_inner;
        end
        k = k + 1;
        log_t(k)     = sim_time;
        log_p(k, :)  = plant.p';
        log_v(k, :)  = plant.v';
        log_eul(k,:) = plant.eul';
        log_tgt(k,:) = target_pos';
        log_pwm(k,:) = plant.motors';

        if isvalid(viz.fig)
            update_live_dashboard(viz, plant.p, target_pos, plant.eul, ...
                rad2deg(plant.eul(3)), rad2deg(plant.eul(1)), rad2deg(plant.eul(2)), ...
                last_cmd, sim_time);
        end

        err = norm(plant.p - target_pos);
        if err < settle_r && t_skill > 0.8 && ~snap_done(si)
            snap_done(si) = true;
            save_matlab_fig(viz.fig, fullfile(fig_dir, sprintf('01_dashboard_%s.png', snap_names{si})));
        end
        if t_skill >= sk.hold - dt_log && ~snap_done(si)
            snap_done(si) = true;
            save_matlab_fig(viz.fig, fullfile(fig_dir, sprintf('01_dashboard_%s.png', snap_names{si})));
        end
    end
end

log_t   = log_t(1:k);
log_p   = log_p(1:k,:);
log_v   = log_v(1:k,:);
log_eul = log_eul(1:k,:);
log_tgt = log_tgt(1:k,:);
log_pwm = log_pwm(1:k,:);

% Final dashboard
if isvalid(viz.fig)
    save_matlab_fig(viz.fig, fullfile(fig_dir, '01_dashboard_final.png'));
end

% Evaluation figure (same layout as plot_results.m)
fig_eval = figure('Name', 'LiteWing 6-DOF Evaluation', 'NumberTitle', 'off', ...
    'Color', 'w', 'Position', [80, 80, 1100, 680]);
subplot(2, 2, [1, 3]);
plot3(log_p(:,1), log_p(:,2), log_p(:,3), 'b-', 'LineWidth', 2); hold on;
wps = [0 0 1; 0.5 0.5 1; 0.5 -0.5 1; -0.5 -0.5 1; -0.5 0.5 1; 0 0 1; 0 0 0];
plot3(wps(:,1), wps(:,2), wps(:,3), 'r*', 'MarkerSize', 10);
grid on; xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('6-DOF path vs skill waypoints');
legend('Newton-Euler path', 'Skill setpoints', 'Location', 'northwest');
view(35, 22);

subplot(2, 2, 2);
plot(log_t, log_p(:,3), 'Color', [0.15 0.45 0.28], 'LineWidth', 1.6); hold on;
plot(log_t, log_tgt(:,3), '--', 'Color', [0.55 0.35 0.15], 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('Z (m)');
title('Altitude tracking');
legend('measured', 'setpoint', 'Location', 'east');

subplot(2, 2, 4);
plot(log_t, rad2deg(log_eul(:,1)), 'r-', 'LineWidth', 1.1); hold on;
plot(log_t, rad2deg(log_eul(:,2)), 'b-', 'LineWidth', 1.1);
grid on; xlabel('Time (s)'); ylabel('Angle (deg)');
title('Attitude (roll / pitch)');
legend('Roll', 'Pitch');
save_matlab_fig(fig_eval, fullfile(fig_dir, '02_sixdof_evaluation.png'));

% Kinematic runner (run_simulation.m) for comparison
run_simulation;
plot_results;
fig_kin = gcf;
set(fig_kin, 'Name', 'LiteWing Kinematic Evaluation');
save_matlab_fig(fig_kin, fullfile(fig_dir, '03_kinematic_evaluation.png'));

% --- Metrics ---
err_vec = log_p - log_tgt;
rmse_xyz = sqrt(mean(err_vec.^2, 1));
rmse_pos = sqrt(mean(sum(err_vec.^2, 2)));
max_abs  = max(abs(err_vec), [], 1);
max_tilt = max(max(abs(rad2deg(log_eul(:,1:2)))));
path_len = sum(sqrt(sum(diff(log_p).^2, 2)));
gf_x = params.workspace.x_limits;
gf_y = params.workspace.y_limits;
gf_z = params.workspace.z_limits;
viol = sum(log_p(:,1) < gf_x(1) | log_p(:,1) > gf_x(2) | ...
           log_p(:,2) < gf_y(1) | log_p(:,2) > gf_y(2) | ...
           log_p(:,3) < gf_z(1) | log_p(:,3) > gf_z(2));

% Takeoff settling: first time Z within 5% of 1.0 m and staying
z_target_tk = 1.0;
in_band = abs(log_p(:,3) - z_target_tk) <= 0.05;
settle_tk = NaN;
for i = 1:numel(in_band)
    if in_band(i) && all(in_band(i:min(i+25, numel(in_band))))
        settle_tk = log_t(i);
        break;
    end
end

hover_idx = log_t > 2.0 & log_t < 3.2;
hover_z_err = mean(abs(log_p(hover_idx,3) - 1.0));
hover_thrust_N = params.m * params.g;

T = table( ...
    {'prompt'; 'execution_path'; 'dt_inner_s'; 'dt_log_s'; 'duration_s'; ...
     'rmse_x_m'; 'rmse_y_m'; 'rmse_z_m'; 'rmse_pos_m'; ...
     'max_abs_x_m'; 'max_abs_y_m'; 'max_abs_z_m'; ...
     'max_tilt_deg'; 'path_length_m'; 'geofence_violations'; ...
     'takeoff_settle_s'; 'hover_abs_z_err_m'; 'mass_kg'; 'hover_thrust_N'; ...
     'workspace_m'}, ...
    {prompt; ...
     'intent_router -> local_planner -> TAKEOFF+MOVE_TO*5+LAND -> 6DOF PID'; ...
     dt_inner; dt_log; log_t(end); ...
     rmse_xyz(1); rmse_xyz(2); rmse_xyz(3); rmse_pos; ...
     max_abs(1); max_abs(2); max_abs(3); ...
     max_tilt; path_len; viol; ...
     settle_tk; hover_z_err; params.m; hover_thrust_N; ...
     'X/Y ±1.5, Z 0–1.5'}, ...
    'VariableNames', {'metric', 'value'});
writetable(T, fullfile(met_dir, 'matlab_6dof_metrics.csv'));

traj = table(log_t, log_p(:,1), log_p(:,2), log_p(:,3), ...
    log_tgt(:,1), log_tgt(:,2), log_tgt(:,3), ...
    rad2deg(log_eul(:,1)), rad2deg(log_eul(:,2)), rad2deg(log_eul(:,3)), ...
    'VariableNames', {'t_s','x_m','y_m','z_m','tx_m','ty_m','tz_m','roll_deg','pitch_deg','yaw_deg'});
writetable(traj, fullfile(met_dir, 'matlab_6dof_trajectory.csv'));

fid = fopen(fullfile(calc_dir, 'matlab_metric_formulas.txt'), 'w');
fprintf(fid, 'RMSE_axis = sqrt(mean((p_axis - tgt_axis).^2))\n');
fprintf(fid, 'RMSE_pos  = sqrt(mean(||p-tgt||^2))\n');
fprintf(fid, 'path_len  = sum(||p[k+1]-p[k]||)\n');
fprintf(fid, 'hover_thrust_N = m * g = %.6f * %.4f = %.6f\n', params.m, params.g, hover_thrust_N);
fprintf(fid, 'takeoff_settle = first t with |Z-1.0|<=0.05 m for 0.5 s\n');
fprintf(fid, 'geofence = count samples outside X/Y ±1.5 m or Z [0, 1.5] m\n');
fclose(fid);

fprintf('\n=== MATLAB capture complete ===\n');
fprintf('RMSE xyz = [%.4f %.4f %.4f] m   pos = %.4f m\n', rmse_xyz, rmse_pos);
fprintf('max |err| xyz = [%.4f %.4f %.4f] m\n', max_abs);
fprintf('max tilt = %.2f deg   path = %.3f m   fence viol = %d\n', max_tilt, path_len, viol);
fprintf('takeoff settle = %.2f s   hover |z| err = %.4f m\n', settle_tk, hover_z_err);
fprintf('figures -> %s\n', fig_dir);

function save_matlab_fig(fig, path)
    if ~isvalid(fig), return; end
    try
        exportgraphics(fig, path, 'Resolution', 140);
    catch
        print(fig, path, '-dpng', '-r140');
    end
    fprintf('[MATLAB] saved %s\n', path);
end

function viz = create_live_dashboard()
    viz.fig = figure('Name', 'LiteWing Live Flight Dashboard', ...
        'NumberTitle', 'off', 'Color', [0.08 0.09 0.12], ...
        'Position', [60, 60, 1280, 720]);
    viz.ax3d = subplot(2, 2, 1);
    hold(viz.ax3d, 'on');
    viz.path3d = animatedline(viz.ax3d, 'Color', [0.30 0.75 1.00], ...
        'LineWidth', 2.0, 'MaximumNumPoints', 800);
    viz.drone3d = plot3(viz.ax3d, 0, 0, 0, 'o', 'MarkerSize', 14, ...
        'MarkerFaceColor', [1.00 0.45 0.20], 'MarkerEdgeColor', 'w', 'LineWidth', 1.2);
    viz.target3d = plot3(viz.ax3d, 0, 0, 0, 'p', 'MarkerSize', 16, ...
        'MarkerFaceColor', [0.40 0.95 0.55], 'MarkerEdgeColor', 'w');
    viz.quad = create_quadrotor_mesh(viz.ax3d);
    grid(viz.ax3d, 'on');
    viz.ax3d.Color = [0.12 0.14 0.18];
    viz.ax3d.XColor = [0.80 0.84 0.90];
    viz.ax3d.YColor = [0.80 0.84 0.90];
    viz.ax3d.ZColor = [0.80 0.84 0.90];
    viz.ax3d.GridColor = [0.35 0.40 0.48];
    xlabel(viz.ax3d, 'X (m)'); ylabel(viz.ax3d, 'Y (m)'); zlabel(viz.ax3d, 'Z (m)');
    title(viz.ax3d, '3D Body (MATLAB mesh — no Simscape Multibody)', 'Color', 'w');
    xlim(viz.ax3d, [-1.6 1.6]); ylim(viz.ax3d, [-1.6 1.6]); zlim(viz.ax3d, [0 1.7]);
    view(viz.ax3d, 35, 22);
    legend(viz.ax3d, {'Trail', 'Drone', 'Target'}, 'TextColor', 'w', ...
        'Color', [0.12 0.14 0.18], 'Location', 'northwest');

    viz.axxy = subplot(2, 2, 2);
    hold(viz.axxy, 'on');
    rectangle(viz.axxy, 'Position', [-1.5 -1.5 3 3], 'EdgeColor', [0.55 0.60 0.70], ...
        'LineStyle', '--', 'LineWidth', 1.2);
    plot(viz.axxy, 0, 0, '+', 'Color', [0.55 0.60 0.70], 'MarkerSize', 10);
    sq = [0.5 0.5; 0.5 -0.5; -0.5 -0.5; -0.5 0.5; 0.5 0.5];
    plot(viz.axxy, sq(:,1), sq(:,2), '--', 'Color', [0.45 0.55 0.35], 'LineWidth', 1.0);
    viz.pathxy = animatedline(viz.axxy, 'Color', [0.30 0.75 1.00], ...
        'LineWidth', 1.8, 'MaximumNumPoints', 800);
    viz.dronexy = plot(viz.axxy, 0, 0, 'o', 'MarkerSize', 14, ...
        'MarkerFaceColor', [1.00 0.45 0.20], 'MarkerEdgeColor', 'w');
    viz.heading = quiver(viz.axxy, 0, 0, 0.25, 0, 0, 'Color', [1.00 0.85 0.30], ...
        'LineWidth', 2.0, 'MaxHeadSize', 0.8);
    viz.targetxy = plot(viz.axxy, 0, 0, 'p', 'MarkerSize', 16, ...
        'MarkerFaceColor', [0.40 0.95 0.55], 'MarkerEdgeColor', 'w');
    grid(viz.axxy, 'on'); axis(viz.axxy, 'equal');
    viz.axxy.Color = [0.12 0.14 0.18];
    viz.axxy.XColor = [0.80 0.84 0.90];
    viz.axxy.YColor = [0.80 0.84 0.90];
    viz.axxy.GridColor = [0.35 0.40 0.48];
    xlabel(viz.axxy, 'X (m)'); ylabel(viz.axxy, 'Y (m)');
    title(viz.axxy, 'Top-Down Arena  [-1.5, 1.5] m', 'Color', 'w');
    xlim(viz.axxy, [-1.7 1.7]); ylim(viz.axxy, [-1.7 1.7]);

    viz.axts = subplot(2, 2, 3);
    hold(viz.axts, 'on');
    viz.lx = animatedline(viz.axts, 'Color', [0.95 0.40 0.40], 'LineWidth', 1.6, 'MaximumNumPoints', 400);
    viz.ly = animatedline(viz.axts, 'Color', [0.40 0.85 0.50], 'LineWidth', 1.6, 'MaximumNumPoints', 400);
    viz.lz = animatedline(viz.axts, 'Color', [0.40 0.70 1.00], 'LineWidth', 1.6, 'MaximumNumPoints', 400);
    grid(viz.axts, 'on');
    viz.axts.Color = [0.12 0.14 0.18];
    viz.axts.XColor = [0.80 0.84 0.90];
    viz.axts.YColor = [0.80 0.84 0.90];
    viz.axts.GridColor = [0.35 0.40 0.48];
    xlabel(viz.axts, 'Time (s)'); ylabel(viz.axts, 'Position (m)');
    title(viz.axts, 'Position vs Time', 'Color', 'w');
    ylim(viz.axts, [-1.6 1.7]);
    legend(viz.axts, {'X', 'Y', 'Z'}, 'TextColor', 'w', ...
        'Color', [0.12 0.14 0.18], 'Location', 'northwest');

    viz.axalt = subplot(2, 2, 4);
    hold(viz.axalt, 'on');
    viz.lz_meas = animatedline(viz.axalt, 'Color', [0.40 0.70 1.00], 'LineWidth', 2.0, 'MaximumNumPoints', 400);
    viz.lz_tgt = animatedline(viz.axalt, 'Color', [0.40 0.95 0.55], 'LineWidth', 1.4, ...
        'LineStyle', '--', 'MaximumNumPoints', 400);
    grid(viz.axalt, 'on');
    viz.axalt.Color = [0.12 0.14 0.18];
    viz.axalt.XColor = [0.80 0.84 0.90];
    viz.axalt.YColor = [0.80 0.84 0.90];
    viz.axalt.GridColor = [0.35 0.40 0.48];
    xlabel(viz.axalt, 'Time (s)'); ylabel(viz.axalt, 'Altitude Z (m)');
    title(viz.axalt, 'Altitude  |  cmd: IDLE', 'Color', 'w');
    ylim(viz.axalt, [-0.05 1.7]);
    legend(viz.axalt, {'Z measured', 'Z target'}, 'TextColor', 'w', ...
        'Color', [0.12 0.14 0.18], 'Location', 'northwest');
end

function h = create_quadrotor_mesh(ax)
    h = hgtransform('Parent', ax);
    s = 0.16;
    arm_c = [0.78 0.82 0.88];
    plot3([-s s], [s -s], [0 0], 'Parent', h, 'Color', arm_c, 'LineWidth', 2.8, 'HandleVisibility', 'off');
    plot3([-s s], [-s s], [0 0], 'Parent', h, 'Color', arm_c, 'LineWidth', 2.8, 'HandleVisibility', 'off');
    th = linspace(0, 2*pi, 28);
    r = 0.055;
    motors = [s s; s -s; -s -s; -s s];
    cols = [0.25 0.75 1.00; 1.00 0.55 0.20; 0.25 0.75 1.00; 1.00 0.55 0.20];
    for i = 1:4
        plot3(r*cos(th) + motors(i,1), r*sin(th) + motors(i,2), zeros(size(th)), ...
            'Parent', h, 'Color', cols(i,:), 'LineWidth', 1.6, 'HandleVisibility', 'off');
    end
    fill3([-0.04 0.04 0.04 -0.04], [-0.03 -0.03 0.03 0.03], [0 0 0 0], [0.18 0.22 0.28], ...
        'Parent', h, 'EdgeColor', [0.9 0.9 0.95], 'HandleVisibility', 'off');
    plot3([0 0.09], [0 0], [0 0], 'Parent', h, 'Color', [1 0.45 0.2], 'LineWidth', 2.4, 'HandleVisibility', 'off');
end

function update_live_dashboard(viz, pos, tgt, eul_rad, yaw_deg, roll_deg, pitch_deg, last_cmd, t)
    addpoints(viz.path3d, pos(1), pos(2), pos(3));
    set(viz.drone3d, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
    set(viz.target3d, 'XData', tgt(1), 'YData', tgt(2), 'ZData', tgt(3));
    if isfield(viz, 'quad') && isvalid(viz.quad)
        Tf = makehgtform('translate', [pos(1), pos(2), pos(3)], ...
            'zrotate', eul_rad(3), 'yrotate', eul_rad(2), 'xrotate', eul_rad(1));
        set(viz.quad, 'Matrix', Tf);
    end
    addpoints(viz.pathxy, pos(1), pos(2));
    set(viz.dronexy, 'XData', pos(1), 'YData', pos(2));
    set(viz.targetxy, 'XData', tgt(1), 'YData', tgt(2));
    hx = 0.28 * cosd(yaw_deg);
    hy = 0.28 * sind(yaw_deg);
    set(viz.heading, 'XData', pos(1), 'YData', pos(2), 'UData', hx, 'VData', hy);
    addpoints(viz.lx, t, pos(1));
    addpoints(viz.ly, t, pos(2));
    addpoints(viz.lz, t, pos(3));
    tmin = max(0, t - 20);
    tmax = max(20, t);
    xlim(viz.axts, [tmin, tmax]);
    addpoints(viz.lz_meas, t, pos(3));
    addpoints(viz.lz_tgt, t, tgt(3));
    xlim(viz.axalt, [tmin, tmax]);
    title(viz.axalt, sprintf('Altitude  |  cmd: %s', last_cmd), 'Color', 'w');
    sgtitle(viz.fig, sprintf( ...
        'LiteWing 6-DOF Newton-Euler  |  %s  |  X=%.2f  Y=%.2f  Z=%.2f m  |  yaw=%.0f  roll=%.1f  pitch=%.1f', ...
        last_cmd, pos(1), pos(2), pos(3), yaw_deg, roll_deg, pitch_deg), ...
        'Color', 'w', 'FontWeight', 'bold');
    drawnow limitrate;
end
