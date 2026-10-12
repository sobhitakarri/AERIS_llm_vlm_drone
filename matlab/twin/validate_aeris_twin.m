function M = validate_aeris_twin(airframe, scene, opts)
% VALIDATE_AERIS_TWIN  Scripted closed-loop flight of the twin, no Python.
%   M = validate_aeris_twin                      onboard estimator in the loop
%   M = validate_aeris_twin('x500')
%   M = validate_aeris_twin(Truth=true)          controller on ground truth
%
% Flies take-off, a box of waypoints, a 90 degree yaw and a landing on the
% Simscape Multibody plant with all contacts, then reports step response,
% tracking and estimator metrics. Figures and metrics go to
% cache/twin_validation.

arguments
    airframe (1,:) char = 'litewing'
    scene (1,:) char = 'scene_lab'
    opts.Truth (1,1) logical = false
    opts.Plot (1,1) logical = true
end

addpath(fileparts(mfilename('fullpath')));
T = aeris.loadParams(airframe, scene);
if opts.Truth, T.est.mode = 0; T.core.est.mode = 0; end
model = build_aeris_twin(T, Render=false, Link=false, Name='aeris_twin_core');

h = T.core.home; z = 1.0;
ws = T.scene.workspace;
d = 0.8 * min([ws.x(2) - h(1), h(1) - ws.x(1), ws.y(2) - h(2), h(2) - ws.y(1), 1.2]);
%        t   mode  x        y        z  yaw   vmax seq
C = [ 0.0   0  h(1)     h(2)     h(3) 0     0   0 0
      1.0   1  h(1)     h(2)     z    0     0   1 0
      6.0   1  h(1)+d   h(2)     z    0     0.6 2 0
     12.0   1  h(1)+d   h(2)+d   z    0     0.6 3 0
     18.0   1  h(1)     h(2)+d   z    0     0.6 4 0
     24.0   1  h(1)     h(2)     z    0     0.6 5 0
     30.0   1  h(1)     h(2)     z    pi/2  0   6 0
     34.0   2  h(1)     h(2)     z    pi/2  0   7 0 ];
tEnd = 42;
assignin('base', 'aeris_cmd', timeseries(C(:, 2:end), C(:, 1)));

wall = tic;
out = sim(model, 'StopTime', num2str(tEnd));
wall = toc(wall);

L = out.logsout;
p = sig(L, 'p'); pe = sig(L, 'p_est'); q = sig(L, 'q'); st = sig(L, 'status');
u = sig(L, 'u'); bat = sig(L, 'battery'); w = sig(L, 'w_rotor');
ref = interp1(st.t, st.x(:, 3:5), p.t, 'previous', 'extrap');
armed = interp1(st.t, st.x(:, 1), p.t, 'previous', 'extrap') > 0.5;
pest = interp1(pe.t, pe.x, p.t, 'linear', 'extrap');
eul = zeros(numel(q.t), 3);
for k = 1:numel(q.t), eul(k, :) = rad2deg(aeris.q2eul(q.x(k, :).')).'; end

fly = armed & p.t > 2 & p.t < 34;
M = struct();
M.airframe = T.air.name;
M.estimator = ternary(opts.Truth, 'ground truth', 'onboard');
M.realtime_factor = tEnd / wall;
M.hover_throttle = mean(mean(u.x(u.t > 4 & u.t < 5.5, :)));
M.hover_rotor_rpm = mean(mean(w.x(w.t > 4 & w.t < 5.5, :))) * 60 / (2 * pi);

% Take-off step: 1.0 s command, settle within 5 cm of the target height.
zc = p.x(:, 3);
inBand = abs(zc - z) < 0.05 & p.t > 1;
last_out = find(~inBand & p.t > 1 & p.t < 6, 1, 'last');
M.takeoff_settle_s = p.t(min(last_out + 1, numel(p.t))) - 1.0;
M.takeoff_overshoot_cm = 100 * max(0, max(zc(p.t < 6)) - z);

% Tracking of the shaped reference (what the controller is asked to follow)
% and arrival accuracy at each waypoint (sampled just before the next one).
err = p.x - ref;
M.tracking_rmse_cm = 100 * sqrt(mean(sum(err(fly, :).^2, 2)));
M.tracking_max_cm = 100 * max(vecnorm(err(fly, :), 2, 2));
arrive = zeros(4, 1);
for k = 3:6
    i = find(p.t >= C(k + 1, 1) - 0.05, 1);
    arrive(k - 2) = 100 * norm(p.x(i, :) - C(k, 3:5));
end
M.waypoint_error_cm = arrive.';
M.max_tilt_deg = max(max(abs(eul(fly, 1:2))));
M.yaw_step_final_deg = eul(find(p.t >= 33.9, 1), 3);

% Estimator against the plant.
ee = pest - p.x;
M.estimator_rmse_cm = 100 * sqrt(mean(sum(ee(fly, :).^2, 2)));
M.estimator_final_xy_cm = 100 * norm(ee(find(p.t >= 33.9, 1), 1:2));

% Landing and energy.
M.landed = st.x(end, 2) > 0.5 && p.x(end, 3) < T.z_rest + 0.01;
M.landing_xy_error_cm = 100 * norm(p.x(end, 1:2) - h(1:2));
M.battery_used_pct = 100 * (bat.x(1, 3) - bat.x(end, 3));
M.min_voltage = min(bat.x(:, 1));

fprintf('\n=== Twin validation: %s, estimator %s ===\n', M.airframe, M.estimator);
fprintf('real-time factor (no rendering)   %.2f\n', M.realtime_factor);
fprintf('hover throttle / rotor speed      %.2f / %.0f rpm\n', M.hover_throttle, M.hover_rotor_rpm);
fprintf('take-off settle / overshoot       %.2f s / %.1f cm\n', M.takeoff_settle_s, M.takeoff_overshoot_cm);
fprintf('tracking RMSE / max               %.1f / %.1f cm\n', M.tracking_rmse_cm, M.tracking_max_cm);
fprintf('waypoint arrival error            %s cm\n', mat2str(round(M.waypoint_error_cm, 1)));
fprintf('max tilt                          %.1f deg\n', M.max_tilt_deg);
fprintf('yaw after 90 deg step             %.1f deg\n', M.yaw_step_final_deg);
fprintf('estimator RMSE / final xy drift   %.1f / %.1f cm\n', M.estimator_rmse_cm, M.estimator_final_xy_cm);
fprintf('landed / landing xy error         %d / %.1f cm\n', M.landed, M.landing_xy_error_cm);
fprintf('battery used / minimum voltage    %.2f %% / %.2f V\n', M.battery_used_pct, M.min_voltage);

outDir = fullfile(T.paths.repo, 'cache', 'twin_validation');
if ~exist(outDir, 'dir'), mkdir(outDir); end
tag = sprintf('%s_%s', T.airframe_id, ternary(opts.Truth, 'truth', 'onboard'));
fid = fopen(fullfile(outDir, ['metrics_' tag '.json']), 'w');
fwrite(fid, jsonencode(M, PrettyPrint=true)); fclose(fid);

if opts.Plot
    f = figure('Visible', 'off', 'Position', [50 50 1300 800], 'Color', 'w');
    tl = tiledlayout(f, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, sprintf('%s twin, estimator: %s', M.airframe, M.estimator));
    nexttile; plot3(p.x(:, 1), p.x(:, 2), p.x(:, 3), 'LineWidth', 1.4); hold on;
    plot3(C(2:end, 3), C(2:end, 4), C(2:end, 5), 'ko--'); grid on; axis equal;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); title('Flight path'); view(35, 25);
    nexttile; plot(p.t, p.x, 'LineWidth', 1.2); hold on; set(gca, 'ColorOrderIndex', 1);
    plot(p.t, ref, '--'); grid on; xlabel('t (s)'); ylabel('m'); title('Position (solid) vs reference (dashed)');
    legend('x', 'y', 'z', 'Location', 'best');
    nexttile; plot(p.t, 100 * vecnorm(err, 2, 2), p.t, 100 * vecnorm(ee, 2, 2), 'LineWidth', 1.2); grid on;
    xlabel('t (s)'); ylabel('cm'); title('Errors'); legend('tracking', 'estimator', 'Location', 'best');
    nexttile; plot(q.t, eul, 'LineWidth', 1.1); grid on; xlabel('t (s)'); ylabel('deg'); title('Attitude');
    legend('roll', 'pitch', 'yaw', 'Location', 'best');
    nexttile; plot(w.t, w.x * 60 / (2 * pi), 'LineWidth', 1); grid on; xlabel('t (s)'); ylabel('rpm'); title('Rotor speeds');
    nexttile; yyaxis left; plot(bat.t, bat.x(:, 1), 'LineWidth', 1.2); ylabel('V');
    yyaxis right; plot(bat.t, bat.x(:, 2), 'LineWidth', 1); ylabel('A'); grid on; xlabel('t (s)'); title('Battery');
    exportgraphics(f, fullfile(outDir, ['flight_' tag '.png']), 'Resolution', 120);
    close(f);
    fprintf('Figure and metrics written to %s\n', outDir);
end
end

function s = sig(L, name)
v = L.get(name).Values;
D = squeeze(v.Data);
if size(D, 1) ~= numel(v.Time), D = D.'; end
s.t = v.Time; s.x = double(D);
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end
