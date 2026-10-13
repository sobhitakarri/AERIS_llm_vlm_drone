function experiments_aeris_twin(which, opts)
% EXPERIMENTS_AERIS_TWIN  Measured experiments on the twin, no Python, no model.
%   experiments_aeris_twin              all experiments
%   experiments_aeris_twin('steps')     one of: timeseries steps gust mass drift mc
%   experiments_aeris_twin('mc', NMC=4) fewer Monte Carlo runs
%
% Each run flies the Simscape Multibody plant with all contacts, writes its
% logged signals at 100 Hz to reports/twin_results/metrics/experiments/ and
% leaves the analysis to backend/scripts/make_thesis_figures.py, so the
% numbers in the report are computed from the files, not typed in.
%
%   timeseries  the validation flight, full signal logs (3 configurations)
%   steps       position, height and yaw steps; ground-truth and onboard feedback
%   gust        wind gust intensity sweep
%   mass        airframe mass sweep (payload), inertia scaled with mass
%   drift       dead reckoning without the camera: error against distance flown
%   mc          Monte Carlo of the validation flight over sensor turn-on biases
%   speed       0.5 m step and return at speed limits 0.2 to 1.0 m/s, onboard estimator

arguments
    which (1,:) char = 'all'
    opts.NMC (1,1) double = 8
    opts.Out (1,:) char = ''      % sub-folder of the experiments folder, e.g. a code version
end

here = fileparts(mfilename('fullpath'));
addpath(here);
repo = fileparts(fileparts(here));
outDir = fullfile(repo, 'reports', 'twin_results', 'metrics', 'experiments', opts.Out);
if ~exist(outDir, 'dir'), mkdir(outDir); end

do = @(name) any(strcmp(which, {'all', name}));
idx = struct('name', {}, 'file', {}, 'params', {});

if do('timeseries')
    for k = 1:3
        cfg = {{'litewing', true}, {'litewing', false}, {'x500', false}};
        c = cfg{k};
        T = aeris.loadParams(c{1}, 'scene_lab');
        if c{2}, T.est.mode = 0; T.core.est.mode = 0; end
        tag = sprintf('ts_%s_%s', c{1}, ternary(c{2}, 'truth', 'onboard'));
        F = runFlight(T, valCmd(T), 42);
        idx = saveRun(outDir, idx, tag, F, struct('airframe', c{1}, 'truth', c{2}));
    end
end

if do('speed')
    for v = [0.2 0.4 0.6 0.8 1.0]
        T = aeris.loadParams('litewing', 'scene_lab');
        h = T.core.home;
        C = [ 0 0 h(1)       h(2) h(3) 0 0 0 0
              1 1 h(1)       h(2) 0.8  0 0 1 0
              6 1 h(1) + 0.5 h(2) 0.8  0 v 2 0
             14 1 h(1)       h(2) 0.8  0 v 3 0 ];
        F = runFlight(T, C, 22);
        tag = sprintf('speed_%03d', round(100 * v));
        idx = saveRun(outDir, idx, tag, F, struct('vmax', v));
    end
end

if do('steps')
    for truth = [true false]
        T = aeris.loadParams('litewing', 'scene_lab');
        if truth, T.est.mode = 0; T.core.est.mode = 0; end
        h = T.core.home;
        C = [ 0  0 h(1)       h(2)       h(3) 0     0 0 0
              1  1 h(1)       h(2)       0.8  0     0 1 0
              6  1 h(1) + 0.5 h(2)       0.8  0     0 2 0
             12  1 h(1) + 0.5 h(2) + 0.5 0.8  0     0 3 0
             18  1 h(1) + 0.5 h(2) + 0.5 1.2  0     0 4 0
             24  1 h(1) + 0.5 h(2) + 0.5 1.2  pi/2  0 5 0
             30  2 h(1) + 0.5 h(2) + 0.5 1.2  pi/2  0 6 0 ];
        F = runFlight(T, C, 36);
        tag = ['steps_' ternary(truth, 'truth', 'onboard')];
        idx = saveRun(outDir, idx, tag, F, struct('truth', truth));
    end
end

if do('gust')
    for sigma = [0 0.04 0.10 0.20 0.30]
        T = aeris.loadParams('litewing', 'scene_lab');
        T.core.wind.sigma = sigma;
        F = runFlight(T, valCmd(T), 42);
        tag = sprintf('gust_%03d', round(1000 * sigma));
        idx = saveRun(outDir, idx, tag, F, struct('gust_sigma', sigma));
    end
end

if do('mass')
    base = jsondecode(fileread(fullfile(here, 'config', 'airframe_litewing.json')));
    for m = [0.033 0.040 0.045 0.055 0.065 0.075]
        A = base;
        A.inertia_kgm2 = base.inertia_kgm2 * (m / base.mass_kg);
        A.mass_kg = m;
        name = sprintf('tmp_m%03d', round(1000 * m));
        fn = fullfile(here, 'config', ['airframe_' name '.json']);
        fid = fopen(fn, 'w'); fwrite(fid, jsonencode(A)); fclose(fid);
        cleanup = onCleanup(@() delete(fn));
        T = aeris.loadParams(name, 'scene_lab');
        F = runFlight(T, valCmd(T), 42);
        tag = sprintf('mass_%03d', round(1000 * m));
        idx = saveRun(outDir, idx, tag, F, struct('mass_kg', m, 'twr', T.core.air.thrust_to_weight, ...
            'w_hover', T.core.air.w_hover));
        clear cleanup
    end
end

if do('drift')
    for k = 1:opts.NMC
        T = aeris.loadParams('litewing', 'scene_lab');
        rng(100 + k);
        T.core.sens.bias_frac = 2 * rand(2, 3) - 1;
        h = T.core.home; d = 1.2; z = 0.8;
        C = [0 0 h(1) h(2) h(3) 0 0 0 0; 1 1 h(1) h(2) z 0 0 1 0];
        t = 7; seq = 2;
        for lap = 1:3
            for c = [d 0; d d; 0 d; 0 0].'
                C(end + 1, :) = [t 1 h(1) + c(1) h(2) + c(2) z 0 0.6 seq 0]; %#ok<AGROW>
                t = t + 6; seq = seq + 1;
            end
        end
        F = runFlight(T, C, t + 2);
        tag = sprintf('drift_%02d', k);
        idx = saveRun(outDir, idx, tag, F, struct('run', k, 'bias_frac', T.core.sens.bias_frac));
    end
end

if do('mc')
    for k = 1:opts.NMC
        T = aeris.loadParams('litewing', 'scene_lab');
        rng(200 + k);
        T.core.sens.bias_frac = 2 * rand(2, 3) - 1;
        F = runFlight(T, valCmd(T), 42);
        tag = sprintf('mc_%02d', k);
        idx = saveRun(outDir, idx, tag, F, struct('run', k, 'bias_frac', T.core.sens.bias_frac));
    end
end

if isempty(which) || strcmp(which, 'all'), which = 'all'; end
fid = fopen(fullfile(outDir, ['index_' which '.json']), 'w');
fwrite(fid, jsonencode(idx, PrettyPrint=true)); fclose(fid);
fprintf('Experiments written to %s\n', outDir);
end

% ----------------------------------------------------------------------
function C = valCmd(T)
% Take-off, a box of four waypoints, a 90 degree yaw step and a landing.
h = T.core.home; z = 1.0;
ws = T.scene.workspace;
d = 0.8 * min([ws.x(2) - h(1), h(1) - ws.x(1), ws.y(2) - h(2), h(2) - ws.y(1), 1.2]);
C = [ 0.0 0 h(1)     h(2)     h(3) 0    0   0 0
      1.0 1 h(1)     h(2)     z    0    0   1 0
      6.0 1 h(1)+d   h(2)     z    0    0.6 2 0
     12.0 1 h(1)+d   h(2)+d   z    0    0.6 3 0
     18.0 1 h(1)     h(2)+d   z    0    0.6 4 0
     24.0 1 h(1)     h(2)     z    0    0.6 5 0
     30.0 1 h(1)     h(2)     z    pi/2 0   6 0
     34.0 2 h(1)     h(2)     z    pi/2 0   7 0 ];
end

function F = runFlight(T, C, tEnd)
model = build_aeris_twin(T, Render=false, Link=false, Name='aeris_twin_core');
assignin('base', 'aeris_cmd', timeseries(C(:, 2:end), C(:, 1)));
wall = tic;
out = sim(model, 'StopTime', num2str(tEnd));
F.wall_s = toc(wall);
L = out.logsout;
t = (0:0.01:tEnd).';
F.t = t;
F.p = resample(L, 'p', t, 'linear');
F.pe = resample(L, 'p_est', t, 'linear');
F.q = resample(L, 'q', t, 'linear');
F.qe = resample(L, 'q_est', t, 'linear');
st = resample(L, 'status', t, 'previous');
F.ref = st(:, 3:6);
F.armed = st(:, 1);
F.sat = st(:, 8);
F.u = resample(L, 'u', t, 'linear');
F.w = resample(L, 'w_rotor', t, 'linear');
F.bat = resample(L, 'battery', t, 'linear');
F.eul = zeros(numel(t), 3);
F.yawe = zeros(numel(t), 1);
for k = 1:numel(t)
    F.eul(k, :) = rad2deg(aeris.q2eul(F.q(k, :).')).';
    ee = aeris.q2eul(F.qe(k, :).');
    F.yawe(k) = rad2deg(ee(3));
end
F.mass = T.core.air.m;
F.T = T;
fprintf('  flight %.0f s simulated in %.1f s\n', tEnd, F.wall_s);
end

function X = resample(L, name, t, method)
v = L.get(name).Values;
D = squeeze(v.Data);
if size(D, 1) ~= numel(v.Time), D = D.'; end
X = interp1(v.Time, double(D), t, method, 'extrap');
end

function idx = saveRun(outDir, idx, tag, F, params)
tab = array2table([F.t F.p F.pe F.ref F.eul F.yawe F.u F.w * 60 / (2 * pi) F.bat F.armed F.sat], 'VariableNames', ...
    {'t', 'px', 'py', 'pz', 'ex', 'ey', 'ez', 'rx', 'ry', 'rz', 'ryaw', 'roll', 'pitch', 'yaw', 'yaw_est', ...
     'u1', 'u2', 'u3', 'u4', 'rpm1', 'rpm2', 'rpm3', 'rpm4', 'vbat', 'ibat', 'soc', 'armed', 'sat'});
file = [tag '.csv'];
writetable(tab, fullfile(outDir, file));
params.mass_kg_plant = F.mass;
params.wall_s = F.wall_s;
idx(end + 1) = struct('name', tag, 'file', file, 'params', params);
fprintf('[%s] written\n', tag);
end

function v = ternary(c, a, b)
if c, v = a; else, v = b; end
end
