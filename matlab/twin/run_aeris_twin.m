function run_aeris_twin(airframe, scene, opts)
% RUN_AERIS_TWIN  Start the live digital twin and wait for Python commands.
%   run_aeris_twin                     LiteWing in the lab scene
%   run_aeris_twin('x500')             Pixhawk-class airframe
%   run_aeris_twin('litewing', 'scene_lab', Render=false)   physics only
%
% Then start the backend:   python -m backend.main --drone twin
%
% The simulation is paced to wall-clock time. A RESTART command from
% Python stops and re-runs it from the initial state; STOP (or closing the
% model) ends this function.

arguments
    airframe (1,:) char = 'litewing'
    scene (1,:) char = 'scene_lab'
    opts.Render (1,1) logical = true
    opts.Truth (1,1) logical = false     % true: controller flies on ground truth
    opts.Hours (1,1) double = 12         % simulated time before the run ends on its own
end

twinDir = fileparts(mfilename('fullpath'));
addpath(twinDir);

T = aeris.loadParams(airframe, scene);
if opts.Truth
    T.est.mode = 0; T.core.est.mode = 0;
end
fprintf('\n=== AERIS digital twin ===\n');
fprintf('Airframe   %s (%.0f g, thrust/weight %.2f, hover %.0f rpm)\n', T.air.name, ...
    1000 * T.air.m, T.air.thrust_to_weight, T.air.w_hover * 60 / (2 * pi));
fprintf('Scene      %s (%d objects, %d calibration markers)\n', T.scene.name, ...
    numel(T.scene.objects), numel(T.scene.markers.items));
fprintf('Link       commands udp:%d  telemetry udp:%d  cameras tcp:%d\n', ...
    T.net.cmd_port, T.net.telem_port, T.net.cam_port);

model = build_aeris_twin(T, Render=opts.Render, Link=true);
% A finite horizon: the Unreal co-simulation ends itself after 20 s when
% the stop time is infinite.
set_param(model, 'EnablePacing', 'on', 'PacingRate', '1', 'StopTime', num2str(opts.Hours * 3600), ...
    'SignalLogging', 'off', 'SaveTime', 'off', 'SaveOutput', 'off', 'SaveState', 'off');   % a live run must not accumulate data
aeris.writeRuntime(T);

setappdata(0, 'AerisTwinRestart', true);
run = 0;
while getappdata(0, 'AerisTwinRestart')
    setappdata(0, 'AerisTwinRestart', false);
    run = run + 1;
    fprintf('[twin] run %d started - waiting for commands\n', run);
    try
        out = sim(model);
        info = out.SimulationMetadata.ExecutionInfo;
        fprintf('[twin] run %d ended (%s)\n', run, info.StopEvent);
        if ~isempty(info.ErrorDiagnostic)
            fprintf(2, '%s\n', info.ErrorDiagnostic.Diagnostic.message);
        end
    catch err
        fprintf(2, '[twin] simulation stopped:\n%s\n', getReport(err, 'extended', 'hyperlinks', 'off'));
        break;
    end
end
fprintf('[twin] stopped.\n');
end
