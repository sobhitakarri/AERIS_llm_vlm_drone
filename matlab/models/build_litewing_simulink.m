function build_litewing_simulink()
% BUILD_LITEWING_SIMULINK
% Creates matlab/models/litewing_full_system.slx
%   Clock -> Level-2 MATLAB S-Function (PID + Newton-Euler)
%   -> Scopes, XY Graph, Aerospace Blockset 6DoF / MATLAB Animation
% 3D visualization is Aerospace Blockset only.

this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, '..', 'plant'));
addpath(fullfile(this_dir, '..', 'scripts'));
addpath(fullfile(this_dir, '..', 'controllers'));
init_params;
this_dir = fileparts(mfilename('fullpath'));

if ~license('test', 'Simulink')
    error('Simulink is required for this model.');
end

model = 'litewing_full_system';
out_file = fullfile(this_dir, [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end
if exist(out_file, 'file')
    try
        delete(out_file);
    catch
    end
end

new_system(model);
open_system(model);
set_param(model, ...
    'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.01', ...
    'StopTime', '18', ...
    'RelTol', '1e-3');

add_block('simulink/Sources/Clock', [model '/Clock'], ...
    'Position', [50 102 80 128]);

add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function', ...
    [model '/LiteWing6DOF'], 'Position', [160 80 300 150]);
set_param([model '/LiteWing6DOF'], 'FunctionName', 'litewing_sfun');

add_block('simulink/Signal Routing/Demux', [model '/Demux'], ...
    'Position', [340 70 345 160], 'Outputs', '9');

add_block('simulink/Signal Routing/Mux', [model '/MuxXYZ'], ...
    'Position', [400 70 405 110], 'Inputs', '3');
add_block('simulink/Signal Routing/Mux', [model '/MuxAtt'], ...
    'Position', [400 120 405 160], 'Inputs', '3');

add_block('simulink/Sinks/Scope', [model '/Position'], ...
    'Position', [470 72 520 108]);
add_block('simulink/Sinks/Scope', [model '/Attitude'], ...
    'Position', [470 122 520 158]);
add_block('simulink/Sinks/XY Graph', [model '/TopDownXY'], ...
    'Position', [470 180 520 230]);
try
    set_param([model '/TopDownXY'], 'xmin', '-1.6', 'xmax', '1.6', ...
        'ymin', '-1.6', 'ymax', '1.6');
catch
end

add_line(model, 'Clock/1', 'LiteWing6DOF/1');
add_line(model, 'LiteWing6DOF/1', 'Demux/1');
add_line(model, 'Demux/1', 'MuxXYZ/1');
add_line(model, 'Demux/2', 'MuxXYZ/2');
add_line(model, 'Demux/3', 'MuxXYZ/3');
add_line(model, 'Demux/4', 'MuxAtt/1');
add_line(model, 'Demux/5', 'MuxAtt/2');
add_line(model, 'Demux/6', 'MuxAtt/3');
add_line(model, 'MuxXYZ/1', 'Position/1');
add_line(model, 'MuxAtt/1', 'Attitude/1');
add_line(model, 'Demux/1', 'TopDownXY/1');
add_line(model, 'Demux/2', 'TopDownXY/2');

added_anim = false;
anim_kind = '';
if license('test', 'Aerospace_Blockset')
    [added_anim, anim_kind] = add_aerospace_animation(model, this_dir);
end

save_system(model, out_file);
fprintf('Saved Simulink model:\n  %s\n', out_file);
if added_anim
    fprintf('Aerospace Blockset %s is in the model — press Run for the 3D window.\n', anim_kind);
else
    fprintf(['[build] Could not add an Aerospace Blockset animation block automatically.\n', ...
        'In Library Browser drag one of these onto litewing_full_system:\n', ...
        '  Aerospace Blockset / Animation / MATLAB-Based Animation / 6DoF Animation\n', ...
        '  Aerospace Blockset / Animation / MATLAB-Based Animation / MATLAB Animation\n', ...
        'Wire: MuxNED [x;y;-z] -> xe,  Deg2Rad(MuxAtt) -> Euler.\n']);
end
fprintf('Mission: takeoff 0-2 s, then a 1 m square, 3 s per corner.\n');
end

function [ok, kind] = add_aerospace_animation(model, models_dir)
% Aerospace Blockset 3D: prefer 6DoF Animation, else MATLAB Animation.
ok = false;
kind = '';
open_asb_libraries();

candidates = asb_anim_candidates();
blk = [model '/AeroAnim'];
src = '';
kind = '';
last_err = '';
for i = 1:size(candidates, 1)
    try
        add_block(candidates{i, 1}, blk, 'Position', [540 248 680 330]);
        src = candidates{i, 1};
        kind = candidates{i, 2};
        break;
    catch ME
        last_err = ME.message;
    end
end
if isempty(src)
    [src, kind] = locate_asb_anim_block();
    if ~isempty(src)
        try
            add_block(src, blk, 'Position', [540 248 680 330]);
        catch ME
            last_err = ME.message;
            src = '';
        end
    end
end
if isempty(src)
    fprintf('[build] Aerospace Blockset animation add_block failed.\n');
    if ~isempty(last_err)
        fprintf('[build] Last error: %s\n', last_err);
    end
    w = which('aerolib');
    if ~isempty(w)
        fprintf('[build] which aerolib: %s\n', w);
    else
        fprintf('[build] aerolib not on the MATLAB path. Run aerolib in the Command Window.\n');
    end
    return;
end
fprintf('[build] Added Aerospace Blockset %s from:\n  %s\n', kind, src);

add_block('simulink/Math Operations/Gain', [model '/Zdown'], ...
    'Gain', '-1', 'Position', [400 268 430 292]);
add_block('simulink/Signal Routing/Mux', [model '/MuxNED'], ...
    'Inputs', '3', 'Position', [460 218 465 268]);
add_block('simulink/Math Operations/Gain', [model '/Deg2Rad'], ...
    'Gain', 'pi/180', 'Position', [460 288 510 318]);
add_line(model, 'Demux/1', 'MuxNED/1');
add_line(model, 'Demux/2', 'MuxNED/2');
add_line(model, 'Demux/3', 'Zdown/1');
add_line(model, 'Zdown/1', 'MuxNED/3');
add_line(model, 'MuxAtt/1', 'Deg2Rad/1');

% 6DoF Animation uses NED altitude-down. MATLAB Animation uses Earth x-y-z.
if strcmp(kind, '6DoF Animation')
    pos_src = 'MuxNED/1';
else
    pos_src = 'MuxXYZ/1';
end
try
    add_line(model, pos_src, 'AeroAnim/1');
    add_line(model, 'Deg2Rad/1', 'AeroAnim/2');
catch ME
    fprintf('[build] Wire %s and Deg2Rad to AeroAnim ports if needed.\n%s\n', pos_src, ME.message);
end

if strcmp(kind, '6DoF Animation')
    try
        set_param(blk, ...
            'u1', '[-2 2 -2 2 -2.2 0.4]', ...
            'u2', '0.05', ...
            'u3', '4', ...
            'u4', '[0.5 0.5 -1.0]', ...
            'u5', 'Fixed position', ...
            'u6', '[2.2 1.8 -0.4]', ...
            'u7', '35', ...
            'u8', 'on');
    catch
        fprintf('[build] Set 6DoF Animation axes [-2 2 -2 2 -2.2 0.4], Size of craft 4, camera [2.2 1.8 -0.4], view 35.\n');
    end
else
    try
        acfile = ensure_litewing_ac3d();
        addpath(models_dir);
        set_param(blk, ...
            'Geometries', ['''' acfile ''''], ...
            'BoundingBoxCoordinates', '[-1.6 1.6 -1.6 1.6 0 1.7]', ...
            'CameraOffset', '[1.8 1.4 0.6]', ...
            'CameraViewAngle', '28', ...
            'SampleTime', '0.05');
    catch
        try
            set_param(blk, 'BoundingBoxCoordinates', '[-1.6 1.6 -1.6 1.6 0 1.7]', ...
                'CameraOffset', '[1.8 1.4 0.6]', 'CameraViewAngle', '28');
        catch
            fprintf('[build] Set MATLAB Animation bounding box to the 3 m room and geometry litewing.ac.\n');
        end
    end
end
ok = true;
end

function C = asb_anim_candidates()
% {library path, kind}
C = {
    'Aerospace Blockset/Animation/MATLAB-Based Animation/6DoF Animation', '6DoF Animation'
    'aerospace/Animation/MATLAB-Based Animation/6DoF Animation', '6DoF Animation'
    'aerolib/Animation MATLAB/6DoF Animation', '6DoF Animation'
    'aerolib/Animation/MATLAB-Based Animation/6DoF Animation', '6DoF Animation'
    'aerolib/Animation/6DoF Animation', '6DoF Animation'
    'asbanim/6DoF Animation', '6DoF Animation'
    'Aerospace Blockset/Animation/MATLAB-Based Animation/MATLAB Animation', 'MATLAB Animation'
    'aerospace/Animation/MATLAB-Based Animation/MATLAB Animation', 'MATLAB Animation'
    'aerolib/Animation MATLAB/MATLAB Animation', 'MATLAB Animation'
    'aerolib/Animation/MATLAB-Based Animation/MATLAB Animation', 'MATLAB Animation'
    'aerolib/Animation/MATLAB Animation', 'MATLAB Animation'
    };
end

function open_asb_libraries()
try
    evalc('aerolib');
catch
end
roots = {'aerolib', 'asbanim', 'asbanim2', 'aeroblks', 'aerospace', 'aeroanim'};
for i = 1:numel(roots)
    try
        load_system(roots{i});
    catch
    end
end
end

function [src, kind] = locate_asb_anim_block()
src = '';
kind = '';
names = {
    '6DoF Animation', '6DoF Animation'
    '6DOF Animation', '6DoF Animation'
    'MATLAB Animation', 'MATLAB Animation'
    };
try
    bds = find_system('type', 'block_diagram');
catch
    bds = {};
end
if ischar(bds)
    bds = {bds};
end
for n = 1:size(names, 1)
    for i = 1:numel(bds)
        try
            hits = find_system(bds{i}, 'LookUnderMasks', 'all', ...
                'FollowLinks', 'on', 'Name', names{n, 1});
        catch
            hits = {};
        end
        if ~isempty(hits)
            if ischar(hits)
                hits = {hits};
            end
            src = hits{1};
            kind = names{n, 2};
            return;
        end
    end
end
end

