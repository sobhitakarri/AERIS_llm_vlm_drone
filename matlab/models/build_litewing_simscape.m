function build_litewing_simscape()
% BUILD_LITEWING_SIMSCAPE
% Programmatically create matlab/models/litewing_plant.slx
%   World -> 6-DOF Joint -> Fuselage brick (33 g)
%   External Force and Torque (body-frame thrust + control torques)
%
% Requires: Simulink + Simscape Multibody.
% The live UDP bridge already runs the same physics in MATLAB
% (Newton-Euler) even if you do not have these toolboxes.

this_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(this_dir, '..', 'scripts'));
addpath(fullfile(this_dir, '..', 'plant'));
addpath(fullfile(this_dir, '..', 'controllers'));
init_params;
this_dir = fileparts(mfilename('fullpath'));

if ~license('test', 'Simulink')
    error(['Simulink license not found. Use matlab_udp_bridge.m instead — ', ...
           'it already runs Newton-Euler 6-DOF + cascaded PID in MATLAB.']);
end
if ~license('test', 'Simscape_Multibody')
    fprintf(2, ['\nSimscape Multibody is not licensed on this MATLAB.\n', ...
        'You have Simscape foundation + Simulink + Aerospace Blockset.\n', ...
        'World Frame / 6-DOF Joint / Brick Solid need Multibody — skip this file.\n\n', ...
        'For Simulink 3D (6DoF Animation), run THIS instead:\n\n', ...
        '  cd(''%s'');\n', ...
        '  build_litewing_simulink\n\n', ...
        'Then press Run on litewing_full_system.\n\n', ...
        'For the live Python twin (not Simulink):\n\n', ...
        '  cd(''%s'');\n', ...
        '  matlab_udp_bridge\n\n'], ...
        strrep(this_dir, '\', '/'), ...
        strrep(fullfile(this_dir, '..', 'scripts'), '\', '/'));
    return;
end

model = 'litewing_plant';
out_file = fullfile(this_dir, [model '.slx']);

if bdIsLoaded(model)
    close_system(model, 0);
end
if exist(out_file, 'file')
    delete(out_file);
end

new_system(model);
open_system(model);
% Gravity is NOT a block_diagram parameter. It is set on Mechanism Configuration.
set_param(model, 'Solver', 'ode23t', 'StopTime', '10');

ok = false;
try
    add_block('nesl_utility/Solver Configuration', [model '/Solver_Config'], ...
        'Position', [30 30 70 70]);
    add_block('sm_lib/Frames and Transforms/World Frame', [model '/World'], ...
        'Position', [120 30 170 80]);
    add_block('sm_lib/Frames and Transforms/Mechanism Configuration', [model '/Mech_Config'], ...
        'Position', [30 110 90 160]);
    add_block('sm_lib/Joints/6-DOF Joint', [model '/Joint6'], ...
        'Position', [250 80 320 150]);
    add_block('sm_lib/Body Elements/Brick Solid', [model '/Fuselage'], ...
        'Position', [420 80 500 150]);
    add_block('sm_lib/Forces and Torques/External Force and Torque', [model '/Wrench'], ...
        'Position', [420 190 510 250]);

    set_mech_gravity([model '/Mech_Config']);

    try
        set_param([model '/Fuselage'], 'Mass', '0.033');
    catch
    end
    try
        set_param([model '/Fuselage'], 'BrickDimensions', '[0.09 0.09 0.02]');
    catch
        try
            set_param([model '/Fuselage'], 'Dimensions', '[0.09 0.09 0.02]');
        catch
        end
    end

    sm_connect(model, 'World', 'Joint6', 'B');
    sm_connect(model, 'Joint6', 'Fuselage', 'F');
    sm_connect(model, 'Fuselage', 'Wrench', 'R');
    sm_connect(model, 'World', 'Solver_Config', 'R');
    sm_connect(model, 'World', 'Mech_Config', 'R');

    add_block('simulink/Sources/In1', [model '/Thrust'], 'Position', [300 190 330 210]);
    add_block('simulink/Sources/In1', [model '/Torque'], 'Position', [300 230 330 250]);
    try
        add_line(model, 'Thrust/1', 'Wrench/Force', 'autorouting', 'on');
        add_line(model, 'Torque/1', 'Wrench/Torque', 'autorouting', 'on');
    catch
        fprintf(['[build] Could not auto-wire Force/Torque signal ports.\n', ...
                 'Open litewing_plant and connect Thrust -> Wrench Force,\n', ...
                 'Torque -> Wrench Torque (Follower frame, Force along +Z).\n']);
    end

    ok = true;
catch ME
    fprintf(2, '[build] Simscape Multibody blocks not available:\n%s\n', ME.message);
    fprintf(['Install Simscape Multibody, or skip this .slx and use\n', ...
             'matlab/scripts/matlab_udp_bridge.m (Newton-Euler twin).\n']);
end

if ok
    save_system(model, out_file);
    fprintf('Saved Simscape plant: %s\n', out_file);
    fprintf('Press Run in Simulink — Mechanics Explorer is the 3D body view.\n');
    fprintf(['Next: Wrench Force/Torque = Follower frame, Force [0;0;T].\n', ...
             'Drive T,tau from matlab/plant/cascaded_controller.m\n']);
else
    close_system(model, 0);
end
end

function set_mech_gravity(blk)
    g = '[0 0 -9.81]';
    names = {'Gravity', 'UniformGravity', 'GravityVector', 'g'};
    for i = 1:numel(names)
        try
            set_param(blk, names{i}, g);
            return;
        catch
        end
    end
    fprintf('[build] Set gravity on Mechanism Configuration to [0 0 -9.81] by hand.\n');
end

function sm_connect(model, src, dst, dst_role)
    src_ports = {'R', 'RConn', 'Rconn', 'LConn', '1'};
    if strcmp(dst_role, 'B')
        dst_ports = {'B', 'BConn', 'Bconn', 'LConn', '1'};
    elseif strcmp(dst_role, 'F')
        dst_ports = {'F', 'FConn', 'Fconn', 'RConn', 'R', '2'};
    else
        dst_ports = {'R', 'RConn', 'Rconn', 'B', '1'};
    end
    for i = 1:numel(src_ports)
        for j = 1:numel(dst_ports)
            try
                add_line(model, [src '/' src_ports{i}], [dst '/' dst_ports{j}], ...
                    'autorouting', 'on');
                return;
            catch
            end
        end
    end
    fprintf('[build] Connect %s -> %s frame ports in the canvas if they are unconnected.\n', src, dst);
end
