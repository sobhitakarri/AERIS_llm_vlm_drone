function model = build_aeris_twin(T, opts)
% BUILD_AERIS_TWIN  Generate the digital-twin Simulink model from T.
%   model = build_aeris_twin(T)
%   model = build_aeris_twin(T, Render=false)   plant + control only (no Unreal)
%   model = build_aeris_twin(T, Link=false)     no UDP/TCP link (scripted tests)
%
% The model is regenerated from the parameter struct every time so the
% scene file and the airframe file stay the only sources of truth:
%
%   Simscape Multibody   6-DOF airframe, per-rotor thrust at the hubs,
%                        contact against the floor, walls and obstacles
%   aeris.Propulsion     motors, rotor aero, wind, battery        (1 kHz)
%   aeris.Sensors        IMU / range / flow models + estimator     (1 kHz)
%   aeris.Controller     position -> attitude -> rate -> mixer     (500 Hz)
%   Simulation 3D        Unreal scene, drone actor, three cameras
%   aeris.Link           UDP commands in, telemetry out
%   aeris.CamServer      camera frames to Python over TCP

arguments
    T struct
    opts.Render (1,1) logical = true
    opts.Link (1,1) logical = true
    opts.Name (1,:) char = 'aeris_twin'
end

model = opts.Name;
outFile = fullfile(T.paths.twin, 'models', [model '.slx']);
if ~exist(fileparts(outFile), 'dir'), mkdir(fileparts(outFile)); end
if bdIsLoaded(model), close_system(model, 0); end
load_system('sm_lib'); load_system('nesl_utility'); load_system('simulink');
new_system(model);
assignin('base', 'T', T);

% Fixed step: the contact model is stiff enough to stall a variable-step
% solver at touchdown, and a fixed step keeps the cost per simulated
% second constant, which real-time pacing needs.
set_param(model, 'SolverType', 'Fixed-step', 'Solver', T.sim.solver, ...
    'FixedStep', num2str(T.sim.step), 'StopTime', 'inf', ...
    'SingleTaskRateTransMsg', 'none', ...
    'UnconnectedInputMsg', 'none', 'UnconnectedOutputMsg', 'none', ...
    'AlgebraicLoopMsg', 'error', ...
    'ReturnWorkspaceOutputs', 'on', 'SignalLogging', 'on', 'SignalLoggingName', 'logsout');

b = @(name) [model '/' name];
SM = struct('world', 'sm_lib/Frames and Transforms/World Frame', ...
    'rt', 'sm_lib/Frames and Transforms/Rigid Transform', ...
    'sensor', 'sm_lib/Frames and Transforms/Transform Sensor', ...
    'joint', 'sm_lib/Joints/6-DOF Joint', ...
    'inertia', 'sm_lib/Body Elements/Inertia', ...
    'brick', 'sm_lib/Body Elements/Brick Solid', ...
    'cyl', 'sm_lib/Body Elements/Cylindrical Solid', ...
    'sphere', 'sm_lib/Body Elements/Spherical Solid', ...
    'wrench', 'sm_lib/Forces and Torques/External Force and Torque', ...
    'contact', 'sm_lib/Forces and Torques/Spatial Contact Force', ...
    'plane', 'sm_lib/Curves and Surfaces/Infinite Plane', ...
    'mech', 'sm_lib/Utilities/Mechanism Configuration', ...
    'solver', 'nesl_utility/Solver Configuration', ...
    's2ps', 'nesl_utility/Simulink-PS Converter', ...
    'ps2s', 'nesl_utility/PS-Simulink Converter');

plantBlocks = {};
    function h = add(lib, name)
        h = add_block(lib, b(name), 'MakeNameUnique', 'off');
    end
    function h = addPlant(lib, name)
        h = add(lib, name);
        plantBlocks{end+1} = b(name);
    end
    function connect(p1, p2)
        add_line(model, p1, p2, 'autorouting', 'smart');
    end
    function geom = attachSolid(frame, name)
        % A block that exports geometry has one frame port and one geometry
        % port; Simscape only accepts the frame line on the frame port, so
        % let it tell us which is which.
        ph = get_param(b(name), 'PortHandles');
        try
            add_line(model, frame, ph.RConn(1), 'autorouting', 'smart');
            geom = ph.LConn(1);
        catch
            add_line(model, frame, ph.LConn(1), 'autorouting', 'smart');
            geom = ph.RConn(1);
        end
    end
    function p = port(name, side, idx)
        ph = get_param(b(name), 'PortHandles');
        p = ph.(side)(idx);
    end

% =====================================================================
% Simscape Multibody plant
% =====================================================================
addPlant(SM.world, 'World');
addPlant(SM.mech, 'Mechanism');
set_param(b('Mechanism'), 'GravityVector', sprintf('[0 0 %.5f]', -T.g));
addPlant(SM.solver, 'Solver');
connect(port('World', 'RConn', 1), port('Mechanism', 'RConn', 1));
connect(port('World', 'RConn', 1), port('Solver', 'RConn', 1));

addPlant(SM.rt, 'HomePose');
set_param(b('HomePose'), 'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', mat2str([T.scene.home.pos(:).' T.z_rest + 0.001]), ...
    'RotationMethod', 'StandardAxis', 'RotationStandardAxis', '+Z', ...
    'RotationAngle', num2str(T.scene.home.yaw_deg));
addPlant(SM.joint, 'FreeBody');
connect(port('World', 'RConn', 1), port('HomePose', 'LConn', 1));
connect(port('HomePose', 'RConn', 1), port('FreeBody', 'LConn', 1));
bodyFrame = port('FreeBody', 'RConn', 1);

% Mass and inertia are the airframe totals; the hull and rotor graphics
% below carry negligible mass so they do not perturb them.
addPlant(SM.inertia, 'Airframe');
set_param(b('Airframe'), 'InertiaType', 'Custom', 'Mass', 'T.air.m', ...
    'CenterOfMass', '[0 0 0]', 'MomentsOfInertia', 'T.air.I', 'ProductsOfInertia', '[0 0 0]');
connect(bodyFrame, port('Airframe', 'RConn', 1));

% Collision hull for bumps against obstacles. It stops short of the feet so
% that resting on the floor is carried by four point contacts: a flat face
% on a plane resolves to a single wandering contact point and creeps.
footR = 0.08 * T.air.L;
clear_z = 0.5 * footR;
addPlant(SM.rt, 'HullPose');
set_param(b('HullPose'), 'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', mat2str([0 0 clear_z / 2], 6));
connect(bodyFrame, port('HullPose', 'LConn', 1));
addPlant(SM.brick, 'Hull');
set_param(b('Hull'), 'BrickDimensions', mat2str(T.air.hull - [0 0 clear_z], 6), 'ExportEntireGeometry', 'on', ...
    'InertiaType', 'Custom', 'Mass', '1e-7', 'MomentsOfInertia', '1e-12*[1 1 1]', ...
    'GraphicDiffuseColor', '[0.10 0.12 0.14]', 'GraphicOpacity', '0.35');
hullGeom = attachSolid(port('HullPose', 'RConn', 1), 'Hull');
footGeom = zeros(1, 4);
for i = 1:4
    leg = sprintf('Leg%d', i); foot = sprintf('Foot%d', i);
    addPlant(SM.rt, leg);
    set_param(b(leg), 'TranslationMethod', 'Cartesian', ...
        'TranslationCartesianOffset', mat2str([T.air.rotor_xy(i, :) -T.air.hull(3)/2 + footR], 6));
    connect(bodyFrame, port(leg, 'LConn', 1));
    addPlant(SM.sphere, foot);
    set_param(b(foot), 'SphereRadius', num2str(footR), 'ExportEntireGeometry', 'on', ...
        'InertiaType', 'Custom', 'Mass', '1e-7', 'MomentsOfInertia', '1e-12*[1 1 1]', ...
        'GraphicDiffuseColor', '[0.03 0.03 0.03]');
    footGeom(i) = attachSolid(port(leg, 'RConn', 1), foot);
end

for i = 1:4
    arm = sprintf('Arm%d', i); thr = sprintf('Thrust%d', i); gfx = sprintf('Rotor%d', i);
    addPlant(SM.rt, arm);
    set_param(b(arm), 'TranslationMethod', 'Cartesian', ...
        'TranslationCartesianOffset', mat2str([T.air.rotor_xy(i, :) T.air.hull(3)/2]));
    connect(bodyFrame, port(arm, 'LConn', 1));
    addPlant(SM.wrench, thr);
    set_param(b(thr), 'EnableForceZ', 'on');
    connect(port(arm, 'RConn', 1), port(thr, 'RConn', 1));
    addPlant(SM.cyl, gfx);
    col = '[0.05 0.05 0.05]'; if T.air.rotor_xy(i, 1) > 0, col = '[0.95 0.45 0.05]'; end
    set_param(b(gfx), 'CylinderRadius', 'T.air.R', 'CylinderLength', '0.002', ...
        'InertiaType', 'Custom', 'Mass', '1e-7', 'MomentsOfInertia', '1e-12*[1 1 1]', ...
        'GraphicDiffuseColor', col, 'GraphicOpacity', '0.5');
    connect(port(arm, 'RConn', 1), port(gfx, 'RConn', 1));
end

addPlant(SM.wrench, 'BodyForce');
set_param(b('BodyForce'), 'EnableForce', 'on', 'ForceResolutionFrame', 'AttachedFrame');
connect(bodyFrame, port('BodyForce', 'RConn', 1));
addPlant(SM.wrench, 'BodyTorque');
set_param(b('BodyTorque'), 'EnableTorque', 'on', 'TorqueResolutionFrame', 'AttachedFrame');
connect(bodyFrame, port('BodyTorque', 'RConn', 1));

% ---- State sensing (one sensor per quantity keeps port order explicit) --
sensors = { ...
    'Sense_p', {'SenseXYZ', 'on'},     'm',     3; ...
    'Sense_v', {'SenseXYZDot', 'on'},  'm/s',   3; ...
    'Sense_a', {'SenseXYZDDot', 'on'}, 'm/s^2', 3; ...
    'Sense_q', {'SenseQ', 'on'},       '1',     4; ...
    'Sense_w', {'SenseOmega', 'on', 'ExpressedInFrame', 'Follower'}, 'rad/s', 3};
for i = 1:size(sensors, 1)
    name = sensors{i, 1};
    addPlant(SM.sensor, name);
    set_param(b(name), 'ObservationFrame', 'World', sensors{i, 2}{:});
    connect(port('World', 'RConn', 1), port(name, 'LConn', 1));
    connect(bodyFrame, port(name, 'RConn', 1));
    conv = [name '_out'];
    addPlant(SM.ps2s, conv);
    set_param(b(conv), 'Unit', sensors{i, 3});
    connect(port(name, 'RConn', 2), port(conv, 'LConn', 1));
end

% ---- Contact: floor, room shell, obstacles -----------------------------
addPlant(SM.plane, 'Floor');
set_param(b('Floor'), 'GraphicPlaneWidth', num2str(T.scene.room.size_m(1)), ...
    'GraphicPlaneHeight', num2str(T.scene.room.size_m(2)), 'GraphicDiffuseColor', '[0.55 0.56 0.57]');
floorGeom = attachSolid(port('World', 'RConn', 1), 'Floor');
addContact('FloorContact', floorGeom, hullGeom, 1);
for i = 1:4
    addContact(sprintf('FootContact%d', i), floorGeom, footGeom(i), 0.25);
end

if T.contact.obstacles
    room = T.scene.room.size_m(:).';
    hx = room(1)/2; hy = room(2)/2; hz = room(3); t = 0.1;
    shell = { ...
        'WallN',   [0  hy+t/2 hz/2], [room(1) t hz]; ...
        'WallS',   [0 -hy-t/2 hz/2], [room(1) t hz]; ...
        'WallE',   [ hx+t/2 0 hz/2], [t room(2) hz]; ...
        'WallW',   [-hx-t/2 0 hz/2], [t room(2) hz]; ...
        'Ceiling', [0 0 hz+t/2],     [room(1) room(2) t]};
    for i = 1:size(shell, 1)
        addObstacle(shell{i, 1}, 'brick', shell{i, 2}, 0, shell{i, 3}, [0.8 0.82 0.84], 0.08);
    end
    for i = 1:numel(T.scene.objects)
        o = T.scene.objects(i);
        if ~o.collide, continue; end
        centre = o.pos(:).' + [0 0 o.size_m(3)/2];
        kind = 'brick';
        if any(strcmpi(o.type, {'drum', 'plant'})), kind = 'cyl'; end
        addObstacle(['Obs_' o.name], kind, centre, o.yaw_deg, o.size_m(:).', o.color(:).', 0.6);
    end
end

    function addContact(name, baseGeom, bodyGeom, share)
        % share: fraction of the vehicle weight this contact carries at rest.
        addPlant(SM.contact, name);
        set_param(b(name), 'NormalStiffness', sprintf('T.contact.k*%g', share), ...
            'NormalDamping', sprintf('T.contact.c*%g', share), ...
            'NormalTransitionRegionWidth', '2e-4', ...
            'CoefficientOfStaticFriction', 'T.contact.mu_s', ...
            'CoefficientOfDynamicFriction', 'T.contact.mu_d', 'FrictionalCriticalVelocity', '0.01');
        connect(baseGeom, port(name, 'LConn', 1));
        connect(bodyGeom, port(name, 'RConn', 1));
    end

    function addObstacle(name, kind, centre, yawDeg, sz, colour, opacity)
        tf = [name '_pose'];
        addPlant(SM.rt, tf);
        set_param(b(tf), 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', mat2str(centre, 6), ...
            'RotationMethod', 'StandardAxis', 'RotationStandardAxis', '+Z', 'RotationAngle', num2str(yawDeg));
        connect(port('World', 'RConn', 1), port(tf, 'LConn', 1));
        if strcmp(kind, 'cyl')
            addPlant(SM.cyl, name);
            set_param(b(name), 'CylinderRadius', num2str(sz(1)/2), 'CylinderLength', num2str(sz(3)));
        else
            addPlant(SM.brick, name);
            set_param(b(name), 'BrickDimensions', mat2str(sz, 6));
        end
        set_param(b(name), 'ExportEntireGeometry', 'on', 'GraphicDiffuseColor', mat2str(colour, 3), ...
            'GraphicOpacity', num2str(opacity));
        addContact([name '_contact'], attachSolid(port(tf, 'RConn', 1), name), hullGeom, 1);
    end

% =====================================================================
% Propulsion, sensing, control
% =====================================================================
sysLib = 'simulink/User-Defined Functions/MATLAB System';

add(sysLib, 'Propulsion');
set_param(b('Propulsion'), 'System', 'aeris.Propulsion', 'P', 'T.core', 'Ts', 'T.Ts.phys', ...
    'SimulateUsing', 'Code generation');
add(sysLib, 'Sensors');
set_param(b('Sensors'), 'System', 'aeris.Sensors', 'P', 'T.core', 'Ts', 'T.Ts.phys', ...
    'SimulateUsing', 'Code generation');
add(sysLib, 'Controller');
set_param(b('Controller'), 'System', 'aeris.Controller', 'P', 'T.core', 'Ts', 'T.Ts.ctrl', ...
    'SimulateUsing', 'Code generation');

% One control period of transport delay between the controller and the
% motors; it also breaks the algebraic loop through the accelerometer.
add('simulink/Discrete/Unit Delay', 'MotorCmdDelay');
set_param(b('MotorCmdDelay'), 'SampleTime', 'T.Ts.ctrl', 'InitialCondition', 'zeros(4,1)');

add('simulink/Signal Routing/Selector', 'Height');
set_param(b('Height'), 'InputPortWidth', '3', 'Indices', '3');
add('simulink/Signal Routing/Selector', 'Vbat');
set_param(b('Vbat'), 'InputPortWidth', '3', 'Indices', '1');
add('simulink/Signal Routing/Demux', 'ThrustSplit');
set_param(b('ThrustSplit'), 'Outputs', '4');

for i = 1:4
    conv = sprintf('Thrust%d_in', i);
    addPlant(SM.s2ps, conv);
    set_param(b(conv), 'Unit', 'N');
    connect(port('ThrustSplit', 'Outport', i), port(conv, 'Inport', 1));
    connect(port(conv, 'RConn', 1), port(sprintf('Thrust%d', i), 'LConn', 1));
end
addPlant(SM.s2ps, 'BodyForce_in');  set_param(b('BodyForce_in'), 'Unit', 'N');
addPlant(SM.s2ps, 'BodyTorque_in'); set_param(b('BodyTorque_in'), 'Unit', 'N*m');
connect(port('BodyForce_in', 'RConn', 1), port('BodyForce', 'LConn', 1));
connect(port('BodyTorque_in', 'RConn', 1), port('BodyTorque', 'LConn', 1));

truth = @(name) port([name '_out'], 'Outport', 1);

% Propulsion(u, v, q, wb, z) -> thrust, Fb, Mb, w_rotor, battery
connect(port('MotorCmdDelay', 'Outport', 1), port('Propulsion', 'Inport', 1));
connect(truth('Sense_v'), port('Propulsion', 'Inport', 2));
connect(truth('Sense_q'), port('Propulsion', 'Inport', 3));
connect(truth('Sense_w'), port('Propulsion', 'Inport', 4));
connect(truth('Sense_p'), port('Height', 'Inport', 1));
connect(port('Height', 'Outport', 1), port('Propulsion', 'Inport', 5));
connect(port('Propulsion', 'Outport', 1), port('ThrustSplit', 'Inport', 1));
connect(port('Propulsion', 'Outport', 2), port('BodyForce_in', 'Inport', 1));
connect(port('Propulsion', 'Outport', 3), port('BodyTorque_in', 'Inport', 1));
connect(port('Propulsion', 'Outport', 5), port('Vbat', 'Inport', 1));

% Sensors(p, v, a, q, wb, ext) -> p_est, v_est, q_est, w_est, raw
connect(truth('Sense_p'), port('Sensors', 'Inport', 1));
connect(truth('Sense_v'), port('Sensors', 'Inport', 2));
connect(truth('Sense_a'), port('Sensors', 'Inport', 3));
connect(truth('Sense_q'), port('Sensors', 'Inport', 4));
connect(truth('Sense_w'), port('Sensors', 'Inport', 5));

% Controller(p, v, q, w, cmd, vbat) -> u, status
for i = 1:4
    connect(port('Sensors', 'Outport', i), port('Controller', 'Inport', i));
end
connect(port('Vbat', 'Outport', 1), port('Controller', 'Inport', 6));
connect(port('Controller', 'Outport', 1), port('MotorCmdDelay', 'Inport', 1));

add('simulink/Signal Routing/Selector', 'Range');
set_param(b('Range'), 'InputPortWidth', '7', 'Indices', '1');
connect(port('Sensors', 'Outport', 5), port('Range', 'Inport', 1));
add('simulink/Sources/Clock', 'Clock');

% Telemetry vector, see aeris.Link for the layout.
add('simulink/Signal Routing/Mux', 'Telemetry');
teleSrc = {truth('Sense_p'), truth('Sense_v'), truth('Sense_q'), truth('Sense_w'), ...
    port('Sensors', 'Outport', 1), port('Sensors', 'Outport', 2), port('Sensors', 'Outport', 3), ...
    port('Controller', 'Outport', 2), port('Propulsion', 'Outport', 4), port('Propulsion', 'Outport', 5), ...
    port('Range', 'Outport', 1), port('Clock', 'Outport', 1)};
set_param(b('Telemetry'), 'Inputs', num2str(numel(teleSrc)));
for i = 1:numel(teleSrc)
    connect(teleSrc{i}, port('Telemetry', 'Inport', i));
end

% ---- Link or scripted command source -----------------------------------
nMov = max(numel(T.movable), 1);
if opts.Link
    add(sysLib, 'Link');
    set_param(b('Link'), 'System', 'aeris.Link', 'T', 'T', 'Ts', 'T.Ts.link', ...
        'SimulateUsing', 'Interpreted execution');
    connect(port('Telemetry', 'Outport', 1), port('Link', 'Inport', 1));
    % One link period of latency on everything the link sends into the
    % loop. It models the radio hop and breaks the algebraic loop through
    % the telemetry the link reads.
    objInit = zeros(nMov, 3);
    for i = 1:numel(T.movable), objInit(i, :) = T.scene.objects(T.movable(i)).pos; end
    delays = {'CmdLatency', 'zeros(8,1)'; 'ObjLatency', mat2str(objInit, 6); 'ExtLatency', 'zeros(4,1)'};
    for i = 1:3
        add('simulink/Discrete/Unit Delay', delays{i, 1});
        set_param(b(delays{i, 1}), 'SampleTime', 'T.Ts.link', 'InitialCondition', delays{i, 2});
        connect(port('Link', 'Outport', i), port(delays{i, 1}, 'Inport', 1));
    end
    cmdSrc = port('CmdLatency', 'Outport', 1);
    objSrc = port('ObjLatency', 'Outport', 1);
    extSrc = port('ExtLatency', 'Outport', 1);
else
    % Scripted tests drive the controller from a workspace timeseries.
    add('simulink/Sources/From Workspace', 'CmdScript');
    set_param(b('CmdScript'), 'VariableName', 'aeris_cmd', 'SampleTime', 'T.Ts.link', ...
        'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value');
    add('simulink/Sources/Constant', 'ObjScript');
    set_param(b('ObjScript'), 'Value', sprintf('zeros(%d,3)', nMov), 'SampleTime', 'T.Ts.link');
    add('simulink/Sources/Constant', 'ExtScript');
    set_param(b('ExtScript'), 'Value', 'zeros(4,1)', 'SampleTime', 'T.Ts.link');
    add('simulink/Sinks/Terminator', 'TelemetryEnd');
    connect(port('Telemetry', 'Outport', 1), port('TelemetryEnd', 'Inport', 1));
    cmdSrc = port('CmdScript', 'Outport', 1);
    objSrc = port('ObjScript', 'Outport', 1);
    extSrc = port('ExtScript', 'Outport', 1);
end
connect(cmdSrc, port('Controller', 'Inport', 5));
connect(extSrc, port('Sensors', 'Inport', 6));

% ---- Logged signals -----------------------------------------------------
logs = {truth('Sense_p'), 'p'; truth('Sense_v'), 'v'; truth('Sense_q'), 'q'; truth('Sense_w'), 'wb'; ...
    port('Sensors', 'Outport', 1), 'p_est'; port('Sensors', 'Outport', 2), 'v_est'; ...
    port('Sensors', 'Outport', 3), 'q_est'; port('Controller', 'Outport', 1), 'u'; ...
    port('Controller', 'Outport', 2), 'status'; port('Propulsion', 'Outport', 1), 'thrust'; ...
    port('Propulsion', 'Outport', 4), 'w_rotor'; port('Propulsion', 'Outport', 5), 'battery'; ...
    cmdSrc, 'cmd'};
for i = 1:size(logs, 1)
    lh = get_param(logs{i, 1}, 'Line');
    if lh > 0
        set_param(lh, 'Name', logs{i, 2});
        Simulink.sdi.markSignalForStreaming(logs{i, 1}, 'on');
    end
end

% =====================================================================
% Unreal scene, drone actor and cameras
% =====================================================================
if opts.Render
    load_system('sim3dlib');
    add('sim3dlib/Simulation 3D Scene Configuration', 'Scene');
    viewer = 'off'; if T.scene.render.viewer, viewer = 'on'; end
    set_param(b('Scene'), 'SceneDesc', 'Empty scene', 'Ts', 'T.Ts.render', 'EnableWindow', viewer);

    add('sim3dlib/Simulation 3D Actor', 'Arena');
    set_param(b('Arena'), 'ActorName', 'Arena', 'SampleTime', 'T.Ts.render', ...
        'InitScriptText', 'aeris.buildScene(World, Actor, evalin(''base'', ''T''));');
    add('simulink/Sinks/Terminator', 'ArenaEnd');
    connect(port('Arena', 'Outport', 1), port('ArenaEnd', 'Inport', 1));
    if ~isempty(T.movable)
        names = arrayfun(@(k) sprintf('%s.Translation', T.scene.objects(k).name), T.movable, 'UniformOutput', false);
        set_param(b('Arena'), 'InputsText', strjoin(names, newline));
        for i = 1:numel(T.movable)
            sel = sprintf('ObjPos%d', i);
            add('simulink/Signal Routing/Selector', sel);
            set_param(b(sel), 'NumberOfDimensions', '2', 'InputPortWidth', sprintf('[%d 3]', nMov), ...
                'IndexOptions', 'Index vector (dialog),Select all', 'Indices', sprintf('%d,[]', i));
            connect(objSrc, port(sel, 'Inport', 1));
            connect(port(sel, 'Outport', 1), port('Arena', 'Inport', i));
        end
    end

    add('sim3dlib/Simulation 3D Actor', 'Drone');
    set_param(b('Drone'), 'ActorName', 'Drone', 'SampleTime', 'T.Ts.render', ...
        'InitScriptText', 'aeris.buildDrone(World, [], evalin(''base'', ''T''), ''Drone'', Actor);', ...
        'InputsText', strjoin({'Drone.Translation', 'Drone.Rotation', 'Drone_Rotor1.Rotation', ...
            'Drone_Rotor2.Rotation', 'Drone_Rotor3.Rotation', 'Drone_Rotor4.Rotation'}, newline));
    add('simulink/Sinks/Terminator', 'DroneEnd');
    connect(port('Drone', 'Outport', 1), port('DroneEnd', 'Inport', 1));
    add(sysLib, 'Visual');
    set_param(b('Visual'), 'System', 'aeris.Visual', 'P', 'T.core', 'Ts', 'T.Ts.render', ...
        'SimulateUsing', 'Interpreted execution');
    connect(truth('Sense_p'), port('Visual', 'Inport', 1));
    connect(truth('Sense_q'), port('Visual', 'Inport', 2));
    connect(port('Propulsion', 'Outport', 4), port('Visual', 'Inport', 3));
    for i = 1:6
        connect(port('Visual', 'Outport', i), port('Drone', 'Inport', i));
    end

    cams = aeris.cameraSpecs(T);
    camNames = fieldnames(cams);
    % Camera feeds. The overhead camera is RGB-D (it stands in for a
    % ceiling-mounted depth camera), the others are plain RGB.
    feeds = {};
    for i = 1:numel(camNames)
        c = cams.(camNames{i});
        blk = ['Cam_' camNames{i}];
        if strcmp(camNames{i}, 'overhead') && T.scene.render.depth
            load_system('uavsim3dlib');
            add('uavsim3dlib/Simulation 3D Camera', blk);
            set_param(b(blk), 'sensorId', num2str(i), 'CoordinateSystem', 'MATLAB', ...
                'FocalLength', mat2str([c.fx c.fy], 8), 'OpticalCenter', mat2str([c.cx c.cy], 8), ...
                'ImageSize', mat2str(c.size([2 1])), 'SampleTime', 'T.Ts.cam', 'offsetFlag', 'on', ...
                'tmountOffset', mat2str(c.pos, 6), 'rmountOffset', mat2str(deg2rad(c.rpy_deg), 8), ...
                'DepthOutportEnabled', 'on');
            feeds(end+1, :) = {port(blk, 'Outport', 1), 'overhead'}; %#ok<AGROW>
            feeds(end+1, :) = {port(blk, 'Outport', 2), 'overhead_depth'}; %#ok<AGROW>
        else
            add('sim3dlib/Simulation 3D Camera Get', blk);
            set_param(b(blk), 'sensorId', num2str(i), 'CoordinateSystem', 'MATLAB', ...
                'HorizontalResolution', num2str(c.size(1)), 'VerticalResolution', num2str(c.size(2)), ...
                'HorizontalFOV', num2str(c.hfov), 'SampleTime', 'T.Ts.cam', 'offsetFlag', 'on', ...
                'tmountOffset', mat2str(c.pos, 6), 'rmountOffset', mat2str(deg2rad(c.rpy_deg), 8));
            if strcmp(camNames{i}, 'fpv')
                set_param(b(blk), 'vehTagList', '{''Scene Origin'', ''Drone''}');
                set_param(b(blk), 'vehTag', 'Drone');
            end
            feeds(end+1, :) = {port(blk, 'Outport', 1), camNames{i}}; %#ok<AGROW>
        end
    end
    if opts.Link
        add(sysLib, 'CamServer');
        set_param(b('CamServer'), 'System', 'aeris.CamServer', 'T', 'T', 'Ts', 'T.Ts.cam', ...
            'SimulateUsing', 'Interpreted execution');
        connect(port('Clock', 'Outport', 1), port('CamServer', 'Inport', 1));
        for i = 1:size(feeds, 1)
            connect(feeds{i, 1}, port('CamServer', 'Inport', 1 + i));
        end
    else
        for i = 1:size(feeds, 1)
            sink = ['CamOut_' feeds{i, 2}];
            add('simulink/Sinks/To Workspace', sink);
            set_param(b(sink), 'VariableName', ['img_' feeds{i, 2}], 'SaveFormat', 'Array', ...
                'MaxDataPoints', '1', 'SampleTime', 'T.Ts.cam');
            connect(feeds{i, 1}, port(sink, 'Inport', 1));
        end
    end
end

% =====================================================================
% Layout
% =====================================================================
try
    Simulink.BlockDiagram.createSubsystem(getSimulinkBlockHandle(plantBlocks), 'Name', 'Plant (Simscape Multibody)');
catch err
    fprintf('[build] plant left ungrouped: %s\n', err.message);
end
try
    Simulink.BlockDiagram.arrangeSystem(model);
catch
end
save_system(model, outFile);
fprintf('[build] %s (%d blocks) saved to %s\n', model, numel(find_system(model, 'Type', 'block')), outFile);
end
