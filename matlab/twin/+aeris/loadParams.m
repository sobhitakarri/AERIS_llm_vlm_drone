function T = loadParams(airframe, scene)
% LOADPARAMS  Build the twin parameter struct from the JSON config files.
%   T = aeris.loadParams()                      LiteWing in the lab scene
%   T = aeris.loadParams('x500', 'scene_lab')   other airframe / scene
%
% Everything downstream (Simscape plant, controller, sensors, Unreal scene,
% the Python link) reads from T, so the JSON files are the only place a
% physical or scene value is written down.

if nargin < 1 || isempty(airframe), airframe = 'litewing'; end
if nargin < 2 || isempty(scene), scene = 'scene_lab'; end

twinDir = fileparts(fileparts(mfilename('fullpath')));
repoDir = fileparts(fileparts(twinDir));
A = jsondecode(fileread(fullfile(twinDir, 'config', ['airframe_' char(airframe) '.json'])));
S = jsondecode(fileread(fullfile(twinDir, 'config', [char(scene) '.json'])));

T = struct();
T.airframe_id = char(airframe);
T.scene_id = char(scene);
T.assets.tex = fullfile(twinDir, 'assets', 'tex');
T.paths.twin = twinDir;
T.paths.repo = repoDir;
T.paths.runtime = fullfile(repoDir, 'cache', 'twin_runtime.json');
T.g = 9.80665;
T.rho = 1.20;

% ---- Rates -------------------------------------------------------------
T.Ts.phys = 1 / A.sensors.imu_rate_hz;       % propulsion + sensors + estimator
T.Ts.ctrl = 1 / A.control.rate_hz;           % attitude / rate loop
T.Ts.pos_div = 5;                            % position loop runs every N control steps
T.Ts.link = 0.02;                            % telemetry + command link
T.Ts.render = 0.10;                          % Unreal frame
T.Ts.cam = 0.20;                             % camera frames to Python

T.sim.solver = 'ode3';
T.sim.step = T.Ts.phys;                      % plant integration step

% ---- Airframe ----------------------------------------------------------
air.name = A.name;
air.m = A.mass_kg;
air.I = A.inertia_kgm2(:).';
air.L = A.arm_m;
air.hull = A.hull_m(:).';
a = air.L / sqrt(2);
air.rotor_xy = [a a; -a a; -a -a; a -a];     % FLU body frame: 1 FL, 2 RL, 3 RR, 4 FR
air.spin = A.rotor.spin(:).';
air.R = A.rotor.radius_m;
air.kT = A.rotor.kT_N_per_rad2s2;
air.kQ = A.rotor.kQ_Nm_per_rad2s2;
air.Jr = A.rotor.inertia_kgm2;
air.w_max = A.rotor.omega_max_rad_s;
air.tau_up = A.rotor.tau_up_s;
air.tau_dn = A.rotor.tau_down_s;
air.k_h = A.aero.rotor_drag_N_s_per_m_rad;
air.cdA = A.aero.cd_area_m2;
air.ground_effect = double(A.aero.ground_effect);
% Thrust [T; tau_x; tau_y; tau_z] = alloc * per-rotor thrust
air.alloc = [ones(1, 4); air.rotor_xy(:, 2).'; -air.rotor_xy(:, 1).'; -air.spin * (air.kQ / air.kT)];
air.alloc_inv = inv(air.alloc);
air.T_max = air.kT * air.w_max^2;
air.w_hover = sqrt(air.m * T.g / (4 * air.kT));
air.thrust_to_weight = 4 * air.T_max / (air.m * T.g);
T.air = air;

b = A.battery;
T.bat.cells = b.cells;
T.bat.capacity_As = b.capacity_Ah * 3600;
T.bat.R = b.r_internal_ohm;
T.bat.soc_pts = b.ocv_soc(:).';
T.bat.ocv_pts = b.ocv_v(:).' * b.cells;
T.bat.v_nom = b.v_nominal;
T.bat.eta = b.drive_efficiency;
T.bat.i_avionics = b.avionics_A;
T.bat.soc0 = 0.95;

s = A.sensors;
T.sens.gyro_sigma = s.gyro_noise_density_rad_s_rtHz * sqrt(A.sensors.imu_rate_hz / 2);
T.sens.gyro_bias = s.gyro_bias_rad_s;
T.sens.gyro_walk = s.gyro_bias_walk_rad_s2_rtHz;
T.sens.accel_sigma = s.accel_noise_density_m_s2_rtHz * sqrt(A.sensors.imu_rate_hz / 2);
T.sens.accel_bias = s.accel_bias_m_s2;
T.sens.tof_div = round(A.sensors.imu_rate_hz / s.tof_rate_hz);
T.sens.tof_sigma = s.tof_sigma_m;
T.sens.tof_max = s.tof_max_m;
T.sens.flow_div = round(A.sensors.imu_rate_hz / s.flow_rate_hz);
T.sens.flow_sigma = s.flow_sigma_m_s_per_m;
T.sens.flow_min_h = s.flow_min_height_m;
T.sens.seed = 20261012;
T.sens.bias_frac = [0.6 -0.4 0.3; 0.5 -0.3 0.4];   % turn-on bias as a fraction of the bound: gyro, accel

% Estimator: Mahony attitude filter + fixed-gain position/velocity observers.
T.est.mode = 1;                 % 1 = onboard estimator, 0 = ground-truth passthrough
T.est.mahony_kp = 0.8;
T.est.mahony_ki = 0.05;
T.est.gyro_boot_s = 0.8;        % boot calibration window: the airframe is still, rotors off
T.est.gyro_trim_gain = 2e-5;    % per IMU sample while landed and still, after boot
T.est.gyro_still = 0.012;       % rad/s: gyro within this of the bias counts as still (about 4 sigma of the noise)
T.est.k_z = [18 60];            % range -> (z, vz)
T.est.k_flow = 14;              % flow velocity -> (vx, vy)
T.est.k_flow_i = 20;            % flow velocity -> horizontal acceleration bias
T.est.k_ext = [1.5 1.0];        % external fix -> (xy, yaw), per second
T.est.terrain_step = 0.12;      % range jump treated as a change of surface, not of height

% ---- Controller gains from bandwidths ----------------------------------
c = A.control;
ctl.kp_rate = c.rate_bandwidth_rad_s * [1 1 0.6];
ctl.ki_rate = 0.10 * ctl.kp_rate.^2;
ctl.kd_rate = [0.0015 0.0015 0] .* ctl.kp_rate;
ctl.kp_att = [c.attitude_bandwidth_rad_s * [1 1], c.yaw_bandwidth_rad_s];
ctl.kp_vel = [c.velocity_bandwidth_rad_s * [1 1], c.climb_rate_bandwidth_rad_s];
ctl.ki_vel = ctl.kp_vel.^2 / 6;
ctl.kp_pos = [c.position_bandwidth_rad_s * [1 1], c.altitude_bandwidth_rad_s];
ctl.tilt_max = deg2rad(c.tilt_max_deg);
ctl.v_max = [c.v_max_xy_m_s c.v_max_xy_m_s c.v_max_z_m_s];
ctl.a_max = c.a_max_m_s2;
ctl.v_land = c.v_land_m_s;
ctl.yaw_rate_max = deg2rad(c.yaw_rate_max_deg_s);
ctl.rate_lpf_hz = 60;
ctl.tumble_angle = deg2rad(70);
ctl.tumble_time = 0.35;
ctl.thrust_min = 0.04 * 4 * air.T_max;
ctl.thrust_max = 0.95 * 4 * air.T_max;
T.ctl = ctl;

% ---- Contact (penalty model sized to the vehicle) ----------------------
T.contact.k = air.m * T.g / 0.0008;
T.contact.c = 2 * 0.9 * sqrt(T.contact.k * air.m);
T.contact.mu_s = 0.6;
T.contact.mu_d = 0.45;
T.contact.obstacles = true;

% ---- Scene -------------------------------------------------------------
if ~isfield(S, 'wind'), S.wind = struct('mean_m_s', [0 0 0], 'gust_sigma_m_s', 0, 'gust_tau_s', 1); end
% Rendering options: scene file values override these defaults.
render = struct('quality', 'high', 'light_type', 'RectLight', 'light_intensity', 2500, 'shadows', true, 'viewer', true, 'depth', true);
if isfield(S, 'render')
    f = fieldnames(S.render);
    for i = 1:numel(f), render.(f{i}) = S.render.(f{i}); end
end
S.render = render;
S.objects = normaliseObjects(S.objects);
if iscell(S.markers.items), S.markers.items = [S.markers.items{:}]; end
T.scene = S;
T.movable = find([S.objects.movable]);
T.z_rest = air.hull(3) / 2;

% Footprints of everything the downward range sensor can see instead of
% the floor: [cx cy half_x half_y yaw top_z] per object.
terrain = zeros(numel(S.objects), 6);
for i = 1:numel(S.objects)
    o = S.objects(i);
    terrain(i, :) = [o.pos(1) o.pos(2) o.size_m(1)/2 o.size_m(2)/2 deg2rad(o.yaw_deg) o.pos(3)+o.size_m(3)];
end

% Numeric-only subset handed to the generated-code blocks.
T.core = struct('g', T.g, 'rho', T.rho, 'air', T.air, 'bat', T.bat, 'sens', T.sens, ...
    'est', T.est, 'ctl', T.ctl, 'z_rest', T.z_rest, ...
    'wind', struct('mean', S.wind.mean_m_s(:).', 'sigma', S.wind.gust_sigma_m_s, 'tau', S.wind.gust_tau_s), ...
    'home', [S.home.pos(:).' T.z_rest], 'home_yaw', deg2rad(S.home.yaw_deg), 'terrain', terrain);
T.core.air = rmfield(T.core.air, 'name');

T.net.cmd_port = 5005;
T.net.telem_port = 5006;
T.net.cam_port = 5007;
T.net.host = '127.0.0.1';
end

function objs = normaliseObjects(raw)
% jsondecode returns a cell array when objects carry different fields.
if isstruct(raw), raw = num2cell(raw); end
defaults = struct('name', '', 'type', 'box', 'pos', [0 0 0], 'yaw_deg', 0, ...
    'size_m', [0.2 0.2 0.2], 'color', [0.6 0.6 0.6], 'material', '', ...
    'movable', false, 'collide', false);
keys = fieldnames(defaults);
objs = repmat(defaults, 1, numel(raw));
for i = 1:numel(raw)
    for k = 1:numel(keys)
        if isfield(raw{i}, keys{k})
            v = raw{i}.(keys{k});
            if isnumeric(v), v = v(:).'; end
            objs(i).(keys{k}) = v;
        end
    end
    objs(i).movable = logical(objs(i).movable);
    objs(i).collide = logical(objs(i).collide);
end
end
