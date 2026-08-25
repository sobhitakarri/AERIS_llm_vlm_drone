% =========================================================================
% Real-Time MATLAB UDP Bridge (Python AI Framework <-> MATLAB/Simulink)
% =========================================================================
% Listens for incoming setpoints from Python on UDP Port 5005
% Streams simulated 6-DOF telemetry back to Python on UDP Port 5006
%
% Socket order (new MATLAB first):
%   1) udpport          (Instrument Control Toolbox, R2020b+)
%   2) classic udp      (removed in recent MATLAB; used only if present)
%   3) Java DatagramSocket (no toolbox required)
%
% Cleanup uses udpportfind / icdevicefind (LegacyMode=false) instead of
% the deprecated instrfind path wherever those APIs exist.
% =========================================================================
clear; clc;
this_dir = fileparts(mfilename('fullpath'));
addpath(this_dir);
addpath(fullfile(this_dir, '..', 'controllers'));
addpath(fullfile(this_dir, '..', 'plant'));
addpath(fullfile(this_dir, '..', 'utils'));
init_params;

disp('====================================================');
disp('   LiteWing MATLAB Real-Time UDP Socket Bridge');
disp('   Plant: Newton-Euler 6-DOF + cascaded PID (500 Hz inner loop)');
disp('   Listening for Python AI commands on Port 5005...');
disp('   Streaming 6-DOF telemetry back to Python on Port 5006...');
disp('====================================================');

% Port Setup
rx_port = 5005;
tx_port = 5006;
target_host = '127.0.0.1';

% 1. Release leftover sockets. Do NOT use instrfind for modern icdevice
%    objects (LegacyMode=false) — MATLAB requires icdevicefind for those.
%    udpport objects are found with udpportfind, not instrfind.
cleanup_existing_sockets();

use_modern_udpport = false;
use_classic_udp = false;
use_java_udp = false;
u_rx = [];
u_tx = [];

% 2. Initialize UDP sockets (modern -> classic -> Java)
udpport_available = (exist('udpport', 'class') == 8) || (exist('udpport', 'file') ~= 0);
if udpport_available
    try
        % Prefer tagged sockets so udpportfind can reclaim them later (R2024a+).
        try
            u_rx = udpport("datagram", "IPV4", ...
                "LocalPort", rx_port, ...
                "EnablePortSharing", true, ...
                "Tag", "LiteWingRx");
            u_tx = udpport("datagram", "IPV4", ...
                "EnablePortSharing", true, ...
                "Tag", "LiteWingTx");
        catch
            u_rx = udpport("datagram", "LocalPort", rx_port);
            u_tx = udpport("datagram");
        end
        use_modern_udpport = true;
        disp('[MATLAB] Connected using modern `udpport` API.');
    catch ME
        fprintf('[MATLAB] `udpport` creation note: %s\n', ME.message);
        u_rx = [];
        u_tx = [];
    end
end

classic_udp_available = ~use_modern_udpport && ...
    ((exist('udp', 'file') ~= 0) || (exist('udp', 'builtin') ~= 0));
if classic_udp_available
    try
        u_rx = udp(target_host, 'LocalPort', rx_port);
        set(u_rx, 'InputBufferSize', 4096);
        set(u_rx, 'Timeout', 0.1);
        fopen(u_rx);

        u_tx = udp(target_host, 'RemotePort', tx_port);
        set(u_tx, 'OutputBufferSize', 4096);
        fopen(u_tx);
        use_classic_udp = true;
        disp('[MATLAB] Connected using classic `udp` API.');
    catch ME
        fprintf('[MATLAB] Classic `udp` creation note: %s\n', ME.message);
        u_rx = [];
        u_tx = [];
    end
end

if ~use_modern_udpport && ~use_classic_udp
    try
        [u_rx, u_tx] = create_java_udp_sockets(rx_port);
        use_java_udp = true;
        disp('[MATLAB] Connected using Java DatagramSocket (no Instrument Control Toolbox).');
    catch ME
        error(['Failed to create UDP socket. Need `udpport` (Instrument Control Toolbox), ' ...
               'classic `udp`, or a working MATLAB JVM.\nDetails: %s'], ME.message);
    end
end

% Close sockets cleanly on Ctrl+C / error / script exit
cleanupObj = onCleanup(@() close_bridge_sockets(u_rx, u_tx, ...
    use_modern_udpport, use_classic_udp, use_java_udp)); %#ok<NASGU>

% Current State Initialization (rigid-body plant)
plant = init_litewing_plant();
target_pos  = [0.0; 0.0; 0.0];
target_yaw  = 0.0;
last_cmd = "IDLE";
sim_time = 0.0;

dt = 0.05;          % 20 Hz UDP / plot
dt_inner = 0.002;   % 500 Hz control + physics
n_sub = round(dt / dt_inner);

% Live flight dashboard (3D path, top-down arena, XYZ vs time)
viz = create_live_dashboard();
disp('[MATLAB] Live flight dashboard opened. Send commands from the Python UI.');

while true
    last_pkt = "";

    % 3. Read Incoming Command Packet from Python (Port 5005)
    if use_modern_udpport
        if u_rx.NumDatagramsAvailable > 0
            datagram = read(u_rx, u_rx.NumDatagramsAvailable, "string");
            last_pkt = datagram(end).Data;
        end
    elseif use_classic_udp
        if u_rx.BytesAvailable > 0
            last_pkt = fscanf(u_rx, '%s');
        end
    else
        last_pkt = java_udp_receive(u_rx);
    end

    if strlength(last_pkt) > 0
        try
            cmd_data = jsondecode(char(last_pkt));
            fprintf('[MATLAB] Received Command: %s\n', cmd_data.cmd);

            last_cmd = string(cmd_data.cmd);
            if strcmp(cmd_data.cmd, 'TAKEOFF')
                target_pos(3) = cmd_data.z;
            elseif strcmp(cmd_data.cmd, 'LAND')
                target_pos = [0.0; 0.0; 0.0];
            elseif strcmp(cmd_data.cmd, 'MOVE_TO')
                target_pos = [cmd_data.x; cmd_data.y; cmd_data.z];
            elseif strcmp(cmd_data.cmd, 'HOVER')
                % Hold current target setpoint
            elseif strcmp(cmd_data.cmd, 'ROTATE')
                if isfield(cmd_data, 'angle')
                    target_yaw = target_yaw + cmd_data.angle * pi/180;
                end
            elseif strcmp(cmd_data.cmd, 'EMERGENCY')
                target_pos = [0.0; 0.0; 0.0];
                plant.T = 0;
            elseif strcmp(cmd_data.cmd, 'CLEAR_PATH')
                if isvalid(viz.fig)
                    clear_live_paths(viz);
                end
            elseif strcmp(cmd_data.cmd, 'HOME') || strcmp(cmd_data.cmd, 'RESET')
                plant = init_litewing_plant();
                target_pos  = [0.0; 0.0; 0.0];
                target_yaw  = 0.0;
                sim_time = 0.0;
                last_cmd = "HOME";
                if isvalid(viz.fig)
                    clear_live_paths(viz);
                end
            end
        catch
        end
    end

    % MATLAB figure buttons / keys: Clear Path, Return Home
    if isvalid(viz.fig)
        if logical(getappdata(viz.fig, 'do_clear'))
            clear_live_paths(viz);
            setappdata(viz.fig, 'do_clear', false);
            last_cmd = "CLEAR_PATH";
            fprintf('[MATLAB] Cleared flight trails.\n');
        end
        if logical(getappdata(viz.fig, 'do_home'))
            plant = init_litewing_plant();
            target_pos  = [0.0; 0.0; 0.0];
            target_yaw  = 0.0;
            sim_time = 0.0;
            last_cmd = "HOME";
            clear_live_paths(viz);
            setappdata(viz.fig, 'do_home', false);
            fprintf('[MATLAB] Reset to home (0, 0, 0).\n');
        end
    end

    % 4. Cascaded PID + Newton-Euler 6-DOF (substepped)
    for k = 1:n_sub
        plant = litewing_closed_loop_step(plant, target_pos, target_yaw, params, dt_inner);
    end

    current_pos = plant.p;
    current_vel = plant.v;
    pos_err = target_pos - current_pos;
    current_yaw = plant.eul(3) * 180/pi;

    % Fake IMU (gyro/accel noise) on reported attitude
    roll_deg  = plant.eul(1) * 180/pi + 0.15 * randn();
    pitch_deg = plant.eul(2) * 180/pi + 0.15 * randn();

    % 5. Format and Send 6-DOF Telemetry Packet back to Python (Port 5006)
    telem.x = current_pos(1);
    telem.y = current_pos(2);
    telem.z = current_pos(3);
    telem.roll = roll_deg;
    telem.pitch = pitch_deg;
    telem.yaw = current_yaw;
    telem.vx = current_vel(1);
    telem.vy = current_vel(2);
    telem.vz = current_vel(3);
    telem.battery_v = 4.1;
    telem.motors = plant.motors;

    sim_time = sim_time + dt;
    if isvalid(viz.fig)
        update_live_dashboard(viz, current_pos, target_pos, plant.eul, ...
            current_yaw, roll_deg, pitch_deg, last_cmd, sim_time);
    end
    litewing_aero_viz(plant.p, plant.eul);

    json_str = jsonencode(telem);

    if use_modern_udpport
        write(u_tx, string(json_str), "string", target_host, tx_port);
    elseif use_classic_udp
        fprintf(u_tx, '%s', json_str);
    else
        java_udp_send(u_tx, json_str, target_host, tx_port);
    end

    pause(dt);
end

% -------------------------------------------------------------------------
% Local helpers (script local functions, R2016b+)
% -------------------------------------------------------------------------
function cleanup_existing_sockets()
    % Modern udpport connections (R2024a+)
    try
        if exist('udpportfind', 'file') || exist('udpportfind', 'builtin')
            old_udp = udpportfind;
            if ~isempty(old_udp)
                delete(old_udp);
            end
            clear old_udp;
        end
    catch
    end

    % icdevice objects created with LegacyMode=false MUST use icdevicefind.
    % instrfind cannot manage those objects and warns/errors on new MATLAB.
    try
        if exist('icdevicefind', 'file') || exist('icdevicefind', 'builtin')
            old_icd = icdevicefind;
            if ~isempty(old_icd)
                delete(old_icd);
            end
            clear old_icd;
        end
    catch
    end

    % Legacy serial/udp/tcpip objects only. Skip instrfind when modern
    % finders exist — it warns and cannot manage udpport / LegacyMode=false icdevice.
    has_modern_finders = (exist('udpportfind', 'file') ~= 0) || ...
                         (exist('icdevicefind', 'file') ~= 0);
    if ~has_modern_finders
        try
            if exist('instrfind', 'file')
                old_objs = instrfind;
                if ~isempty(old_objs)
                    fclose(old_objs);
                    delete(old_objs);
                end
                clear old_objs;
            end
        catch
        end
    end
end

function viz = create_live_dashboard()
    viz.fig = figure('Name', 'LiteWing Live Flight Dashboard', ...
        'NumberTitle', 'off', 'Color', [0.08 0.09 0.12], ...
        'Position', [60, 60, 1280, 720]);

    % --- 3D trajectory ---
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

    % --- Top-down arena ---
    viz.axxy = subplot(2, 2, 2);
    hold(viz.axxy, 'on');
    rectangle(viz.axxy, 'Position', [-1.5 -1.5 3 3], 'EdgeColor', [0.55 0.60 0.70], ...
        'LineStyle', '--', 'LineWidth', 1.2);
    plot(viz.axxy, 0, 0, '+', 'Color', [0.55 0.60 0.70], 'MarkerSize', 10);
    sq = [0.5 0.5; 0.5 -0.5; -0.5 -0.5; -0.5 0.5; 0.5 0.5];
    plot(viz.axxy, sq(:,1), sq(:,2), '--', 'Color', [0.45 0.55 0.35], ...
        'LineWidth', 1.0);
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

    % --- XYZ vs time ---
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

    % --- Altitude + status ---
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

    sgtitle(viz.fig, 'LiteWing 6-DOF  |  waiting for Python commands...', 'Color', 'w', 'FontWeight', 'bold');

    setappdata(viz.fig, 'do_clear', false);
    setappdata(viz.fig, 'do_home', false);
    viz.fig.KeyPressFcn = @(~, evt) dashboard_keypress(viz.fig, evt);

    uicontrol(viz.fig, 'Style', 'pushbutton', 'String', 'Clear Path', ...
        'Units', 'pixels', 'Position', [16, 10, 110, 28], ...
        'FontWeight', 'bold', 'BackgroundColor', [0.20 0.32 0.48], 'ForegroundColor', 'w', ...
        'Callback', @(~,~) setappdata(viz.fig, 'do_clear', true));
    uicontrol(viz.fig, 'Style', 'pushbutton', 'String', 'Return Home', ...
        'Units', 'pixels', 'Position', [134, 10, 120, 28], ...
        'FontWeight', 'bold', 'BackgroundColor', [0.18 0.42 0.28], 'ForegroundColor', 'w', ...
        'Callback', @(~,~) setappdata(viz.fig, 'do_home', true));
    uicontrol(viz.fig, 'Style', 'text', 'String', '  Keys: C = clear path,  H = home (0,0,0)', ...
        'Units', 'pixels', 'Position', [264, 10, 280, 26], ...
        'BackgroundColor', [0.08 0.09 0.12], 'ForegroundColor', [0.70 0.74 0.80], ...
        'HorizontalAlignment', 'left');
end

function h = create_quadrotor_mesh(ax)
% Visual X-quad (scaled ~0.16 m so tilt is visible in the 3 m arena).
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

function dashboard_keypress(fig, evt)
    if ~isvalid(fig), return; end
    if strcmpi(evt.Key, 'c')
        setappdata(fig, 'do_clear', true);
    elseif strcmpi(evt.Key, 'h')
        setappdata(fig, 'do_home', true);
    end
end

function clear_live_paths(viz)
    if ~isfield(viz, 'fig') || ~isvalid(viz.fig), return; end
    clearpoints(viz.path3d);
    clearpoints(viz.pathxy);
    clearpoints(viz.lx);
    clearpoints(viz.ly);
    clearpoints(viz.lz);
    clearpoints(viz.lz_meas);
    clearpoints(viz.lz_tgt);
end

function update_live_dashboard(viz, pos, tgt, eul_rad, yaw_deg, roll_deg, pitch_deg, last_cmd, t)
    addpoints(viz.path3d, pos(1), pos(2), pos(3));
    set(viz.drone3d, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
    set(viz.target3d, 'XData', tgt(1), 'YData', tgt(2), 'ZData', tgt(3));
    if isfield(viz, 'quad') && isvalid(viz.quad)
        T = makehgtform('translate', [pos(1), pos(2), pos(3)], ...
            'zrotate', eul_rad(3), 'yrotate', eul_rad(2), 'xrotate', eul_rad(1));
        set(viz.quad, 'Matrix', T);
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
        'LiteWing 6-DOF Newton-Euler  |  %s  |  X=%.2f  Y=%.2f  Z=%.2f m  |  yaw=%.0f°  roll=%.1f  pitch=%.1f', ...
        last_cmd, pos(1), pos(2), pos(3), yaw_deg, roll_deg, pitch_deg), ...
        'Color', 'w', 'FontWeight', 'bold');

    drawnow limitrate;
end

function [rx_sock, tx_sock] = create_java_udp_sockets(rx_port)
    rx_sock = java.net.DatagramSocket();
    rx_sock.setReuseAddress(true);
    rx_sock.setSoTimeout(100);
    rx_sock.bind(java.net.InetSocketAddress(rx_port));

    tx_sock = java.net.DatagramSocket();
    tx_sock.setReuseAddress(true);
end

function pkt = java_udp_receive(rx_sock)
    pkt = "";
    buf = javaArray('byte', 4096);
    packet = java.net.DatagramPacket(buf, 4096);
    try
        rx_sock.receive(packet);
        n = packet.getLength();
        raw = packet.getData();
        pkt = native2unicode(typecast(int8(raw(1:n)), 'uint8'), 'UTF-8');
    catch
        % SocketTimeoutException = no datagram this tick
    end
end

function java_udp_send(tx_sock, json_str, host, port)
    addr = java.net.InetAddress.getByName(host);
    payload = int8(unicode2native(char(json_str), 'UTF-8'));
    packet = java.net.DatagramPacket(payload, numel(payload), addr, port);
    tx_sock.send(packet);
end

function close_bridge_sockets(u_rx, u_tx, use_modern_udpport, use_classic_udp, use_java_udp)
    try
        if use_modern_udpport
            if ~isempty(u_rx), delete(u_rx); end
            if ~isempty(u_tx), delete(u_tx); end
        elseif use_classic_udp
            if ~isempty(u_rx), fclose(u_rx); delete(u_rx); end
            if ~isempty(u_tx), fclose(u_tx); delete(u_tx); end
        elseif use_java_udp
            if ~isempty(u_rx), u_rx.close(); end
            if ~isempty(u_tx), u_tx.close(); end
        end
    catch
    end
end
