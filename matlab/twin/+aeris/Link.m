classdef Link < matlab.System
    % LINK  Command and telemetry link between the twin and the Python stack.
    %
    %   UDP in  (T.net.cmd_port)   JSON skill commands
    %   UDP out (T.net.telem_port) JSON telemetry at the link rate
    %
    % Commands: TAKEOFF{z} MOVE_TO{x,y,z,velocity,yaw} HOVER ROTATE{angle}
    %           LAND EMERGENCY HOME SET_OBJECT{name,x,y,z} EXT_POSE{x,y,yaw}
    %           RESTART STOP
    % The setpoints go to the flight controller; nothing here touches
    % motors. Telemetry reports the estimator's state (what the vehicle
    % knows) at top level and the plant's true state under "truth", which
    % the Python side uses for scoring only.

    properties (Nontunable)
        T = struct()        % full twin parameter struct
        Ts = 0.02
    end

    properties (Access = private)
        rx
        tx
        mode
        target
        yawT
        vmax
        seq
        objPos
        ext
        extAge
        lastCmd
        nRx
    end

    methods (Access = protected)
        function setupImpl(obj)
            n = obj.T.net;
            aeris.Link.releasePorts();
            obj.rx = udpport("datagram", "IPV4", "LocalPort", n.cmd_port, "Tag", "AerisTwinRx");
            obj.tx = udpport("datagram", "IPV4", "Tag", "AerisTwinTx");
            % The default 512-byte datagram would split a telemetry packet.
            obj.tx.OutputDatagramSize = 8192;
            obj.resetStates();
            aeris.writeRuntime(obj.T);
        end

        function resetImpl(obj)
            if ~isempty(obj.mode), obj.resetStates(); end
        end

        function releaseImpl(obj)
            try, delete(obj.rx); catch, end
            try, delete(obj.tx); catch, end
        end

        function [cmd, objOut, extOut] = stepImpl(obj, tele)
            % tele = [p(3) v(3) q(4) wb(3) | p_est(3) v_est(3) q_est(4) |
            %         status(8) | w_rotor(4) | battery(3) | range | t]
            pT = tele(1:3); vT = tele(4:6); qT = tele(7:10);
            pE = tele(14:16); vE = tele(17:19); qE = tele(20:23);
            st = tele(24:31); wr = tele(32:35); bat = tele(36:38);
            range = tele(39); t = tele(40);

            % ---- Commands ------------------------------------------------
            while obj.rx.NumDatagramsAvailable > 0
                d = read(obj.rx, 1, "string");
                try
                    c = jsondecode(char(d.Data));
                    obj.handle(c, pE, st);
                    obj.nRx = obj.nRx + 1;
                catch err
                    fprintf('[twin] bad command packet: %s\n', err.message);
                end
            end

            obj.extAge = obj.extAge + obj.Ts;
            if obj.extAge > 0.5, obj.ext(1) = 0; end       % external fix goes stale

            cmd = [obj.mode; obj.target; obj.yawT; obj.vmax; obj.seq; 0];
            objOut = obj.objPos;
            extOut = obj.ext;

            % ---- Telemetry -----------------------------------------------
            eE = rad2deg(aeris.q2eul(qE(:)));
            eT = rad2deg(aeris.q2eul(qT(:)));
            m.t = t;
            m.x = pE(1); m.y = pE(2); m.z = pE(3);
            m.vx = vE(1); m.vy = vE(2); m.vz = vE(3);
            m.roll = eE(1); m.pitch = eE(2); m.yaw = eE(3);
            m.armed = st(1) > 0.5;
            m.landed = st(2) > 0.5;
            m.mode = obj.mode;
            m.seq = obj.seq;
            m.last_cmd = obj.lastCmd;
            m.ref = st(3:6).';
            m.target = [obj.target.' obj.yawT];
            m.saturated = st(8) > 0.5;
            m.battery_v = bat(1); m.current_a = bat(2); m.battery_pct = 100 * bat(3);
            m.motors = wr(:).';
            m.range = range;
            m.truth = struct('x', pT(1), 'y', pT(2), 'z', pT(3), ...
                'vx', vT(1), 'vy', vT(2), 'vz', vT(3), ...
                'roll', eT(1), 'pitch', eT(2), 'yaw', eT(3));
            mov = obj.T.movable;
            gt = struct();
            for i = 1:numel(mov)
                gt.(obj.T.scene.objects(mov(i)).name) = obj.objPos(i, :);
            end
            m.objects_truth = gt;
            try
                write(obj.tx, jsonencode(m), "string", obj.T.net.host, obj.T.net.telem_port);
            catch
            end
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end

        function n = getNumInputsImpl(~), n = 1; end
        function n = getNumOutputsImpl(~), n = 3; end
        function [a, b, c] = getOutputSizeImpl(obj)
            n = 1;
            if isfield(obj.T, 'movable'), n = max(numel(obj.T.movable), 1); end
            a = [8 1]; b = [n 3]; c = [4 1];
        end
        function [a, b, c] = getOutputDataTypeImpl(~), a = 'double'; b = 'double'; c = 'double'; end
        function [a, b, c] = isOutputComplexImpl(~), a = false; b = false; c = false; end
        function [a, b, c] = isOutputFixedSizeImpl(~), a = true; b = true; c = true; end
        function [n1, n2, n3] = getOutputNamesImpl(~), n1 = 'cmd'; n2 = 'objects'; n3 = 'ext'; end
    end

    methods (Access = private)
        function resetStates(obj)
            home = [obj.T.scene.home.pos(:); obj.T.z_rest];
            obj.mode = 0;
            obj.target = home;
            obj.yawT = deg2rad(obj.T.scene.home.yaw_deg);
            obj.vmax = 0;
            obj.seq = 0;
            mov = obj.T.movable;
            obj.objPos = zeros(max(numel(mov), 1), 3);
            for i = 1:numel(mov)
                obj.objPos(i, :) = obj.T.scene.objects(mov(i)).pos;
            end
            obj.ext = zeros(4, 1);
            obj.extAge = inf;
            obj.lastCmd = 'IDLE';
            obj.nRx = 0;
        end

        function handle(obj, c, pE, st)
            name = upper(char(c.cmd));
            ws = obj.T.scene.workspace;
            flying = st(1) > 0.5;
            switch name
                case 'TAKEOFF'
                    z = getf(c, 'z', 1.0);
                    if flying
                        obj.target(3) = clampTo(z, ws.z);
                    else
                        obj.target = [pE(1); pE(2); clampTo(z, ws.z)];
                    end
                    obj.vmax = getf(c, 'velocity', 0);
                    obj.mode = 1; obj.seq = obj.seq + 1;
                case 'MOVE_TO'
                    obj.target = [clampTo(getf(c, 'x', obj.target(1)), ws.x); ...
                                  clampTo(getf(c, 'y', obj.target(2)), ws.y); ...
                                  clampTo(getf(c, 'z', obj.target(3)), ws.z)];
                    obj.vmax = getf(c, 'velocity', 0);
                    if isfield(c, 'yaw') && ~isempty(c.yaw), obj.yawT = deg2rad(c.yaw); end
                    obj.mode = 1; obj.seq = obj.seq + 1;
                case 'HOVER'
                    if flying, obj.target = st(3:5); obj.target = obj.target(:); end
                case 'ROTATE'
                    obj.yawT = wrapAngle(obj.yawT + deg2rad(getf(c, 'angle', 0)));
                case 'SET_YAW'
                    obj.yawT = wrapAngle(deg2rad(getf(c, 'yaw', 0)));
                case 'LAND'
                    if flying, obj.mode = 2; end
                case 'EMERGENCY'
                    obj.mode = 3;
                case 'HOME'
                    home = obj.T.scene.home.pos(:);
                    if flying
                        obj.target = [home; max(obj.target(3), ws.z(1) + 0.3)];
                        obj.mode = 1; obj.seq = obj.seq + 1;
                    end
                case 'SET_OBJECT'
                    mov = obj.T.movable;
                    for i = 1:numel(mov)
                        if strcmpi(obj.T.scene.objects(mov(i)).name, char(c.name))
                            cur = obj.objPos(i, :);
                            obj.objPos(i, :) = [getf(c, 'x', cur(1)) getf(c, 'y', cur(2)) getf(c, 'z', cur(3))];
                        end
                    end
                case 'EXT_POSE'
                    obj.ext = [1; getf(c, 'x', pE(1)); getf(c, 'y', pE(2)); deg2rad(getf(c, 'yaw', 1e6))];     % no yaw key: position-only fix
                    obj.extAge = 0;
                case 'RESTART'
                    setappdata(0, 'AerisTwinRestart', true);
                    set_param(bdroot(gcs), 'SimulationCommand', 'stop');
                case 'STOP'
                    setappdata(0, 'AerisTwinRestart', false);
                    set_param(bdroot(gcs), 'SimulationCommand', 'stop');
                case {'CLEAR_PATH', 'PING'}
                    % Nothing to do in the plant.
                otherwise
                    fprintf('[twin] unknown command %s\n', name);
                    return;
            end
            obj.lastCmd = name;
        end
    end

    methods (Static)
        function releasePorts()
            % A previous run that ended abnormally can leave the ports bound.
            try
                old = [udpportfind(Tag="AerisTwinRx"), udpportfind(Tag="AerisTwinTx")];
                if ~isempty(old), delete(old); end
            catch
            end
        end
    end
end

function v = getf(c, name, default)
if isfield(c, name) && ~isempty(c.(name)) && isnumeric(c.(name))
    v = double(c.(name));
else
    v = default;
end
end

function v = clampTo(v, lim)
v = max(lim(1), min(lim(2), v));
end

function a = wrapAngle(a)
a = atan2(sin(a), cos(a));
end
