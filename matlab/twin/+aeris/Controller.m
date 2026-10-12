classdef Controller < matlab.System
    % CONTROLLER  Cascaded multicopter flight controller.
    %
    %   setpoint shaping -> position P -> velocity PI -> thrust vector
    %   -> attitude (quaternion) P -> body-rate PID -> mixer -> motor commands
    %
    % Same structure as the Crazyflie / PX4 multicopter cascade, so what is
    % tuned here maps onto firmware gains. Gains come from the bandwidths in
    % the airframe file; nothing is tuned against a particular scene.
    %
    % cmd = [mode x y z yaw v_max seq 0]
    %   mode 0 disarmed, 1 fly to the setpoint, 2 land, 3 motor kill
    %   seq increments on every new command, which re-arms after a landing
    % status = [armed landed ref_x ref_y ref_z ref_yaw thrust_cmd saturated]
    %#codegen

    properties (Nontunable)
        P = struct()
        Ts = 0.002
    end

    properties (Access = private)
        armed
        landed
        lastSeq
        refP
        refV
        refYaw
        intV
        intW
        wFilt
        wPrev
        groundTimer
        tumbleTimer
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.resetStates();
        end

        function resetImpl(obj)
            obj.resetStates();
        end

        function [u, status] = stepImpl(obj, p, v, q, w, cmd, vbat)
            C = obj.P.ctl; air = obj.P.air; dt = obj.Ts; g = obj.P.g;
            p = p(:); v = v(:); q = q(:); w = w(:);
            mode = cmd(1); tgt = cmd(2:4); tgt = tgt(:); yawT = cmd(5);
            vmaxCmd = cmd(6); seq = cmd(7);
            R = aeris.q2rotm(q);
            eul = aeris.q2eul(q);

            % ---- Arming state machine -----------------------------------
            if mode == 3 || mode == 0
                obj.armed = false;
                if mode == 0, obj.landed = true; end
            elseif mode == 1 && ~obj.armed && seq ~= obj.lastSeq
                obj.armed = true; obj.landed = false;
                obj.refP = p; obj.refV = zeros(3, 1); obj.refYaw = eul(3);
                obj.intV = zeros(3, 1); obj.intW = zeros(3, 1);
                obj.wFilt = w; obj.wPrev = w; obj.groundTimer = 0; obj.tumbleTimer = 0;
            end
            obj.lastSeq = seq;

            % Tumble protection: cut the motors if the vehicle stays far
            % from upright (after a collision), as the firmware does.
            if obj.armed && R(3, 3) < cos(C.tumble_angle)
                obj.tumbleTimer = obj.tumbleTimer + dt;
                if obj.tumbleTimer > C.tumble_time
                    obj.armed = false; obj.landed = true;
                end
            else
                obj.tumbleTimer = 0;
            end

            if ~obj.armed
                u = zeros(4, 1);
                obj.refP = p; obj.refV = zeros(3, 1); obj.refYaw = eul(3);
                status = [0; double(obj.landed); obj.refP; obj.refYaw; 0; 0];
                return;
            end

            % ---- Setpoint shaping ---------------------------------------
            vmax = C.v_max(:);
            if vmaxCmd > 0
                vmax(1:2) = min(vmax(1:2), vmaxCmd);
                vmax(3) = min(vmax(3), max(vmaxCmd, 0.2));
            end
            if mode == 2
                tgt = [obj.refP(1:2); -0.5];
                vmax(3) = C.v_land;
            end
            e = tgt - obj.refP;
            dxy = norm(e(1:2));
            vDes = zeros(3, 1);
            if dxy > 1e-4
                spd = min([vmax(1), sqrt(1.6 * C.a_max * dxy), 4 * dxy]);
                vDes(1:2) = spd * e(1:2) / dxy;
            end
            vDes(3) = sign(e(3)) * min([vmax(3), sqrt(1.6 * C.a_max * abs(e(3))), 4 * abs(e(3))]);
            dv = vDes - obj.refV;
            refA = dv / dt;
            an = norm(refA);
            if an > C.a_max, refA = refA * (C.a_max / an); end
            obj.refV = obj.refV + refA * dt;
            obj.refP = obj.refP + obj.refV * dt;

            dyaw = atan2(sin(yawT - obj.refYaw), cos(yawT - obj.refYaw));
            stepYaw = max(-C.yaw_rate_max * dt, min(C.yaw_rate_max * dt, dyaw));
            obj.refYaw = obj.refYaw + stepYaw;
            yawRateFF = stepYaw / dt;

            % ---- Position and velocity loops ----------------------------
            vSp = obj.refV + C.kp_pos(:) .* (obj.refP - p);
            vSp = max(-1.3 * vmax, min(1.3 * vmax, vSp));
            eV = vSp - v;
            aSp = refA + C.kp_vel(:) .* eV + obj.intV;
            aSp(3) = max(-0.6 * g, min(aSp(3), 1.5 * g));
            aXYmax = (aSp(3) + g) * tan(C.tilt_max);
            aXY = norm(aSp(1:2));
            satXY = aXY > aXYmax;
            if satXY, aSp(1:2) = aSp(1:2) * (aXYmax / aXY); end

            F = air.m * (aSp + [0; 0; g]);
            Tcmd = F.' * R(:, 3);
            satT = Tcmd > C.thrust_max || Tcmd < C.thrust_min;
            Tcmd = max(C.thrust_min, min(C.thrust_max, Tcmd));

            % Integrate only where the actuator has authority left.
            iGain = C.ki_vel(:) * dt;
            if ~satXY, obj.intV(1:2) = obj.intV(1:2) + iGain(1:2) .* eV(1:2); end
            if ~satT, obj.intV(3) = obj.intV(3) + iGain(3) * eV(3); end
            obj.intV = max(-0.4 * g, min(0.4 * g, obj.intV));

            % ---- Attitude loop ------------------------------------------
            zb = F / max(norm(F), 1e-6);
            xc = [cos(obj.refYaw); sin(obj.refYaw); 0];
            yb = cross(zb, xc); yb = yb / max(norm(yb), 1e-6);
            xb = cross(yb, zb);
            qDes = aeris.rotm2q([xb yb zb]);
            qErr = aeris.qmul([q(1); -q(2:4)], qDes);
            if qErr(1) < 0, qErr = -qErr; end
            wSp = 2 * C.kp_att(:) .* qErr(2:4);
            wSp(3) = wSp(3) + yawRateFF * R(3, 3);
            wSp(3) = max(-C.yaw_rate_max, min(C.yaw_rate_max, wSp(3)));

            % ---- Body-rate loop -----------------------------------------
            alpha = dt * 2 * pi * C.rate_lpf_hz;
            alpha = alpha / (1 + alpha);
            obj.wFilt = obj.wFilt + alpha * (w - obj.wFilt);
            eW = wSp - obj.wFilt;
            wDot = (obj.wFilt - obj.wPrev) / dt;
            obj.wPrev = obj.wFilt;
            acc = C.kp_rate(:) .* eW + obj.intW - C.kd_rate(:) .* wDot;
            I = air.I(:);
            tau = I .* acc + cross(w, I .* w);

            % ---- Mixer with yaw-first desaturation ----------------------
            base = air.alloc_inv * [Tcmd; tau(1); tau(2); 0];
            dyawT = air.alloc_inv(:, 4) * tau(3);
            kYaw = 1;
            for i = 1:4
                if dyawT(i) > 1e-12
                    kYaw = min(kYaw, max(0, (air.T_max - base(i)) / dyawT(i)));
                elseif dyawT(i) < -1e-12
                    kYaw = min(kYaw, max(0, (0 - base(i)) / dyawT(i)));
                end
            end
            Tm = base + kYaw * dyawT;
            satM = kYaw < 1 || any(Tm < 0) || any(Tm > air.T_max);
            Tm = max(0, min(air.T_max, Tm));
            if ~satM
                obj.intW = obj.intW + C.ki_rate(:) .* eW * dt;
                lim = 0.3 * C.kp_rate(:) * 3;
                obj.intW = max(-lim, min(lim, obj.intW));
            end

            % Thrust -> rotor speed -> duty, compensated for pack voltage.
            wCmd = sqrt(Tm / air.kT);
            u = wCmd / (air.w_max * max(vbat, 0.5 * obj.P.bat.v_nom) / obj.P.bat.v_nom);
            u = max(0.05, min(1, u));

            % ---- Touchdown detection while landing ----------------------
            if mode == 2
                % Down and stopped: either at floor height, or stalled on a
                % surface while the descent reference keeps going (a landing
                % on a table or box).
                stalled = obj.refP(3) < p(3) - 0.12 && abs(v(3)) < 0.06;
                if (p(3) < obj.P.z_rest + 0.03 && abs(v(3)) < 0.15) || stalled
                    obj.groundTimer = obj.groundTimer + dt;
                else
                    obj.groundTimer = 0;
                end
                if obj.groundTimer > 0.4
                    obj.armed = false; obj.landed = true;
                    u = zeros(4, 1);
                end
            else
                obj.groundTimer = 0;
            end

            status = [double(obj.armed); double(obj.landed); obj.refP; obj.refYaw; Tcmd; double(satM || satT || satXY)];
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end

        function n = getNumInputsImpl(~), n = 6; end
        function n = getNumOutputsImpl(~), n = 2; end
        function [a, b] = getOutputSizeImpl(~), a = [4 1]; b = [8 1]; end
        function [a, b] = getOutputDataTypeImpl(~), a = 'double'; b = 'double'; end
        function [a, b] = isOutputComplexImpl(~), a = false; b = false; end
        function [a, b] = isOutputFixedSizeImpl(~), a = true; b = true; end
        function [n1, n2, n3, n4, n5, n6] = getInputNamesImpl(~)
            n1 = 'p'; n2 = 'v'; n3 = 'q'; n4 = 'w'; n5 = 'cmd'; n6 = 'vbat';
        end
        function [n1, n2] = getOutputNamesImpl(~), n1 = 'u'; n2 = 'status'; end
    end

    methods (Access = private)
        function resetStates(obj)
            obj.armed = false;
            obj.landed = true;
            obj.lastSeq = -1;
            obj.refP = zeros(3, 1);
            obj.refV = zeros(3, 1);
            obj.refYaw = 0;
            obj.intV = zeros(3, 1);
            obj.intW = zeros(3, 1);
            obj.wFilt = zeros(3, 1);
            obj.wPrev = zeros(3, 1);
            obj.groundTimer = 0;
            obj.tumbleTimer = 0;
        end
    end
end
