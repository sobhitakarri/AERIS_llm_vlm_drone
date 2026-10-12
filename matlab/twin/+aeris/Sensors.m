classdef Sensors < matlab.System
    % SENSORS  Onboard sensor models and the state estimator.
    %
    % Truth from the Simscape plant is turned into what the airframe can
    % actually measure, then fused the way the firmware would:
    %   IMU      gyro + accelerometer with white noise, turn-on bias, bias walk
    %   Range    downward time-of-flight to whatever surface is below
    %            (floor, table top, box), tilt-dependent, range-limited
    %   Flow     optical-flow velocity in the body frame, noise grows with height
    % Estimator: Mahony attitude filter, fixed-gain observers for height and
    % horizontal velocity, dead-reckoned horizontal position. Horizontal
    % position and yaw therefore drift, exactly as on the real vehicle,
    % unless an external fix (overhead camera) is supplied on `ext`.
    %
    % P.est.mode = 0 bypasses all of this and passes truth through.
    %#codegen

    properties (Nontunable)
        P = struct()
        Ts = 0.001
    end

    properties (Access = private)
        q           % attitude estimate, body -> world
        bg          % gyro bias estimate
        p
        v
        gyroBias
        ba          % horizontal acceleration correction, world frame
        terrain     % estimated height of the surface under the range sensor
        accelBias
        k
        range
        init
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.resetStates();
        end

        function resetImpl(obj)
            obj.resetStates();
        end

        function [pe, ve, qe, we, aux] = stepImpl(obj, p, v, a, q, wb, ext)
            S = obj.P.sens; E = obj.P.est; dt = obj.Ts; g = obj.P.g;
            p = p(:); v = v(:); a = a(:); q = q(:); wb = wb(:);
            R = aeris.q2rotm(q);

            if ~obj.init
                obj.q = q; obj.p = p; obj.v = v; obj.init = true;
            end
            obj.k = obj.k + 1;

            % ---- Sensor models ------------------------------------------
            obj.gyroBias = obj.gyroBias + S.gyro_walk * sqrt(dt) * randn(3, 1);
            gyro = wb + obj.gyroBias + S.gyro_sigma * randn(3, 1);
            f_b = R.' * (a + [0; 0; g]);                       % specific force
            acc = f_b + obj.accelBias + S.accel_sigma * randn(3, 1);

            % Surface under the vehicle: the tallest footprint it is over.
            surf = 0;
            ter = obj.P.terrain;
            for i = 1:size(ter, 1)
                c = cos(ter(i, 5)); s = sin(ter(i, 5));
                dx = p(1) - ter(i, 1); dy = p(2) - ter(i, 2);
                lx = c * dx + s * dy; ly = -s * dx + c * dy;
                if abs(lx) <= ter(i, 3) && abs(ly) <= ter(i, 4) && ter(i, 6) < p(3) + 0.02
                    surf = max(surf, ter(i, 6));
                end
            end
            clearance = max(p(3) - obj.P.z_rest - surf, 0) + obj.P.z_rest;

            newRange = false;
            if mod(obj.k, S.tof_div) == 0
                cosTilt = R(3, 3);
                if cosTilt > 0.5
                    r = clearance / cosTilt + S.tof_sigma * randn();
                    if r > 0 && r < S.tof_max
                        obj.range = r; newRange = true;
                    end
                end
            end
            newFlow = false; flow = zeros(2, 1);
            if mod(obj.k, S.flow_div) == 0 && clearance > S.flow_min_h && R(3, 3) > 0.7
                vb = R.' * v;
                flow = vb(1:2) + S.flow_sigma * max(clearance, 0.1) * randn(2, 1);
                newFlow = true;
            end

            if E.mode == 0
                pe = p; ve = v; qe = q; we = wb;
                aux = [obj.range; acc; gyro];
                return;
            end

            % ---- Gyro bias calibration while sitting on the ground ---------
            % Fast at boot, when the airframe is known to be still. After
            % that only a slow trim, and only while the gyro agrees with the
            % current bias: as the rotors spin up the airframe creeps on its
            % feet, and tracking that rotation would be taken off the pad as
            % a yaw-rate bias (heading then drifts for the whole flight).
            onGround = obj.range < obj.P.z_rest + 0.015;
            if onGround
                if obj.k * dt < E.gyro_boot_s
                    if norm(gyro) < 0.2
                        obj.bg = obj.bg + (gyro - obj.bg) / obj.k;      % running mean of the boot window
                    end
                elseif norm(gyro - obj.bg) < E.gyro_still
                    obj.bg = obj.bg + E.gyro_trim_gain * (gyro - obj.bg);
                end
            end

            % ---- Attitude: Mahony complementary filter -------------------
            Re = aeris.q2rotm(obj.q);
            w_corr = gyro - obj.bg;
            an = norm(acc);
            if an > 0.5 * g && an < 1.5 * g
                e = cross(acc / an, Re(3, :).');               % measured vs predicted gravity
                % Gravity says nothing about heading. Letting this error
                % integrate into the yaw-axis bias makes yaw drift whenever
                % the airframe is tilted and accelerating, so the yaw-axis
                % bias is only ever set by the calibration on the ground.
                obj.bg(1:2) = obj.bg(1:2) - E.mahony_ki * e(1:2) * dt;
                w_corr = w_corr + E.mahony_kp * e;
            end
            obj.q = obj.q + 0.5 * aeris.qmul(obj.q, [0; w_corr]) * dt;
            obj.q = obj.q / norm(obj.q);
            Re = aeris.q2rotm(obj.q);

            % ---- Translation: accelerometer prediction + corrections ----
            a_w = Re * acc - [0; 0; g];
            a_w(1:2) = a_w(1:2) + obj.ba;
            if onGround
                % Range sensor says the airframe is sitting on its legs:
                % hold the translation states, as firmware does while landed.
                a_w = zeros(3, 1);
                obj.v = 0.9 * obj.v;
            end
            obj.p = obj.p + obj.v * dt + 0.5 * a_w * dt^2;
            obj.v = obj.v + a_w * dt;

            if newRange
                dtm = dt * S.tof_div;
                % Height = range to the surface below + height of that surface.
                innov = obj.range * Re(3, 3) + obj.terrain - obj.p(3);
                if abs(innov) > E.terrain_step
                    % The surface below changed (flew over a table edge):
                    % move the terrain estimate, not the vehicle.
                    obj.terrain = obj.terrain - innov;
                else
                    obj.p(3) = obj.p(3) + min(E.k_z(1) * dtm, 1) * innov;
                    obj.v(3) = obj.v(3) + E.k_z(2) * dtm * innov;
                end
            end
            if newFlow
                dtm = dt * S.flow_div;
                v_meas = Re(:, 1:2) * flow;                     % rotate body xy velocity to world
                innov = v_meas(1:2) - obj.v(1:2);
                obj.v(1:2) = obj.v(1:2) + min(E.k_flow * dtm, 1) * innov;
                % Integral term: absorbs the horizontal acceleration error
                % left by accelerometer bias, which would otherwise show up
                % as a steady velocity (and so position-rate) offset.
                obj.ba = obj.ba + E.k_flow_i * dtm * innov;
            end

            % ---- External fix (overhead camera): [valid x y yaw] --------
            % yaw beyond +-100 rad means "no heading in this fix".
            if ext(1) > 0.5
                obj.p(1:2) = obj.p(1:2) + min(E.k_ext(1) * dt, 1) * (ext(2:3) - obj.p(1:2));
                if abs(ext(4)) < 100                           % a fix may carry position only
                    eul = aeris.q2eul(obj.q);
                    dy = atan2(sin(ext(4) - eul(3)), cos(ext(4) - eul(3)));
                    half = 0.5 * min(E.k_ext(2) * dt, 1) * dy;
                    obj.q = aeris.qmul([cos(half); 0; 0; sin(half)], obj.q);
                end
            end

            pe = obj.p; ve = obj.v; qe = obj.q; we = gyro - obj.bg;
            aux = [obj.range; acc; gyro];
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end

        function n = getNumInputsImpl(~), n = 6; end
        function n = getNumOutputsImpl(~), n = 5; end
        function [a, b, c, d, e] = getOutputSizeImpl(~)
            a = [3 1]; b = [3 1]; c = [4 1]; d = [3 1]; e = [7 1];
        end
        function [a, b, c, d, e] = getOutputDataTypeImpl(~)
            a = 'double'; b = 'double'; c = 'double'; d = 'double'; e = 'double';
        end
        function [a, b, c, d, e] = isOutputComplexImpl(~)
            a = false; b = false; c = false; d = false; e = false;
        end
        function [a, b, c, d, e] = isOutputFixedSizeImpl(~)
            a = true; b = true; c = true; d = true; e = true;
        end
        function [n1, n2, n3, n4, n5, n6] = getInputNamesImpl(~)
            n1 = 'p'; n2 = 'v'; n3 = 'a'; n4 = 'q'; n5 = 'wb'; n6 = 'ext';
        end
        function [n1, n2, n3, n4, n5] = getOutputNamesImpl(~)
            n1 = 'p_est'; n2 = 'v_est'; n3 = 'q_est'; n4 = 'w_est'; n5 = 'raw';
        end
    end

    methods (Access = private)
        function resetStates(obj)
            S = obj.P.sens;
            obj.q = [1; 0; 0; 0];
            obj.bg = zeros(3, 1);
            obj.p = zeros(3, 1);
            obj.v = zeros(3, 1);
            obj.ba = zeros(2, 1);
            obj.terrain = 0;
            % Turn-on biases: fixed fractions of the datasheet bound so a
            % run is repeatable; the bias walk adds the stochastic part.
            obj.gyroBias = S.gyro_bias * [0.6; -0.4; 0.3];
            obj.accelBias = S.accel_bias * [0.5; -0.3; 0.4];
            obj.k = 0;
            obj.range = 0;
            obj.init = false;
        end
    end
end
