classdef Propulsion < matlab.System
    % PROPULSION  Motors, rotor aerodynamics, wind and battery for the twin.
    %
    % Runs at the physics rate. Takes the four motor commands (0..1) and
    % the true vehicle state from the Simscape plant, returns the wrench
    % the plant applies:
    %   thrust  per-rotor thrust along body +z, applied at each hub [N]
    %   Fb      body-frame force from rotor and parasitic drag [N]
    %   Mb      body-frame torque: rotor reaction, spin-up and gyroscopic [N m]
    %
    % Rotors are not spun as physical bodies in Simscape (at ~11 000 rpm
    % that forces a sub-0.1 ms step). Their angular momentum enters
    % analytically instead, which is the same simplification RotorS and
    % the PX4 Gazebo plugins make.
    %#codegen

    properties (Nontunable)
        P = struct()        % numeric twin parameters (T.core from aeris.loadParams)
        Ts = 0.001
    end

    properties (Access = private)
        w           % rotor speeds [rad/s]
        soc
        vbat
        gust        % coloured-noise wind state, world frame [m/s]
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.w = zeros(4, 1);
            obj.soc = obj.P.bat.soc0;
            obj.vbat = interp1(obj.P.bat.soc_pts, obj.P.bat.ocv_pts, obj.P.bat.soc0);
            obj.gust = zeros(3, 1);
        end

        function resetImpl(obj)
            obj.w = zeros(4, 1);
            obj.soc = obj.P.bat.soc0;
            obj.vbat = interp1(obj.P.bat.soc_pts, obj.P.bat.ocv_pts, obj.P.bat.soc0);
            obj.gust = zeros(3, 1);
        end

        function [thrust, Fb, Mb, wr, bat] = stepImpl(obj, u, v, q, wb, z)
            air = obj.P.air; b = obj.P.bat; dt = obj.Ts;
            u = min(max(u(:), 0), 1);

            % Brushed/BLDC drive: speed follows duty and sags with the pack.
            w_cmd = air.w_max * u * (obj.vbat / b.v_nom);
            w_cmd = min(w_cmd, 1.15 * air.w_max);
            tau = air.tau_dn * ones(4, 1);
            tau(w_cmd > obj.w) = air.tau_up;
            w_dot = (w_cmd - obj.w) ./ tau;
            obj.w = max(obj.w + w_dot * dt, 0);
            w = obj.w;

            % Ground effect (Cheeseman-Bennett), bounded close to the floor.
            ge = 1;
            if air.ground_effect > 0
                h = max(z, 0.5 * air.R);
                ge = 1 / (1 - min((air.R / (4 * h))^2, 0.30));
            end
            thrust = air.kT * w.^2 * ge;

            % Air-relative velocity in the body frame.
            wind = obj.P.wind;
            if wind.sigma > 0
                a = dt / wind.tau;
                obj.gust = obj.gust * (1 - a) + wind.sigma * sqrt(2 * a) * randn(3, 1);
            end
            R = aeris.q2rotm(q(:));
            v_air = R.' * (v(:) - (wind.mean(:) + obj.gust));

            % Rotor H-force (linear in airspeed and total rotor speed) plus
            % quadratic parasitic drag of the frame.
            Fb = -air.k_h * sum(w) * [v_air(1); v_air(2); 0] ...
                 - 0.5 * obj.P.rho * air.cdA * norm(v_air) * v_air;

            % Yaw reaction, spin-up torque and gyroscopic coupling.
            s = air.spin(:);
            Q = air.kQ * w.^2;
            h_rot = air.Jr * sum(s .* w);
            Mb = [0; 0; -sum(s .* (Q + air.Jr * w_dot))] - cross(wb(:), [0; 0; h_rot]);

            % Battery: mechanical power through the drive efficiency.
            i_bat = sum(Q .* w) / (b.eta * max(obj.vbat, 0.5 * b.v_nom)) + b.i_avionics;
            obj.soc = max(obj.soc - i_bat * dt / b.capacity_As, 0);
            ocv = interp1(b.soc_pts, b.ocv_pts, obj.soc);
            obj.vbat = max(ocv - i_bat * b.R, 0.6 * b.v_nom);

            wr = w;
            bat = [obj.vbat; i_bat; obj.soc];
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end

        function n = getNumInputsImpl(~), n = 5; end
        function n = getNumOutputsImpl(~), n = 5; end
        function [a, b, c, d, e] = getOutputSizeImpl(~)
            a = [4 1]; b = [3 1]; c = [3 1]; d = [4 1]; e = [3 1];
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
        function [n1, n2, n3, n4, n5] = getInputNamesImpl(~)
            n1 = 'u'; n2 = 'v'; n3 = 'q'; n4 = 'wb'; n5 = 'z';
        end
        function [n1, n2, n3, n4, n5] = getOutputNamesImpl(~)
            n1 = 'thrust'; n2 = 'Fb'; n3 = 'Mb'; n4 = 'w_rotor'; n5 = 'battery';
        end
    end
end
