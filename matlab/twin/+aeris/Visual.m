classdef Visual < matlab.System
    % VISUAL  Converts plant state into Unreal actor transforms.
    %   Rotors are drawn at a slowed rate (proportional to true speed) so
    %   the blades stay visible at the render frame rate.

    properties (Nontunable)
        P = struct()
        Ts = 0.04
    end

    properties (Access = private)
        ang
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.ang = zeros(4, 1);
        end

        function resetImpl(obj)
            obj.ang = zeros(4, 1);
        end

        function [trans, rot, r1, r2, r3, r4] = stepImpl(obj, p, q, w)
            e = aeris.q2eul(q(:));
            trans = p(:).';
            rot = e(:).';
            visRate = 25 * (w(:) / obj.P.air.w_max) .* obj.P.air.spin(:);   % rad/s on screen
            obj.ang = mod(obj.ang + visRate * obj.Ts, 2 * pi);
            r1 = [0 0 obj.ang(1)]; r2 = [0 0 obj.ang(2)];
            r3 = [0 0 obj.ang(3)]; r4 = [0 0 obj.ang(4)];
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end

        function n = getNumInputsImpl(~), n = 3; end
        function n = getNumOutputsImpl(~), n = 6; end
        function varargout = getOutputSizeImpl(~), [varargout{1:6}] = deal([1 3]); end
        function varargout = getOutputDataTypeImpl(~), [varargout{1:6}] = deal('double'); end
        function varargout = isOutputComplexImpl(~), [varargout{1:6}] = deal(false); end
        function varargout = isOutputFixedSizeImpl(~), [varargout{1:6}] = deal(true); end
    end
end
