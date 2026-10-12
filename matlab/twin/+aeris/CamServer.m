classdef CamServer < matlab.System
    % CAMSERVER  Streams the rendered camera frames to Python over TCP.
    %
    % One client at a time on T.net.cam_port. Each frame is sent as
    %   'AERS' | stream id (uint8) | format (uint8) | 2 pad | width (uint16)
    %   | height (uint16) | sim time (float32) | payload, row-major
    % format 0: RGB, 3 bytes per pixel. format 1: depth, uint16 millimetres.
    % Stream ids follow the feed order in aeris.cameraSpecs. Frames are
    % dropped, not queued, when no client is connected.

    properties (Nontunable)
        T = struct()
        Ts = 0.08
    end

    properties (Access = private)
        srv
        sent
    end

    methods (Access = protected)
        function setupImpl(obj)
            try
                old = tcpserverfind(Tag="AerisTwinCam");
                if ~isempty(old), delete(old); end
            catch
            end
            obj.srv = tcpserver(obj.T.net.host, obj.T.net.cam_port, "Tag", "AerisTwinCam", "Timeout", 1);
            obj.sent = 0;
        end

        function releaseImpl(obj)
            try, delete(obj.srv); catch, end
        end

        function stepImpl(obj, t, varargin)
            if isempty(obj.srv) || ~obj.srv.Connected, return; end
            try
                for id = 1:numel(varargin)
                    img = varargin{id};
                    [h, w, ch] = size(img);
                    if ch == 3
                        fmt = 0;
                        payload = reshape(permute(img, [3 2 1]), 1, []);
                    else
                        fmt = 1;
                        mm = uint16(min(double(img) * 1000, 65535));
                        payload = typecast(reshape(mm.', 1, []), 'uint8');
                    end
                    header = [uint8('AERS'), uint8([id fmt 0 0]), ...
                        typecast(uint16([w h]), 'uint8'), typecast(single(t), 'uint8')];
                    write(obj.srv, [header, payload], "uint8");
                end
                obj.sent = obj.sent + 1;
            catch err
                fprintf('[twin] camera stream: %s\n', err.message);
            end
        end

        function sts = getSampleTimeImpl(obj)
            sts = createSampleTime(obj, 'Type', 'Discrete', 'SampleTime', obj.Ts);
        end

        function n = getNumInputsImpl(obj)
            % The block queries its port count before the mask value of T
            % has been applied, so tolerate the empty default.
            n = 1;
            if isfield(obj.T, 'scene')
                [~, feeds] = aeris.cameraSpecs(obj.T);
                n = 1 + numel(feeds);
            end
        end
        function n = getNumOutputsImpl(~), n = 0; end
    end
end
