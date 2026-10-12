classdef MeshBuilder < handle
    % Accumulates primitives into one triangle mesh so each scene object
    % reaches Unreal as a handful of actors instead of dozens.
    %
    % All primitives are placed with a local pose (translation, ZYX Euler in
    % degrees) and carry a per-vertex colour. Units are metres, z up.

    properties
        V = zeros(0, 3)
        N = zeros(0, 3)
        F = zeros(0, 3)
        T = zeros(0, 2)
        C = zeros(0, 3)
    end

    methods
        function box(obj, sz, pos, col, rpy, uvScale)
            if nargin < 5 || isempty(rpy), rpy = [0 0 0]; end
            if nargin < 6, uvScale = 1; end
            [v, n, f, t] = sim3d.utils.Geometry.box(sz);
            obj.append(v, n, f, t * uvScale, pos, rpy, col);
        end

        function cyl(obj, diameter, height, pos, col, rpy, segments)
            if nargin < 6 || isempty(rpy), rpy = [0 0 0]; end
            if nargin < 7, segments = 24; end
            [v, n, f, t] = sim3d.utils.Geometry.cylinder([diameter diameter height], segments);
            obj.append(v, n, f, t, pos, rpy, col);
        end

        function cone(obj, diameter, height, pos, col, rpy, segments)
            if nargin < 6 || isempty(rpy), rpy = [0 0 0]; end
            if nargin < 7, segments = 24; end
            [v, n, f, t] = sim3d.utils.Geometry.cone([diameter diameter height], segments);
            obj.append(v, n, f, t, pos, rpy, col);
        end

        function sphere(obj, diameter, pos, col, squash)
            if nargin < 5, squash = [1 1 1]; end
            [v, n, f, t] = sim3d.utils.Geometry.icosphere(diameter * [1 1 1], 2);
            v = v .* squash;
            n = n ./ squash;
            n = n ./ max(vecnorm(n, 2, 2), 1e-9);
            obj.append(v, n, f, t, pos, [0 0 0], col);
        end

        function revolve(obj, profileZR, pos, col, rpy, segments)
            % profileZR: rows of [z radius], bottom to top.
            if nargin < 5 || isempty(rpy), rpy = [0 0 0]; end
            if nargin < 6, segments = 28; end
            [v, n, f, t] = sim3d.utils.Geometry.revolution(profileZR, segments, true, true);
            obj.append(v, n, f, t, pos, rpy, col);
        end

        function quad(obj, p1, p2, p3, p4, col, uv)
            % Single-sided quad, counter-clockwise seen from its front.
            if nargin < 7, uv = [0 0; 1 0; 1 1; 0 1]; end
            v = [p1; p2; p3; p4];
            nrm = cross(p2 - p1, p4 - p1);
            nrm = nrm / max(norm(nrm), 1e-9);
            % sim3d faces are zero-based and wound clockwise about the normal.
            obj.append(v, repmat(nrm, 4, 1), [0 3 2; 0 2 1], uv, [0 0 0], [0 0 0], col);
        end

        function append(obj, v, n, f, t, pos, rpy, col)
            R = aeris.MeshBuilder.rotm(rpy);
            v = v * R.' + pos(:).';
            n = n * R.';
            base = size(obj.V, 1);
            obj.V = [obj.V; v];
            obj.N = [obj.N; n];
            obj.F = [obj.F; f + base];
            if isempty(t), t = zeros(size(v, 1), 2); end
            obj.T = [obj.T; t];
            obj.C = [obj.C; repmat(col(:).', size(v, 1), 1)];
        end

        function actor = emit(obj, world, parent, name, opts)
            % opts: Texture, Color, Metallic, Shininess, Specular,
            %       Transparency, Shadows, Translation, Rotation (deg), Movable
            arguments
                obj
                world
                parent
                name
                opts.Texture = ''
                opts.Color = []
                opts.Metallic = 0.0
                opts.Shininess = 0.25
                opts.Specular = 0.3
                opts.Transparency = 0
                opts.Shadows = true
                opts.Translation = [0 0 0]
                opts.Rotation = [0 0 0]
                opts.Movable = false
                opts.TwoSided = false
            end
            actor = sim3d.Actor(ActorName=name, CoordinateSystem='MATLAB');
            obj.fill(actor, opts);
            if isempty(parent)
                add(world, actor);
            else
                add(world, actor, parent);
            end
        end
    end

    methods

        function fill(obj, actor, opts)
            % Put the accumulated mesh and material on an existing actor.
            if nargin < 3, opts = struct(); end
            d = struct('Texture', '', 'Color', [], 'Metallic', 0.0, 'Shininess', 0.25, ...
                'Specular', 0.3, 'Transparency', 0, 'Shadows', true, 'Translation', [0 0 0], ...
                'Rotation', [0 0 0], 'Movable', false, 'TwoSided', false);
            f = fieldnames(d);
            for i = 1:numel(f)
                if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
            end
            actor.CoordinateSystem = 'MATLAB';
            if opts.Movable
                actor.Mobility = sim3d.utils.MobilityTypes.Movable;
            end
            createMesh(actor, obj.V, obj.N, obj.F, obj.T, obj.C);
            if ~isempty(opts.Texture)
                actor.Texture = opts.Texture;
                actor.Color = [1 1 1];
                actor.VertexBlend = 0;
            elseif ~isempty(opts.Color)
                actor.Color = opts.Color;
                actor.VertexBlend = 0;
            else
                actor.Color = [1 1 1];
                actor.VertexBlend = 1;
            end
            actor.Metallic = opts.Metallic;
            actor.Shininess = opts.Shininess;
            actor.Specular = opts.Specular;
            actor.Transparency = opts.Transparency;
            actor.Shadows = opts.Shadows;
            actor.TwoSided = opts.TwoSided;
            actor.Collisions = false;
            actor.Translation = opts.Translation;
            actor.Rotation = deg2rad(opts.Rotation);
        end
    end

    methods (Static)
        function R = rotm(rpyDeg)
            r = deg2rad(rpyDeg(1)); p = deg2rad(rpyDeg(2)); y = deg2rad(rpyDeg(3));
            Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
            Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
            Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
            R = Rz * Ry * Rx;
        end
    end
end
