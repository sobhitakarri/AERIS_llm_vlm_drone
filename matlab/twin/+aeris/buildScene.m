function buildScene(world, root, T)
% BUILDSCENE  Populate the Unreal world from the scene description in T.
%   world : sim3d.World
%   root  : parent sim3d.Actor ([] when the world is driven from MATLAB)
%   T     : twin parameter struct from aeris.loadParams
%
% Every object in T.scene.objects becomes one actor named after the object
% (plus child actors for textured parts), so movable objects can be driven
% by name from Simulink.

tex = @(f) fullfile(T.assets.tex, f);
room = T.scene.room.size_m(:).';
hx = room(1) / 2; hy = room(2) / 2; hz = room(3);

% ---- Shell -------------------------------------------------------------
mb = aeris.MeshBuilder;
tile = 2.0;   % metres of floor covered by one texture repeat
mb.quad([-hx -hy 0], [hx -hy 0], [hx hy 0], [-hx hy 0], [1 1 1], ...
    [0 0; room(1)/tile 0; room(1)/tile room(2)/tile; 0 room(2)/tile]);
mb.emit(world, root, 'Floor', Texture=tex('floor_concrete.png'), Shininess=0.35, Specular=0.25, Shadows=false);

mb = aeris.MeshBuilder;
wt = 2.5;
mb.quad([-hx hy 0], [hx hy 0], [hx hy hz], [-hx hy hz], [1 1 1], [0 0; room(1)/wt 0; room(1)/wt hz/wt; 0 hz/wt]);
mb.quad([hx -hy 0], [-hx -hy 0], [-hx -hy hz], [hx -hy hz], [1 1 1], [0 0; room(1)/wt 0; room(1)/wt hz/wt; 0 hz/wt]);
mb.quad([hx hy 0], [hx -hy 0], [hx -hy hz], [hx hy hz], [1 1 1], [0 0; room(2)/wt 0; room(2)/wt hz/wt; 0 hz/wt]);
mb.quad([-hx -hy 0], [-hx hy 0], [-hx hy hz], [-hx -hy hz], [1 1 1], [0 0; room(2)/wt 0; room(2)/wt hz/wt; 0 hz/wt]);
mb.emit(world, root, 'Walls', Texture=tex('wall_plaster.png'), Shininess=0.08, Specular=0.1, Shadows=true, TwoSided=true);

mb = aeris.MeshBuilder;
mb.quad([-hx hy hz], [hx hy hz], [hx -hy hz], [-hx -hy hz], [1 1 1], [0 0; room(1)/1.2 0; room(1)/1.2 room(2)/1.2; 0 room(2)/1.2]);
mb.emit(world, root, 'Ceiling', Texture=tex('ceiling_tile.png'), Shininess=0.05, Shadows=true, TwoSided=true);

% Skirting, door, structural truss and the light fittings share one mesh.
mb = aeris.MeshBuilder;
skirt = [0.16 0.17 0.19];
mb.box([room(1) 0.02 0.10], [0  hy-0.01 0.05], skirt);
mb.box([room(1) 0.02 0.10], [0 -hy+0.01 0.05], skirt);
mb.box([0.02 room(2) 0.10], [ hx-0.01 0 0.05], skirt);
mb.box([0.02 room(2) 0.10], [-hx+0.01 0 0.05], skirt);
mb.box([0.04 0.95 2.05], [hx-0.02 -1.55 1.025], [0.42 0.30 0.20]);        % door leaf
mb.box([0.05 1.05 0.06], [hx-0.025 -1.55 2.08], [0.85 0.85 0.85]);        % door head
mb.cyl(0.03, 0.12, [hx-0.07 -1.18 1.0], [0.75 0.75 0.78], [0 90 0], 12);  % handle
truss = [0.11 0.11 0.12];
for sx = [-1 1]
    for sy = [-1 1]
        mb.cyl(0.05, hz, [sx*(hx-0.12) sy*(hy-0.12) hz/2], truss, [], 10);
        mb.box([0.12 0.09 0.09], [sx*(hx-0.22) sy*(hy-0.22) hz-0.35], [0.05 0.05 0.06], [0 25 atan2d(-sy, -sx)]);
    end
end
mb.cyl(0.04, room(1)-0.24, [0  hy-0.12 hz-0.12], truss, [0 90 0], 10);
mb.cyl(0.04, room(1)-0.24, [0 -hy+0.12 hz-0.12], truss, [0 90 0], 10);
mb.cyl(0.04, room(2)-0.24, [ hx-0.12 0 hz-0.12], truss, [90 0 0], 10);
mb.cyl(0.04, room(2)-0.24, [-hx+0.12 0 hz-0.12], truss, [90 0 0], 10);
mb.emit(world, root, 'Fixtures', Shininess=0.3, Metallic=0.1);

lightXY = [-1.6 -1.2; 1.6 -1.2; -1.6 1.2; 1.6 1.2; 0 0];
if hx < 2.5, lightXY = lightXY .* [hx/3 hy/2.5]; end
mb = aeris.MeshBuilder;
for i = 1:size(lightXY, 1)
    mb.box([1.15 0.30 0.02], [lightXY(i, :) hz-0.012], [1 1 1]);
end
mb.emit(world, root, 'LightPanels', Color=[1 1 1], Shininess=0.9, Specular=1.0, Shadows=false);

nL = size(lightXY, 1);
R = T.scene.render;
lamp = sim3d.Light(ActorName='CeilingLights', NumberOfLights=nL, LightType=R.light_type);
lamp.CoordinateSystem = 'MATLAB';
lamp.Translation = [lightXY, repmat(hz-0.06, nL, 1)];
lamp.Rotation = repmat([0 pi/2 0], nL, 1);
lamp.Intensity = repmat(R.light_intensity, nL, 1);
lamp.AttenuationRadius = repmat(9, nL, 1);
lamp.Temperature = repmat(5200, nL, 1);
if strcmp(R.light_type, 'RectLight')
    lamp.SourceWidth = repmat(0.30, nL, 1);
    lamp.SourceHeight = repmat(1.15, nL, 1);
end
addTo(world, lamp, root);
try
    world.GraphicsQuality = R.quality;
catch
end

% ---- Floor markings ----------------------------------------------------
ws = T.scene.workspace;
mb = aeris.MeshBuilder;
tape = [0.93 0.76 0.08]; w = 0.05; z = 0.0015;
x0 = ws.x(1); x1 = ws.x(2); y0 = ws.y(1); y1 = ws.y(2);
mb.quad([x0 y0-w z], [x1 y0-w z], [x1 y0 z], [x0 y0 z], tape);
mb.quad([x0 y1 z], [x1 y1 z], [x1 y1+w z], [x0 y1+w z], tape);
mb.quad([x0-w y0-w z], [x0 y0-w z], [x0 y1+w z], [x0-w y1+w z], tape);
mb.quad([x1 y0-w z], [x1+w y0-w z], [x1+w y1+w z], [x1 y1+w z], tape);
mb.emit(world, root, 'FloorTape', Shininess=0.2, Shadows=false);

home = T.scene.home.pos(:).';
mb = aeris.MeshBuilder;
r = 0.30;
mb.quad([home(1)-r home(2)-r 0.002], [home(1)+r home(2)-r 0.002], [home(1)+r home(2)+r 0.002], [home(1)-r home(2)+r 0.002], [1 1 1], [0 1; 1 1; 1 0; 0 0]);
mb.emit(world, root, 'Helipad', Texture=tex('helipad.png'), Shininess=0.1, Shadows=false);

mk = T.scene.markers;
half = mk.size_m / 2 * (600 / 480);   % printed tile includes the white quiet zone
for i = 1:numel(mk.items)
    c = mk.items(i).pos(:).';
    mb = aeris.MeshBuilder;
    mb.quad([c(1)-half c(2)-half 0.003], [c(1)+half c(2)-half 0.003], [c(1)+half c(2)+half 0.003], [c(1)-half c(2)+half 0.003], ...
        [1 1 1], [0 1; 1 1; 1 0; 0 0]);
    mb.emit(world, root, sprintf('Marker%d', mk.items(i).id), ...
        Texture=tex(sprintf('aruco_%d.png', mk.items(i).id)), Shininess=0.02, Specular=0.02, Shadows=false);
end

mb = aeris.MeshBuilder;
mb.quad([-hx+0.004 0.9 1.15], [-hx+0.004 0.2 1.15], [-hx+0.004 0.2 2.13], [-hx+0.004 0.9 2.13], [1 1 1], [0 1; 1 1; 1 0; 0 0]);
mb.emit(world, root, 'Poster', Texture=tex('poster_safety.png'), Shininess=0.1, Shadows=false);

% ---- Objects -----------------------------------------------------------
for i = 1:numel(T.scene.objects)
    o = T.scene.objects(i);
    groups = aeris.objectMesh(o, T);
    main = [];
    for g = 1:numel(groups)
        gr = groups(g);
        args = {'Shininess', gr.shininess, 'Metallic', gr.metallic, 'Specular', gr.specular, ...
                'Transparency', gr.transparency, 'Movable', o.movable};
        if ~isempty(gr.texture), args = [args, {'Texture', tex(gr.texture)}]; end %#ok<AGROW>
        if g == 1
            main = gr.mesh.emit(world, root, o.name, args{:}, ...
                Translation=o.pos(:).', Rotation=[0 0 o.yaw_deg]);
        else
            gr.mesh.emit(world, main, sprintf('%s_part%d', o.name, g), args{:});
        end
    end
end

if ~T.scene.render.shadows
    names = fieldnames(world.Actors);
    for i = 1:numel(names)
        a = world.Actors.(names{i});
        if isprop(a, 'Shadows') && ~any(strcmp(names{i}, {'Walls', 'Ceiling'}))
            a.Shadows = false;
        end
    end
end
end

function addTo(world, actor, root)
if isempty(root)
    add(world, actor);
else
    add(world, actor, root);
end
end
