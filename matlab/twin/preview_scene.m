function preview_scene(outDir, dronePose, airframe, scene)
% PREVIEW_SCENE  Render the twin's Unreal scene without running the plant.
%   preview_scene()                 opens the scene and saves stills to cache/twin_preview
%   preview_scene(dir, [x y z yawDeg])
% Use it to check a scene file or the lighting before a full simulation.

if nargin < 3, airframe = 'litewing'; end
if nargin < 4, scene = 'scene_lab'; end
T = aeris.loadParams(airframe, scene);
if nargin < 1 || isempty(outDir), outDir = fullfile(T.paths.repo, 'cache', 'twin_preview'); end
if nargin < 2 || isempty(dronePose), dronePose = [0.2 -0.2 1.0 30]; end
if ~exist(outDir, 'dir'), mkdir(outDir); end

world = sim3d.World(Output=@grab, SampleTime=0.05, StopTime=1.5);
aeris.buildScene(world, [], T);
drone = aeris.buildDrone(world, [], T);
drone.Translation = dronePose(1:3);
drone.Rotation = [0 0 deg2rad(dronePose(4))];

cams = aeris.cameraSpecs(T);
names = fieldnames(cams);
world.UserData.cams = struct();
for i = 1:numel(names)
    c = cams.(names{i});
    if strcmp(names{i}, 'fpv')
        cam = sim3d.sensors.IdealCamera(ActorName='cam_fpv', CoordinateSystem='MATLAB', ...
            ImageSize=c.size([2 1]), HorizontalFieldOfView=c.hfov, Translation=c.pos, Rotation=deg2rad(c.rpy_deg));
        add(world, cam, drone);
    else
        cam = sim3d.sensors.IdealCamera(ActorName=['cam_' names{i}], CoordinateSystem='MATLAB', ...
            ImageSize=c.size([2 1]), HorizontalFieldOfView=c.hfov, Translation=c.pos, Rotation=deg2rad(c.rpy_deg));
        add(world, cam);
    end
    world.UserData.cams.(names{i}) = cam;
end
world.UserData.outDir = outDir;
world.UserData.n = 0;
run(world);
close(world);
delete(world);
fprintf('Stills written to %s\n', outDir);
end

function grab(world)
world.UserData.n = world.UserData.n + 1;
if world.UserData.n == 24
    names = fieldnames(world.UserData.cams);
    for i = 1:numel(names)
        img = read(world.UserData.cams.(names{i}));
        imwrite(img, fullfile(world.UserData.outDir, [names{i} '.png']));
    end
end
end
