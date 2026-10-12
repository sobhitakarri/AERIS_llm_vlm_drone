function writeRuntime(T)
% WRITERUNTIME  Publish what the Python side needs to talk to this twin.
%   cache/twin_runtime.json carries the network ports, camera intrinsics,
%   the surveyed calibration-marker positions, the workspace limits and the
%   airframe summary. Object poses are included under "ground_truth" for
%   scoring; the planner and perception code do not read that section.

[cams, feeds] = aeris.cameraSpecs(T);
names = fieldnames(cams);
R = struct();
R.created = char(datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ss'));
R.airframe = struct('id', T.airframe_id, 'name', T.air.name, 'mass_kg', T.air.m, ...
    'arm_m', T.air.L, 'thrust_to_weight', T.air.thrust_to_weight, ...
    'v_max', T.ctl.v_max, 'hover_rad_s', T.air.w_hover);
R.scene = T.scene_id;
R.net = T.net;
R.workspace = T.scene.workspace;
R.room = T.scene.room;
R.home = struct('pos', [T.scene.home.pos(:).' T.z_rest], 'yaw_deg', T.scene.home.yaw_deg);
R.markers = T.scene.markers;
R.cameras = struct();
for i = 1:numel(names)
    c = cams.(names{i});
    R.cameras.(names{i}) = struct('id', find(strcmp(feeds, names{i})), 'width', c.size(1), 'height', c.size(2), ...
        'fx', c.fx, 'fy', c.fy, 'cx', c.cx, 'cy', c.cy, 'hfov_deg', c.hfov, ...
        'rate_hz', 1 / T.Ts.cam);
end
R.streams = struct();
for i = 1:numel(feeds), R.streams.(feeds{i}) = i; end
R.rates = T.Ts;
gt = struct();
for i = 1:numel(T.scene.objects)
    o = T.scene.objects(i);
    gt.(o.name) = struct('type', o.type, 'pos', o.pos, 'size_m', o.size_m, ...
        'yaw_deg', o.yaw_deg, 'movable', o.movable);
end
R.ground_truth = struct('objects', gt, 'cameras', cams);

d = fileparts(T.paths.runtime);
if ~exist(d, 'dir'), mkdir(d); end
fid = fopen(T.paths.runtime, 'w');
fwrite(fid, jsonencode(R, PrettyPrint=true));
fclose(fid);
end
