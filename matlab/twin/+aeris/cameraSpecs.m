function [cams, feeds] = cameraSpecs(T)
% CAMERASPECS  Resolved camera poses and pinhole intrinsics for the scene.
%   Poses use the world frame (x forward, y left, z up); rpy_deg is the
%   [roll pitch yaw] the Unreal camera actor is given. The FPV pose is
%   relative to the drone body.
%   feeds lists the streams sent to Python, in stream-id order; the
%   overhead camera also delivers a depth image.

C = T.scene.cameras;
cams = struct();

o = C.overhead;
yaw = 0; if isfield(o, 'yaw_deg'), yaw = o.yaw_deg; end
cams.overhead = spec(o, o.pos(:).', [0 90 yaw]);

f = C.fpv;
cams.fpv = spec(f, f.mount_m(:).', [0 f.tilt_down_deg 0]);

if isfield(C, 'chase')
    ch = C.chase;
    d = ch.look_at(:).' - ch.pos(:).';
    yaw = atan2d(d(2), d(1));
    pitch = atan2d(-d(3), hypot(d(1), d(2)));
    cams.chase = spec(ch, ch.pos(:).', [0 pitch yaw]);
end

names = fieldnames(cams);
feeds = {};
for i = 1:numel(names)
    feeds{end+1} = names{i}; %#ok<AGROW>
    if strcmp(names{i}, 'overhead') && T.scene.render.depth, feeds{end+1} = 'overhead_depth'; end %#ok<AGROW>
end
end

function s = spec(c, pos, rpy)
s.pos = pos;
s.rpy_deg = rpy;
s.hfov = c.hfov_deg;
s.size = c.size_px(:).';                       % [width height]
s.fx = (s.size(1) / 2) / tand(s.hfov / 2);
s.fy = s.fx;
s.cx = s.size(1) / 2;
s.cy = s.size(2) / 2;
s.rate_hz = c.rate_hz;
end
