function [H, world_pos] = homography_utils(pixel_uv, src_pts, dst_pts)
% HOMOGRAPHY_UTILS Computes homography matrix H and transforms pixel (u,v) -> world (X,Y)
%
% Inputs:
%   pixel_uv : [u; v] 2x1 pixel coordinate
%   src_pts  : 4x2 matrix of source camera pixel calibration points
%   dst_pts  : 4x2 matrix of destination physical world calibration points (m)

if nargin < 2 || isempty(src_pts)
    src_pts = [0, 0; 640, 0; 640, 480; 0, 480];
end
if nargin < 3 || isempty(dst_pts)
    dst_pts = [-1.5, 1.5; 1.5, 1.5; 1.5, -1.5; -1.5, -1.5];
end

% Compute Homography matrix H using projective transformation
tform = fitgeotform2d(src_pts, dst_pts, 'projective');
H = tform.T';

if nargin > 0 && ~isempty(pixel_uv)
    p = [pixel_uv(1); pixel_uv(2); 1.0];
    w = H * p;
    world_pos = [w(1)/w(3); w(2)/w(3)];
else
    world_pos = [];
end
end
