function y = litewing_square_ref(t)
% LITEWING_SQUARE_REF Waypoint command [xd; yd; zd; yaw] vs time.
% 0-2 s takeoff to 1 m, then 3 s per square corner.
y = zeros(4, 1);
y(3) = 1.0;
if t < 2
    return;
end
corners = [
     0.5,  0.5;
     0.5, -0.5;
    -0.5, -0.5;
    -0.5,  0.5;
     0.5,  0.5
];
k = min(floor((t - 2) / 3) + 1, size(corners, 1));
y(1) = corners(k, 1);
y(2) = corners(k, 2);
end
