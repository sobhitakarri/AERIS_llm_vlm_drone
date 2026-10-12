function e = q2eul(q)
% Q2EUL  [roll; pitch; yaw] in radians (ZYX sequence) from a scalar-first quaternion.
%#codegen
w = q(1); x = q(2); y = q(3); z = q(4);
sp = 2*(w*y - z*x);
sp = max(-1, min(1, sp));
e = [atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
     asin(sp);
     atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))];
end
