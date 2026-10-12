function R = q2rotm(q)
% Q2ROTM  Rotation matrix (body -> world) from a scalar-first unit quaternion.
%#codegen
w = q(1); x = q(2); y = q(3); z = q(4);
R = [1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y);
     2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x);
     2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y)];
end
