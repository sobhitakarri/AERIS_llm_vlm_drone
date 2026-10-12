function q = rotm2q(R)
% ROTM2Q  Scalar-first unit quaternion from a rotation matrix (Shepperd).
%#codegen
tr = R(1,1) + R(2,2) + R(3,3);
if tr > 0
    s = sqrt(tr + 1) * 2;
    q = [0.25*s; (R(3,2) - R(2,3))/s; (R(1,3) - R(3,1))/s; (R(2,1) - R(1,2))/s];
elseif R(1,1) > R(2,2) && R(1,1) > R(3,3)
    s = sqrt(1 + R(1,1) - R(2,2) - R(3,3)) * 2;
    q = [(R(3,2) - R(2,3))/s; 0.25*s; (R(1,2) + R(2,1))/s; (R(1,3) + R(3,1))/s];
elseif R(2,2) > R(3,3)
    s = sqrt(1 + R(2,2) - R(1,1) - R(3,3)) * 2;
    q = [(R(1,3) - R(3,1))/s; (R(1,2) + R(2,1))/s; 0.25*s; (R(2,3) + R(3,2))/s];
else
    s = sqrt(1 + R(3,3) - R(1,1) - R(2,2)) * 2;
    q = [(R(2,1) - R(1,2))/s; (R(1,3) + R(3,1))/s; (R(2,3) + R(3,2))/s; 0.25*s];
end
q = q / norm(q);
end
