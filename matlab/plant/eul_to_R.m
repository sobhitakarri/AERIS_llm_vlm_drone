function R = eul_to_R(eul)
% EUL_TO_R ZYX Euler angles (roll, pitch, yaw) -> world-from-body rotation.
phi = eul(1); th = eul(2); psi = eul(3);
cz = cos(psi); sz = sin(psi);
cy = cos(th);  sy = sin(th);
cx = cos(phi); sx = sin(phi);
Rz = [cz -sz 0; sz cz 0; 0 0 1];
Ry = [cy 0 sy; 0 1 0; -sy 0 cy];
Rx = [1 0 0; 0 cx -sx; 0 sx cx];
R = Rz * Ry * Rx;
end
