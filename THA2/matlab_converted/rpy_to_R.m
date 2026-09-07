function R = rpy_to_R(rpy)
% Computes R = Rz(yaw) * Ry(pitch) * Rx(roll).

rpy = rpy(:);
r = rpy(1);
p = rpy(2);
y = rpy(3);
R = Rz(y) * Ry(p) * Rx(r);
end
