function T_inv = inv_SE3(T)
% Inverse of homogeneous transformation.

R = T(1:3, 1:3);
p = T(1:3, 4);
T_inv = eye(4);
T_inv(1:3, 1:3) = R.';
T_inv(1:3, 4) = -R.' * p;
end
