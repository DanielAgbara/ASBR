function Ad_T_inv = adjoint_inverse(T)
% Inverse adjoint matrix of T.

R = T(1:3, 1:3);
p = T(1:3, 4);
Ad_T_inv = [R.' zeros(3,3); -R.' * skew(p), R.'];
end
