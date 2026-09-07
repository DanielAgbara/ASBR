function [b_tip, b_post] = pivot_calibration(T)
% Calculate:
%           b = R^(-1)*(-p)
% Input: 
%           T (4x4xn)
% Output:
%           b_tip: the pivoting point wrt Dynamic Reference Frame (DRF)
%           b_post: the povoting point wrt the world/tracker frame

R_stack = zeros(3*size(T, 3), 6);
p_stack = zeros(3*size(T, 3), 1);

for i = 1 : size(T, 3)
    idx = 3*i-2:3*i;
    R_stack(idx, 1:3) = T(1:3, 1:3, i);
    R_stack(idx, 4:6) = -eye(3);
    p_stack(idx) = -T(1:3, 4, i);
end

x = pinv(R_stack) * p_stack;

b_tip = x(1:3);
b_post = x(4:6);
end