function J_b = body_jacobian(B_list, theta)
% Body Jacobian.

theta = theta(:);
n = numel(B_list);
J_b = zeros(6, n);
J_b(:,n) = B_list{n};

T = eye(4);
for i = n-1:-1:1
    T = T * exp_screw_axis(B_list{i+1}, -theta(i+1));
    J_b(:,i) = adjoint(T) * B_list{i};
end
end
