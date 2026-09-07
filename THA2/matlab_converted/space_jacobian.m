function J = space_jacobian(S_list, theta)
% Space Jacobian.

theta = theta(:);
n = numel(S_list);
J = zeros(6, n);
J(:,1) = S_list{1};

T = eye(4);
for i = 2:n
    T = T * exp_screw_axis(S_list{i-1}, theta(i-1));
    J(:,i) = adjoint(T) * S_list{i};
end
end
