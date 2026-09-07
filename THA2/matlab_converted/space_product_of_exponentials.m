function T_ee = space_product_of_exponentials(M, S_list, theta)
% Computes T(theta) = exp([S1]theta1)...exp([Sn]thetan)M.

theta = theta(:);
T = eye(4);
for i = 1:numel(S_list)
    T = T * exp_screw_axis(S_list{i}, theta(i));
end
T_ee = T * M;
end
