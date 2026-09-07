function T_ee = body_product_of_exponentials(M, B_list, theta)
% Computes T(theta) = M exp([B1]theta1)...exp([Bn]thetan).

theta = theta(:);
T = M;
for i = 1:numel(B_list)
    T = T * exp_screw_axis(B_list{i}, theta(i));
end
T_ee = T;
end
