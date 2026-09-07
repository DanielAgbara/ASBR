function T = exp_screw_axis(S, theta)
% Computes T = exp([S] theta) in SE(3).

hat_S = vec_to_se3(S);
T = exp_screw_hat(hat_S, theta);
end
