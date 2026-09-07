function w = joint_limit_objective(theta, q_min, q_max)
% Distance-from-joint-limits objective.

span = q_max(:) - q_min(:);
q_bar = (q_min(:) + q_max(:)) / 2;
theta = theta(:);
w = -0.5 * mean(((theta - q_bar) ./ span) .^ 2);
end
