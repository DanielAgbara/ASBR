function w = manipulability_objective(theta, B_list)
% Objective based on body Jacobian manipulability.

J = body_jacobian(B_list, theta);
w = manipulability(J);
end
