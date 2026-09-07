function J_star = damped_least_square_inverse(J, k)
% Damped least-squares pseudoinverse.

if nargin < 2
    k = 0.01;
end
JJT = J * J.';
J_star = J.' / (JJT + k^2 * eye(size(JJT,1)));
end
