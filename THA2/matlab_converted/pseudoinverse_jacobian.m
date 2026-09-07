function J_dagger = pseudoinverse_jacobian(J)
% Pseudoinverse matching the Python implementation.

[m, n] = size(J);
r = rank(J);

if n > m
    J_dagger = J.' / (J * J.');
else
    J_dagger = (J.' * J) \ J.';
end
end
