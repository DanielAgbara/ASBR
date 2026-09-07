function T = maximum_likelihood(a, b)
% 
% Inputs:
%           a: moving model set (3xn)
%           b: fixed/statac scene set (3xn)
% Returns:
%           T: 

if size(a, 2) < 3 || size(b, 2) < 3
    error('At least 3 points are required');
end

N = size(a, 2);
a_bar = sum(a, 2) / N;
b_bar = sum(b, 2) / N;

a_tilde = a - a_bar;
b_tilde = b - b_bar;

H = a_tilde * b_tilde';
[U, S, V] = svd(H, 'econ');

R = V * U';

% FIX: enforce proper rotation
if det(R) < 0
    % Correct the reflection so the estimate belongs to SO(3).
    V(:, end) = -V(:, end);
    R = V * U';
end

p = b_bar - R * a_bar;

T = [R, p;
     0, 0, 0, 1];

end

