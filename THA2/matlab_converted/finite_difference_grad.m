function grad = finite_difference_grad(w_func, theta, eps_val)
% Central-difference gradient.

if nargin < 3
    eps_val = 1e-6;
end

theta = theta(:);
grad = zeros(size(theta));

for i = 1:length(theta)
    t1 = theta;
    t2 = theta;
    t1(i) = t1(i) + eps_val;
    t2(i) = t2(i) - eps_val;
    grad(i) = (w_func(t1) - w_func(t2)) / (2 * eps_val);
end
end
