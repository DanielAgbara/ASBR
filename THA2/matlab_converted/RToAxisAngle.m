function [w_hat, theta] = RToAxisAngle(R, eps_val)
% Rotation axis and angle from a rotation matrix.

if nargin < 2
    eps_val = 1e-6;
end

R = double(R);
tr_R = trace(R);
I = eye(3);

if norm(R - I, 'fro') < eps_val
    warning('Rotation axis is undefined');
    w_hat = [0; 0; 1];
    theta = 0.0;
    return;
end

if abs(tr_R + 1) < eps_val
    diagv = 1 + diag(R);
    [~, idx] = max(diagv);
    theta = pi;
    w_hat = R(:, idx);
    w_hat(idx) = w_hat(idx) + 1.0;
    denom = sqrt(2 * (1 + R(idx, idx)));
    w_hat = w_hat / denom;
else
    theta = acos((tr_R - 1) / 2);
    theta = min(max(theta, 0), pi);
    W = (R - R.') / (2 * sin(theta));
    w_hat = [W(3,2); W(1,3); W(2,1)];
end

w_hat = w_hat / norm(w_hat);
end
