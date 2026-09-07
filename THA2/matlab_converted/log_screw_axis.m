function [S, theta] = log_screw_axis(T)
% Computes T = exp([S] theta) -> S, theta.

R = T(1:3, 1:3);
p = T(1:3, 4);
[w, theta] = RToAxisAngle(R);

if norm(w * theta) < 1e-12
    pnorm = norm(p);
    if pnorm < 1e-12
        S = zeros(6,1);
        theta = 0.0;
    else
        v = p / pnorm;
        S = [zeros(3,1); v];
        theta = pnorm;
    end
    return;
end

w_hat = skew(w);
G_inv = eye(3) / theta - 0.5 * w_hat + (1/theta - 0.5 / tan(theta/2)) * (w_hat * w_hat);
v = G_inv * p;
S = [w; v];
end
