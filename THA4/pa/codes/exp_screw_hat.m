function T = exp_screw_hat(hat_S, theta)
% Computes T = exp([S] theta) in SE(3).

hat_w = hat_S(1:3, 1:3);
v = hat_S(1:3, 4);
w = [hat_w(3,2); hat_w(1,3); hat_w(2,1)];
wn = norm(w);
T = eye(4);

if wn < 1e-12
    T(1:3, 1:3) = eye(3);
    T(1:3, 4) = v * theta;
    return;
end

R = eye(3) + sin(theta) * hat_w + (1 - cos(theta)) * (hat_w * hat_w);
G = eye(3) * theta + (1 - cos(theta)) * hat_w + (theta - sin(theta)) * (hat_w * hat_w);

T(1:3, 1:3) = R;
T(1:3, 4) = G * v;
end
