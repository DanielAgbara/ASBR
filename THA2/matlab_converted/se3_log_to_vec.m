function V = se3_log_to_vec(T)
R = T(1:3,1:3);
p = T(1:3,4);

if norm(R - eye(3), 'fro') < 1e-12
    V = [0;0;0; p];
    return;
end

[w_hat, theta] = so3_log(R);
w = so3_vee(w_hat);

G_inv = eye(3)/theta - 0.5*w_hat + ...
    (1/theta - 0.5*cot(theta/2)) * (w_hat*w_hat);

v = G_inv * p;

V = [w*theta;
     v*theta];
end

function [w_hat, theta] = so3_log(R)
x = (trace(R)-1)/2;
x = min(max(x,-1),1);
theta = acos(x);

if abs(theta) < 1e-12
    w_hat = zeros(3,3);
else
    w_hat = (R - R.')/(2*sin(theta));
end
end

function w = so3_vee(w_hat)
w = [w_hat(3,2); w_hat(1,3); w_hat(2,1)];
end