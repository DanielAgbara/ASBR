function R = AxisAngleToR(w, theta)
% Rodrigues formula.

w = double(w(:));
wn = norm(w);
if wn < 1e-12
    R = eye(3);
    return;
end

w = w / wn;
w_hat = skew(w);
R = eye(3) + sin(theta) * w_hat + (1 - cos(theta)) * (w_hat * w_hat);
end
