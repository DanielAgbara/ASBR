function T = screw_to_T(twist, theta, eps_val)
% Exponential map: T = exp([S]*theta) in SE(3).
%
% Inputs:
%   twist   - 6x1 vector [w; v]
%             if ||w|| ~ 0 => pure translation
%             else         => rotation + translation
%   theta   - scalar motion parameter
%   eps_val - tolerance (optional, default = 1e-12)
%
% Output:
%   T       - 4x4 homogeneous transform

    if nargin < 3
        eps_val = 1e-12;
    end

    w = twist(1:3, 1);
    v = twist(4:6, 1);

    [w_hat, w_norm] = normalize(w, eps_val);

    if w_norm < eps_val
        R = eye(3);
        p = v * theta;
    else
        W = skew(w_hat);
        R = eye(3) + sin(theta) * W + (1 - cos(theta)) * (W * W);
        G = eye(3) * theta + (1 - cos(theta)) * W + (theta - sin(theta)) * (W * W);
        p = G * v;
    end

    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = p;
end