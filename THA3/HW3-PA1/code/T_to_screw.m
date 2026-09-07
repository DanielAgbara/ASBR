function [screw_params, theta] = T_to_screw(T, eps_val)
% Given T in SE(3), recover screw parameters {q, s, h} and theta.
%
% Returns:
%   screw_params = {q, s, h}
%   theta
%
% Note: for theta ~ 0, the screw axis is not unique (near pure translation).
%
% Requires:
%   - RToAxisAngle(R): returns [s, theta], where s is a unit axis
%   - skew(s): returns the 3x3 skew-symmetric matrix of s

    if nargin < 2
        eps_val = 1e-9;
    end

    T = double(T);

    if ~isequal(size(T), [4, 4])
        error('T must be 4x4.');
    end

    R = T(1:3, 1:3);
    p = T(1:3, 4);

    [s, theta] = RToAxisAngle(R);  % s is unit axis from your rotation code

    % Near-identity rotation case (translation / tiny rotation)
    if abs(theta) < eps_val
        pnorm = norm(p);

        if pnorm < eps_val
            screw_params = {zeros(3,1), [0; 0; 1], 0.0};
            theta = 0.0;
            return;
        end

        s = p / pnorm;
        q = zeros(3,1);
        h = Inf;

        screw_params = {q, s, h};
        return;
    end

    % Use the inverse of the left Jacobian (G_inv) to recover v from p
    W = skew(s);
    cot_half = 1.0 / tan(theta / 2.0);

    G_inv = (1.0 / theta) * eye(3) ...
          - 0.5 * W ...
          + (1.0 / theta - 0.5 * cot_half) * (W * W);

    v = G_inv * p;

    % Screw parameters
    q = cross(s, v) / (norm(s)^2);   % norm(s)=1 usually
    h = (s.' * v) / (norm(s)^2);

    screw_params = {q, s, h};
end