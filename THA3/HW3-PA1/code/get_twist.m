function twist = get_twist(q, s, h)
% Build a 6x1 twist S = [w; v] from screw axis parameters {q, s, h}.
%
% Inputs:
%   q - point on axis in R^3 (3x1 or 1x3)
%   s - axis direction (will be normalized)
%   h - pitch
%
% Output:
%   twist - 6x1 vector [w; v]
%
% For unit w:
%   v = -w x q + h*w

    q = double(q(:));
    s = double(s(:));

    [w, wn] = normalize(s);
    if wn == 0
        error('s must be non-zero.');
    end

    v = -cross(w, q) + h * w;

    twist = [w; v];
end