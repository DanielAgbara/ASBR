function pts = screw_axis_line_points(q, s, span, n)
% Points on the screw axis line: q + alpha*s for alpha in [-span, span].
%
% Inputs:
%   q    - point on axis
%   s    - axis direction
%   span - half-length of line segment (optional, default = 5.0)
%   n    - number of points (optional, default = 200)
%
% Output:
%   pts  - Nx3 array of points on the axis line

    if nargin < 3 || isempty(span)
        span = 5.0;
    end
    if nargin < 4 || isempty(n)
        n = 200;
    end

    q = double(q(:));
    s = double(s(:));

    [s, sn] = normalize(s);
    if sn == 0
        error('s must be non-zero.');
    end

    alphas = linspace(-span, span, n).';
    pts = q.' + alphas * s.';
end