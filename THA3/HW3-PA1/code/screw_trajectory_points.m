function traj = screw_trajectory_points(S, theta, p0_world, n)
% Trajectory of a world point p0_world under the screw motion exp([S] t),
% for t in [0, theta].
%
% Inputs:
%   S         - screw parameters {q, s, h}
%   theta     - motion amount
%   p0_world  - initial point in world coordinates
%   n         - number of trajectory samples (optional, default = 200)
%
% Output:
%   traj      - Nx3 array of trajectory points

    if nargin < 4 || isempty(n)
        n = 200;
    end

    q = double(S{1}(:));
    s = double(S{2}(:));
    h = S{3};

    p0_world = double(p0_world(:));

    [s, sn] = normalize(s);
    if sn == 0
        error('s must be non-zero.');
    end

    twist = get_twist(q, s, h);

    ts = linspace(0.0, theta, n);
    p0_h = [p0_world; 1.0];

    traj = zeros(n, 3);
    for i = 1:n
        t = ts(i);
        Tt = screw_to_T(twist, t);
        pt = Tt * p0_h;
        traj(i, :) = pt(1:3).';
    end
end