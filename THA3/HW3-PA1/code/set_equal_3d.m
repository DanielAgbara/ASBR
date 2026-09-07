function set_equal_3d(ax, pts)
% Set 3D limits so x/y/z appear with roughly equal scale.
%
% Inputs:
%   ax  - axes handle
%   pts - Nx3 array of points that must be visible

    pts = double(pts);
    X = pts(:, 1);
    Y = pts(:, 2);
    Z = pts(:, 3);

    xmid = 0.5 * (min(X) + max(X));
    ymid = 0.5 * (min(Y) + max(Y));
    zmid = 0.5 * (min(Z) + max(Z));

    r = max([max(X) - min(X), ...
             max(Y) - min(Y), ...
             max(Z) - min(Z)]) * 0.55;

    if r == 0
        r = 1.0;
    end

    xlim(ax, [xmid - r, xmid + r]);
    ylim(ax, [ymid - r, ymid + r]);
    zlim(ax, [zmid - r, zmid + r]);

    axis(ax, 'equal');
end