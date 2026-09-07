function plot_ellipsoid_with_axes(ax, A, center, scale, color_rgb)

A = 0.5 * (A + A.');

[V,D] = eig(A);
lam = diag(D);

% sort descending
[lam, idx] = sort(lam, 'descend');
V = V(:, idx);

lam(lam < 1e-12) = 1e-12;
r = scale * sqrt(lam);

[X,Y,Z] = ellipsoid(0,0,0,r(1),r(2),r(3),24);
pts = V * [X(:)'; Y(:)'; Z(:)'];

Xr = reshape(pts(1,:), size(X)) + center(1);
Yr = reshape(pts(2,:), size(Y)) + center(2);
Zr = reshape(pts(3,:), size(Z)) + center(3);

surf(ax, Xr, Yr, Zr, ...
    'FaceColor', color_rgb, ...
    'FaceAlpha', 0.18, ...
    'EdgeColor', color_rgb, ...
    'EdgeAlpha', 0.15);

% principal axes
for i = 1:3
    dir = V(:,i) * r(i);
    p1 = center - dir;
    p2 = center + dir;

    plot3(ax, [p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], ...
        '-', 'Color', color_rgb, 'LineWidth', 2.0);
end
end