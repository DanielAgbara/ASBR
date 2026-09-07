function plot_ellipsoid_from_metric(ax, A, center, scale)

A = 0.5 * (A + A.');

[V,D] = eig(A);
lam = diag(D);

lam(lam < 1e-12) = 1e-12;

r = scale * sqrt(lam);

[X,Y,Z] = ellipsoid(0,0,0,r(1),r(2),r(3),20);
pts = V * [X(:)'; Y(:)'; Z(:)'];

Xr = reshape(pts(1,:), size(X)) + center(1);
Yr = reshape(pts(2,:), size(Y)) + center(2);
Zr = reshape(pts(3,:), size(Z)) + center(3);

surf(ax, Xr, Yr, Zr, ...
    'FaceAlpha', 0.2, ...
    'EdgeColor', 'none');
end