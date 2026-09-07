function h_wall = draw_virtual_wall(ax, wall, view_span)

center = wall.center(:);
n = wall.normal(:) / norm(wall.normal);

if nargin < 3 || isempty(view_span)
    xl = xlim(ax);
    yl = ylim(ax);
    zl = zlim(ax);
    view_span = max([diff(xl), diff(yl), diff(zl)]);
    if view_span <= 0
        view_span = 1;
    end
end

if abs(n(1)) < 0.9
    temp = [1; 0; 0];
else
    temp = [0; 1; 0];
end

t1 = cross(n, temp);
t1 = t1 / norm(t1);
t2 = cross(n, t1);

L = 0.35 * view_span;

[u, v] = meshgrid(linspace(-L, L, 10), linspace(-L, L, 10));

pts = center + t1*u(:).' + t2*v(:).';

X = reshape(pts(1,:), size(u));
Y = reshape(pts(2,:), size(u));
Z = reshape(pts(3,:), size(u));

h_wall = surf(ax, X, Y, Z, ...
    'FaceColor', [0.2 0.6 1.0], ...
    'FaceAlpha', 0.30, ...
    'EdgeColor', 'none', ...
    'DisplayName', 'Virtual wall');

end