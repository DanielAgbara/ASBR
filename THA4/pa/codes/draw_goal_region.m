function draw_goal_region(ax, p_goal, r)

res = 25;
[X, Y, Z] = sphere(res);

X = r * X + p_goal(1);
Y = r * Y + p_goal(2);
Z = r * Z + p_goal(3);

surf(ax, X, Y, Z, ...
    'FaceColor', [0.2 0.8 0.2], ...
    'FaceAlpha', 0.1, ...
    'EdgeColor', 'none');

end