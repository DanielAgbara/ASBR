function plot_robot_motion_snapshot2(T_list, wall, p_goal, r, num_snapshots)

fs = 20;

if nargin < 2
    wall = [];
end

has_wall = ~isempty(wall) && isstruct(wall);

if nargin < 5
    num_snapshots = 5;
end

N = size(T_list, 3);
idx = unique(round(linspace(1, N, num_snapshots)));
num_snapshots = length(idx);

p = squeeze(T_list(1:3,4,:));

fig = figure('Color','w', 'Position', [100 100 800 700]);
ax = axes(fig);
hold(ax, 'on');
grid(ax, 'on');

%% ===== Axis limits =====
all_pts = [p, p_goal(:)];

if has_wall
    all_pts = [all_pts, wall.center(:)];
end

pad = max(3*r, 0.005);

xlims = [min(all_pts(1,:)) - pad, max(all_pts(1,:)) + pad];
ylims = [min(all_pts(2,:)) - pad, max(all_pts(2,:)) + pad];
zlims = [min(all_pts(3,:)) - pad, max(all_pts(3,:)) + pad];

if diff(xlims) < 1e-6, xlims = xlims + [-0.005 0.005]; end
if diff(ylims) < 1e-6, ylims = ylims + [-0.005 0.005]; end
if diff(zlims) < 1e-6, zlims = zlims + [-0.005 0.005]; end

view_span = max([diff(xlims), diff(ylims), diff(zlims)]);
axis_len = 0.08 * view_span;

%% ===== Goal tolerance region =====
draw_goal_region(ax, p_goal, r);

%% ===== Wall =====
if has_wall
    h_wall = draw_virtual_wall(ax, wall, view_span);

    n_scale = 0.12 * view_span;
    quiver3(ax, ...
        wall.center(1), wall.center(2), wall.center(3), ...
        n_scale*wall.normal(1), n_scale*wall.normal(2), n_scale*wall.normal(3), ...
        'LineWidth', 2.0, ...
        'MaxHeadSize', 1.0, ...
        'HandleVisibility','off');
end

%% ===== Full trajectory =====
plot3(ax, p(1,:), p(2,:), p(3,:), 'k--', ...
    'LineWidth', 1.0, ...
    'HandleVisibility','off');

%% ===== Snapshot trajectories and frames =====
cmap = lines(num_snapshots);

for i = 1:num_snapshots
    k = idx(i);

    T_k = T_list(:,:,k);
    R_k = T_k(1:3,1:3);
    p_k = T_k(1:3,4);

    plot3(ax, p(1,1:k), p(2,1:k), p(3,1:k), ...
        '-', ...
        'Color', cmap(i,:), ...
        'LineWidth', 2.0, ...
        'HandleVisibility','off');

    plot3(ax, p_k(1), p_k(2), p_k(3), 'o', ...
        'Color', cmap(i,:), ...
        'MarkerFaceColor', cmap(i,:), ...
        'MarkerSize', 6, ...
        'LineWidth', 1.2, ...
        'HandleVisibility','off');

    draw_frame(ax, R_k, p_k, axis_len);
end

%% ===== Draw initial and goal last so they are visible =====
p_init = p(:,1);

h_init = plot3(ax, p_init(1), p_init(2), p_init(3), 'ko', ...
    'MarkerSize', 10, ...
    'MarkerFaceColor', 'k', ...
    'LineWidth', 1.5);

h_goal = plot3(ax, p_goal(1), p_goal(2), p_goal(3), 'r*', ...
    'MarkerSize', 13, ...
    'LineWidth', 1.8);

if has_wall
    legend(ax, [h_init, h_goal, h_wall], ...
        {'Initial tool position', 'Goal position', 'Virtual wall'}, ...
        'Location', 'best');
else
    legend(ax, [h_init, h_goal], ...
        {'Initial tool position', 'Goal position'}, ...
        'Location', 'best');
end

%% ===== Axis settings =====
axis(ax, 'equal');
daspect(ax, [1 1 1]);
view(ax, -135, 25);

xlim(ax, xlims);
ylim(ax, ylims);
zlim(ax, zlims);

xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');
zlabel(ax, 'Z [m]');

title(ax, 'Close-up Motion Snapshots Near Goal', ...
    'FontSize', fs+2, ...
    'FontWeight', 'bold');

set(ax, 'FontSize', fs);
ax.LineWidth = 1.2;

end