function plot_robot_motion_snapshots(robot, theta_history, T_list, wall, p_goal, r, num_snapshots)

fs = 20;

if nargin < 4
    wall = [];
end
has_wall = ~isempty(wall) && isstruct(wall);

if nargin < 6
    p_goal = [];
    r = [];
end
has_goal_region = ~isempty(p_goal) && ~isempty(r);

if nargin < 7
    num_snapshots = 5;
end

N = size(theta_history, 1);

% ===== snapshot indices =====
idx = unique(round(linspace(1, N, num_snapshots)));
num_snapshots = length(idx);

% ===== end-effector path =====
p = squeeze(T_list(1:3,4,:));

margin = 0.5;
xlims = [-0.4, 0.8];
ylims = [min(p(2,:)) - margin, 0.6];
zlims = [0, 1.2];

% ===== main figure =====
fig = figure('Color','w', 'Position', [100 100 900 700]);
ax = axes(fig);
hold(ax, 'on');
grid(ax, 'on');

% ===== base trajectory =====
plot3(ax, p(1,:), p(2,:), p(3,:), 'k--', 'LineWidth', 1.0);

% ===== wall =====
if has_wall
    draw_virtual_wall(ax, wall);
end

% ===== colors =====
cmap = lines(num_snapshots);

% ===== draw snapshots =====
for i = 1:num_snapshots
    k = idx(i);

    theta_k = theta_history(k,:).';

    % Draw robot snapshot as independent graphics
    draw_robot_snapshot_copy(ax, robot, theta_k, 0.25);

    % Partial trajectory up to this snapshot
    plot3(ax, p(1,1:k), p(2,1:k), p(3,1:k), ...
        '-', ...
        'Color', cmap(i,:), ...
        'LineWidth', 2.0);
end

% ===== goal region =====
if has_goal_region
    draw_goal_region(ax, p_goal, r);
end

p_init = p(:,1);
h_init = plot3(ax, p_init(1), p_init(2), p_init(3), ...
    'ko', ...
    'MarkerFaceColor', 'k', ...
    'MarkerSize', 9, ...
    'LineWidth', 1.5);

if has_goal_region
    h_goal = plot3(ax, p_goal(1), p_goal(2), p_goal(3), ...
        'rp', ...
        'MarkerFaceColor', 'r', ...
        'MarkerSize', 12, ...
        'LineWidth', 1.5);

    legend(ax, [h_init, h_goal], ...
        {'Initial tool position', 'Goal position'}, ...
        'Location', 'best');
else
    legend(ax, h_init, ...
        {'Initial tool position'}, ...
        'Location', 'best');
end

% ===== wall normal =====
if has_wall
    n_scale = 0.12;
    quiver3(ax, ...
        wall.center(1), wall.center(2), wall.center(3), ...
        n_scale*wall.normal(1), n_scale*wall.normal(2), n_scale*wall.normal(3), ...
        'LineWidth', 2.0, ...
        'MaxHeadSize', 1.0);
end

% ===== axis settings =====
axis(ax, 'equal');
daspect(ax, [1 1 1]);
view(ax, 135, 25);

xlim(ax, xlims);
ylim(ax, ylims);
zlim(ax, zlims);

xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');
zlabel(ax, 'Z [m]');

set(ax, 'FontSize', fs);

end

function draw_robot_snapshot_copy(ax_main, robot, q, face_alpha)

if nargin < 4
    face_alpha = 0.25;
end

fig_tmp = figure('Visible', 'off');
ax_tmp = axes(fig_tmp);
hold(ax_tmp, 'on');

show(robot, q, ...
    'Parent', ax_tmp, ...
    'PreservePlot', true, ...
    'Frames', 'off');

drawnow;

% Copy robot graphics
h_tmp = allchild(ax_tmp);
h_new = copyobj(h_tmp, ax_main);

% Find only newly copied objects
patches = findobj(h_new, 'Type', 'Patch');

for i = 1:length(patches)

    % Original patch: transparent face only
    patches(i).FaceAlpha = face_alpha;
    patches(i).EdgeColor = 'none';

    % Duplicate patch: opaque edge only
    h_edge = copyobj(patches(i), ax_main);
    h_edge.FaceColor = 'none';
    h_edge.FaceAlpha = 0;
    h_edge.EdgeColor = 'k';
    h_edge.EdgeAlpha = 1.0;
    h_edge.LineWidth = 0.5;
end

close(fig_tmp);

hold(ax_main, 'on');

end