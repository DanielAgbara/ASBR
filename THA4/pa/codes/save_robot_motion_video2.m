function save_robot_motion_video2(T_list, filename, wall, p_goal, r)

fs = 20;

has_wall = ~isempty(wall);

N = size(T_list, 3);
frame_idx = 1:N;

p = squeeze(T_list(1:3,4,:));

fig = figure('Color','w', 'Position', [100 100 800 700]);
ax = axes(fig);

% ===== Axis limits: include full path + goal region + wall =====
all_pts = [p, p_goal(:)];

if has_wall
    all_pts = [all_pts, wall.center(:)];
end

pad = max(3*r, 0.005);   % at least 5 mm padding

xlims = [min(all_pts(1,:)) - pad, max(all_pts(1,:)) + pad];
ylims = [min(all_pts(2,:)) - pad, max(all_pts(2,:)) + pad];
zlims = [min(all_pts(3,:)) - pad, max(all_pts(3,:)) + pad];

% Avoid zero-width limits
if diff(xlims) < 1e-6, xlims = xlims + [-0.005 0.005]; end
if diff(ylims) < 1e-6, ylims = ylims + [-0.005 0.005]; end
if diff(zlims) < 1e-6, zlims = zlims + [-0.005 0.005]; end

% ===== Frame scale: based on visible workspace =====
view_span = max([diff(xlims), diff(ylims), diff(zlims)]);
axis_len = 0.08 * view_span;

video = VideoWriter(filename, 'MPEG-4');
video.FrameRate = 5;
open(video);

for k = frame_idx
    % cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');

    axis(ax, 'equal');
    daspect(ax, [1 1 1]);
    view(ax, -135, 25);

    xlim(ax, xlims);
    ylim(ax, ylims);
    zlim(ax, zlims);

    xlabel(ax, 'X [m]');
    ylabel(ax, 'Y [m]');
    zlabel(ax, 'Z [m]');

    title(ax, sprintf('Close-up View Near Goal (iter %d/%d)', k-1, N-1), ...
        'FontSize', fs+2, 'FontWeight', 'bold');

    set(ax, 'FontSize', fs);

    % ===== Goal tolerance region =====
    draw_goal_region(ax, p_goal, r);

    % ===== Goal point =====
    plot3(ax, p_goal(1), p_goal(2), p_goal(3), 'go', ...
        'MarkerSize', 9, ...
        'MarkerFaceColor', 'g', ...
        'LineWidth', 1.5);

    % ===== Wall =====
    if has_wall
        draw_virtual_wall(ax, wall, view_span);

        n_scale = 0.12 * view_span;
        quiver3(ax, ...
            wall.center(1), wall.center(2), wall.center(3), ...
            n_scale*wall.normal(1), ...
            n_scale*wall.normal(2), ...
            n_scale*wall.normal(3), ...
            'LineWidth', 2.0, ...
            'MaxHeadSize', 1.0);
    end

    % ===== Full path =====
    plot3(ax, p(1,:), p(2,:), p(3,:), 'k--', ...
        'LineWidth', 1.0);

    % ===== Current path =====
    plot3(ax, p(1,1:k), p(2,1:k), p(3,1:k), 'r-', ...
        'LineWidth', 2.5);

    % ===== Start point =====
    plot3(ax, p(1,1), p(2,1), p(3,1), 'bo', ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', 'b', ...
        'LineWidth', 1.5);

    % ===== Current tool-tip point =====
    T_k = T_list(:,:,k);
    R_k = T_k(1:3,1:3);
    p_k = T_k(1:3,4);

    plot3(ax, p_k(1), p_k(2), p_k(3), 'ko', ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', 'k', ...
        'LineWidth', 1.5);

    % ===== End-effector frame =====
    draw_frame(ax, R_k, p_k, axis_len);

    xlim(ax, xlims);
    ylim(ax, ylims);
    zlim(ax, zlims);

    frame = getframe(fig);
    writeVideo(video, frame);
end

close(video);
end