function save_robot_motion_video(robot, theta_history, T_list, filename, wall, p_goal, r)

fs = 20;

if nargin < 5
    wall = [];
end
has_wall = ~isempty(wall) && isstruct(wall);

if nargin < 7
    p_goal = [];
    r = [];
end

has_goal_region = ~isempty(p_goal) && ~isempty(r);

N = size(theta_history, 1);
frame_idx = 1:N;

fig = figure('Color','w', 'Position', [100 100 900 700]);
ax = axes(fig);

p = squeeze(T_list(1:3,4,:));

margin = 0.5;
xlims = [min(p(1,:)) - margin, max(p(1,:)) + margin];
ylims = [min(p(2,:)) - margin, max(p(2,:)) + margin];
zlims = [0, max(p(3,:)) + margin];

video = VideoWriter(filename, 'MPEG-4');
video.FrameRate = 5;
open(video);

for k = frame_idx
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');

    title(ax, sprintf('IK Convergence: Robot Motion (iter %d/%d)', k-1, N-1), ...
        'FontSize', fs+2, 'FontWeight', 'bold');

    set(ax, 'FontSize', fs);

    if has_wall
        draw_virtual_wall(ax, wall);
    end

    plot3(ax, p(1,:), p(2,:), p(3,:), 'k--', 'LineWidth', 1.0);
    plot3(ax, p(1,1:k), p(2,1:k), p(3,1:k), 'r-', 'LineWidth', 2.0);

    theta_k = theta_history(k,:).';
    draw_robot(ax, robot, theta_k);

    p_k = p(:,k);
    plot3(ax, p_k(1), p_k(2), p_k(3), 'ko', ...
        'MarkerSize', 8, 'MarkerFaceColor', 'k', 'LineWidth', 1.5);

    if has_goal_region
        draw_goal_region(ax, p_goal, r);
    end

    if has_wall
        n_scale = 0.12;
        quiver3(ax, ...
            wall.center(1), wall.center(2), wall.center(3), ...
            n_scale*wall.normal(1), n_scale*wall.normal(2), n_scale*wall.normal(3), ...
            'LineWidth', 2.0, 'MaxHeadSize', 1.0);
    end

    
    axis(ax, 'equal');
    daspect(ax, [1 1 1]);
    view(ax, 135, 25);

    xlim(ax, xlims);
    ylim(ax, ylims);
    zlim(ax, zlims);

    xlabel(ax, 'X [m]');
    ylabel(ax, 'Y [m]');
    zlabel(ax, 'Z [m]');

    frame = getframe(fig);
    writeVideo(video, frame);
end

close(video);
end
