function save_robot_motion_video(robot, theta_history, T_list, Jw_cell, Jv_cell, filename)
fs = 20;

N = size(theta_history, 1);
frame_step = 1;
frame_idx = unique([1:frame_step:N, N]);

fig = figure('Color','w', 'Position', [100 100 900 700]);
ax = axes(fig);

p = squeeze(T_list(1:3,4,:));

% ===== Fixed global axis limits =====
margin = 0.5;
xmin = min(p(1,:)) - margin; xmax = max(p(1,:)) + margin;
ymin = min(p(2,:)) - margin; ymax = max(p(2,:)) + margin;
zmin = min(p(3,:)) - margin; zmax = max(p(3,:)) + margin;

xlims = [-0.2 xmax];
ylims = [ymin ymax];
zlims = [0 zmax];

v = VideoWriter(filename, 'MPEG-4');
v.FrameRate = 5;
open(v);

for k = frame_idx
    cla(ax);
    hold(ax, 'on');
    grid(ax, 'on');

    % fix axes
    axis(ax, 'manual');
    axis(ax, 'equal');
    daspect(ax, [1 1 1]);
    view(ax, 135, 25);

    xlabel(ax, 'X [m]', 'FontSize', fs);
    ylabel(ax, 'Y [m]', 'FontSize', fs);
    zlabel(ax, 'Z [m]', 'FontSize', fs);
    title(ax, sprintf('IK Convergence: Robot Motion (iter %d/%d)', k-1, N-1) , ...
        'FontSize', fs+2, 'FontWeight', 'bold');

    set(ax, 'FontSize', fs)

    xlim(ax, xlims);
    ylim(ax, ylims);
    zlim(ax, zlims);

    % full EE path
    plot3(ax, p(1,:), p(2,:), p(3,:), 'k--', 'LineWidth', 1.0);

    % EE path up to current iteration
    plot3(ax, p(1,1:k), p(2,1:k), p(3,1:k), 'r-', 'LineWidth', 2.0);

    % current robot pose
    theta_k = theta_history(k,:).';
    draw_robot(ax, robot, theta_k);

    % fix axes
    axis(ax, 'manual');
    axis(ax, 'equal');
    daspect(ax, [1 1 1]);
    view(ax, 135, 25);
    xlim(ax, xlims);
    ylim(ax, ylims);
    zlim(ax, zlims);

    % current EE point
    p_k = p(:,k);
    plot3(ax, p_k(1), p_k(2), p_k(3), 'ko', ...
        'MarkerSize', 8, 'MarkerFaceColor', 'k', 'LineWidth', 1.5);

    % manipulability ellipsoids at end-effector
    J_w = Jw_cell{k};
    J_v = Jv_cell{k};

    A_w = J_w * J_w.';
    A_v = J_v * J_v.';

    plot_ellipsoid_with_axes(ax, A_v, p_k, 0.2, [0 0 1]); % linear
    plot_ellipsoid_with_axes(ax, A_w, p_k, 0.2, [1 0 0]); % angular

    % fix axes
    axis(ax, 'manual');
    axis(ax, 'equal');
    daspect(ax, [1 1 1]);
    xlim(ax, xlims);
    ylim(ax, ylims);
    zlim(ax, zlims);

    frame = getframe(fig);
    writeVideo(v, frame);
end

close(v);
% close(fig);
end