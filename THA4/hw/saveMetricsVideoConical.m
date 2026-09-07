function saveMetricsVideoConical(log, x_s, x_t, alpha, filename, desired_video_time)

N = length(log.t);
frame_idx = 1:N;

video = VideoWriter(filename, 'MPEG-4');
video.FrameRate = N / desired_video_time;
open(video);

fig = figure('Color','w', 'Renderer','opengl');

screen = get(0, 'ScreenSize');
target_ratio = 6/2;

W = screen(3);
H = W / target_ratio;

set(fig, 'Position', [100, 100, W, H]);

fs = 18;

for k = frame_idx
    clf(fig);

    tiledlayout(2,2, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    %% (1,1) Distance from cone axis vs cone boundary
    ax1 = nexttile;
    hold(ax1,'on'); grid(ax1,'on');

    plot(ax1, log.t, log.r_perp, 'LineWidth', 2);
    plot(ax1, log.t, log.cone_dist, '--', 'LineWidth', 1.5);
    xline(ax1, log.t(k), 'k--', 'LineWidth', 1.2);
    plot(ax1, log.t(k), log.r_perp(k), 'ko', 'MarkerFaceColor','k');

    ylabel(ax1, 'Distance [m]');
    title(ax1, 'Cone Boundary');
    legend(ax1, 'r_\perp', 'cone boundary', 'Location','best');
    xlim(ax1, [log.t(1), log.t(end)]);
    set(ax1, 'FontSize', fs);

    %% (1,2) Speed
    ax2 = nexttile;
    hold(ax2,'on'); grid(ax2,'on');

    plot(ax2, log.t, log.speed, 'LineWidth', 2);
    xline(ax2, log.t(k), 'k--', 'LineWidth', 1.2);
    plot(ax2, log.t(k), log.speed(k), 'ko', 'MarkerFaceColor','k');

    ylabel(ax2, '||v|| [m/s]');
    title(ax2, 'Speed');
    xlim(ax2, [log.t(1), log.t(end)]);
    set(ax2, 'FontSize', fs);

    %% (2,1) Axial interaction
    ax3 = nexttile;
    hold(ax3,'on'); grid(ax3,'on');
    
    plot(ax3, log.t, log.F_axis_interaction, 'LineWidth', 2);
    yline(ax3, 0, '--', 'LineWidth', 1.2);
    xline(ax3, log.t(k), 'k--', 'LineWidth', 1.2);
    plot(ax3, log.t(k), log.F_axis_interaction(k), 'ko', 'MarkerFaceColor','k');
    
    ylabel(ax3, 'Force [N]');
    title(ax3, sprintf('Axial Interaction: %.3f N', log.F_axis_interaction(k)));
    subtitle(ax3, '+ assist, - resist');
    xlim(ax3, [log.t(1), log.t(end)]);
    set(ax3, 'FontSize', fs);

    %% (2,2) Normal interaction
    ax4 = nexttile;
    hold(ax4,'on'); grid(ax4,'on');
    
    plot(ax4, log.t, log.F_norm_interaction, 'LineWidth', 2);
    yline(ax4, 0, '--', 'LineWidth', 1.2);
    xline(ax4, log.t(k), 'k--', 'LineWidth', 1.2);
    plot(ax4, log.t(k), log.F_norm_interaction(k), 'ko', 'MarkerFaceColor','k');
    
    xlabel(ax4, 'Time [s]');
    ylabel(ax4, 'Force [N]');
    title(ax4, sprintf('Normal Interaction: %.3f N', log.F_norm_interaction(k)));
    subtitle(ax4, '+ assist, - resist');
    xlim(ax4, [log.t(1), log.t(end)]);
    set(ax4, 'FontSize', fs);

    drawnow;
    writeVideo(video, getframe(fig));
end

close(video);

end