function saveMetricsVideoTubular(log, R, filename, desired_video_time)

N = length(log.t);
frame_idx = 1:N;

video = VideoWriter(filename, 'MPEG-4');
video.FrameRate = N / desired_video_time;
open(video);

fig = figure('Color','w', 'Renderer','opengl');

%% ===== Screen-based sizing =====
screen = get(0, 'ScreenSize');
target_ratio = 6/2;

W = screen(3);
H = W / target_ratio;

set(fig, 'Position', [100, 100, W, H]);

fs = 18;

for k = frame_idx
    clf(fig);

    tiledlayout(1,2, ...
        'TileSpacing','compact', ...
        'Padding','compact');

    %% (1) Distance and spring force overlay
    ax1 = nexttile;
    hold(ax1,'on'); grid(ax1,'on');

    yyaxis(ax1,'left');
    plot(ax1, log.t, log.d, 'LineWidth', 2);
    yline(ax1, R, '--', 'LineWidth', 1.5);
    plot(ax1, log.t(k), log.d(k), 'ko', 'MarkerFaceColor','k');
    ylabel(ax1, 'd(t) [m]');

    yyaxis(ax1,'right');
    plot(ax1, log.t, log.F_spring, 'LineWidth', 2);
    plot(ax1, log.t(k), log.F_spring(k), 'ks', 'MarkerFaceColor','k');
    ylabel(ax1, '||F_{spring}|| [N]');

    xline(ax1, log.t(k), 'k--', 'LineWidth', 1.2);

    xlabel(ax1, 'Time [s]');
    xlim(ax1, [log.t(1), log.t(end)]);
    title(ax1, 'Distance and Spring Force');
    set(ax1, 'FontSize', fs);

    %% (2) Tangential force
    ax2 = nexttile;
    hold(ax2,'on'); grid(ax2,'on');

    plot(ax2, log.t, log.F_tangent, 'LineWidth', 2);
    xline(ax2, log.t(k), 'k--', 'LineWidth', 1.2);
    plot(ax2, log.t(k), log.F_tangent(k), 'ko', 'MarkerFaceColor','k');

    xlabel(ax2, 'Time [s]');
    ylabel(ax2, '||F_{tangent}|| [N]');
    xlim(ax2, [log.t(1), log.t(end)]);
    title(ax2, 'Tangential Guidance Force');
    set(ax2, 'FontSize', fs);

    drawnow;
    writeVideo(video, getframe(fig));
end

close(video);

end