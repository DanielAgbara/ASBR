function plotMetricsConical(log)

fig = figure('Color','w','Renderer','opengl');

%% ===== Screen-based sizing =====
screen = get(0, 'ScreenSize');
target_ratio = 6/2;

W = screen(3);
H = W / target_ratio;

set(fig, 'Position', [100, 100, W, H]);

fs = 18;

tiledlayout(1,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

%% (1) Cone violation distance + spring force overlay
ax1 = nexttile;
hold(ax1,'on'); grid(ax1,'on');

yyaxis(ax1,'left');
plot(ax1, log.t, log.r_perp, 'LineWidth', 2);
plot(ax1, log.t, log.cone_dist, '--', 'LineWidth', 1.5);
ylabel(ax1, 'Distance [m]');

yyaxis(ax1,'right');
F_spring_norm = vecnorm(log.F_spring, 2, 1);
plot(ax1, log.t, F_spring_norm, 'LineWidth', 2);
ylabel(ax1, '||F_{spring}|| [N]');
ylim([0, 7])

xlabel(ax1, 'Time [s]');
xlim(ax1, [log.t(1), log.t(end)]);
title(ax1, 'Cone Distance and Spring Force');

legend(ax1, ...
    '||n_{\perp}||', ...
    'd_{cone}', ...
    '||F_{spring}||', ...
    'Location','best');

set(ax1, 'FontSize', fs);

%% (2) Tangential force
ax2 = nexttile;
hold(ax2,'on'); grid(ax2,'on');

F_tan_norm = vecnorm(log.F_tangent, 2, 1);
plot(ax2, log.t, F_tan_norm, 'LineWidth', 2);

xlabel(ax2, 'Time [s]');
ylabel(ax2, '||F_{tangent}|| [N]');
xlim(ax2, [log.t(1), log.t(end)]);
title(ax2, 'Tangential Guidance Force');
ylim(ax2, [0, 1]);

set(ax2, 'FontSize', fs);

end