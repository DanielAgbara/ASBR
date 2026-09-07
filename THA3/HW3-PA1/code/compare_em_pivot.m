clc; clear; close all;

[data_debug, ~] = loadDebugData(7);

max_iter = 100;
tol = 1e-6;

%% registration + pivot calibration
G0 = mean(data_debug.empivot.G(:, :, 1), 2);     % 3x1
g  = data_debug.empivot.G(:, :, 1) - G0;         % 3xNg

Nf = data_debug.empivot.Nf;
Ng = size(data_debug.empivot.G, 2);

F_g = zeros(4, 4, Nf);
for i = 1:Nf
    F_g(:, :, i) = iterative_closest_point(g, data_debug.empivot.G(:, :, i), max_iter, tol);
end

[em_tip, em_post] = pivot_calibration(F_g);

%% precompute points for plotting
frame_colors = lines(Nf);

G0_world      = zeros(3, Nf);   % transformed local origin (= transformed G0)
tip_world     = zeros(3, Nf);   % F_g * em_tip
g_world_all   = zeros(3, Ng, Nf); % transformed model points

for i = 1:Nf
    T = F_g(:, :, i);
    R = T(1:3, 1:3);
    p = T(1:3, 4);

    % transformed local origin
    G0_world(:, i) = p;

    % transformed tip
    tip_world(:, i) = R * em_tip + p;

    % transformed template points
    g_world_all(:, :, i) = R * g + p;
end

%% quantitative check
tip_err = vecnorm(tip_world - em_post, 2, 1);

% metrics
mean_abs_err = mean(tip_err);
rms_err      = sqrt(mean(tip_err.^2));
max_abs_err  = max(tip_err);

fprintf('Pivot calibration check:\n');
fprintf('Mean absolute error = %.4f\n', mean_abs_err);
fprintf('RMS error           = %.4f\n', rms_err);
fprintf('Max absolute error  = %.4f\n', max_abs_err);
%% plot
all_pts = reshape(data_debug.empivot.G, 3, []);
valid = ~any(isnan(all_pts), 1);
all_pts = all_pts(:, valid);

xmin = min(all_pts(1,:)); xmax = max(all_pts(1,:));
ymin = min(all_pts(2,:)); ymax = max(all_pts(2,:));
zmin = min(all_pts(3,:)); zmax = max(all_pts(3,:));

% margin
mx = 0.05 * (xmax - xmin + eps);
my = 0.05 * (ymax - ymin + eps);
mz = 0.05 * (zmax - zmin + eps);

xlim_all = [xmin-mx, xmax+mx];
ylim_all = [ymin-my, ymax+my];
zlim_all = [zmin-mz, zmax+mz];

figure(1); clf;

subplot(2,2,1);
plot_pivot_scene(data_debug, F_g, em_tip, Nf);
title('Top view');
view(0, 90);
xlim(xlim_all); ylim(ylim_all); zlim(zlim_all);

subplot(2,2,2);
plot_pivot_scene(data_debug, F_g, em_tip, Nf);
title('Auxiliary view');
view(45, 25);
% xlim(xlim_all); ylim(ylim_all); zlim(zlim_all);

subplot(2,2,3);
plot_pivot_scene(data_debug, F_g, em_tip, Nf);
title('Front view');
view(0, 0);
xlim(xlim_all); ylim(ylim_all); zlim(zlim_all);

subplot(2,2,4);
plot_pivot_scene(data_debug, F_g, em_tip, Nf);
title('Right view');
view(90, 0);
% xlim(xlim_all); ylim(ylim_all); zlim(zlim_all);

figure(2); clf; hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('F_{g}*p_{tip} vs p_{post}');
view(45, 25);

ax = gca;
ax.FontSize = 15;
ax.LineWidth = 1.2;

for i = 1:Nf
    c = frame_colors(i,:);

    if i == 1

        plot3(tip_world(1,i), tip_world(2,i), tip_world(3,i), 'o', ...
            'Color', [28/255, 49/255, 41/255], ...
            'MarkerFaceColor', [28/255, 49/255, 41/255], ...
            'MarkerSize', 6, ...
            'DisplayName', 'F_{g} p_{tip}');

        plot3([tip_world(1,i), em_post(1)], ...
              [tip_world(2,i), em_post(2)], ...
              [tip_world(3,i), em_post(3)], ...
              '--', 'Color', [28/255, 49/255, 41/255], 'LineWidth', 0.8, ...
              'DisplayName', 'error');
    else
        % legend 제외
        plot3(tip_world(1,i), tip_world(2,i), tip_world(3,i), 'o', ...
            'Color', [28/255, 49/255, 41/255], ...
            'MarkerFaceColor', [28/255, 49/255, 41/255], ...
            'MarkerSize', 6, ...
            'HandleVisibility', 'off');

        plot3([tip_world(1,i), em_post(1)], ...
              [tip_world(2,i), em_post(2)], ...
              [tip_world(3,i), em_post(3)], ...
              '--', 'Color', [28/255, 49/255, 41/255], 'LineWidth', 0.8, ...
              'HandleVisibility', 'off');
    end
end

% pivot point
plot3(em_post(1), em_post(2), em_post(3), 'ok', ...
    'MarkerSize', 10, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'p_{post}');

legend('Location', 'best');
hold off;

%% Helpers

function plot_pivot_scene(data_debug, F_g, em_tip, Nf)
    hold on; grid on; axis padded
    xlabel('X'); ylabel('Y'); zlabel('Z');

    ax = gca;
    ax.FontSize = 18;
    ax.LineWidth = 1.2;

    colors = lines(Nf);

    for i = 1:Nf
        c = colors(i,:);

        Gi = data_debug.empivot.G(:,:,i);

        if size(Gi,2) >= 4
            try
                K = convhull(Gi(1,:)', Gi(2,:)', Gi(3,:)');
                trisurf(K, Gi(1,:)', Gi(2,:)', Gi(3,:)', ...
                    'FaceColor', c, ...
                    'FaceAlpha', 0.3, ...
                    'EdgeColor', c, ...
                    'EdgeAlpha', 0.0, ...
                    'LineWidth', 0.5, ...
                    'HandleVisibility', 'off');
            catch
            end
        end

        plot3(Gi(1,:), Gi(2,:), Gi(3,:), 'o', ...
            'Color', c, ...
            'MarkerFaceColor', c, ...
            'MarkerSize', 2, ...
            'HandleVisibility', 'off');

        o = F_g(1:3,4,i);
        plot3(o(1), o(2), o(3), '*', ...
            'Color', c, ...
            'MarkerSize', 7, ...
            'LineWidth', 1.2, ...
            'HandleVisibility', 'off');

        v = F_g(1:3,1:3,i) * em_tip;
        t = o + v;

        plot3(t(1), t(2), t(3), '.', ...
            'Color', c, ...
            'MarkerSize', 10, ...
            'HandleVisibility', 'off');

        quiver3(o(1), o(2), o(3), ...
            v(1), v(2), v(3), ...
            0, ...
            'Color', c, ...
            'LineWidth', 1, ...
            'MaxHeadSize', 0.0, ...
            'HandleVisibility', 'off');
    end


    hold off
end