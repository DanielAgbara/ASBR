clc; clear;

[data_debug, ~] = loadDebugData(1);

max_iter = 100;
tol = 1e-6;

%% registration + optical pivot calibration
H_new = zeros(size(data_debug.optpivot.H));

for i = 1:data_debug.optpivot.Nf
    F_d(:, :, i) = iterative_closest_point( ...
        data_debug.calbody.d, ...
        data_debug.optpivot.D(:, :, i), ...
        max_iter, tol);

    H_new(:, :, i) = F_d(1:3, 1:3, i) \ data_debug.optpivot.H(:, :, i) ...
                   - F_d(1:3, 1:3, i) \ F_d(1:3, 4, i);
end

H0 = mean(H_new(:, :, 1), 2);
h  = H_new(:, :, 1) - H0;

Nf = data_debug.optpivot.Nf;
Nh = size(H_new, 2);

F_h = zeros(4, 4, Nf);
for i = 1:Nf
    F_h(:, :, i) = iterative_closest_point(h, H_new(:, :, i), max_iter, tol);
end

[opt_tip, opt_post] = pivot_calibration(F_h);

%% precompute points for plotting
frame_colors = lines(Nf);

H0_world      = zeros(3, Nf);      % transformed local origin
tip_world     = zeros(3, Nf);      % F_h * opt_tip
h_world_all   = zeros(3, Nh, Nf);  % transformed template points

for i = 1:Nf
    T = F_h(:, :, i);
    R = T(1:3, 1:3);
    p = T(1:3, 4);

    % transformed local origin
    H0_world(:, i) = p;

    % transformed tip
    tip_world(:, i) = R * opt_tip + p;

    % transformed template points
    h_world_all(:, :, i) = R * h + p;
end

%% quantitative check
tip_err = vecnorm(tip_world - opt_post, 2, 1);

% metrics
mean_abs_err = mean(tip_err);
rms_err      = sqrt(mean(tip_err.^2));
max_abs_err  = max(tip_err);

fprintf('Pivot calibration check:\n');
fprintf('Mean absolute error = %.4f\n', mean_abs_err);
fprintf('RMS error           = %.4f\n', rms_err);
fprintf('Max absolute error  = %.4f\n', max_abs_err);


%% plot limits
all_pts = reshape(H_new, 3, []);
valid = ~any(isnan(all_pts), 1);
all_pts = all_pts(:, valid);

xmin = min(all_pts(1,:)); xmax = max(all_pts(1,:));
ymin = min(all_pts(2,:)); ymax = max(all_pts(2,:));
zmin = min(all_pts(3,:)); zmax = max(all_pts(3,:));

mx = 0.05 * (xmax - xmin + eps);
my = 0.05 * (ymax - ymin + eps);
mz = 0.05 * (zmax - zmin + eps);

xlim_all = [xmin-mx, xmax+mx];
ylim_all = [ymin-my, ymax+my];
zlim_all = [zmin-mz, zmax+mz];

%% figure 1 : multi-view optical pivot scene
figure(1); clf;

subplot(2,2,1);
plot_optical_pivot_scene(H_new, F_h, opt_tip, Nf);
title('Top view');
view(0, 90);
xlim(xlim_all); ylim(ylim_all); zlim(zlim_all);

subplot(2,2,2);
plot_optical_pivot_scene(H_new, F_h, opt_tip, Nf);
title('Auxiliary view');
view(45, 25);
% xlim(xlim_all); ylim(ylim_all); zlim(zlim_all);

subplot(2,2,3);
plot_optical_pivot_scene(H_new, F_h, opt_tip, Nf);
title('Front view');
view(0, 0);
xlim(xlim_all); ylim(ylim_all); zlim(zlim_all);

subplot(2,2,4);
plot_optical_pivot_scene(H_new, F_h, opt_tip, Nf);
title('Right view');
view(90, 0);
% xlim(xlim_all); ylim(ylim_all); zlim(zlim_all);

%% figure 2 : transformed tip vs pivot point
figure(2); clf; hold on; grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('F_{h}*p_{tip} vs p_{post}');
view(45, 25);

ax = gca;
ax.FontSize = 15;
ax.LineWidth = 1.2;

for i = 1:Nf
    if i == 1
        plot3(tip_world(1,i), tip_world(2,i), tip_world(3,i), 'o', ...
            'Color', [28/255, 49/255, 41/255], ...
            'MarkerFaceColor', [28/255, 49/255, 41/255], ...
            'MarkerSize', 6, ...
            'DisplayName', 'F_{h} p_{tip}');

        plot3([tip_world(1,i), opt_post(1)], ...
              [tip_world(2,i), opt_post(2)], ...
              [tip_world(3,i), opt_post(3)], ...
              '--', 'Color', [28/255, 49/255, 41/255], 'LineWidth', 0.8, ...
              'DisplayName', 'error');
    else
        plot3(tip_world(1,i), tip_world(2,i), tip_world(3,i), 'o', ...
            'Color', [28/255, 49/255, 41/255], ...
            'MarkerFaceColor', [28/255, 49/255, 41/255], ...
            'MarkerSize', 6, ...
            'HandleVisibility', 'off');

        plot3([tip_world(1,i), opt_post(1)], ...
              [tip_world(2,i), opt_post(2)], ...
              [tip_world(3,i), opt_post(3)], ...
              '--', 'Color', [28/255, 49/255, 41/255], 'LineWidth', 0.8, ...
              'HandleVisibility', 'off');
    end
end

plot3(opt_post(1), opt_post(2), opt_post(3), 'ok', ...
    'MarkerSize', 10, ...
    'LineWidth', 1.5, ...
    'DisplayName', 'p_{post}');

legend('Location', 'best');
hold off;

%% Helpers

function plot_optical_pivot_scene(H_new, F_h, opt_tip, Nf)
    hold on; grid on; axis padded
    xlabel('X'); ylabel('Y'); zlabel('Z');

    ax = gca;
    ax.FontSize = 18;
    ax.LineWidth = 1.2;

    colors = lines(Nf);

    for i = 1:Nf
        c = colors(i,:);

        Hi = H_new(:, :, i);

        if size(Hi,2) >= 4
            try
                K = convhull(Hi(1,:)', Hi(2,:)', Hi(3,:)');
                trisurf(K, Hi(1,:)', Hi(2,:)', Hi(3,:)', ...
                    'FaceColor', c, ...
                    'FaceAlpha', 0.3, ...
                    'EdgeColor', c, ...
                    'EdgeAlpha', 0.0, ...
                    'LineWidth', 0.5, ...
                    'HandleVisibility', 'off');
            catch
            end
        end

        plot3(Hi(1,:), Hi(2,:), Hi(3,:), 'o', ...
            'Color', c, ...
            'MarkerFaceColor', c, ...
            'MarkerSize', 2, ...
            'HandleVisibility', 'off');

        o = F_h(1:3,4,i);
        plot3(o(1), o(2), o(3), '*', ...
            'Color', c, ...
            'MarkerSize', 7, ...
            'LineWidth', 1.2, ...
            'HandleVisibility', 'off');

        v = F_h(1:3,1:3,i) * opt_tip;
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