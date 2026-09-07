clc; clear; close all;

[data_good, ~] = loadDebugData(1);
[data_bad,  ~] = loadDebugData(7);

% raw data: 3 x N x T
Dg = data_good.calreadings.D;
Ag = data_good.calreadings.A;
Cg = data_good.calreadings.C;

Db = data_bad.calreadings.D;
Ab = data_bad.calreadings.A;
Cb = data_bad.calreadings.C;

% each set independently centered frame-by-frame
Dg_rel = make_relative_per_frame(Dg);
Db_rel = make_relative_per_frame(Db);

Ag_rel = make_relative_per_frame(Ag);
Ab_rel = make_relative_per_frame(Ab);

Cg_rel = make_relative_per_frame(Cg);
Cb_rel = make_relative_per_frame(Cb);

% statistics
stats.D.good = compute_mean_variance_per_index(Dg_rel);
stats.D.bad  = compute_mean_variance_per_index(Db_rel);

stats.A.good = compute_mean_variance_per_index(Ag_rel);
stats.A.bad  = compute_mean_variance_per_index(Ab_rel);

stats.C.good = compute_mean_variance_per_index(Cg_rel);
stats.C.bad  = compute_mean_variance_per_index(Cb_rel);

figure('Color','w');

%% summary panel
subplot(2,2,1)
axis off

header1 = sprintf('%-1s | %6s %6s | %6s %6s\n', ...
    '', 'good', '', 'bad', '');

header2 = sprintf('%-1s | %6s %6s | %6s %6s\n', ...
    '', 'mean', 'max', 'mean', 'max');

sep = repmat('-', 1, 35);

row_D = sprintf('%-1s | %6.2f %6.2f | %6.2f %6.2f\n', ...
    'D', ...
    stats.D.good.mean_var, stats.D.good.max_var, ...
    stats.D.bad.mean_var,  stats.D.bad.max_var);

row_A = sprintf('%-1s | %6.2f %6.2f | %6.2f %6.2f\n', ...
    'A', ...
    stats.A.good.mean_var, stats.A.good.max_var, ...
    stats.A.bad.mean_var,  stats.A.bad.max_var);

row_C = sprintf('%-1s | %6.2f %6.2f | %6.2f %6.2f\n', ...
    'C', ...
    stats.C.good.mean_var, stats.C.good.max_var, ...
    stats.C.bad.mean_var,  stats.C.bad.max_var);

table_text = sprintf('%s%s%s\n%s%s%s', ...
    header1, header2, sep, ...
    row_D, row_A, row_C);

text(0.05, 0.9, table_text, ...
    'Units', 'normalized', ...
    'FontName', 'Courier', ...
    'FontSize', 11, ...
    'VerticalAlignment', 'top');

title('Variance Summary')

%% D
subplot(2,2,2)
h1 = plot3(squeeze(Dg_rel(1,:,:)), squeeze(Dg_rel(2,:,:)), squeeze(Dg_rel(3,:,:)), ...
    '.b', 'MarkerSize', 6);
hold on
h2 = plot3(squeeze(Db_rel(1,:,:)), squeeze(Db_rel(2,:,:)), squeeze(Db_rel(3,:,:)), ...
    '.r', 'MarkerSize', 6);

plot_ellipsoids_per_index(Dg_rel, 'b');
plot_ellipsoids_per_index(Db_rel, 'r');

title('D relative distribution')
grid on
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal
view(3)
legend([h1(1), h2(1)], {'good', 'bad'}, 'Location', 'best')
hold off


%% A
subplot(2,2,3)
h1 = plot3(squeeze(Ag_rel(1,:,:)), squeeze(Ag_rel(2,:,:)), squeeze(Ag_rel(3,:,:)), ...
    '.b', 'MarkerSize', 6);
hold on
h2 = plot3(squeeze(Ab_rel(1,:,:)), squeeze(Ab_rel(2,:,:)), squeeze(Ab_rel(3,:,:)), ...
    '.r', 'MarkerSize', 6);

plot_ellipsoids_per_index(Ag_rel, 'b');
plot_ellipsoids_per_index(Ab_rel, 'r');

title('A relative distribution')
grid on
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal
view(3)
legend([h1(1), h2(1)], {'good', 'bad'}, 'Location', 'best')
hold off


%% C
subplot(2,2,4)
h1 = plot3(squeeze(Cg_rel(1,:,:)), squeeze(Cg_rel(2,:,:)), squeeze(Cg_rel(3,:,:)), ...
    '.b', 'MarkerSize', 6);
hold on
h2 = plot3(squeeze(Cb_rel(1,:,:)), squeeze(Cb_rel(2,:,:)), squeeze(Cb_rel(3,:,:)), ...
    '.r', 'MarkerSize', 6);

plot_ellipsoids_per_index(Cg_rel, 'b');
plot_ellipsoids_per_index(Cb_rel, 'r');

title('C relative distribution')
grid on
xlabel('X'); ylabel('Y'); zlabel('Z');
axis equal
view(3)
legend([h1(1), h2(1)], {'good', 'bad'}, 'Location', 'best')
hold off
set(findall(gcf,'-property','FontSize'),'FontSize',15)

%% =========================
% local functions
% ==========================

function Xrel = make_relative_per_frame(Xraw)
    % Xraw: 3 x N x T
    [dim, ~, T] = size(Xraw);

    if dim ~= 3
        error('Input must have size 3 x N x T');
    end

    Xrel = nan(size(Xraw));

    for t = 1:T
        Xt = Xraw(:,:,t);   % 3 x N

        valid = ~any(isnan(Xt), 1);
        if ~any(valid)
            continue;
        end

        center = mean(Xt(:, valid), 2);   % 3 x 1
        Xrel(:,:,t) = Xt - center;
    end
end

function stats = compute_mean_variance_per_index(Xraw)
    % Xraw: 3 x N x T
    [dim, N, ~] = size(Xraw);

    if dim ~= 3
        error('Input must have size 3 x N x T');
    end

    var_list = nan(N,1);

    for i = 1:N
        X = squeeze(Xraw(:, i, :))';   % T x 3
        X = X(~any(isnan(X), 2), :);

        if size(X,1) < 2
            continue;
        end

        Sigma = cov(X);
        var_list(i) = trace(Sigma) / 3;   % scalar variance
    end

    valid = ~isnan(var_list);

    stats.var_per_index = var_list(valid);

    stats.mean_var = mean(stats.var_per_index);
    stats.max_var  = max(stats.var_per_index);
    stats.std_var  = std(stats.var_per_index);

    stats.n_valid  = sum(valid);
end

function plot_ellipsoids_per_index(Xraw, color)
    % Xraw: 3 x N x T
    [dim, N, ~] = size(Xraw);

    if dim ~= 3
        error('Input must have size 3 x N x T');
    end

    for i = 1:N
        X = squeeze(Xraw(:, i, :))';   % T x 3
        X = X(~any(isnan(X), 2), :);

        if size(X,1) < 4
            continue;
        end

        plot_single_ellipsoid(X, color);
    end
end


function plot_single_ellipsoid(X, color)
    % X: M x 3
    mu = mean(X, 1);
    Sigma = cov(X);

    if any(~isfinite(Sigma), 'all')
        return;
    end

    Sigma = (Sigma + Sigma.') / 2;
    Sigma = Sigma + 1e-12 * eye(3);

    [V, D] = eig(Sigma);
    lam = diag(D);

    if any(lam <= 0)
        return;
    end

    k = 2;  % 2-sigma
    radii = k * sqrt(lam);

    [xs, ys, zs] = sphere(20);
    sphere_pts = [xs(:)'; ys(:)'; zs(:)'];   % 3 x M

    ellipsoid_pts = V * diag(radii) * sphere_pts + mu(:);

    xs_e = reshape(ellipsoid_pts(1,:), size(xs));
    ys_e = reshape(ellipsoid_pts(2,:), size(ys));
    zs_e = reshape(ellipsoid_pts(3,:), size(zs));

    surf(xs_e, ys_e, zs_e, ...
        'FaceColor', color, ...
        'FaceAlpha', 0.12, ...
        'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
end