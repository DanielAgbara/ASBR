function fig = visualize_registration_frame(data, out, k)
%VISUALIZE_REGISTRATION_FRAME
% Per-frame registration visualization in the optical frame.
%
% Full scene:
%   - Optical frame at origin
%   - EM frame using F_D
%   - Object frame using F_A
%   - observed and predicted points for A, D, and C
%
% Subplots:
%   - D vs F_D d
%   - A vs F_A a
%   - F_D C vs F_A c

    if nargin < 3 || isempty(k)
        k = 1;
    end

    % ---------------------------------------------------------
    % Transforms
    % ---------------------------------------------------------
    F_A = out.F_a(:, :, k);   % object -> optical
    F_D = out.F_d(:, :, k);   % EM -> optical

    R_opt_from_obj = F_A(1:3,1:3);
    p_opt_from_obj = F_A(1:3,4);

    R_opt_from_em  = F_D(1:3,1:3);
    p_opt_from_em  = F_D(1:3,4);

    % ---------------------------------------------------------
    % Model points
    % ---------------------------------------------------------
    a_model = data.calbody.a;
    d_model = data.calbody.d;
    c_model = data.calbody.c;

    % ---------------------------------------------------------
    % Observed points
    % ---------------------------------------------------------
    A_obs_opt = data.calreadings.A(:, :, k).';
    D_obs_opt = data.calreadings.D(:, :, k).';

    C_obs_em  = data.calreadings.C(:, :, k);
    C_obs_opt = (R_opt_from_em * C_obs_em + p_opt_from_em).';

    % ---------------------------------------------------------
    % Predicted points
    % ---------------------------------------------------------
    A_pred_opt = (R_opt_from_obj * a_model + p_opt_from_obj).';
    D_pred_opt = (R_opt_from_em  * d_model + p_opt_from_em).';
    C_pred_opt = (R_opt_from_obj * c_model + p_opt_from_obj).';

    % ---------------------------------------------------------
    % Frame origins
    % ---------------------------------------------------------
    oOpt = [0; 0; 0];
    oEM  = p_opt_from_em;
    oObj = p_opt_from_obj;

    pts_scene = [A_obs_opt; D_obs_opt; C_obs_opt; ...
                 A_pred_opt; D_pred_opt; C_pred_opt; ...
                 oOpt.'; oEM.'; oObj.'];

    xyz_span = max(pts_scene, [], 1) - min(pts_scene, [], 1);
    L = max(10, 0.15 * max(xyz_span));

    % ---------------------------------------------------------
    % Colors
    % ---------------------------------------------------------
    % Observed
    color_A_obs = [0.20 0.20 0.20];   % dark gray
    color_D_obs = [0.35 0.35 0.35];   % medium gray
    color_C_obs = [0.55 0.55 0.55];   % light gray

    % Predicted (requested)
    color_A_pred = [0.00 0.45 0.74];  % blue    -> F_A a
    color_D_pred = [0.85 0.33 0.10];  % orange  -> F_D d
    color_C_pred = [0.93 0.69 0.13];  % yellow  -> F_A c

    color_residual = [0.55 0.55 0.55];

    % ---------------------------------------------------------
    % Figure
    % ---------------------------------------------------------
    fig = figure( ...
        'Color', 'w', ...
        'Name', sprintf('%s | Registration Frame %d', data.file_name, k));

    tl = tiledlayout(fig, 2, 2, ...
        'Padding', 'compact', ...
        'TileSpacing', 'compact');

    % =========================================================
    % FULL SCENE
    % =========================================================
    ax1 = nexttile(tl, 1);
    style_axes(ax1);
    hold(ax1, 'on');
    view(ax1, 35, 22);

    draw_frame_clean(ax1, oOpt, eye(3),        L, 'Optical');
    draw_frame_clean(ax1, oEM,  R_opt_from_em, L, 'EM');
    draw_frame_clean(ax1, oObj, R_opt_from_obj, L, 'Object');

    % Observed first
    plot3(ax1, A_obs_opt(:,1), A_obs_opt(:,2), A_obs_opt(:,3), ...
        'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', color_A_obs, ...
        'MarkerEdgeColor', 'k', ...
        'DisplayName', 'A');

    plot3(ax1, D_obs_opt(:,1), D_obs_opt(:,2), D_obs_opt(:,3), ...
        'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', color_D_obs, ...
        'MarkerEdgeColor', 'k', ...
        'DisplayName', 'D');

    plot3(ax1, C_obs_opt(:,1), C_obs_opt(:,2), C_obs_opt(:,3), ...
        'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', color_C_obs, ...
        'MarkerEdgeColor', 'k', ...
        'DisplayName', 'F_D C');

    % Residual lines
    draw_match_lines(ax1, A_pred_opt, A_obs_opt, color_residual);
    draw_match_lines(ax1, D_pred_opt, D_obs_opt, color_residual);
    draw_match_lines(ax1, C_pred_opt, C_obs_opt, color_residual);

    % Predicted last so X markers stay visible
    plot3(ax1, A_pred_opt(:,1), A_pred_opt(:,2), A_pred_opt(:,3), ...
        'x', 'MarkerSize', 11, 'LineWidth', 2.2, ...
        'Color', color_A_pred, ...
        'DisplayName', 'F_A a');

    plot3(ax1, D_pred_opt(:,1), D_pred_opt(:,2), D_pred_opt(:,3), ...
        'x', 'MarkerSize', 11, 'LineWidth', 2.2, ...
        'Color', color_D_pred, ...
        'DisplayName', 'F_D d');

    plot3(ax1, C_pred_opt(:,1), C_pred_opt(:,2), C_pred_opt(:,3), ...
        'x', 'MarkerSize', 11, 'LineWidth', 2.2, ...
        'Color', color_C_pred, ...
        'DisplayName', 'F_A c');

    xlabel(ax1, 'X');
    ylabel(ax1, 'Y');
    zlabel(ax1, 'Z');
    title(ax1, sprintf('Full scene in optical frame | Frame %d', k));
    set_equal_3d(ax1, pts_scene);
    daspect(ax1, [1 1 1]);
    legend(ax1, 'Location', 'bestoutside');

    % =========================================================
    % D vs F_D d
    % =========================================================
    ax2 = nexttile(tl, 2);
    plot_pair_subplot(ax2, D_obs_opt, D_pred_opt, ...
        'D vs F_D d', color_D_obs, color_D_pred, color_residual, ...
        'D', 'F_D d');

    % =========================================================
    % A vs F_A a
    % =========================================================
    ax3 = nexttile(tl, 3);
    plot_pair_subplot(ax3, A_obs_opt, A_pred_opt, ...
        'A vs F_A a', color_A_obs, color_A_pred, color_residual, ...
        'A', 'F_A a');

    % =========================================================
    % F_D C vs F_A c
    % =========================================================
    ax4 = nexttile(tl, 4);
    plot_pair_subplot(ax4, C_obs_opt, C_pred_opt, ...
        'F_D C vs F_A c', color_C_obs, color_C_pred, color_residual, ...
        'F_D C', 'F_A c');
end

function style_axes(ax)
    grid(ax, 'on');
    ax.FontSize = 15;
    ax.LineWidth = 1.2;
end

function draw_match_lines(ax, P1, P2, colorVal)
    for i = 1:size(P1, 1)
        plot3(ax, ...
            [P1(i,1), P2(i,1)], ...
            [P1(i,2), P2(i,2)], ...
            [P1(i,3), P2(i,3)], ...
            '--', ...
            'Color', colorVal, ...
            'LineWidth', 0.9, ...
            'HandleVisibility', 'off');
    end
end

function plot_pair_subplot(ax, obsPts, predPts, ttl, obsColor, predColor, lineColor, obsName, predName)

    style_axes(ax);
    hold(ax, 'on');
    view(ax, 35, 22);

    % Observed first
    plot3(ax, obsPts(:,1), obsPts(:,2), obsPts(:,3), ...
        'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', obsColor, ...
        'MarkerEdgeColor', 'k', ...
        'DisplayName', obsName);

    % Residuals
    for i = 1:size(obsPts, 1)
        plot3(ax, ...
            [predPts(i,1), obsPts(i,1)], ...
            [predPts(i,2), obsPts(i,2)], ...
            [predPts(i,3), obsPts(i,3)], ...
            '--', ...
            'Color', lineColor, ...
            'LineWidth', 0.9, ...
            'HandleVisibility', 'off');
    end

    % Predicted last
    plot3(ax, predPts(:,1), predPts(:,2), predPts(:,3), ...
        'x', 'MarkerSize', 11, 'LineWidth', 2.2, ...
        'Color', predColor, ...
        'DisplayName', predName);

    pts = [obsPts; predPts];
    set_equal_3d(ax, pts);
    daspect(ax, [1 1 1]);

    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    title(ax, ttl);
    legend(ax, 'Location', 'best');
end

function draw_frame_clean(ax, origin, R, L, name)

    if nargin < 3 || isempty(R)
        R = eye(3);
    end
    if nargin < 4 || isempty(L)
        L = 1;
    end
    if nargin < 5
        name = '';
    end

    origin = origin(:);
    colors = {'r', 'g', 'b'};

    hold(ax, 'on');

    for i = 1:3
        v = R(:,i) * L;
        quiver3(ax, ...
            origin(1), origin(2), origin(3), ...
            v(1), v(2), v(3), 0, ...
            'Color', colors{i}, ...
            'LineWidth', 2, ...
            'MaxHeadSize', 0.18, ...
            'HandleVisibility', 'off');
    end

    if ~isempty(name)
        text(ax, origin(1), origin(2), origin(3), ['  ' name], ...
            'FontWeight', 'bold', ...
            'FontSize', 11, ...
            'HandleVisibility', 'off');
    end
end