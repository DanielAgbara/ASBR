function fig = visualize_pivot_calibration_frame(data_debug, pivot_out, em_frame_idx, opt_frame_idx)
% Combined pivot calibration visualization
%
% Figure layout:
%   - Main plot: Optical frame, EM frame, optical probe frame, EM probe frame
%   - Subplot 1: Optical-frame view
%   - Subplot 2: EM-frame view
%
% Conventions:
%   Optical frame is the global plotting reference for the main plot.
%   EM frame is placed in optical coordinates using F_d_opt(:,:,opt_frame_idx).
%   Optical probe frame is placed in optical coordinates.
%   EM probe frame is placed in EM coordinates, then mapped into optical for the main plot.

    if nargin < 3 || isempty(em_frame_idx)
        em_frame_idx = 1;
    end
    if nargin < 4 || isempty(opt_frame_idx)
        opt_frame_idx = 1;
    end

    em_frame_idx = max(1, min(em_frame_idx, data_debug.empivot.Nf));
    opt_frame_idx = max(1, min(opt_frame_idx, data_debug.optpivot.Nf));

    %% ============================================================
    % Optical-pivot data
    % ============================================================

    % EM frame in optical coordinates
    F_D = pivot_out.F_d_opt(:, :, opt_frame_idx);   % EM -> optical
    R_opt_from_em = F_D(1:3,1:3);
    p_opt_from_em = F_D(1:3,4);

    % Optical probe local frame in EM coordinates
    F_H = pivot_out.F_h(:, :, opt_frame_idx);

    % Optical probe frame in optical coordinates
    T_opt_from_optprobe = F_D * F_H;
    R_opt_from_optprobe = T_opt_from_optprobe(1:3,1:3);
    p_opt_from_optprobe = T_opt_from_optprobe(1:3,4);

    % Observed optical probe markers H_j in optical coordinates
    H_obs_opt = data_debug.optpivot.H(:, :, opt_frame_idx).';      % NH x 3

    % Predicted optical probe markers from local h, mapped to optical
    h_in_em = F_H(1:3,1:3) * pivot_out.h + F_H(1:3,4);             % 3 x NH
    h_in_opt = (R_opt_from_em * h_in_em + p_opt_from_em).';        % NH x 3

    % EM base markers D in optical coordinates
    D_obs_opt = data_debug.optpivot.D(:, :, opt_frame_idx).';      % ND x 3
    D_pred_opt = (R_opt_from_em * data_debug.calbody.d + p_opt_from_em).'; % ND x 3

    % Optical pivot tip/post in optical coordinates
    opt_tip_in_em = F_H(1:3,1:3) * pivot_out.opt_tip + F_H(1:3,4);
    opt_tip_in_opt = R_opt_from_em * opt_tip_in_em + p_opt_from_em;
    opt_post_in_opt = R_opt_from_em * pivot_out.opt_post + p_opt_from_em;

    %% ============================================================
    % EM-pivot data
    % ============================================================

    G_obs_em = data_debug.empivot.G(:, :, em_frame_idx).';         % NG x 3
    F_G = pivot_out.F_g(:, :, em_frame_idx);                       % EM probe -> EM

    R_em_from_emprobe = F_G(1:3,1:3);
    p_em_from_emprobe = F_G(1:3,4);

    % EM probe frame mapped into optical for main plot
    R_opt_from_emprobe = R_opt_from_em * R_em_from_emprobe;
    p_opt_from_emprobe = R_opt_from_em * p_em_from_emprobe + p_opt_from_em;

    % EM pivot markers mapped into optical for main plot
    G_obs_opt = (R_opt_from_em * data_debug.empivot.G(:, :, em_frame_idx) + p_opt_from_em).'; % NG x 3
    g_in_em = F_G(1:3,1:3) * pivot_out.g + F_G(1:3,4);             % 3 x NG
    g_in_opt = (R_opt_from_em * g_in_em + p_opt_from_em).';        % NG x 3

    % EM pivot tip/post
    em_tip_in_em = F_G(1:3,1:3) * pivot_out.em_tip + F_G(1:3,4);
    em_tip_in_opt = R_opt_from_em * em_tip_in_em + p_opt_from_em;
    em_post_in_opt = R_opt_from_em * pivot_out.em_post + p_opt_from_em;

    %% ============================================================
    % Shared axis limits
    % ============================================================

    all_pts = [ ...
        H_obs_opt; h_in_opt; ...
        D_obs_opt; D_pred_opt; ...
        opt_tip_in_opt.'; opt_post_in_opt.'; ...
        G_obs_opt; g_in_opt; ...
        em_tip_in_opt.'; em_post_in_opt.'; ...
        [0 0 0]; ...
        p_opt_from_em.'; ...
        p_opt_from_optprobe.'; ...
        p_opt_from_emprobe.' ...
    ];

    xmin = min(all_pts(:,1)); xmax = max(all_pts(:,1));
    ymin = min(all_pts(:,2)); ymax = max(all_pts(:,2));
    zmin = min(all_pts(:,3)); zmax = max(all_pts(:,3));

    xmid = (xmin + xmax) / 2;
    ymid = (ymin + ymax) / 2;
    zmid = (zmin + zmax) / 2;

    r = max([xmax - xmin, ymax - ymin, zmax - zmin]) * 0.60;
    if r == 0
        r = 1.0;
    end

    xlim_global = [xmid - r, xmid + r];
    ylim_global = [ymid - r, ymid + r];
    zlim_global = [zmid - r, zmid + r];

    L = max(10, 0.15 * max([xmax - xmin, ymax - ymin, zmax - zmin]));

    %% ============================================================
    % Figure layout
    % ============================================================

    fig = figure( ...
        'Color', 'w', ...
        'Name', sprintf('Pivot Calibration | EM frame %d | OPT frame %d', ...
        em_frame_idx, opt_frame_idx));

    tl = tiledlayout(fig, 2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    %% ============================================================
    % Main plot: all frames in optical coordinates
    % ============================================================
    ax1 = nexttile(tl, [2 1]);
    hold(ax1, 'on');
    grid(ax1, 'on');
    view(ax1, 35, 22);

    % Frames
    draw_frame_clean(ax1, [0;0;0], eye(3), L, 'Optical');
    draw_frame_clean(ax1, p_opt_from_em, R_opt_from_em, L, 'EM');
    draw_frame_clean(ax1, p_opt_from_optprobe, R_opt_from_optprobe, L, 'Opt Probe');
    draw_frame_clean(ax1, p_opt_from_emprobe, R_opt_from_emprobe, L, 'EM Probe');

    % Optical pivot scene
    plot3(ax1, D_obs_opt(:,1), D_obs_opt(:,2), D_obs_opt(:,3), ...
        'o', 'MarkerSize', 7, ...
        'MarkerFaceColor', [0.00 0.45 0.74], ...
        'MarkerEdgeColor', [0.00 0.45 0.74], ...
        'DisplayName', 'D_j');

    plot3(ax1, D_pred_opt(:,1), D_pred_opt(:,2), D_pred_opt(:,3), ...
        '+', 'MarkerSize', 9, 'LineWidth', 1.5, ...
        'Color', [0.00 0.45 0.74], ...
        'DisplayName', 'F_D d_j');

    plot3(ax1, H_obs_opt(:,1), H_obs_opt(:,2), H_obs_opt(:,3), ...
        'o', 'MarkerSize', 7, ...
        'MarkerFaceColor', [0.85 0.33 0.10], ...
        'MarkerEdgeColor', [0.85 0.33 0.10], ...
        'DisplayName', 'H_j');

    plot3(ax1, h_in_opt(:,1), h_in_opt(:,2), h_in_opt(:,3), ...
        '+', 'MarkerSize', 9, 'LineWidth', 1.5, ...
        'Color', [0.85 0.33 0.10], ...
        'DisplayName', 'F_H h_j');

    % EM pivot scene
    plot3(ax1, G_obs_opt(:,1), G_obs_opt(:,2), G_obs_opt(:,3), ...
        'o', 'MarkerSize', 7, ...
        'MarkerFaceColor', [0.20 0.65 0.20], ...
        'MarkerEdgeColor', [0.20 0.65 0.20], ...
        'DisplayName', 'G_j');

    plot3(ax1, g_in_opt(:,1), g_in_opt(:,2), g_in_opt(:,3), ...
        '+', 'MarkerSize', 9, 'LineWidth', 1.5, ...
        'Color', [0.20 0.65 0.20], ...
        'DisplayName', 'F_G g_j');

    % Posts and tips
    plot3(ax1, opt_post_in_opt(1), opt_post_in_opt(2), opt_post_in_opt(3), ...
        'p', 'MarkerSize', 12, ...
        'MarkerFaceColor', [0.49 0.18 0.56], ...
        'MarkerEdgeColor', [0.49 0.18 0.56], ...
        'DisplayName', 'Opt b_{post}');

    plot3(ax1, opt_tip_in_opt(1), opt_tip_in_opt(2), opt_tip_in_opt(3), ...
        'd', 'MarkerSize', 10, ...
        'MarkerFaceColor', [0.20 0.20 0.20], ...
        'MarkerEdgeColor', [0.20 0.20 0.20], ...
        'DisplayName', 'Opt tip');

    plot3(ax1, em_post_in_opt(1), em_post_in_opt(2), em_post_in_opt(3), ...
        'p', 'MarkerSize', 12, ...
        'MarkerFaceColor', [0.80 0.20 0.20], ...
        'MarkerEdgeColor', [0.80 0.20 0.20], ...
        'DisplayName', 'EM b_{post}');

    plot3(ax1, em_tip_in_opt(1), em_tip_in_opt(2), em_tip_in_opt(3), ...
        'd', 'MarkerSize', 10, ...
        'MarkerFaceColor', [0.10 0.10 0.10], ...
        'MarkerEdgeColor', [0.10 0.10 0.10], ...
        'DisplayName', 'EM tip');

    % Tip vectors
    quiver3(ax1, p_opt_from_optprobe(1), p_opt_from_optprobe(2), p_opt_from_optprobe(3), ...
        opt_tip_in_opt(1)-p_opt_from_optprobe(1), ...
        opt_tip_in_opt(2)-p_opt_from_optprobe(2), ...
        opt_tip_in_opt(3)-p_opt_from_optprobe(3), ...
        0, 'k', 'LineWidth', 2, 'MaxHeadSize', 0.2, ...
        'DisplayName', 'Opt b_{tip}');

    quiver3(ax1, p_opt_from_emprobe(1), p_opt_from_emprobe(2), p_opt_from_emprobe(3), ...
        em_tip_in_opt(1)-p_opt_from_emprobe(1), ...
        em_tip_in_opt(2)-p_opt_from_emprobe(2), ...
        em_tip_in_opt(3)-p_opt_from_emprobe(3), ...
        0, 'Color', [0.4 0.1 0.1], 'LineWidth', 2, 'MaxHeadSize', 0.2, ...
        'DisplayName', 'EM b_{tip}');

    xlabel(ax1, 'X');
    ylabel(ax1, 'Y');
    zlabel(ax1, 'Z');
    title(ax1, sprintf('Combined pivot scene | EM %d | OPT %d', em_frame_idx, opt_frame_idx));

    xlim(ax1, xlim_global);
    ylim(ax1, ylim_global);
    zlim(ax1, zlim_global);
    axis(ax1, 'equal');
    axis(ax1, 'vis3d');
    legend(ax1, 'Location', 'bestoutside');

    %% ============================================================
    % Subplot 2: optical-frame view
    % ============================================================
    ax2 = nexttile(tl, 2);
    plot_opt_frame_subplot(ax2, data_debug, pivot_out, opt_frame_idx, ...
        xlim_global, ylim_global, zlim_global, L);

    %% ============================================================
    % Subplot 3: EM-frame view
    % ============================================================
    ax3 = nexttile(tl, 4);
    plot_em_frame_subplot(ax3, data_debug, pivot_out, em_frame_idx, L);
end


function plot_opt_frame_subplot(ax, data_debug, pivot_out, frame_idx, ...
    xlim_global, ylim_global, zlim_global, L)

    H_obs = data_debug.optpivot.H(:, :, frame_idx).';
    F_D = pivot_out.F_d_opt(:, :, frame_idx);
    F_H = pivot_out.F_h(:, :, frame_idx);

    R_opt_from_em = F_D(1:3,1:3);
    p_opt_from_em = F_D(1:3,4);

    T_opt_from_probe = F_D * F_H;
    R_opt_from_probe = T_opt_from_probe(1:3,1:3);
    p_opt_from_probe = T_opt_from_probe(1:3,4);

    h_in_em = F_H(1:3,1:3) * pivot_out.h + F_H(1:3,4);
    h_in_opt = (R_opt_from_em * h_in_em + p_opt_from_em).';

    tip_in_em = F_H(1:3,1:3) * pivot_out.opt_tip + F_H(1:3,4);
    tip_in_opt = R_opt_from_em * tip_in_em + p_opt_from_em;
    post_in_opt = R_opt_from_em * pivot_out.opt_post + p_opt_from_em;

    hold(ax, 'on');
    grid(ax, 'on');
    view(ax, 35, 22);

    draw_frame_clean(ax, [0;0;0], eye(3), L, 'Optical');
    draw_frame_clean(ax, p_opt_from_em, R_opt_from_em, L, 'EM');
    draw_frame_clean(ax, p_opt_from_probe, R_opt_from_probe, L, 'Opt Probe');

    plot3(ax, H_obs(:,1), H_obs(:,2), H_obs(:,3), ...
        'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', [0.85 0.33 0.10], ...
        'MarkerEdgeColor', [0.85 0.33 0.10], ...
        'DisplayName', 'H_j');

    plot3(ax, h_in_opt(:,1), h_in_opt(:,2), h_in_opt(:,3), ...
        '+', 'MarkerSize', 8, 'LineWidth', 1.4, ...
        'Color', [0.00 0.45 0.74], ...
        'DisplayName', 'F_H h_j');

    plot3(ax, post_in_opt(1), post_in_opt(2), post_in_opt(3), ...
        'p', 'MarkerSize', 10, ...
        'MarkerFaceColor', [0.49 0.18 0.56], ...
        'MarkerEdgeColor', [0.49 0.18 0.56], ...
        'DisplayName', 'b_{post}');

    plot3(ax, tip_in_opt(1), tip_in_opt(2), tip_in_opt(3), ...
        'd', 'MarkerSize', 8, ...
        'MarkerFaceColor', [0.20 0.20 0.20], ...
        'MarkerEdgeColor', [0.20 0.20 0.20], ...
        'DisplayName', 'tip');

    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    title(ax, sprintf('Optical-frame view | frame %d', frame_idx));

    xlim(ax, xlim_global);
    ylim(ax, ylim_global);
    zlim(ax, zlim_global);
    axis(ax, 'equal');
    axis(ax, 'vis3d');
    legend(ax, 'Location', 'best');
end


function plot_em_frame_subplot(ax, data_debug, pivot_out, frame_idx, L)

    G_obs = data_debug.empivot.G(:, :, frame_idx).';
    T = pivot_out.F_g(:, :, frame_idx);

    R = T(1:3,1:3);
    p = T(1:3,4);

    g_in_em = (R * pivot_out.g + p).';
    tip_in_em = R * pivot_out.em_tip + p;

    pts = [G_obs; g_in_em; pivot_out.em_post.'; tip_in_em.'];
    xmin = min(pts(:,1)); xmax = max(pts(:,1));
    ymin = min(pts(:,2)); ymax = max(pts(:,2));
    zmin = min(pts(:,3)); zmax = max(pts(:,3));
    xmid = (xmin + xmax)/2;
    ymid = (ymin + ymax)/2;
    zmid = (zmin + zmax)/2;
    r = max([xmax-xmin, ymax-ymin, zmax-zmin]) * 0.60;
    if r == 0
        r = 1;
    end

    hold(ax, 'on');
    grid(ax, 'on');
    view(ax, 35, 22);

    draw_frame_clean(ax, [0;0;0], eye(3), L, 'EM');
    draw_frame_clean(ax, p, R, L, 'EM Probe');

    plot3(ax, G_obs(:,1), G_obs(:,2), G_obs(:,3), ...
        'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', [0.20 0.65 0.20], ...
        'MarkerEdgeColor', [0.20 0.65 0.20], ...
        'DisplayName', 'G_j');

    plot3(ax, g_in_em(:,1), g_in_em(:,2), g_in_em(:,3), ...
        '+', 'MarkerSize', 8, 'LineWidth', 1.4, ...
        'Color', [0.85 0.33 0.10], ...
        'DisplayName', 'F_G g_j');

    plot3(ax, pivot_out.em_post(1), pivot_out.em_post(2), pivot_out.em_post(3), ...
        'p', 'MarkerSize', 10, ...
        'MarkerFaceColor', [0.80 0.20 0.20], ...
        'MarkerEdgeColor', [0.80 0.20 0.20], ...
        'DisplayName', 'b_{post}');

    plot3(ax, tip_in_em(1), tip_in_em(2), tip_in_em(3), ...
        'd', 'MarkerSize', 8, ...
        'MarkerFaceColor', [0.10 0.10 0.10], ...
        'MarkerEdgeColor', [0.10 0.10 0.10], ...
        'DisplayName', 'tip');

    xlabel(ax, 'X');
    ylabel(ax, 'Y');
    zlabel(ax, 'Z');
    title(ax, sprintf('EM-frame view | frame %d', frame_idx));

    xlim(ax, [xmid-r, xmid+r]);
    ylim(ax, [ymid-r, ymid+r]);
    zlim(ax, [zmid-r, zmid+r]);
    axis(ax, 'equal');
    axis(ax, 'vis3d');
    legend(ax, 'Location', 'best');
end