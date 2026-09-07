function result = visualizeEyeInHandChain( ...
    q_robot_nf, q_camera_nf, t_robot_nf, t_camera_nf, X_nf, ...
    q_robot_ny, q_camera_ny, t_robot_ny, t_camera_ny, X_ny)

    % =========================
    % compute both
    % =========================
    data_nf = compute_chain(q_robot_nf, q_camera_nf, t_robot_nf, t_camera_nf, X_nf);
    data_ny = compute_chain(q_robot_ny, q_camera_ny, t_robot_ny, t_camera_ny, X_ny);

    % ============================================================
    % Figure 1: subplot (noise-free vs noisy)
    % ============================================================
    figure(1); clf
    set(gcf, 'Units', 'normalized', 'Position', [0.05 0.08 0.9 0.75]);

    % ---------------- LEFT: noise-free ----------------
    subplot(1,2,1)
    draw_chain_subplot(data_nf, 'noise-free');

    % ---------------- RIGHT: noisy ----------------
    subplot(1,2,2)
    draw_chain_subplot(data_ny, 'noisy');

    % ============================================================
    % Figure 2: compare p_o
    % ============================================================
    figure(2); clf
    set(gcf, 'Units', 'normalized', 'Position', [0.1 0.1 0.65 0.7]);
    
    ax = gca;
    ax.FontSize = 20;
    ax.LineWidth = 1.0;
    ax.Clipping = 'off';
    
    hold on; grid on; axis equal;
    view([0.05, -0.3, 0.1]);
    rotate3d on;
    axis vis3d;
    
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('Object origin comparison');
    
    h_nf = plot3(data_nf.p_o(1,:), data_nf.p_o(2,:), data_nf.p_o(3,:), '.', ...
        'MarkerSize', 18, 'DisplayName', 'noise-free');
    
    h_ny = plot3(data_ny.p_o(1,:), data_ny.p_o(2,:), data_ny.p_o(3,:), '.', ...
        'MarkerSize', 18, 'DisplayName', 'noisy');
    
    h_nf_mean = plot3(data_nf.p_o_mean(1), data_nf.p_o_mean(2), data_nf.p_o_mean(3), 'o', ...
        'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'nf mean');
    
    h_ny_mean = plot3(data_ny.p_o_mean(1), data_ny.p_o_mean(2), data_ny.p_o_mean(3), 's', ...
        'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'ny mean');
    
    h_ell_nf = plot_single_ellipsoid(data_nf.p_o.', [0 0.4470 0.7410], 2);
    h_ell_ny = plot_single_ellipsoid(data_ny.p_o.', [0.8500 0.3250 0.0980], 2);
    
    legend('Location', 'best');

    % =========================
    % return
    % =========================
    result.noise_free = data_nf;
    result.noisy = data_ny;
end


function data = compute_chain(q_robot, q_camera, t_robot, t_camera, X)

    N = size(q_robot, 2);

    T_bg = zeros(4,4,N);   % ^bT_g
    T_co = zeros(4,4,N);   % ^cT_o
    T_bc = zeros(4,4,N);   % ^bT_c
    T_bo = zeros(4,4,N);   % ^bT_o

    p_b = zeros(3,1);      %#ok<NASGU>
    p_g_all = zeros(3,N);
    p_c_all = zeros(3,N);
    p_o_all = zeros(3,N);

    for i = 1:N
        R_bg = quaternionToR(q_robot(:,i));
        R_co = quaternionToR(q_camera(:,i));

        T_bg(:,:,i) = eye(4);
        T_bg(1:3,1:3,i) = R_bg;
        T_bg(1:3,4,i)   = t_robot(:,i);

        T_co(:,:,i) = eye(4);
        T_co(1:3,1:3,i) = R_co;
        T_co(1:3,4,i)   = t_camera(:,i);

        T_bc(:,:,i) = T_bg(:,:,i) * X;
        T_bo(:,:,i) = T_bc(:,:,i) * T_co(:,:,i);

        p_g_all(:,i) = T_bg(1:3,4,i);
        p_c_all(:,i) = T_bc(1:3,4,i);
        p_o_all(:,i) = T_bo(1:3,4,i);
    end

    p_o_mean = mean(p_o_all, 2);
    pos_err = vecnorm(p_o_all - p_o_mean, 2, 1);

    rot_err_deg = zeros(1,N);
    R_ref = T_bo(1:3,1:3,1);

    for i = 1:N
        R_i = T_bo(1:3,1:3,i);
        R_rel = R_ref' * R_i;
        c = (trace(R_rel) - 1) / 2;
        c = max(-1, min(1, c));
        rot_err_deg(i) = rad2deg(acos(c));
    end

    data.T_bg = T_bg;
    data.T_gc = X;
    data.T_co = T_co;
    data.T_bc = T_bc;
    data.T_bo = T_bo;

    data.p_g = p_g_all;
    data.p_c = p_c_all;
    data.p_o = p_o_all;
    data.p_o_mean = p_o_mean;

    data.pos_err = pos_err;
    data.rot_err_deg = rot_err_deg;
end


function draw_chain_subplot(data, title_str)

    ax = gca;
    ax.FontSize = 18;
    ax.LineWidth = 1.0;
    ax.Clipping = 'off';

    hold on; grid on; axis equal;
    view([0.05, -0.3, 0.1]);
    axis vis3d;

    xlabel('X'); ylabel('Y'); zlabel('Z');
    title(title_str);

    color_gb = [0 0.4470 0.7410];
    color_cg = [0.8500 0.3250 0.0980];
    color_oc = [0.4660 0.6740 0.1880];

    N = size(data.p_g, 2);
    p_b = zeros(3,1);

    for i = 1:N
        R_bg = data.T_bg(1:3,1:3,i);
        R_bc = data.T_bc(1:3,1:3,i);

        p_g = data.p_g(:,i);
        p_c = data.p_c(:,i);
        p_o = data.p_o(:,i);

        v_gb = p_b - p_g;
        v_cg = -R_bg * data.T_gc(1:3,4);
        v_oc = -R_bc * data.T_co(1:3,4,i);

        quiver3(p_g(1), p_g(2), p_g(3), v_gb(1), v_gb(2), v_gb(3), 0, ...
            'Color', color_gb, 'LineWidth', 1);

        quiver3(p_c(1), p_c(2), p_c(3), v_cg(1), v_cg(2), v_cg(3), 0, ...
            'Color', color_cg, 'LineWidth', 1);

        quiver3(p_o(1), p_o(2), p_o(3), v_oc(1), v_oc(2), v_oc(3), 0, ...
            'Color', color_oc, 'LineWidth', 1);

        plot3(p_g(1), p_g(2), p_g(3), '.', 'Color', color_gb, 'MarkerSize', 16);
        plot3(p_c(1), p_c(2), p_c(3), '.', 'Color', color_cg, 'MarkerSize', 16);
        plot3(p_o(1), p_o(2), p_o(3), '.', 'Color', color_oc, 'MarkerSize', 12);
    end

    plot3(0,0,0,'k.','MarkerSize',20);

    plot_single_ellipsoid(data.p_o.', [0 0.6 0], 2);
end


function h = plot_single_ellipsoid(X, color, k)
% plot_single_ellipsoid
% X : Mx3, each row is one 3D point
% color : RGB or short color char
% k : sigma scale (e.g., 2 for 2-sigma)

    if nargin < 3
        k = 2;
    end

    if size(X,2) ~= 3
        error('X must be M x 3');
    end

    if size(X,1) < 2
        h = gobjects(1);
        return;
    end

    mu = mean(X, 1);
    Sigma = cov(X);

    if any(~isfinite(Sigma), 'all')
        h = gobjects(1);
        return;
    end

    Sigma = (Sigma + Sigma.') / 2;
    Sigma = Sigma + 1e-12 * eye(3);

    [V, D] = eig(Sigma);
    lam = diag(D);

    if any(lam <= 0)
        h = gobjects(1);
        return;
    end

    radii = k * sqrt(lam);

    [xs, ys, zs] = sphere(30);
    sphere_pts = [xs(:)'; ys(:)'; zs(:)'];

    ellipsoid_pts = V * diag(radii) * sphere_pts + mu(:);

    xs_e = reshape(ellipsoid_pts(1,:), size(xs));
    ys_e = reshape(ellipsoid_pts(2,:), size(ys));
    zs_e = reshape(ellipsoid_pts(3,:), size(zs));

    h = surf(xs_e, ys_e, zs_e, ...
        'FaceColor', color, ...
        'FaceAlpha', 0.25, ...
        'EdgeColor', 'none');
end