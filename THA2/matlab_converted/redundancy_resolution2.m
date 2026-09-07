function [theta_sol, theta_history, w_err_history, v_err_history] = redundancy_resolution2( ...
    M_ee, B_list, theta_init, T_sd, max_iters, tol_w, tol_v, ...
    q_min, q_max, k0, secondary_mode)

%{
Computes numerical inverse kinematics for end-effector pose
using the pseudo-inverse method with redundancy resolution.

Secondary objective modes:
    'manip_only'          -> manipulability maximization only
    'joint_limits_only'   -> joint-limit avoidance only
    'manip_and_limits'    -> both manipulability and joint-limit avoidance
    'none'                -> no secondary objective

Returns:
    theta_sol       final joint solution
    theta_history   joint history
    w_err_final     final angular error norm
    v_err_final     final linear error norm
    w_err_history   angular error norm at each stored step
    v_err_history   linear error norm at each stored step
%}

    if nargin < 5 || isempty(max_iters)
        max_iters = 500;
    end

    if nargin < 6 || isempty(tol_w)
        tol_w = 1e-6;
    end

    if nargin < 7 || isempty(tol_v)
        tol_v = 1e-6;
    end

    if nargin < 10 || isempty(k0)
        k0 = 0.1;
    end

    if nargin < 11 || isempty(secondary_mode)
        secondary_mode = 'manip_and_limits';
    end

    theta = theta_init(:);
    n = length(theta);
    theta_history = theta.';

    w_err_history = [];
    v_err_history = [];

    for k = 1:max_iters
        % Current pose and body-frame error
        T_sb = body_product_of_exponentials(M_ee, B_list, theta);
        T_bs = inv_SE3(T_sb);
        T_bd = T_bs * T_sd;

        % Body twist error
        [S, th] = log_screw_axis(T_bd);
        V_b = S * th;
        w_b = V_b(1:3);
        v_b = V_b(4:6);

        w_err = norm(w_b);
        v_err = norm(v_b);

        w_err_history = [w_err_history; w_err]; %#ok<AGROW>
        v_err_history = [v_err_history; v_err]; %#ok<AGROW>

        % Check convergence
        if w_err <= tol_w && v_err <= tol_v
            break;
        end

        % Body Jacobian
        J_b = body_jacobian(B_list, theta);

        % Right pseudoinverse
        J_pinv = pseudoinverse_jacobian(J_b);

        % Primary task
        q_dot_task = J_pinv * V_b;

        % Secondary task weights based on selected mode
        switch lower(secondary_mode)
            case 'manip_only'
                w_manip = 1.0;
                w_limit = 0.0;

            case 'joint_limits_only'
                w_manip = 0.0;
                w_limit = 1.0;
                k0 = 0.01;

            case 'manip_and_limits'
                w_manip = 1.0;
                w_limit = 0.2;

            case 'none'
                w_manip = 0.0;
                w_limit = 0.0;

            otherwise
                error(['secondary_mode must be one of: ', ...
                       '''manip_only'', ''joint_limits_only'', ', ...
                       '''manip_and_limits'', or ''none''.']);
        end

        grad_sec = calculate_secondary_gradient_rr( ...
            B_list, theta, q_min, q_max, w_manip, w_limit);

        % Null-space projector
        P = eye(n) - J_pinv * J_b;

        % Total update
        dtheta = q_dot_task + P * (k0 * grad_sec);

        % Update and clamp
        theta = theta + dtheta;
        if ~isempty(q_min) && ~isempty(q_max)
            theta = max(min(theta, q_max(:)), q_min(:));
        end

        theta_history = [theta_history; theta.']; %#ok<AGROW>
    end

    theta_sol = theta;

    % Final error recomputation at returned solution
    T_sb = body_product_of_exponentials(M_ee, B_list, theta_sol);
    T_bd = inv_SE3(T_sb) * T_sd;
    [S, th] = log_screw_axis(T_bd);
    V_b = S * th;

    w_err_final = norm(V_b(1:3));
    v_err_final = norm(V_b(4:6));
end


function grad = calculate_manipulability_gradient_rr(B_list, q)
%{
Numerically approximates the gradient of the manipulability measure.
%}

    n = length(q);
    grad = zeros(n, 1);
    eps_step = 1e-4;

    get_w = @(angles) sqrt(max(det(body_jacobian(B_list, angles) * body_jacobian(B_list, angles)'), 0));

    w_now = get_w(q);

    for i = 1:n
        q_temp = q;
        q_temp(i) = q_temp(i) + eps_step;
        w_plus = get_w(q_temp);
        grad(i) = (w_plus - w_now) / eps_step;
    end
end


function grad_lim = calculate_joint_limit_gradient_rr(q, q_min, q_max)
%{
Computes a gradient that pushes joints toward the center of their limits.
%}

    if isempty(q_min) || isempty(q_max)
        grad_lim = zeros(length(q), 1);
        return;
    end

    q = q(:);
    q_min = q_min(:);
    q_max = q_max(:);

    q_center = 0.5 * (q_min + q_max);
    q_range  = q_max - q_min;

    grad_lim = -(q - q_center) ./ (q_range.^2);
end


function grad_total = calculate_secondary_gradient_rr( ...
    B_list, q, q_min, q_max, w_manip, w_limit)
%{
Combines manipulability maximization and joint-limit avoidance.
Weights determine which objective is active.
%}

    if w_manip ~= 0
        grad_manip = calculate_manipulability_gradient_rr(B_list, q);
    else
        grad_manip = zeros(length(q), 1);
    end

    if w_limit ~= 0
        grad_limit = calculate_joint_limit_gradient_rr(q, q_min, q_max);
    else
        grad_limit = zeros(length(q), 1);
    end

    grad_total = w_manip * grad_manip + w_limit * grad_limit;
end