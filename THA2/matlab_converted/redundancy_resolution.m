function [theta, theta_history, norm_w_b_hist, norm_v_b_hist] = redundancy_resolution( ...
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
% Preallocate histories
theta_history   = zeros(max_iters, n);
norm_w_b_hist   = zeros(max_iters, 1);
norm_v_b_hist   = zeros(max_iters, 1);
iter_count = 0;
for k = 1:max_iters
    iter_count = iter_count + 1;
    % Current pose and body-frame error
    T_sb = body_product_of_exponentials(M_ee, B_list, theta);
    T_bs = inv_SE3(T_sb);
    T_bd = T_bs * T_sd;

    % Body twist error
    [S, th] = log_screw_axis(T_bd);
    V_b = S * th;
    w_b = V_b(1:3);
    v_b = V_b(4:6);

    % Save histories
    theta_history(iter_count, :) = theta.';
    norm_w_b_hist(iter_count) = norm(w_b);
    norm_v_b_hist(iter_count) = norm(v_b);

    % Check convergence
    if norm(w_b) <= tol_w && norm(v_b) <= tol_v
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

    grad_sec = calculate_secondary_gradient( ...
        B_list, theta, q_min, q_max, w_manip, w_limit);

    % Null-space projector
    P = eye(n) - J_pinv * J_b;

    % Total update
    dtheta = q_dot_task + P * (k0 * grad_sec);
    disp(q_dot_task)
    disp(P * (k0 * grad_sec))

    % Update and clamp
    theta = theta + dtheta;
    if ~isempty(q_min) && ~isempty(q_max)
        theta = max(min(theta, q_max(:)), q_min(:));
    end
end
fprintf('||w_b||=%.20f, ||v_b||=%.20f\n', norm(w_b), norm(v_b))
% Trim unused preallocated rows
theta_history = theta_history(1:iter_count, :);
norm_w_b_hist = norm_w_b_hist(1:iter_count);
norm_v_b_hist = norm_v_b_hist(1:iter_count);
end


function grad = calculate_manipulability_gradient(B_list, q)
%{
Numerically approximates the gradient of the manipulability measure
using central differences.
%}
    eps_step = 1e-3;

    q = q(:);
    n = length(q);
    grad = zeros(n,1);

    get_w = @(angles) sqrt(max(det(body_jacobian(B_list, angles) * body_jacobian(B_list, angles)'), 0));

    for i = 1:n
        dq = zeros(n,1);
        dq(i) = eps_step;

        w_plus  = get_w(q + dq);
        w_minus = get_w(q - dq);

        grad(i) = (w_plus - w_minus) / (2*eps_step);
    end
end


function grad_lim = calculate_joint_limit_gradient(q, q_min, q_max)
%{
Numerically approximates the gradient of the joint-centering objective
using central differences.

w(q) = -(1/n) * sum( (q_i - q_center_i)^2 / (2*(q_max_i - q_min_i)^2) )
%}

    if isempty(q_min) || isempty(q_max)
        grad_lim = zeros(length(q), 1);
        return;
    end

    eps_step = 1e-3;

    q = q(:);
    q_min = q_min(:);
    q_max = q_max(:);

    n = length(q);
    grad_lim = zeros(n,1);

    q_center = 0.5 * (q_min + q_max);
    q_range  = q_max - q_min;

    get_w = @(angles) -(1/n) * sum( ((angles - q_center).^2) ./ (2 * (q_range.^2)) );

    for i = 1:n
        dq = zeros(n,1);
        dq(i) = eps_step;

        w_plus  = get_w(q + dq);
        w_minus = get_w(q - dq);

        grad_lim(i) = (w_plus - w_minus) / (2 * eps_step);
    end
end


function grad_total = calculate_secondary_gradient( ...
    B_list, q, q_min, q_max, w_manip, w_limit)
%{
Combines manipulability maximization and joint-limit avoidance.
Weights determine which objective is active.
%}

    if w_manip ~= 0
        grad_manip = calculate_manipulability_gradient(B_list, q);
    else
        grad_manip = zeros(length(q), 1);
    end

    if w_limit ~= 0
        grad_limit = calculate_joint_limit_gradient(q, q_min, q_max);
    else
        grad_limit = zeros(length(q), 1);
    end

    grad_total = w_manip * grad_manip + w_limit * grad_limit;
end