function [theta, theta_history, norm_w_b_hist, norm_v_b_hist] = numerical_inverse_kinematics_pose(M_ee, B_list, theta_init, T_sd, varargin)
% Numerical IK for end-effector pose using the body Jacobian.

opts.max_iters = 100;
opts.tol_w = 1e-6;
opts.tol_v = 1e-6;
opts.tol_manipulability = 1e-3;
opts.q_min = deg2rad([-170 -120 -170 -120 -170 -120 -175]).';
opts.q_max = deg2rad([ 170  120  170  120  170  120  175]).';
opts.objective_func = [];
opts.objective_args = {};
opts.k_null = 0.1;
opts.k_damping = 0.1;
opts.print_iterations = true;
opts = parse_opts(opts, varargin{:});

theta = theta_init(:);
n = length(theta);

% Preallocate histories
theta_history   = zeros(opts.max_iters, n);
norm_w_b_hist   = zeros(opts.max_iters, 1);
norm_v_b_hist   = zeros(opts.max_iters, 1);
iter_count = 0;

for i = 0:opts.max_iters
    iter_count = iter_count + 1;
    T_sb = body_product_of_exponentials(M_ee, B_list, theta);
    T_bs = inv_SE3(T_sb);
    T_bd = T_bs * T_sd;

    [S, th] = log_screw_axis(T_bd);
    V_b = S * th;
    w_b = V_b(1:3);
    v_b = V_b(4:6);

    % Save histories
    theta_history(iter_count, :) = theta.';
    norm_w_b_hist(iter_count) = norm(w_b);
    norm_v_b_hist(iter_count) = norm(v_b);
    if opts.print_iterations
        theta_deg = rad2deg(theta);
        parts = arrayfun(@(j) sprintf('theta%d=%.2fdeg', j, theta_deg(j)), 1:length(theta_deg), 'UniformOutput', false);
        fprintf('Iteration %d: (%s), (x, y, z)=(%.3f, %.3f, %.3f), ||w_b||=%.3f, ||v_b||=%.3f\n', ...
            i, strjoin(parts, ', '), T_sb(1,4), T_sb(2,4), T_sb(3,4), norm(w_b), norm(v_b));
    end

    if norm(w_b) <= opts.tol_w && norm(v_b) <= opts.tol_v
        fprintf('||w_b||=%.20f, ||v_b||=%.20f\n', norm(w_b), norm(v_b))
        break;
    end

    J_b = body_jacobian(B_list, theta);
    % Inspect J_b here when diagnosing singularities.
    % J_dagger = pseudoinverse_jacobian(J_b);
    if opts.tol_manipulability < manipulability(J_b)
        J_dagger = pseudoinverse_jacobian(J_b);
    else
        J_dagger = damped_least_square_inverse(J_b, opts.k_damping);
    end

    dq = J_dagger * V_b;

    if ~isempty(opts.objective_func)
        P = eye(length(theta)) - J_dagger * J_b;
        w_func = @(th_) call_objective(opts.objective_func, th_, opts.objective_args);
        dot_q_0 = opts.k_null * finite_difference_grad(w_func, theta);
        dq = dq + P * dot_q_0;
    end

    theta = theta + dq;
    if ~isempty(opts.q_min) && ~isempty(opts.q_max)
        theta = min(max(theta, opts.q_min(:)), opts.q_max(:));
    end
    
end
% Trim unused preallocated rows
theta_history = theta_history(1:iter_count, :);
norm_w_b_hist = norm_w_b_hist(1:iter_count);
norm_v_b_hist = norm_v_b_hist(1:iter_count);

end

function val = call_objective(f, th, args)
if isempty(args)
    val = f(th);
elseif iscell(args)
    val = f(th, args{:});
else
    val = f(th, args);
end
end

function opts = parse_opts(opts, varargin)
if mod(numel(varargin),2) ~= 0
    error('Optional arguments must be name-value pairs.');
end
for k = 1:2:numel(varargin)
    name = varargin{k};
    value = varargin{k+1};
    if ~isfield(opts, name)
        error('Unknown option: %s', name);
    end
    opts.(name) = value;
end
end
