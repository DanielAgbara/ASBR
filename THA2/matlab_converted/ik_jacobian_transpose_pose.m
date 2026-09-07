function [theta, theta_history, norm_w_b_hist, norm_v_b_hist] = ik_jacobian_transpose_pose(M_ee, B_list, theta_init, T_sd, varargin)
% Numerical IK for pose using Jacobian transpose.

opts.max_iters = 100;
opts.tol_w = 1e-6;
opts.tol_v = 1e-6;
opts.q_min = [];
opts.q_max = [];
opts.K = eye(6);
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

    Vb = se3_log_to_vec(T_bd);
    w_b = Vb(1:3);
    v_b = Vb(4:6);

    % Save histories
    theta_history(iter_count, :) = theta.';
    norm_w_b_hist(iter_count) = norm(w_b);
    norm_v_b_hist(iter_count) = norm(v_b);
    if opts.print_iterations
        theta_deg = rad2deg(theta);
        parts = arrayfun(@(j) sprintf('theta%d=%.2fdeg', j, theta_deg(j)), ...
            1:length(theta_deg), 'UniformOutput', false);

        fprintf('Iteration %d: (%s), (x,y,z)=(%.3f, %.3f, %.3f), ||w_b||=%.3e, ||v_b||=%.3e\n', ...
            i, strjoin(parts, ', '), T_sb(1,4), T_sb(2,4), T_sb(3,4), norm(w_b), norm(v_b));
    end

    if norm(w_b) < opts.tol_w && norm(v_b) < opts.tol_v
        break;
    end

    J_b = body_jacobian(B_list, theta);
    dq  = J_b.' * opts.K * Vb;

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