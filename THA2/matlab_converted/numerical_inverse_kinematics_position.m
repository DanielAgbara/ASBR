function [theta, theta_history] = numerical_inverse_kinematics_position(M_ee, B_list, theta_init, p_des, varargin)
% Numerical IK for end-effector position using the body Jacobian.

opts.max_iters = 100;
opts.tol_converge = 1e-6;
opts.tol_manipulability = 1e-3;
opts.q_min = [];
opts.q_max = [];
opts.objective_func = [];
opts.objective_args = {};
opts.k_null = 0.1;
opts.k_damping = 0.01;
opts.print_iterations = true;
opts = parse_opts(opts, varargin{:});

theta = theta_init(:);
p_des = p_des(:);
theta_history = theta.';

for i = 0:opts.max_iters-1
    T_ee = body_product_of_exponentials(M_ee, B_list, theta);
    p_ee = T_ee(1:3, 4);
    error_vec = p_des - p_ee;

    if opts.print_iterations
        theta_deg = rad2deg(theta);
        parts = arrayfun(@(j) sprintf('theta%d=%.2fdeg', j, theta_deg(j)), 1:length(theta_deg), 'UniformOutput', false);
        fprintf('Iteration %d: (%s), (x,y,z)=(%.3f, %.3f, %.3f), ||error||=%.3e\n', ...
            i, strjoin(parts, ', '), p_ee(1), p_ee(2), p_ee(3), norm(error_vec));
    end

    if norm(error_vec) < opts.tol_converge
        break;
    end

    J_b = body_jacobian(B_list, theta);
    J_v = J_b(4:6, :);
    if opts.tol_manipulability < manipulability(J_v)
        J_dagger = pseudoinverse_jacobian(J_v);
    else
        J_dagger = damped_least_square_inverse(J_v, opts.k_damping);
    end

    dq = J_dagger * error_vec;

    if ~isempty(opts.objective_func)
        P = eye(length(theta)) - J_dagger * J_v;
        w_func = @(th) call_objective(opts.objective_func, th, opts.objective_args);
        dot_q_0 = opts.k_null * finite_difference_grad(w_func, theta);
        dq = dq + P * dot_q_0;
    end

    theta = theta + dq;
    if ~isempty(opts.q_min) && ~isempty(opts.q_max)
        theta = min(max(theta, opts.q_min(:)), opts.q_max(:));
    end
    theta_history(end+1, :) = theta.'; %#ok<AGROW>
end
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
