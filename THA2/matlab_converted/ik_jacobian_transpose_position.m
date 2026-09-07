function [theta, theta_history] = ik_jacobian_transpose_position(M_ee, B_list, theta_init, p_des, varargin)
% Numerical IK for position using Jacobian transpose.

opts.max_iters = 100;
opts.tol_converge = 1e-6;
opts.q_min = deg2rad([-170 -120 -170 -120 -170 -120 -175]).';
opts.q_max = deg2rad([ 170  120  170  120  170  120  175]).';
opts.K = eye(3);
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
    dq = J_v.' * opts.K * error_vec;

    theta = theta + dq;
    if ~isempty(opts.q_min) && ~isempty(opts.q_max)
        theta = min(max(theta, opts.q_min(:)), opts.q_max(:));
    end
    theta_history(end+1, :) = theta.'; %#ok<AGROW>
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
