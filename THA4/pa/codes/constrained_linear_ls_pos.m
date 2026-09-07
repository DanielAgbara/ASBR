function [q, q_history, norm_err_hist] = constrained_linear_ls_pos( ...
    M_tool, S_list, q_init, p_goal, r, varargin)
%{
Computes numerical inverse kinematics for end-effector pose
using the pseudo-inverse method with redundancy resolution.

Options (name-value pairs):
    max_iters      maximum iterations                (default: 500)
    tol_w          angular error tolerance            (default: 1e-6)
    tol_v          linear error tolerance             (default: 1e-6)
    q_min          joint lower limits (rad)           (default: KUKA iiwa)
    q_max          joint upper limits (rad)           (default: KUKA iiwa)
%}

opts.max_iters      = 500;
opts.tol            = 1e-6;
opts.q_min          = deg2rad([-170 -120 -170 -120 -170 -120 -175]).';
opts.q_max          = deg2rad([ 170  120  170  120  170  120  175]).';
opts.optimizer      = optimoptions('lsqlin', 'Display', 'off');
opts.wall           = false;
opts.wall_center    = p_goal + [0.00; -0.001; 0.00];
opts.wall_normal    = [0.0, -1.0, 0.0];
opts = parse_opts(opts, varargin{:});


q = q_init(:);
n = length(q);

% Preallocate histories
q_history = zeros(opts.max_iters, n);
norm_err_hist = zeros(opts.max_iters, 1);
iter_count    = 0;

for k = 1:opts.max_iters
    iter_count = iter_count + 1;

    % Current pose and body-frame error
    T_sb = space_product_of_exponentials(M_tool, S_list, q);
    t = T_sb(1:3, 4);

    % Save histories
    q_history(iter_count, :) = q.';
    norm_err_hist(iter_count)    = norm(t - p_goal);

    % Check convergence
    if norm(t - p_goal) <= opts.tol
        break;
    end

    % Body Jacobian and pseudoinverse
    J_s = space_jacobian(S_list, q);
    J_alpha   = J_s(1:3, :);
    J_epsilon = J_s(4:6, :);
    
    C = -skew(t)*J_alpha + J_epsilon;
    d = p_goal - t;

    A = -2*d'*C;
    b = r^2 - d'*d;

    if opts.wall
        wall = make_wall(opts.wall_center, opts.wall_normal);

        A_wall = -wall.normal' * C;
        b_wall =  wall.normal' * (t - wall.center);
    
        A = [A;
             A_wall];
    
        b = [b;
             b_wall];
    end
    [delta_q,~,~,exitflag,output] = lsqlin( ...
        C, d, A, b, [], [], ...
        opts.q_min - q, opts.q_max - q, ...
        zeros(size(q)), opts.optimizer);
    
    if exitflag <= 0
        warning('lsqlin failed at iter %d. exitflag = %d', k, exitflag);
        disp(output.message);
        break;
    end
    q = q + delta_q;
end

% fprintf('||err||=%.4e\n', norm(t - p_goal));

% Trim unused preallocated rows
q_history = q_history(1:iter_count, :);
norm_err_hist = norm_err_hist(1:iter_count);
end

function opts = parse_opts(opts, varargin)
if mod(numel(varargin), 2) ~= 0
    error('Optional arguments must be name-value pairs.');
end
for k = 1:2:numel(varargin)
    name  = varargin{k};
    value = varargin{k+1};
    if ~isfield(opts, name)
        error('Unknown option: %s', name);
    end
    opts.(name) = value;
end
end

