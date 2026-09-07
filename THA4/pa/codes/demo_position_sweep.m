%% =========================
%  distance_iteration_sweep.m
%  Distance vs IK iteration simulation
%  Position-only constrained IK
%  Random initial configuration with approximately uniform workspace sampling
%% =========================
clear; clc; close all;

%% =========================
% Import URDF robot model
%% =========================
urdfPath = fullfile(fileparts(which('demo_position_sweep')), '..', 'kuka_lbr_iiwa_support', 'urdf', 'lbr_iiwa_14_r820.urdf');
robot = importrobot(urdfPath);
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81];

%% =========================
% Add cylindrical tool to end-effector
%% =========================
toolLength = 0.100;
toolRadius = 0.0025;

parentName = 'tool0';

toolBody = rigidBody('cylindrical_tool');
toolJoint = rigidBodyJoint('cylindrical_tool_fixed_joint', 'fixed');
setFixedTransform(toolJoint, eye(4));
toolBody.Joint = toolJoint;

T_cyl_center = eye(4);
T_cyl_center(1:3,1:3) = Ry(pi/2);
T_cyl_center(1:3,4)   = [toolLength/2; 0; 0];

addVisual(toolBody, "Cylinder", [toolRadius toolLength], T_cyl_center);
addBody(robot, toolBody, parentName);

%% =========================
% Joint limits
%% =========================
q_min = deg2rad([-170 -120 -170 -120 -170 -120 -175]).';
q_max = deg2rad([ 170  120  170  120  170  120  175]).';

%% =========================
% KUKA LBR iiwa 14 R820 data
%% =========================
T_ee_tool = eye(4);
T_ee_tool(1:3,4) = [0; 0; toolLength];

M_ee = [1 0 0 0;
        0 1 0 0;
        0 0 1 1.306;
        0 0 0 1];

M_tool = M_ee * T_ee_tool;

S1 = screw_axis_from_w_q([0; 0; 1], [0; 0; 0.1575]);
S2 = screw_axis_from_w_q([0; 1; 0], [0; 0; 0.3600]);
S3 = screw_axis_from_w_q([0; 0; 1], [0; 0; 0.5645]);
S4 = screw_axis_from_w_q([0; -1; 0], [0; 0; 0.7800]);
S5 = screw_axis_from_w_q([0; 0; 1], [0; 0; 0.9645]);
S6 = screw_axis_from_w_q([0; 1; 0], [0; 0; 1.1800]);
S7 = screw_axis_from_w_q([0; 0; 1], [0; 0; 1.2610]);

S_mat = [S1 S2 S3 S4 S5 S6 S7];
B_mat = adjoint_inverse(M_tool) * S_mat;

S_list = num2cell(S_mat, 1);
B_list = num2cell(B_mat, 1);

%% =========================
% Solver settings
%% =========================
max_iters = 100;
tol       = 1e-4;

% For Case 1, use the actual distance constraint radius.
% If you want unconstrained behavior, set r = 1e6.
r = 1e6;

optim = optimoptions('lsqlin', ...
    'Display', 'off', ...
    'Algorithm', 'active-set');

%% =========================
% Distance sweep settings
%% =========================
dist_array = [ ...
    0.001 0.002 0.003 0.004 0.005 ...
    0.006 0.007 0.008 0.009 0.010 ...
    0.020 0.030 0.040 0.050 0.060 ...
    0.070 0.080 0.090 0.100 ...
    0.200 0.300 0.400 0.500 ...
    0.600 0.700 0.800 0.900];

num_dist   = length(dist_array);
num_trials = 100;

direct_sample_threshold = 0.20;
max_goal_attempts = 2000000;

distance_tol_array = max(0.0005, 0.02 * dist_array);

rng(1);

%% =========================
% Build approximate workspace-uniform sample pool
%% =========================
num_workspace_samples = 80000;
voxel_size = 0.04;   % [m], smaller = more uniform but fewer samples per voxel

fprintf('Building workspace sample pool...\n');

Q_pool = nan(7, num_workspace_samples);
P_pool = nan(3, num_workspace_samples);

for s = 1:num_workspace_samples
    q_rand = q_min + (q_max - q_min) .* rand(size(q_min));
    T_rand = body_product_of_exponentials(M_tool, B_list, q_rand);
    p_rand = T_rand(1:3,4);

    Q_pool(:,s) = q_rand;
    P_pool(:,s) = p_rand;
end

% Cartesian voxelization of workspace points
p_min = min(P_pool, [], 2);
voxel_id = floor((P_pool - p_min) ./ voxel_size);

[unique_voxels, ~, voxel_idx] = unique(voxel_id.', 'rows');
num_voxels = size(unique_voxels, 1);

voxel_members = cell(num_voxels, 1);
for s = 1:num_workspace_samples
    voxel_members{voxel_idx(s)}(end+1) = s;
end

fprintf('Workspace pool complete: %d samples, %d occupied voxels.\n', ...
    num_workspace_samples, num_voxels);

%% =========================
% Data storage
%% =========================
iter_data = nan(num_dist, num_trials);
actual_dist_data = nan(num_dist, num_trials);
lin_model_error_max = nan(num_dist, num_trials);
constraint_violation_max = nan(num_dist, num_trials);
constraint_distance_max = nan(num_dist, num_trials);

%% =========================
% Simulation
%% =========================
for i = 1:num_dist

    d_target = dist_array(i);
    d_tol    = distance_tol_array(i);

    fprintf('\nTarget distance = %.4f m | distance tol = %.6f m\n', ...
        d_target, d_tol);

    for j = 1:num_trials

        %% =========================
        % Approximately uniform workspace sampling for initial pose
        %% =========================
        voxel_choice = randi(num_voxels);
        members = voxel_members{voxel_choice};
        init_idx = members(randi(numel(members)));

        q_init = Q_pool(:, init_idx);
        p_tip  = P_pool(:, init_idx);

        %% =========================
        % Goal sampling
        %% =========================
        if d_target < direct_sample_threshold

            % Near-distance sampling:
            % Generate a goal exactly d_target away from the sampled initial tip.
            u = randn(3,1);
            u = u / norm(u);

            p_goal = p_tip + d_target * u;
            actual_dist = norm(p_goal - p_tip);

        else

            % Far-distance sampling:
            % Select a reachable goal from the workspace pool whose Cartesian
            % distance from the initial tip is close to d_target.
            found_goal = false;
            goal_attempt_count = 0;

            while ~found_goal && goal_attempt_count < max_goal_attempts

                goal_attempt_count = goal_attempt_count + 1;

                goal_idx = randi(num_workspace_samples);
                p_candidate = P_pool(:, goal_idx);

                actual_dist_candidate = norm(p_candidate - p_tip);

                if abs(actual_dist_candidate - d_target) <= d_tol
                    p_goal = p_candidate;
                    actual_dist = actual_dist_candidate;
                    found_goal = true;
                end
            end

            if ~found_goal
                warning('Could not find reachable goal for d = %.4f m, trial %d.', ...
                    d_target, j);

                iter_data(i,j) = NaN;
                actual_dist_data(i,j) = NaN;
                lin_model_error_max(i,j) = NaN;
                constraint_violation_max(i,j) = NaN;
                constraint_distance_max(i,j) = NaN;
                continue;
            end
        end

        actual_dist_data(i,j) = actual_dist;

        %% =========================
        % IK solve
        %% =========================
        try
            [theta_sol, theta_history, ~] = constrained_linear_ls_pos( ...
                M_tool, ...
                S_list, ...
                q_init, ...
                p_goal, ...
                r, ...
                'max_iters', max_iters, ...
                'tol', tol, ...
                'q_min', q_min, ...
                'q_max', q_max, ...
                'optimizer', optim);

            num_iter = size(theta_history,1);

            % Treat max-iteration termination as failure.
            if num_iter >= max_iters
                iter_data(i,j) = NaN;
                lin_model_error_max(i,j) = NaN;
                constraint_violation_max(i,j) = NaN;
                constraint_distance_max(i,j) = NaN;
                continue;
            end

            iter_data(i,j) = num_iter;

            %% =========================
            % Maximum linearization error over iterations
            %% =========================
            N_hist = size(theta_history,1);

            if N_hist >= 2

                lin_model_error_hist = nan(N_hist-1,1);

                for k_hist = 1:N_hist-1

                    qk  = theta_history(k_hist,:).';
                    qkp = theta_history(k_hist+1,:).';
                    dq  = qkp - qk;

                    T_k    = body_product_of_exponentials(M_tool, B_list, qk);
                    T_true = body_product_of_exponentials(M_tool, B_list, qkp);

                    t_k    = T_k(1:3,4);
                    t_true = T_true(1:3,4);

                    Js = space_jacobian(S_list, qk);

                    J_alpha   = Js(1:3,:);
                    J_epsilon = Js(4:6,:);

                    alpha   = J_alpha * dq;
                    epsilon = J_epsilon * dq;

                    t_lin = t_k + cross(alpha, t_k) + epsilon;

                    lin_model_error_hist(k_hist) = norm(t_true - t_lin);
                end

                lin_model_error_max(i,j) = max(lin_model_error_hist, [], 'omitnan');

            else
                lin_model_error_max(i,j) = 0;
            end

            %% =========================
            % Actual constraint violation over iterations
            %% =========================
            distance_hist = nan(N_hist,1);
            violation_hist = nan(N_hist,1);

            for k_hist = 1:N_hist

                qk = theta_history(k_hist,:).';

                T_k = body_product_of_exponentials(M_tool, B_list, qk);
                t_k = T_k(1:3,4);

                distance_hist(k_hist) = norm(t_k - p_goal);
                violation_hist(k_hist) = max(distance_hist(k_hist) - r, 0);
            end

            % The first configuration can be outside the distance bound when
            % initial distance > r. For constraint verification of the generated
            % motion, evaluate violation after the first update.
            if N_hist >= 2
                constraint_violation_max(i,j) = max(violation_hist(2:end), [], 'omitnan');
                constraint_distance_max(i,j) = max(distance_hist(2:end), [], 'omitnan');
            else
                constraint_violation_max(i,j) = max(violation_hist, [], 'omitnan');
                constraint_distance_max(i,j) = max(distance_hist, [], 'omitnan');
            end

        catch ME
            fprintf('  trial %03d failed at d = %.4f m, actual d = %.4f m: %s\n', ...
                j, d_target, actual_dist, ME.message);

            iter_data(i,j) = NaN;
            lin_model_error_max(i,j) = NaN;
            constraint_violation_max(i,j) = NaN;
            constraint_distance_max(i,j) = NaN;
        end
    end

    fprintf('  completed trials = %d | valid IK trials = %d\n', ...
        num_trials, sum(~isnan(iter_data(i,:))));
end

%% =========================
% Statistics
%% =========================
actual_dist_median = median(actual_dist_data, 2, 'omitnan');
actual_dist_min    = min(actual_dist_data, [], 2, 'omitnan');
actual_dist_max    = max(actual_dist_data, [], 2, 'omitnan');

iter_median = median(iter_data, 2, 'omitnan');
lin_model_error_max_median = median(lin_model_error_max, 2, 'omitnan');
constraint_violation_median = median(constraint_violation_max, 2, 'omitnan');
constraint_distance_median = median(constraint_distance_max, 2, 'omitnan');

num_valid = sum(~isnan(iter_data), 2);

summary_table = table( ...
    dist_array(:), ...
    actual_dist_median, ...
    actual_dist_min, ...
    actual_dist_max, ...
    iter_median, ...
    lin_model_error_max_median, ...
    constraint_distance_median, ...
    constraint_violation_median, ...
    num_valid, ...
    'VariableNames', { ...
        'TargetDistance_m', ...
        'MedianActualDistance_m', ...
        'MinActualDistance_m', ...
        'MaxActualDistance_m', ...
        'MedianIterations', ...
        'MedianMaxLinearizationError_m', ...
        'MedianMaxActualDistanceAfterFirstStep_m', ...
        'MedianMaxConstraintViolationAfterFirstStep_m', ...
        'NumValidTrials'});

disp(summary_table);

%% =========================
% Plot 1: Iteration box plot
%% =========================
figure('Color','w'); 
grid on;

boxplot(iter_data.', ...
    'Labels', string(dist_array), ...
    'Whisker', 1.5);

xlabel('Target ||p_{goal} - p_{tip,0}|| [m]');
ylabel('IK iterations');
title('Iteration Distribution vs Initial Distance');

yscale('log');

ax = gca;
ax.FontSize = 16;
ax.LineWidth = 1.2;
ax.XTickLabelRotation = 45;

%% =========================
% Plot 2: Max linearization error box plot
%% =========================
figure('Color','w'); 
grid on;

boxplot(lin_model_error_max.', ...
    'Labels', string(dist_array), ...
    'Whisker', 1.5);

xlabel('Target ||p_{goal} - p_{tip,0}|| [m]');
ylabel('Max ||p_{true} - p_{lin}|| [m]');
title('Maximum Linearization Error Distribution vs Initial Distance');

ax = gca;
ax.FontSize = 16;
ax.LineWidth = 1.2;
ax.XTickLabelRotation = 45;

%% =========================
% Plot 3: Median max linearization error
%% =========================
figure('Color','w');
hold on; grid on;

plot(dist_array, lin_model_error_max_median, ...
    'o-', ...
    'LineWidth', 2, ...
    'MarkerSize', 8);

xlabel('Target ||p_{goal} - p_{tip,0}|| [m]');
ylabel('Median max ||p_{true} - p_{lin}|| [m]');
title('Median Maximum Linearization Error vs Initial Distance');

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.2;

%% =========================
% Plot 4: Actual constraint violation box plot
%% =========================
figure('Color','w'); 
grid on;

boxplot(constraint_violation_max.', ...
    'Labels', string(dist_array), ...
    'Whisker', 1.5);

xlabel('Target ||p_{goal} - p_{tip,0}|| [m]');
ylabel('Max violation max(||p_{tip} - p_{goal}|| - r, 0) [m]');
title('Actual Constraint Violation After First Step vs Initial Distance');

ax = gca;
ax.FontSize = 16;
ax.LineWidth = 1.2;
ax.XTickLabelRotation = 45;

%% =========================
% Plot 5: Maximum actual distance from goal after first step
%% =========================
figure('Color','w'); 
grid on;

boxplot(constraint_distance_max.', ...
    'Labels', string(dist_array), ...
    'Whisker', 1.5);

xlabel('Target ||p_{goal} - p_{tip,0}|| [m]');
ylabel('Max ||p_{tip} - p_{goal}|| after first step [m]');
title('Maximum Actual Distance from Goal After First Step');

yline(r, 'r--', 'LineWidth', 2, 'Label', 'r');

ax = gca;
ax.FontSize = 16;
ax.LineWidth = 1.2;
ax.XTickLabelRotation = 45;

%% =========================
% Plot 6: Median actual constraint violation
%% =========================
figure('Color','w');
hold on; grid on;

plot(dist_array, constraint_violation_median, ...
    'o-', ...
    'LineWidth', 2, ...
    'MarkerSize', 8);

xlabel('Target ||p_{goal} - p_{tip,0}|| [m]');
ylabel('Median max violation [m]');
title('Median Actual Constraint Violation vs Initial Distance');

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.2;