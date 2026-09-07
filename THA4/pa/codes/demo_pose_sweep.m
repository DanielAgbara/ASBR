%% =========================
%  pos_and_ori_weight_distance_sweep.m
%  Position + orientation regulation
%  Random workspace-uniform initial configuration
%  Random direction goal sampling
%  Weight ratio + distance sweep
%% =========================
clear; clc; close all;

%% =========================
% Import URDF robot model
%% =========================
urdfPath = fullfile(fileparts(which('demo_pose_sweep')), '..', 'kuka_lbr_iiwa_support', 'urdf', 'lbr_iiwa_14_r820.urdf');
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
r         = 0.003;

zeta = 1.0;

optim = optimoptions('lsqlin', ...
    'Display', 'off', ...
    'Algorithm', 'active-set');

wall = false;

%% =========================
% Sweep settings
%% =========================
weight_ratio_array = [1e-6 0.1 0.5 1 2 10 1e6];
dist_array = [0.003 0.03 0.3];

num_weight = length(weight_ratio_array);
num_dist   = length(dist_array);
num_trials = 100;

rng(1);

%% =========================
% Build approximate workspace-uniform sample pool
%% =========================
num_workspace_samples = 80000;
voxel_size = 0.04;

fprintf('Building workspace sample pool...\n');

Q_pool = nan(7, num_workspace_samples);
P_pool = nan(3, num_workspace_samples);

for s = 1:num_workspace_samples
    q_rand = q_min + (q_max - q_min) .* rand(size(q_min));
    T_rand = body_product_of_exponentials(M_tool, B_list, q_rand);

    Q_pool(:,s) = q_rand;
    P_pool(:,s) = T_rand(1:3,4);
end

p_min = min(P_pool, [], 2);
voxel_id = floor((P_pool - p_min) ./ voxel_size);

[~, ~, voxel_idx] = unique(voxel_id.', 'rows');
num_voxels = max(voxel_idx);

voxel_members = cell(num_voxels, 1);
for s = 1:num_workspace_samples
    voxel_members{voxel_idx(s)}(end+1) = s;
end

fprintf('Workspace pool complete: %d samples, %d occupied voxels.\n', ...
    num_workspace_samples, num_voxels);

%% =========================
% Data storage
%% =========================
median_step_angle_deg = nan(num_weight, num_trials, num_dist);
max_step_angle_deg    = nan(num_weight, num_trials, num_dist);
final_angle_deg       = nan(num_weight, num_trials, num_dist);
max_angle_from_init_deg = nan(num_weight, num_trials, num_dist);
final_pos_error       = nan(num_weight, num_trials, num_dist);
actual_dist_data      = nan(num_weight, num_trials, num_dist);
num_iter_data         = nan(num_weight, num_trials, num_dist);

%% =========================
% Simulation
%% =========================
for id = 1:num_dist

    d_target = dist_array(id);

    fprintf('\n=========================\n');
    fprintf('Distance case: %.4f m\n', d_target);
    fprintf('=========================\n');

    for iw = 1:num_weight

        eta_over_zeta = weight_ratio_array(iw);

        zeta_i = zeta;
        eta_i  = eta_over_zeta * zeta_i;

        fprintf('\nDistance = %.4f m | eta/zeta = %.4g\n', ...
            d_target, eta_over_zeta);

        for j = 1:num_trials

            %% =========================
            % Workspace-uniform initial pose
            %% =========================
            voxel_choice = randi(num_voxels);
            members = voxel_members{voxel_choice};
            init_idx = members(randi(numel(members)));

            q_init = Q_pool(:, init_idx);
            p_tip  = P_pool(:, init_idx);

            T_init = body_product_of_exponentials(M_tool, B_list, q_init);
            z_init = T_init(1:3,1:3) * [0;0;1];

            %% =========================
            % Random direction goal sampling
            %% =========================
            u = randn(3,1);
            u = u / norm(u);

            p_goal = p_tip + d_target * u;
            actual_dist = norm(p_goal - p_tip);

            actual_dist_data(iw,j,id) = actual_dist;

            if wall
                wall_center = p_goal + [0.00; -0.001; 0.00];
                wall_normal = [0.0; -1.0; 0.0];
                wall_normal = wall_normal / norm(wall_normal);
            else
                wall_center = [];
                wall_normal = [];
            end

            %% =========================
            % IK solve
            %% =========================
            try
                [theta_sol, theta_history, ~] = constrained_linear_ls_pos_and_ori( ...
                    M_tool, ...
                    S_list, ...
                    q_init, ...
                    p_goal, ...
                    r, ...
                    'max_iters', max_iters, ...
                    'tol', tol, ...
                    'q_min', q_min, ...
                    'q_max', q_max, ...
                    'w_dist', zeta_i, ...
                    'w_ori', eta_i, ...
                    'optimizer', optim, ...
                    'wall', wall, ...
                    'wall_center', wall_center, ...
                    'wall_normal', wall_normal);

                N_hist = size(theta_history,1);

                if N_hist >= max_iters
                    continue;
                end

                num_iter_data(iw,j,id) = N_hist;

                %% =========================
                % Step-to-step orientation deviation
                %% =========================
                if N_hist >= 2

                    step_angle_hist = nan(N_hist-1,1);
                    angle_from_init_hist = nan(N_hist,1);

                    for k_hist = 1:N_hist

                        q_curr = theta_history(k_hist,:).';
                        T_curr = body_product_of_exponentials(M_tool, B_list, q_curr);
                        z_curr = T_curr(1:3,1:3) * [0;0;1];

                        cos_init = dot(z_init, z_curr) / (norm(z_init)*norm(z_curr));
                        cos_init = min(max(cos_init, -1), 1);

                        angle_from_init_hist(k_hist) = acosd(cos_init);

                        if k_hist >= 2
                            q_prev = theta_history(k_hist-1,:).';
                            T_prev = body_product_of_exponentials(M_tool, B_list, q_prev);
                            z_prev = T_prev(1:3,1:3) * [0;0;1];

                            cos_step = dot(z_prev, z_curr) / (norm(z_prev)*norm(z_curr));
                            cos_step = min(max(cos_step, -1), 1);

                            step_angle_hist(k_hist-1) = acosd(cos_step);
                        end
                    end

                    median_step_angle_deg(iw,j,id) = median(step_angle_hist, 'omitnan');
                    max_step_angle_deg(iw,j,id)    = max(step_angle_hist, [], 'omitnan');

                    final_angle_deg(iw,j,id) = angle_from_init_hist(end);
                    max_angle_from_init_deg(iw,j,id) = max(angle_from_init_hist, [], 'omitnan');

                else
                    median_step_angle_deg(iw,j,id) = 0;
                    max_step_angle_deg(iw,j,id)    = 0;
                    final_angle_deg(iw,j,id)       = 0;
                    max_angle_from_init_deg(iw,j,id) = 0;
                end

                %% =========================
                % Final position error
                %% =========================
                T_final = body_product_of_exponentials(M_tool, B_list, theta_history(end,:).');
                p_final = T_final(1:3,4);

                final_pos_error(iw,j,id) = norm(p_final - p_goal);

            catch ME
                fprintf('  trial %03d failed | d = %.4f | eta/zeta = %.4g: %s\n', ...
                    j, d_target, eta_over_zeta, ME.message);
            end
        end

        fprintf('  valid trials = %d / %d\n', ...
            sum(~isnan(median_step_angle_deg(iw,:,id))), num_trials);
    end
end

%% =========================
% Statistics
%% =========================
median_step_angle_summary = squeeze(median(median_step_angle_deg, 2, 'omitnan'));
final_angle_summary       = squeeze(median(final_angle_deg, 2, 'omitnan'));
max_step_angle_summary    = squeeze(median(max_step_angle_deg, 2, 'omitnan'));
max_angle_from_init_summary = squeeze(median(max_angle_from_init_deg, 2, 'omitnan'));
final_pos_error_summary   = squeeze(median(final_pos_error, 2, 'omitnan'));
num_valid_summary         = squeeze(sum(~isnan(median_step_angle_deg), 2));

%% =========================
% Summary table
%% =========================
rows = {};

for id = 1:num_dist
    for iw = 1:num_weight
        rows(end+1,:) = { ...
            dist_array(id), ...
            weight_ratio_array(iw), ...
            median_step_angle_summary(iw,id), ...
            final_angle_summary(iw,id), ...
            max_step_angle_summary(iw,id), ...
            max_angle_from_init_summary(iw,id), ...
            final_pos_error_summary(iw,id), ...
            num_valid_summary(iw,id)};
    end
end

summary_table = cell2table(rows, ...
    'VariableNames', { ...
        'Distance_m', ...
        'EtaOverZeta', ...
        'MedianStepAngleDeviation_deg', ...
        'MedianFinalAngleDeviation_deg', ...
        'MedianMaximumStepAngleDeviation_deg', ...
        'MedianMaximumAngleFromInitial_deg', ...
        'MedianFinalPositionError_m', ...
        'NumValidTrials'});

disp(summary_table);

legend_labels = strings(1, num_weight);
for iw = 1:num_weight
    legend_labels(iw) = sprintf('\\eta/\\zeta = %.3g', weight_ratio_array(iw));
end

%% =========================
% Figure 1: Step angle deviation median
%% =========================
figure('Color','w');
hold on; grid on;

for iw = 1:num_weight
    plot(dist_array, median_step_angle_summary(iw,:), ...
        'o-', ...
        'LineWidth', 2, ...
        'MarkerSize', 8);
end

xlabel('Target distance [m]');
ylabel('Median step angle deviation [deg]');
title('Median Step-to-Step Tool-Axis Deviation');
legend(legend_labels, 'Location', 'bestoutside');

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.2;
ax.XScale = 'log';

%% =========================
% Figure 2: Final median deviation
%% =========================
figure('Color','w');
hold on; grid on;

for iw = 1:num_weight
    plot(dist_array, final_angle_summary(iw,:), ...
        'o-', ...
        'LineWidth', 2, ...
        'MarkerSize', 8);
end

xlabel('Target distance [m]');
ylabel('Median final angle deviation [deg]');
title('Median Final Tool-Axis Deviation');
legend(legend_labels, 'Location', 'bestoutside');

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.2;
ax.XScale = 'log';

%% =========================
% Figure 3: Maximum step angle deviation
%% =========================
figure('Color','w');
hold on; grid on;

for iw = 1:num_weight
    plot(dist_array, max_step_angle_summary(iw,:), ...
        'o-', ...
        'LineWidth', 2, ...
        'MarkerSize', 8);
end

xlabel('Target distance [m]');
ylabel('Median maximum step angle deviation [deg]');
title('Maximum Step-to-Step Tool-Axis Deviation');
legend(legend_labels, 'Location', 'bestoutside');

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.2;
ax.XScale = 'log';

%% =========================
% Figure 4: Maximum angle deviation from initial direction
%% =========================
figure('Color','w');
hold on; grid on;

for iw = 1:num_weight
    plot(dist_array, max_angle_from_init_summary(iw,:), ...
        'o-', ...
        'LineWidth', 2, ...
        'MarkerSize', 8);
end

xlabel('Target distance [m]');
ylabel('Median maximum angle deviation from initial [deg]');
title('Maximum Tool-Axis Deviation from Initial Direction');
legend(legend_labels, 'Location', 'bestoutside');

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.2;
ax.XScale = 'log';

%% =========================
% Optional Figure 5: Final position error
%% =========================
figure('Color','w');
hold on; grid on;

for iw = 1:num_weight
    plot(dist_array, final_pos_error_summary(iw,:), ...
        'o-', ...
        'LineWidth', 2, ...
        'MarkerSize', 8);
end

yline(tol, 'k--', 'LineWidth', 1.5, 'Label', 'tol');
yline(r, 'r--', 'LineWidth', 1.5, 'Label', 'r');

xlabel('Target distance [m]');
ylabel('Median final position error [m]');
title('Median Final Position Error');
legend(legend_labels, 'Location', 'bestoutside');

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.2;
ax.XScale = 'log';

%% =========================
% Figure 6: Step deviation vs eta/zeta (d = 0.003, 0.03)
%% =========================
figure('Color','w');
cla
hold on; grid on;

dist_plot_idx = [1, 2]; % 0.003, 0.03

for k = 1:length(dist_plot_idx)
    id = dist_plot_idx(k);

    semilogx(weight_ratio_array, ...
        median_step_angle_summary(:,id), ...
        'o-', ...
        'LineWidth', 2, ...
        'MarkerSize', 8);
end

xlabel('\eta / \zeta');
ylabel('Median step angle deviation [deg]');
title('Step-to-step Orientation Deviation vs Weight');

legend('d = 0.003 m','d = 0.03 m', 'Location','best');

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.2;
hold off
xscale('log')
yscale('log')
%% =========================
% Figure 7: Final deviation vs eta/zeta (d = 0.003, 0.03)
%% =========================
figure('Color','w');
cla
hold on; grid on;

dist_plot_idx = [1, 2]; % 0.003, 0.03

for k = 1:length(dist_plot_idx)
    id = dist_plot_idx(k);

    semilogx(weight_ratio_array, ...
        final_angle_summary(:,id), ...
        'o-', ...
        'LineWidth', 2, ...
        'MarkerSize', 8);
end

xlabel('\eta / \zeta');
ylabel('Median step angle deviation [deg]');
title('Final Orientation Deviation vs Weight');

legend('d = 0.003 m','d = 0.03 m', 'Location','best');

ax = gca;
ax.FontSize = 18;
ax.LineWidth = 1.2;
xscale('log')
yscale('log')