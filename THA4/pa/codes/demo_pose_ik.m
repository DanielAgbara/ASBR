%% =========================
%  PA_h_make_videos.m
%  Simple pose IK test
%% =========================
clear; clc; close all;

%% =========================
% Import URDF robot model
%% =========================
urdfPath = fullfile(fileparts(which('demo_pose_ik')), '..', 'kuka_lbr_iiwa_support', 'urdf', 'lbr_iiwa_14_r820.urdf');
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

addVisual(toolBody, ...
    "Cylinder", ...
    [toolRadius toolLength], ...
    T_cyl_center);

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

% Space-frame screw axes (6x1 each)
S1 = screw_axis_from_w_q([0; 0; 1], [0; 0; 0.1575]);
S2 = screw_axis_from_w_q([0; 1; 0], [0; 0; 0.3600]);
S3 = screw_axis_from_w_q([0; 0; 1], [0; 0; 0.5645]);
S4 = screw_axis_from_w_q([0; -1; 0], [0; 0; 0.7800]);
S5 = screw_axis_from_w_q([0; 0; 1], [0; 0; 0.9645]);
S6 = screw_axis_from_w_q([0; 1; 0], [0; 0; 1.1800]);
S7 = screw_axis_from_w_q([0; 0; 1], [0; 0; 1.2610]);

% Put them into 6x7 matrix first
S_mat = [S1 S2 S3 S4 S5 S6 S7];

% Convert to body-frame screw axes: B = Ad_{M^{-1}} S
B_mat = adjoint_inverse(M_tool) * S_mat;

S_list = num2cell(S_mat, 1);
B_list = num2cell(B_mat, 1);

%% ========= ================
% Initial configuration
%% =========================
theta_a = deg2rad([60; 60; 60; 60; 60; 60; 60]);
T_sb = body_product_of_exponentials(M_tool, B_list, theta_a);
p_tip = T_sb(1:3, 4);

p_goal = p_tip + [0.0; 0.04; 0.0];

%% =========================
% Solver settings
%% =========================
max_iters = 10;
tol       = 1e-5;
optim     = optimoptions('lsqlin', 'Display', 'off', 'Algorithm', 'active-set');
r         = 0.003;

w_dist      = 1.0;
w_ori       = 1.0;

wall            = true;
wall_center     = p_goal + [0.00; -0.03; 0.00];
wall_normal     = [0.0, -1.0, 0.0];
wall_normal = wall_normal / norm(wall_normal);
%% =========================
% Solve pose IK using Jacobian Transpose
%% =========================
[theta_sol, theta_history, norm_err_hist] = constrained_linear_ls_pos_and_ori( ...
    M_tool, ...
    S_list, ...
    theta_a, ...
    p_goal, ...
    r, ...
    'max_iters', max_iters, ...
    'tol', tol, ...
    'q_min', q_min, ...
    'q_max', q_max, ...
    'w_dist', w_dist, ...
    'w_ori', w_ori, ...
    'optimizer', optim, ...
    'wall', wall, ...
    'wall_center', wall_center, ...
    'wall_normal', wall_normal);

%% =========================
% Print result
%% =========================
T_init = body_product_of_exponentials(M_tool, B_list, theta_history(1,:).');
T_final = body_product_of_exponentials(M_tool, B_list, theta_history(end,:).');
p_init = T_init(1:3,4);
p_final = T_final(1:3,4);

disp('Initial end-effector pose T_init =');
disp(T_init);

disp('Final end-effector pose T_final =');
disp(T_final);

disp('Solved joint angles [rad]:');
disp(theta_sol);

disp('Solved joint angles [deg]:');
disp(rad2deg(theta_sol));

fprintf('Initial tool-tip distance to goal: %.6f m\n', norm(p_init - p_goal));
fprintf('Final tool-tip distance to goal:   %.6f m\n', norm(p_final - p_goal));
fprintf('Goal tolerance:                    %.6f m\n', tol);

N = size(theta_history, 1);
fprintf('Total IK iterations stored: %d\n', N);

%% =========================
% Precompute pose / Jacobian / metrics
%% =========================
T_list      = zeros(4,4,N);
p_list      = zeros(3,N);

cond_w_hist = zeros(N,1);
cond_v_hist = zeros(N,1);

iso_w_hist  = zeros(N,1);
iso_v_hist  = zeros(N,1);

manip_hist  = zeros(N,1);

Jv_cell = cell(N,1);
Jw_cell = cell(N,1);

for k = 1:N
    theta_k = theta_history(k,:).';

    T_sb = body_product_of_exponentials(M_tool, B_list, theta_k);
    J_b  = body_jacobian(B_list, theta_k);

    T_list(:,:,k) = T_sb;
    p_list(:,k)   = T_sb(1:3,4);

    ell_data = manipulability_ellipsoid(J_b);

    % Angular part
    eigvals_w = ell_data{1}{2};
    if min(eigvals_w) < 1e-12
        iso_w_hist(k)  = inf;
        cond_w_hist(k) = inf;
    else
        iso_w_hist(k)  = sqrt(max(eigvals_w)/min(eigvals_w));
        cond_w_hist(k) = max(eigvals_w)/min(eigvals_w);
    end

    % Linear part
    eigvals_v = ell_data{2}{2};
    if min(eigvals_v) < 1e-12
        iso_v_hist(k)  = inf;
        cond_v_hist(k) = inf;
    else
        iso_v_hist(k)  = sqrt(max(eigvals_v)/min(eigvals_v));
        cond_v_hist(k) = max(eigvals_v)/min(eigvals_v);
    end

    manip_hist(k) = manipulability(J_b);

    Jw_cell{k} = J_b(1:3,:);
    Jv_cell{k} = J_b(4:6,:);
end

%% =========================
% Save videos
%% =========================
if wall 
    wall_viz = make_wall(wall_center, wall_normal);
else
    wall_viz = [];
end
num_snapshots = 100;

% plot_robot_motion_snapshots(robot, theta_history, T_list, wall, p_goal, r, num_snapshots)

% save_robot_motion_video(robot, theta_history, T_list, 'pos_and_ori_robot_motion.mp4', wall_viz, p_goal, r);

% save_metrics_video(T_list, Jw_cell, Jv_cell, ...
%     norm_w_b_hist, norm_v_b_hist, ...
%     cond_w_hist, cond_v_hist, ...
%     iso_w_hist, iso_v_hist,...
%     manip_hist, ...
%     'pos_and_ori_metrics.mp4');

% save_robot_motion_video2( ...
%     T_list, ...
%     'pos_and_ori_closeup_motion.mp4', ...
%     wall_viz, ...
%     p_goal, ...
%     r);
plot_robot_motion_snapshot2(T_list, wall_viz, p_goal, r, num_snapshots)

fprintf('Saved videos:\n');
fprintf('  - robot_motion.mp4\n');
fprintf('  - ik_metrics.mp4\n');
