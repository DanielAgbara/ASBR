%% =========================
%  PA_h_make_videos.m
%  Simple pose IK test
%% =========================
clear; clc; close all;
export_videos = false; % Set true to render MP4 diagnostics after solving.

%% =========================
% Import URDF robot model
%% =========================
urdfPath = fullfile(fileparts(which('demo_redundancy_resolution')), '..', 'kuka_lbr_iiwa_support', 'urdf', 'lbr_iiwa_14_r820.urdf');
robot = importrobot(urdfPath);
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.81];

%% =========================
% Joint limits
%% =========================
q_min = deg2rad([-170 -120 -170 -120 -170 -120 -175]).';
q_max = deg2rad([ 170  120  170  120  170  120  175]).';

%% =========================
% KUKA LBR iiwa 14 R820 data
%% =========================
M_ee = [1 0 0 0;
        0 1 0 0;
        0 0 1 1.306;
        0 0 0 1];

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
B_mat = adjoint_inverse(M_ee) * S_mat;

% If your MATLAB functions expect cell arrays:
S_list = {S_mat(:,1), S_mat(:,2), S_mat(:,3), S_mat(:,4), ...
          S_mat(:,5), S_mat(:,6), S_mat(:,7)};

B_list = {B_mat(:,1), B_mat(:,2), B_mat(:,3), B_mat(:,4), ...
          B_mat(:,5), B_mat(:,6), B_mat(:,7)};

%% ========= ================
% Initial configuration
%% =========================
% theta_a = deg2rad([20; 45; -20; -60; 10; 50; 0]);
theta_a = deg2rad([0, 110, 0, -100, 0, 110, 0]');
%% =========================
% Desired target pose
%% =========================
% R_sd = Rz(pi/3)*Ry(pi/3)*Rx(pi/3);
% p_sd = [0.45; 0.05; 0.75];
R_sd = eye(3);
p_sd = [0.52, 0, 0.58]';

T_sd = [R_sd, p_sd;
        0 0 0 1];

%% =========================
% Solver settings
%% =========================
max_iters = 101;
tol_w     = 1e-6;
tol_v     = 1e-6;
k0 = 1e17;
secondary_mode = 'joint_limits_only';
%% =========================
% Solve pose IK
%% =========================
[theta_sol, theta_history,norm_w_b_hist, norm_v_b_hist] = redundancy_resolution( ...
    M_ee, ...
    B_list, ...
    theta_a, ...
    T_sd, ...
    max_iters, tol_w, tol_v, ...
    q_min, q_max, k0, secondary_mode);

%% =========================
% Print result
%% =========================
T_init = body_product_of_exponentials(M_ee, B_list, theta_history(1,:).');
T_final = body_product_of_exponentials(M_ee, B_list, theta_history(end,:).');

disp('Initial end-effector pose T_init =');
disp(T_init);

disp('Final end-effector pose T_final =');
disp(T_final);

disp('Solved joint angles [rad]:');
disp(theta_sol);

disp('Solved joint angles [deg]:');
disp(rad2deg(theta_sol));

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

    T_sb = body_product_of_exponentials(M_ee, B_list, theta_k);
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
% save_robot_motion_video(robot, theta_history, T_list, Jw_cell, Jv_cell, 'redun_robot_motion.mp4');

if export_videos
save_metrics_video2(theta_history, T_list, Jw_cell, Jv_cell, ...
    norm_w_b_hist, norm_v_b_hist, ...
    cond_w_hist, cond_v_hist, ...
    iso_w_hist, iso_v_hist,...
    manip_hist, ...
    'redun_metrics.mp4');

fprintf('Saved videos:\n');
fprintf('  - redun_metrics.mp4\n');
end
