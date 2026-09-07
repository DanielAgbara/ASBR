clc; clear; close all;
export_videos = false; % Set true to render the simulation video.

%% Parameters
m = 1.0;
dt = 0.1;
T  = 20.0;
time = 0:dt:T;

K = 10.0;
B = 1.0;
tau = 2.0;
d_max = 0.5;
d_min = 0.15;
F_max = 0.5;

F_input_mag = 1.0;
omega = 0.0;

desired_video_time = 20;

N = length(time);



view_option = 'all';

%% Conical VF geometry
x_s = [0, 0, 0]';      % cone base / start point
x_t = [10, 0, 0]';     % target point
alpha = deg2rad(12); % cone half-angle [rad]

curve_vis = [
    linspace(x_s(1), x_t(1), 300)', ...
    linspace(x_s(2), x_t(2), 300)', ...
    linspace(x_s(3), x_t(3), 300)'
];

% Approximate cone radius at start point for visualizer tube mesh
R_vis = norm(x_t - x_s) * tan(alpha);

%% Initial state
p= [0; 3; 3];
v = [0; 0; 0];

p_prev = p;
t_prev = -dt;


%% Visualization initialization
vis = initConicalImpedanceVisualizer(x_s, x_t, alpha, p, view_option);

%% Simulation loop
log = struct();

rng(1);

for k = 1:length(time)
    t = time(k);

    %% ===== Axis-following user input + noise =====

    dir_to_target = x_t - p;
    dir_to_target = dir_to_target / norm(dir_to_target);
    
    F_input_base = F_input_mag * dir_to_target;
    % noise
    noise_mag = 1;
    F_noise = noise_mag * randn(3,1);
    F_disturb = zeros(3,1);

    if t > 6 && t < 7
        F_disturb = 2.0 * [0; 1; 1];
    end

    F_input = F_input_base + F_noise + F_disturb;

    %% ===== Virtual fixture =====
    [F, info] = conicalImpedance( ...
        p, t, p_prev, t_prev, x_s, x_t, alpha, K, B, tau, d_max, d_min, F_max);

    F_actual = F_input + F;
    a = F_actual / m;

    %% ===== Save log before state update =====
    log.t(k)          = t;
    log.p(:,k)        = p;
    log.v(:,k)        = v;

    log.F_input(:,k)  = F_input;
    log.F(:,k)        = F;
    log.F_actual(:,k) = F_actual;

    log.d(k)          = info.penetration;
    log.cone_dist(k)  = info.cone_dist;
    log.r_perp(k)     = info.n_perp_norm;
    log.speed(k)      = norm(v);
    log.F_spring(:, k) = info.F_spring;
    log.F_tangent(:, k) = info.F_tangent;

    a_axis = info.a;

    % Decompose input
    F_in_axis = dot(F_input, a_axis) * a_axis;
    F_in_norm = F_input - F_in_axis;

    % Decompose output
    F_out_axis = dot(F, a_axis) * a_axis;
    F_out_norm = F - F_out_axis;

    log.F_axis_interaction(k) = dot(F_out_axis, F_in_axis) / max(norm(F_in_axis), 1e-9);
    log.F_norm_interaction(k) = dot(F_out_norm, F_in_norm) / max(norm(F_in_norm), 1e-9);

    log.info(k) = info;

    %% ===== Dynamics update =====
    p_prev = p;
    t_prev = t;

    v = v + a * dt;
    p = p + v * dt;

    if dot(p - x_t, x_t - x_s) > 0
        break;
    end
end

%% Save videos after simulation
if export_videos
    saveSimulationVideoConical(log, x_s, x_t, alpha, view_option, 'conical_k10_b1.mp4', desired_video_time);
end
% saveMetricsVideoConical(log, x_s, x_t, alpha, 'conical_metrics.mp4', desired_video_time);
plotSimulationSnapshotsConical(log, x_s, x_t, alpha, view_option);
plotMetricsConical(log);
