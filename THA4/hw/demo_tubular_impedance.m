clc; clear; close all;

rng(1); 

%% Parameters
m = 1.0;
dt = 0.1;
T  = 40.0;
time = 0:dt:T;

R = 0.25;
K = 20.0;
B = 3.0;
tau = 2.0;
d_max = 0.5;
d_min = 0.05;
F_max = 0.5;

F_input_mag = 1.0;

desired_video_time = 20;

N = length(time);

view_option = 'all';

%% Reference curve
s_min = 0;
s_max = 10;

curve = {
    @(s) s;
    @(s) sin(s);
    @(s) cos(s)
};

s_plot = linspace(s_min, s_max, 300);

curve_vis = [
    curve{1}(s_plot);
    curve{2}(s_plot);
    curve{3}(s_plot)
];

%% Initial state
p = [0, 2, 2]';
v = [0, 0, 0]';

p_prev = p;
t_prev = -dt;

%% Visualization initialization
vis = initTubularImpedanceVisualizer(curve_vis, p, R, view_option);

%% Simulation loop
log = struct();

s = 0;

for k = 1:length(time)
    t = time(k);

    %% ===== Tangent-following user input + noise =====
    s_user = min(max(s, s_min), s_max);

    ds = 1e-3;

    p_f = [
        curve{1}(s_user + ds);
        curve{2}(s_user + ds);
        curve{3}(s_user + ds)
    ];

    p_b = [
        curve{1}(s_user - ds);
        curve{2}(s_user - ds);
        curve{3}(s_user - ds)
    ];

    tangent = p_f - p_b;

    if norm(tangent) > 1e-9
        tangent = tangent / norm(tangent);
    else
        tangent = [1;0;0];
    end

    % noise
    noise_mag = 0.25;
    F_noise = noise_mag * randn(3,1);

    % disturbance (forces exit from tube)
    F_disturb = zeros(3,1);
    if t > 7 && t < 10
        F_disturb = 1.5 * [0; 1; 1];
    end

    if t > 15 && t < 20
        F_disturb = 2 * [0; -1; -1];
    end

    F_input = F_input_mag * tangent + F_noise + F_disturb;

    %% ===== Virtual fixture =====
    [F, s, info] = tubularImpedance( ...
        p, t, p_prev, t_prev, curve, s, R, K, B, tau, d_max, d_min, F_max);

    F_actual = F_input + F;
    a = F_actual / m;

    %% ===== Logging =====
    log.t(:,k)        = t;
    log.p(:,k)        = p;
    log.v(:,k)        = v;
    log.s(:,k)        = s;

    log.F_input(:,k)  = F_input;
    log.F_spring(:,k) = info.F_spring;
    log.F(:,k)        = F;
    log.F_actual(:,k) = F_actual;
    log.F_tangent(:,k)= info.F_tangent;

    log.d(k)          = info.d_norm;
    log.speed(k)      = norm(v);

    if norm(F_input) > 1e-9 && norm(F) > 1e-9
        log.force_align(k) = dot(F_input, F) / (norm(F_input)*norm(F));
    else
        log.force_align(k) = 0;
    end

    log.info(k) = info;

    %% ===== Dynamics =====
    p_prev = p;
    t_prev = t;

    v = v + a * dt;
    p = p + v * dt;

    if p(1) > s_max
        break
    end
end
%% Save videos after simulation
% saveSimulationVideoTubular(log, curve_vis, R, view_option, 'k10_b4.mp4', desired_video_time);
% saveMetricsVideoTubular(log, R, 'tubular_metrics.mp4', desired_video_time);
plotSimulationSnapshotsTubular(log, curve_vis, R, view_option);
% plotMetricsTubular(log, R)