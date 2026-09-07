%% ============================================================
% Load Data
% ============================================================

clc; clear; close all;

[data_debug, file_name] = loadDebugData(3);

max_iter = 100;
tol = 1e-3;

%% ============================================================
% 1. Compute Expected C
% ============================================================

F_a = zeros(4, 4, data_debug.calreadings.Nf);
F_d = zeros(4, 4, data_debug.calreadings.Nf);

for i = 1:data_debug.calreadings.Nf
    % ICP: calibration object (optical)
    F_a(:, :, i) = iterative_closest_point( ...
        data_debug.calbody.a, ...
        data_debug.calreadings.A(:, :, i), ...
        max_iter, tol);

    % ICP: EM base
    F_d(:, :, i) = iterative_closest_point( ...
        data_debug.calbody.d, ...
        data_debug.calreadings.D(:, :, i), ...
        max_iter, tol);
end

% Compute expected C
C_expected = zeros(size(data_debug.calreadings.C));

for i = 1:data_debug.calreadings.Nf
    T_c = F_d(:, :, i) \ F_a(:, :, i);   % F_d^{-1} * F_a
    C_expected(:, :, i) = ...
        T_c(1:3, 1:3) * data_debug.calbody.c + T_c(1:3, 4);
end

%% ============================================================
% 2. EM Pivot Calibration
% ============================================================

G0 = mean(data_debug.empivot.G(:, :, 1), 2);
g = data_debug.empivot.G(:, :, 1) - G0;

F_g = zeros(4, 4, data_debug.empivot.Nf);

for i = 1:data_debug.empivot.Nf
    F_g(:, :, i) = iterative_closest_point(g, ...
        data_debug.empivot.G(:, :, i), ...
        max_iter, tol);
end

[em_tip, em_post] = pivot_calibration(F_g);

em_post_real = [data_debug.output.em_x;
                data_debug.output.em_y;
                data_debug.output.em_z];

%% ============================================================
% 3. Optical Pivot Calibration
% ============================================================

H_new = zeros(size(data_debug.optpivot.H));

for i = 1:data_debug.optpivot.Nf
    F_d(:, :, i) = iterative_closest_point( ...
        data_debug.calbody.d, ...
        data_debug.optpivot.D(:, :, i), ...
        max_iter, tol);

    % Transform H into EM frame
    H_new(:, :, i) = F_d(1:3,1:3,i)' * ...
        (data_debug.optpivot.H(:, :, i) - F_d(1:3,4,i));
end

H0 = mean(H_new(:, :, 1), 2);
h = H_new(:, :, 1) - H0;

F_h = zeros(4, 4, data_debug.optpivot.Nf);

for i = 1:data_debug.optpivot.Nf
    F_h(:, :, i) = iterative_closest_point(h, ...
        H_new(:, :, i), ...
        max_iter, tol);
end

[opt_tip, opt_post] = pivot_calibration(F_h);

opt_post_real = [data_debug.output.opt_x;
                 data_debug.output.opt_y;
                 data_debug.output.opt_z];

%% ============================================================
% 4. Error Reporting
% ============================================================

fprintf('\n================ REGISTRATION REPORT ================\n');

%% ---- C_expected errors ----
C_err = C_expected - data_debug.output.C;
C_err_abs = abs(C_err);

C_mean_abs = mean(C_err_abs, 'all');
C_rms = sqrt(mean(C_err.^2, 'all'));
C_max_abs = max(C_err_abs, [], 'all');

fprintf('\n[Expected C]\n');
fprintf('Mean absolute error: %0.6f\n', C_mean_abs);
fprintf('RMS error          : %0.6f\n', C_rms);
fprintf('Max absolute error : %0.6f\n', C_max_abs);

% Per-frame RMS
C_frame_rms = zeros(data_debug.calreadings.Nf, 1);

for i = 1:data_debug.calreadings.Nf
    Ei = C_err(:, :, i);
    C_frame_rms(i) = sqrt(mean(Ei(:).^2));
end

fprintf('\nPer-frame RMS error for C_expected:\n');
disp(table((1:data_debug.calreadings.Nf).', C_frame_rms, ...
    'VariableNames', {'Frame', 'RMS_Error'}));

%% ---- EM Pivot errors ----
em_err = em_post - em_post_real;
em_err_norm = norm(em_err);

fprintf('\n[EM Pivot Calibration]\n');
fprintf('Estimated post    : [%0.6f, %0.6f, %0.6f]\n', em_post);
fprintf('Ground truth post : [%0.6f, %0.6f, %0.6f]\n', em_post_real);
fprintf('Component error   : [%0.6f, %0.6f, %0.6f]\n', em_err);
fprintf('Error norm        : %0.6f\n', em_err_norm);

fprintf('Estimated tip     : [%0.6f, %0.6f, %0.6f]\n', em_tip);

%% ---- Optical Pivot errors ----
opt_err = opt_post - opt_post_real;
opt_err_norm = norm(opt_err);

fprintf('\n[Optical Pivot Calibration]\n');
fprintf('Estimated post    : [%0.6f, %0.6f, %0.6f]\n', opt_post);
fprintf('Ground truth post : [%0.6f, %0.6f, %0.6f]\n', opt_post_real);
fprintf('Component error   : [%0.6f, %0.6f, %0.6f]\n', opt_err);
fprintf('Error norm        : %0.6f\n', opt_err_norm);

fprintf('Estimated tip     : [%0.6f, %0.6f, %0.6f]\n', opt_tip);

fprintf('\n=====================================================\n');

%% ============================================================
% 5. Optional: Save Report to File
% ============================================================

fid = fopen('registration_report.txt', 'w');

fprintf(fid, '===== Expected C =====\n');
fprintf(fid, 'Mean abs error: %0.6f\n', C_mean_abs);
fprintf(fid, 'RMS error     : %0.6f\n', C_rms);
fprintf(fid, 'Max abs error : %0.6f\n\n', C_max_abs);

fprintf(fid, '===== EM Pivot =====\n');
fprintf(fid, 'Error norm: %0.6f\n\n', em_err_norm);

fprintf(fid, '===== Optical Pivot =====\n');
fprintf(fid, 'Error norm: %0.6f\n', opt_err_norm);

fclose(fid);