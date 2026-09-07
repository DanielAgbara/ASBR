clc, clear, close all

[data_test, file_name] = loadTestData(4);

%% Compute expected values for C_i
max_iter = 100;
tol = 1e-3;

F_a = zeros(4, 4, data_test.calreadings.Nf);
F_d = zeros(4, 4, data_test.calreadings.Nf);
for i = 1 : data_test.calreadings.Nf
    F_a(:, :, i) = iterative_closest_point(data_test.calbody.a, data_test.calreadings.A(:, :, i), max_iter, tol);
    F_d(:, :, i) = iterative_closest_point(data_test.calbody.d, data_test.calreadings.D(:, :, i), max_iter, tol);
end

C_expected = zeros(size(data_test.calreadings.C));
for i = 1 : data_test.calreadings.Nf
    T_c = F_d(:, :, i)\F_a(:, :, i);
    C_expected(:, :, i) = T_c(1:3, 1:3) * data_test.calbody.c + T_c(1:3, 4);
end
%% Electromagnetic Pivot Calibration
G0 = mean(data_test.empivot.G(:, :, 1), 2);
g = data_test.empivot.G(:, :, 1) - G0;

F_g = zeros(4, 4, data_test.empivot.Nf);
for i = 1 : data_test.empivot.Nf
    F_g(:, :, i) = iterative_closest_point(g, data_test.empivot.G(:, :, i), max_iter, tol);
end

[em_tip, em_post] = pivot_calibration(F_g);

%% Optical Pivot Calibration
H_new = zeros(size(data_test.optpivot.H));
for i = 1 : data_test.optpivot.Nf
    F_d(:, :, i) = iterative_closest_point(data_test.calbody.d, data_test.optpivot.D(:, :, i), max_iter, tol);
    H_new(:, :, i) = F_d(1:3, 1:3, i) \ data_test.optpivot.H(:, :, i) - F_d(1:3, 1:3, i) \ F_d(1:3, 4, i);
end

H0 = mean(H_new(:, :, 1), 2);
h = H_new(:, :, 1) - H0;

F_h = zeros(4, 4, data_test.optpivot.Nf);
for i = 1 : data_test.optpivot.Nf
    F_h(:, :, i) = iterative_closest_point(h, H_new(:, :, i), max_iter, tol);
end

[opt_tip, opt_post] = pivot_calibration(F_h);

%% Save Test Data
saveOutputData(data_test.calbody.Nc, data_test.calreadings.Nf, em_post, opt_post, C_expected, file_name)