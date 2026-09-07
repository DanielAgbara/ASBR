clc; clear; close all;

%% Data
datasets = {'a','b','c','d','e','f','g'};

rms1 = [0.003, 0.284, 0.2467, 0.0076, 0.9996, 1.1296, 1.1891];
rms2 = [0.0033, 0.004, 0.9888, 0.0034, 1.3004, 2.7325, 3.0461];
rms2 = [0.0046, 0.0039, 0.0038, 0.0078, 0.0074, 0.0055, 0.0072];

%% Correlation
R = corrcoef(rms1, rms2);
corr_val = R(1,2);

fprintf('Correlation coefficient = %.4f\n', corr_val);

%% Figure 1: Line comparison
figure(1); clf; hold on; grid on;

plot(1:length(rms1), rms1, '-o', 'LineWidth', 2, 'DisplayName', 'RMS 1');
plot(1:length(rms2), rms2, '-s', 'LineWidth', 2, 'DisplayName', 'RMS 2');

xticks(1:length(datasets));
xticklabels(datasets);

xlabel('Dataset');
ylabel('RMS Error');
title('RMS Error Comparison');
legend('Location','best');

%% Figure 2: Scatter + correlation
figure(2); clf; hold on; grid on;

%% Correlation
R = corrcoef(rms1, rms2);
r = R(1,2);

%% Linear fit
p = polyfit(rms1, rms2, 1);
x_fit = linspace(min(rms1), max(rms1), 100);
y_fit = polyval(p, x_fit);

%% Plot

scatter(rms1, rms2, 80, 'filled');

% dataset labels
for i = 1:length(datasets)
    text(rms1(i), rms2(i), ['  ' datasets{i}], 'FontSize', 12);
end

% regression line
plot(x_fit, y_fit, 'LineWidth', 2);

xlabel('Registration RMS');
ylabel('Optical Pivot Calibration RMS');
title(sprintf('corr = %.3f, slope = %.3f', r, p(1)));

legend({'Data', 'Linear fit'}, 'Location','best');
axis tight;
set(findall(gcf,'-property','FontSize'),'FontSize',20)