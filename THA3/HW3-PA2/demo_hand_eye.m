%% Clear Data
clc, clear

[q_Robot_config, q_camera_config, t_Robot_config, t_camera_config] = data_quaternion();
q_robot_nf  = q_Robot_config(:, [4 1 2 3])';
q_camera_nf = q_camera_config(:, [4 1 2 3])';
t_robot_nf = t_Robot_config';
t_camera_nf = t_camera_config';

[q_A, q_B, t_A, t_B] = getRelativeQuaternion(q_robot_nf, q_camera_nf, t_robot_nf, t_camera_nf);
X_nf = eyeInHandQuaternion(q_A, q_B, t_A, t_B);

fprintf('=== Noise-free ===\n');
fprintf('Transformation Matrix X: \n')
disp(X_nf);

[q_Robot_config, q_camera_config, t_Robot_config, t_camera_config] = data_quaternion_noisy();
q_robot_ny  = q_Robot_config(:, [4 1 2 3])';
q_camera_ny = q_camera_config(:, [4 1 2 3])';
t_robot_ny = t_Robot_config';
t_camera_ny = t_camera_config';

[q_A, q_B, t_A, t_B] = getRelativeQuaternion(q_robot_ny, q_camera_ny, t_robot_ny, t_camera_ny);
X_ny = eyeInHandQuaternion(q_A, q_B, t_A, t_B);

fprintf('=== Noisy ===\n');
fprintf('Transformation Matrix X: \n')
disp(X_ny);

result = visualizeEyeInHandChain( ...
    q_robot_nf, q_camera_nf, t_robot_nf, t_camera_nf, X_nf, ...
    q_robot_ny, q_camera_ny, t_robot_ny, t_camera_ny, X_ny);

fprintf('Mean object position in base frame (noise-free):\n');
disp(result.noise_free.p_o_mean);

fprintf('Position error to mean (noise-free):\n');
fprintf('  mean = %.6f\n', mean(result.noise_free.pos_err));
fprintf('  max  = %.6f\n', max(result.noise_free.pos_err));
fprintf('  std  = %.6f\n', std(result.noise_free.pos_err));

fprintf('Rotation error relative to first object pose (noise-free):\n');
fprintf('  mean = %.6f deg\n', mean(result.noise_free.rot_err_deg));
fprintf('  max  = %.6f deg\n', max(result.noise_free.rot_err_deg));

fprintf('\n');

fprintf('Mean object position in base frame (noisy):\n');
disp(result.noisy.p_o_mean);

fprintf('Position error to mean (noisy):\n');
fprintf('  mean = %.6f\n', mean(result.noisy.pos_err));
fprintf('  max  = %.6f\n', max(result.noisy.pos_err));
fprintf('  std  = %.6f\n', std(result.noisy.pos_err));

fprintf('Rotation error relative to first object pose (noisy):\n');
fprintf('  mean = %.6f deg\n', mean(result.noisy.rot_err_deg));
fprintf('  max  = %.6f deg\n', max(result.noisy.rot_err_deg));
