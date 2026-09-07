function [q_A, q_B, t_A, t_B] = getRelativeQuaternion(q_rob, q_cam, t_rob, t_cam)
data_num = size(q_rob, 2);
T_rob = repmat(eye(4), 1, 1, data_num);
T_cam = repmat(eye(4), 1, 1, data_num);

for i = 1 : data_num
    T_rob(1:3, 1:3, i) = quaternionToR(q_rob(:, i));
    T_rob(1:3, 4, i) = t_rob(:, i);
    T_cam(1:3, 1:3, i) = quaternionToR(q_cam(:, i));
    T_cam(1:3, 4, i) = t_cam(:, i);
end

A = zeros(4, 4, data_num - 1);
B = zeros(4, 4, data_num - 1);
q_A = zeros(4, data_num - 1);
q_B = zeros(4, data_num - 1);
t_A = zeros(3, data_num - 1);
t_B = zeros(3, data_num - 1);
for i = 1 : data_num - 1
    A(:, :, i) = invSE3(T_rob(:, :, i)) * T_rob(:, :, i + 1);
    B(:, :, i) = T_cam(:, :, i) * invSE3(T_cam(:, :, i + 1));
    q_A(:, i) = RToQuaternion(A(1:3, 1:3, i));
    q_B(:, i) = RToQuaternion(B(1:3, 1:3, i));
    t_A(:, i) = A(1:3, 4, i);
    t_B(:, i) = B(1:3, 4, i);
end

end
