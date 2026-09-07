function T = eyeInHandQuaternion(q_A, q_B, t_A, t_B)
if ~(size(q_B, 2) == size(t_B, 2) ...
        && size(q_B, 2) == size(q_A, 2) ...
        && size(q_B, 2) == size(t_A, 2))
    error("The number of data does not match!")
else
    num_data = size(q_B, 2);
end

% Solving for R
s_A = q_A(1, :);
v_A = q_A(2:4, :);
s_B = q_B(1, :);
v_B = q_B(2:4, :);

M = zeros(num_data * 4, 4);

for i = 1 : num_data
    M(4*i-3, 1) = s_A(i) - s_B(i);
    M(4*i-3, 2:4) = -(v_A(:, i) - v_B(:, i))';
    M(4*i-2:4*i, 1) = v_A(:, i) - v_B(:, i);
    M(4*i-2:4*i, 2:4) = (s_A(i) - s_B(i)) * eye(3) + skew(v_A(:, i) + v_B(:, i));
end

[~, ~, V] = svd(M);

q = V(:, end);
R = quaternionToR(q);

% Solving for p
lht_stack = zeros(3 * num_data, 3);
rht_stack = zeros(3 * num_data, 1);
for i = 1 : num_data
    lht_stack(3*i-2:3*i, :) = quaternionToR(q_A(:, i)) - eye(3);
    rht_stack(3*i-2:3*i, 1) = R*t_B(:, i) - t_A(:, i);
end

p = lht_stack \ rht_stack;

T = eye(4);
T(1:3, 1:3) = R;
T(1:3, 4) = p;

ax_xb_err = zeros(1, num_data);

for i = 1:num_data
    RA = quaternionToR(q_A(:,i));
    RB = quaternionToR(q_B(:,i));

    Ai = eye(4);
    Ai(1:3,1:3) = RA;
    Ai(1:3,4)   = t_A(:,i);

    Bi = eye(4);
    Bi(1:3,1:3) = RB;
    Bi(1:3,4)   = t_B(:,i);

    ax_xb_err(i) = norm(Ai*T - T*Bi, 'fro');
end

end