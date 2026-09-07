function [q_S, t_S, q_X, t_X, q_E, t_E, q_A, t_A, q_B, t_B, ...
          S_all, X_true, E_all, A_all, B_all] = ...
          generateEyeInHandPoseTestData(num_poses)
% generateEyeInHandPoseTestData
%
% Generate synthetic eye-in-hand test data using absolute poses:
%   S_i : fixed object -> camera
%   X   : camera -> gripper   (constant)
%   E_i : gripper -> base
%
% Such that:
%   E_i * X * S_i = C   (constant for all i)
%
% Then relative motions are computed as:
%   A_i = E_i^{-1} * E_{i+1}
%   B_i = S_i * S_{i+1}^{-1}
%
% so that:
%   A_i * X = X * B_i
%
% Quaternion convention:
%   q = [q0; q1; q2; q3]   (scalar-first)
%
% Inputs:
%   num_poses : number of absolute poses (must be >= 2)
%
% Outputs:
%   q_S, t_S : 4xN, 3xN  for S_i
%   q_X, t_X : 4x1, 3x1  for X
%   q_E, t_E : 4xN, 3xN  for E_i
%   q_A, t_A : 4x(N-1), 3x(N-1) for A_i
%   q_B, t_B : 4x(N-1), 3x(N-1) for B_i
%   S_all    : 4x4xN
%   X_true   : 4x4
%   E_all    : 4x4xN
%   A_all    : 4x4x(N-1)
%   B_all    : 4x4x(N-1)

    if nargin < 1
        num_poses = 10;
    end

    if num_poses < 2
        error('num_poses must be at least 2.');
    end

    % ------------------------------------------------------------
    % 1) Choose constant X = camera -> gripper
    % ------------------------------------------------------------
    R_X = axangToRotm([0; 0; 1], deg2rad(25)) * ...
          axangToRotm([0; 1; 0], deg2rad(-15));
    t_X = [0.04; -0.02; 0.10];

    X_true = eye(4);
    X_true(1:3,1:3) = R_X;
    X_true(1:3,4)   = t_X;

    q_X = rotmToQuaternion(R_X);
    t_X = t_X;

    % ------------------------------------------------------------
    % 2) Choose constant world/object relationship:
    %    C = object -> base  (constant)
    %    E_i * X * S_i = C
    % -> S_i = X^{-1} * E_i^{-1} * C
    % ------------------------------------------------------------
    R_C = axangToRotm([1; 0; 0], deg2rad(10)) * ...
          axangToRotm([0; 1; 0], deg2rad(20));
    t_C = [0.65; 0.15; 0.30];

    C = eye(4);
    C(1:3,1:3) = R_C;
    C(1:3,4)   = t_C;

    X_inv = invSE3(X_true);

    % ------------------------------------------------------------
    % 3) Construct deterministic E_i = gripper -> base
    %    (no random; use simple varying constants)
    % ------------------------------------------------------------
    E_all = zeros(4,4,num_poses);
    S_all = zeros(4,4,num_poses);

    q_E = zeros(4, num_poses);
    t_E = zeros(3, num_poses);

    q_S = zeros(4, num_poses);
    t_S = zeros(3, num_poses);

    for i = 1:num_poses
        % Example deterministic motion trajectory
        ang_z = deg2rad(-20 + 8*(i-1));
        ang_y = deg2rad( 10 - 3*(i-1));
        ang_x = deg2rad(  5 + 2*(i-1));

        R_E = axangToRotm([0;0;1], ang_z) * ...
              axangToRotm([0;1;0], ang_y) * ...
              axangToRotm([1;0;0], ang_x);

        p_E = [0.45 + 0.03*(i-1);
               -0.10 + 0.02*(i-1);
                0.55 - 0.015*(i-1)];

        E_i = eye(4);
        E_i(1:3,1:3) = R_E;
        E_i(1:3,4)   = p_E;

        % From E_i * X * S_i = C
        S_i = X_inv * invSE3(E_i) * C;

        E_all(:,:,i) = E_i;
        S_all(:,:,i) = S_i;

        q_E(:,i) = rotmToQuaternion(E_i(1:3,1:3));
        t_E(:,i) = E_i(1:3,4);

        q_S(:,i) = rotmToQuaternion(S_i(1:3,1:3));
        t_S(:,i) = S_i(1:3,4);
    end

    % ------------------------------------------------------------
    % 4) Relative motions
    %    A_i = E_i^{-1} E_{i+1}
    %    B_i = S_i S_{i+1}^{-1}
    % ------------------------------------------------------------
    q_A = zeros(4, num_poses-1);
    t_A = zeros(3, num_poses-1);

    q_B = zeros(4, num_poses-1);
    t_B = zeros(3, num_poses-1);

    A_all = zeros(4,4,num_poses-1);
    B_all = zeros(4,4,num_poses-1);

    for i = 1:num_poses-1
        A_i = invSE3(E_all(:,:,i)) * E_all(:,:,i+1);
        B_i = S_all(:,:,i) * invSE3(S_all(:,:,i+1));

        A_all(:,:,i) = A_i;
        B_all(:,:,i) = B_i;

        q_A(:,i) = rotmToQuaternion(A_i(1:3,1:3));
        t_A(:,i) = A_i(1:3,4);

        q_B(:,i) = rotmToQuaternion(B_i(1:3,1:3));
        t_B(:,i) = B_i(1:3,4);
    end
end

function Tinv = invSE3(T)
    R = T(1:3,1:3);
    t = T(1:3,4);

    Tinv = eye(4);
    Tinv(1:3,1:3) = R.';
    Tinv(1:3,4)   = -R.' * t;
end

function R = axangToRotm(axis, angle)
    axis = axis / norm(axis);
    x = axis(1); y = axis(2); z = axis(3);

    K = [  0  -z   y;
           z   0  -x;
          -y   x   0 ];

    R = eye(3) + sin(angle)*K + (1-cos(angle))*(K*K);
end

function q = rotmToQuaternion(R)
% q = [q0; q1; q2; q3], scalar-first

    tr = trace(R);

    if tr > 0
        S = sqrt(tr + 1.0) * 2;
        q0 = 0.25 * S;
        q1 = (R(3,2) - R(2,3)) / S;
        q2 = (R(1,3) - R(3,1)) / S;
        q3 = (R(2,1) - R(1,2)) / S;
    elseif (R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
        S = sqrt(1.0 + R(1,1) - R(2,2) - R(3,3)) * 2;
        q0 = (R(3,2) - R(2,3)) / S;
        q1 = 0.25 * S;
        q2 = (R(1,2) + R(2,1)) / S;
        q3 = (R(1,3) + R(3,1)) / S;
    elseif R(2,2) > R(3,3)
        S = sqrt(1.0 + R(2,2) - R(1,1) - R(3,3)) * 2;
        q0 = (R(1,3) - R(3,1)) / S;
        q1 = (R(1,2) + R(2,1)) / S;
        q2 = 0.25 * S;
        q3 = (R(2,3) + R(3,2)) / S;
    else
        S = sqrt(1.0 + R(3,3) - R(1,1) - R(2,2)) * 2;
        q0 = (R(2,1) - R(1,2)) / S;
        q1 = (R(1,3) + R(3,1)) / S;
        q2 = (R(2,3) + R(3,2)) / S;
        q3 = 0.25 * S;
    end

    q = [q0; q1; q2; q3];
    q = q / norm(q);

    if q(1) < 0
        q = -q;
    end
end