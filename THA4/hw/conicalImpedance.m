function [F, info] = conicalImpedance(x_a, t, p_prev, t_prev, x_s, x_t, alpha, K, B, tau, d_max, d_min, F_max)
% Input
% x_a    : (3x1) current end-effector position
% t      : current time
% p_prev : (3x1) previous end-effector position
% t_prev : previous time
% x_s    : (3x1) start point of the cone axis
% x_t    : (3x1) target/apex point of the cone axis
% alpha  : cone half-angle [rad]
% K      : stiffness gain for attraction force
% B      : damping gain
% tau    : ramp-up time for stiffness and tangential guidance force
% d_max  : maximum distance used for saturated attraction displacement
% d_min  : distance threshold for scaling tangential force near the axis
% F_max  : maximum tangential guidance force magnitude
%
% Output
% F     : (3x1) computed conical impedance force
% info  : structure containing intermediate variables for visualization/debugging

x_a    = x_a(:);
p_prev = p_prev(:);
x_s    = x_s(:);
x_t    = x_t(:);

% Time ramp
if t < tau
    k = K * (t / tau);
    f_t = t / tau;
else
    k = K;
    f_t = 1;
end

% Cone axis unit direction
axis_vec = x_t - x_s;
axis_len = norm(axis_vec);

if axis_len < 1e-12
    error('x_s and x_t are identical. Cannot define cone axis.');
end

a = axis_vec / axis_len;

r_apex = x_t - x_a;

n_parallel = (a * a') * r_apex;
n_perp     = (eye(3) - a * a') * r_apex;

n_perp_norm = norm(n_perp);

cone_dist = tan(alpha) * norm(n_parallel);
n_hat_perp = n_perp / n_perp_norm;

if n_perp_norm <= cone_dist
    penetration = 0;
    D = zeros(3,1);

elseif n_perp_norm <= cone_dist + d_max
    penetration = n_perp_norm - cone_dist;
    D = penetration * n_hat_perp;

else
    penetration = n_perp_norm - cone_dist;
    D = d_max * n_hat_perp;
end

v_e = (x_a - p_prev) / (t - t_prev);

F_spring = k * D;
F_damping = -B * v_e;
F_attraction = F_spring + F_damping;

if n_perp_norm >= d_min
    f_d = 1;
else
    f_d = n_perp_norm / d_min;
end

F_tangent = f_d * f_t * F_max * a;

F = F_attraction + F_tangent;
p_c = x_a + n_perp;

% Debug / visualization info
info.a = a;
info.p_c = p_c;
info.n_parallel = n_parallel;
info.n_perp = n_perp;
info.n_perp_norm = n_perp_norm;
info.n_hat_perp = n_hat_perp;
info.cone_dist = cone_dist;
info.penetration = penetration;
info.D = D;
info.k = k;
info.f_d = f_d;
info.f_t = f_t;
info.F_spring = F_spring;
info.F_damping = F_damping;
info.F_attraction = F_attraction;
info.F_tangent = F_tangent;
end