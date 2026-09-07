function [F, s_c, info] = tubularImpedance(p_e, t, p_prev, t_prev, curve, s, R, K, B, tau, d_max, d_min, F_max)
% Input
% p_e   : (3x1) current end-effector position
% t     : current time
% p_prev: (3x1) previous end-effector position
% t_prev: previous time
% curve : 1x3 cell array of parametric curve functions
%         curve = {@(s)x(s); @(s)y(s); @(s)z(s)}
% s     : current curve parameter estimate
% R     : tube radius. Attraction force is applied only when d_norm >= R
% K     : stiffness gain for attraction force
% B     : damping gain
% tau   : ramp-up time for stiffness and tangential guidance force
% d_max : maximum distance used for saturated attraction displacement
% d_min : distance threshold for scaling tangential force near the curve
% F_max : maximum tangential guidance force magnitude
%
% Output
% F     : (3x1) computed tubular impedance force
% s_c   : updated curve parameter corresponding to the estimated closest point
% info  : structure containing intermediate variables for visualization/debugging
%         info.p_c          : estimated closest point on the curve
%         info.d            : vector from p_e to p_c
%         info.d_norm       : distance from p_e to p_c
%         info.u            : unit tangent vector at p_c
%         info.D            : saturated attraction displacement vector
%         info.k            : time-ramped stiffness
%         info.f_d          : distance-dependent tangent force scale
%         info.f_t          : time-dependent tangent force scale
%         info.F_attraction : attraction force component
%         info.F_tangent    : tangential guidance force component

[p_c, u, s_c] = closestPointOnCurve(p_e, curve, s);

% Time
if t < tau
    k = K*(t/tau);
    f_t = t/tau;
else
    k = K;
    f_t = 1;
end

% F_Attraction (Normal Compoent) of output F
d = p_c - p_e;
d_norm = norm(d);

if d_norm <= R
    D = zeros(3,1);

elseif d_norm <= R + d_max
    n = d / d_norm;
    D = (d_norm - R) * n;

else
    n = d / d_norm;
    D = d_max * n;
end

v_e = (p_e - p_prev)/(t - t_prev);

F_spring = k * D;
F_damping = -B * v_e;
F_attraction = F_spring + F_damping;

if d_norm >= d_min
    f_d = 1;
else
    f_d = d_norm / d_min;
end

F_tangent = f_d*f_t*F_max*u;

F = F_attraction + F_tangent;

info.p_c = p_c;
info.d = d;
info.d_norm = d_norm;
info.u = u;
info.D = D;
info.k = k;
info.f_d = f_d;
info.f_t = f_t;
info.F_spring = F_spring;
info.F_damping = F_damping;
info.F_attraction = F_attraction;
info.F_tangent = F_tangent;
end