function [p_c, t_c, s_c] = closestPointOnCurve(p_e, parametricCurve, s)
% Input
% p_e             : (3x1) current end-effector position
% parametricCurve : 3x1 cell array of parametric curve functions
%                   parametricCurve = {@(s)x(s), @(s)y(s), @(s)z(s)}
% s               : current curve parameter estimate
%
% Output
% p_c  : (3x1) estimated closest point on the curve, r(s_c)
% t_c  : (3x1) unit tangent vector at the estimated closest point
% s_c  : updated curve parameter, s_c = s + Delta_s

    h = 1e-6;
    r = evalCurve(parametricCurve, s);
    r_prime = numericalDerivative(parametricCurve, s, h);

    r_prime_norm = norm(r_prime);
    if r_prime_norm < 1e-12
        error('Curve derivative is too small.');
    end

    t = r_prime / r_prime_norm;
    delta_r = p_e - r;
    Delta_s = dot(t, delta_r) / r_prime_norm;
    s_c = s + Delta_s;
    p_c = evalCurve(parametricCurve, s_c);
    r_prime_c = numericalDerivative(parametricCurve, s_c, h);
    t_c = r_prime_c / norm(r_prime_c);
end

function p = evalCurve(parametricCurve, s)
% Evaluate the parametric curve at parameter s.
%
% Input
% parametricCurve : 1x3 cell array {@(s)x(s), @(s)y(s), @(s)z(s)}
% s               : curve parameter
%
% Output
% p               : (1x3) curve point r(s)
    p = [
        parametricCurve{1}(s);
        parametricCurve{2}(s);
        parametricCurve{3}(s)
    ];
end

function r_prime = numericalDerivative(parametricCurve, s, h)
% Compute numerical derivative r'(s) using central difference.
%
% Input
% parametricCurve : 1x3 cell array {@(s)x(s), @(s)y(s), @(s)z(s)}
% s               : curve parameter
% h               : finite difference step size
%
% Output
% r_prime         : (1x3) numerical derivative of the curve at s
    r_plus = evalCurve(parametricCurve, s + h);
    r_minus = evalCurve(parametricCurve, s - h);
    r_prime = (r_plus - r_minus) / (2*h);
end