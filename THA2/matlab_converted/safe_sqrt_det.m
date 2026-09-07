function val = safe_sqrt_det(A)
A = 0.5 * (A + A.');
d = det(A);
if ~isfinite(d) || d < 0
    if abs(d) < 1e-12
        d = 0;
    else
        d = 0;
    end
end
val = sqrt(d);
end