function x_cap = cap_inf(x, xmax)
x_cap = x;
x_cap(~isfinite(x_cap)) = xmax;
x_cap(x_cap > xmax) = xmax;
end