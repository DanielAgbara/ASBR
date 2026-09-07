function y = replace_inf(y)
    finite_vals = y(isfinite(y));
    if isempty(finite_vals)
        y(:) = 0;
        return;
    end

    ymax = max(finite_vals);
    y(~isfinite(y)) = 1.1 * ymax; % 살짝 위
end