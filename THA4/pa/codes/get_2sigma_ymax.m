function ymax = get_2sigma_ymax(x)
x = x(isfinite(x));

if isempty(x)
    ymax = 1;
    return;
end

mu = mean(x);
sigma = std(x);

ymax = mu + 2*sigma;

if ~isfinite(ymax) || ymax <= 0
    ymax = max(x);
end

if ymax <= 0
    ymax = 1;
end

% Ensure current max is visible if it is below 2 sigma bound logic
ymax = max(ymax, max(x));
ymax = 1.05 * ymax;
end