function ticks = nice_3ticks(y)
    y = y(isfinite(y));
    if isempty(y)
        ticks = [0 0.5 1];
        return;
    end

    ymax = max(y);

    if ymax == 0
        ticks = [0 0.5 1];
        return;
    end

    % 👉 핵심: ymax 기반 step
    raw_step = ymax / 2;

    step = nice_step(raw_step);

    % 0부터 시작
    t_low  = 0;
    t_mid  = step;
    t_high = ceil(ymax / step) * step;

    ticks = [t_low, t_mid, t_high];

    % 중복 제거
    ticks = unique(ticks, 'stable');

    % 보정 (혹시 step 너무 커서 2개만 나오는 경우)
    if numel(ticks) < 3
        ticks = [0, t_high/2, t_high];
    end
end

function step = nice_step(x)
    p = 10^floor(log10(x));
    f = x / p;

    if f <= 1
        nf = 1;
    elseif f <= 2
        nf = 2;
    elseif f <= 5
        nf = 5;
    else
        nf = 10;
    end

    step = nf * p;
end