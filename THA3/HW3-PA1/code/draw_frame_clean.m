function draw_frame_clean(ax, origin, R, L, name)

    if nargin < 3 || isempty(R)
        R = eye(3);
    end
    if nargin < 4 || isempty(L)
        L = 1;
    end
    if nargin < 5
        name = '';
    end

    origin = origin(:);
    colors = {'r','g','b'};

    hold(ax, 'on');

    for i = 1:3
        v = R(:,i) * L;
        quiver3(ax, origin(1), origin(2), origin(3), ...
            v(1), v(2), v(3), 0, ...
            'Color', colors{i}, ...
            'LineWidth', 2, ...
            'MaxHeadSize', 0.2, ...
            'HandleVisibility', 'off');
    end

    if ~isempty(name)
        text(ax, origin(1), origin(2), origin(3), ['  ' name], ...
            'FontWeight', 'bold', ...
            'HandleVisibility', 'off');
    end
end