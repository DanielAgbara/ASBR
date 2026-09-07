function draw_frame(ax, origin, R, L, name)
% Draw a coordinate frame at 'origin' with orientation R.
% Columns of R are the axis directions in world coordinates.
%
% Inputs:
%   ax     - axes handle
%   origin - 3x1 or 1x3 origin
%   R      - 3x3 rotation matrix (optional, default = eye(3))
%   L      - axis length (optional, default = 0.5)
%   name   - frame label (optional)

    if nargin < 3 || isempty(R)
        R = eye(3);
    end
    if nargin < 4 || isempty(L)
        L = 0.5;
    end
    if nargin < 5
        name = [];
    end

    origin = double(origin(:));
    colors = {'r', 'g', 'b'};  % x,y,z

    hold(ax, 'on');

    for i = 1:3
        v = R(:, i) * L;
        quiver3(ax, origin(1), origin(2), origin(3), ...
                v(1), v(2), v(3), ...
                0, ...
                'Color', colors{i}, ...
                'LineWidth', 2, ...
                'MaxHeadSize', 0.15);
    end

    if ~isempty(name)
        text(ax, origin(1), origin(2), origin(3), name);
    end
end


