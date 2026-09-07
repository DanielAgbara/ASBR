function draw_robot(ax, robot, q)

% Draw robot with frames ON
show(robot, q, ...
    'Parent', ax, ...
    'PreservePlot', false, ...
    'Frames', 'on');

hold(ax, 'on');

% ===== set transparency =====
patches = findobj(ax, 'Type', 'Patch');

if ~isempty(patches)
    for i = 1:length(patches)
        patches(i).FaceAlpha = 0.3;   % mesh transparency
        patches(i).EdgeAlpha = 0.8;
    end
end

end