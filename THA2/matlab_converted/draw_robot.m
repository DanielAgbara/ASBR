function draw_robot(ax, robot, q)

% Draw robot with frames ON
show(robot, q, ...
    'Parent', ax, ...
    'PreservePlot', false, ...
    'Frames', 'on');   % <-- joint frames 표시

hold(ax, 'on');

% ===== set transparency =====
patches = findobj(ax, 'Type', 'Patch');

if ~isempty(patches)
    for i = 1:length(patches)
        patches(i).FaceAlpha = 0.5;   % mesh transparency
        patches(i).EdgeAlpha = 1.0;
    end
end

end