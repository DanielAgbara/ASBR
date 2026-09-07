function export_kuka_preview
%EXPORT_KUKA_PREVIEW Render the bundled robot model for the README.
root = fileparts(fileparts(mfilename('fullpath')));
model = fullfile(root, 'THA2', 'kuka_lbr_iiwa_support', 'urdf', 'lbr_iiwa_14_r820.urdf');
robot = importrobot(model);
robot.DataFormat = 'column';
f = figure('Color','white', 'Position',[100 100 1000 750]);
ax = axes(f);
show(robot, deg2rad([20;45;-20;-60;10;50;0]), ...
    'Visuals','on', 'Frames','off', 'Parent',ax);
axis(ax,[-0.25 0.95 -0.6 0.6 -0.05 1.0]);
view(ax,135,20);
ax.Units = 'normalized';
ax.Position = [0.18 0.17 0.63 0.66];
if ~isempty(ax.Toolbar), ax.Toolbar.Visible='off'; end
annotation(f,'textbox',[0.05 0.93 0.9 0.05], 'String','THA2 | KUKA LBR iiwa 14 R820', ...
    'EdgeColor','none','HorizontalAlignment','center','FontSize',18,'FontWeight','bold');
drawnow;
exportgraphics(f, fullfile(root,'docs','assets','tha2-kuka.png'), 'Resolution',150);
close(f);
end
