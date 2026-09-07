function verify_matlab
%VERIFY_MATLAB Numerical smoke checks and selected reproducible demo figures.
root = fileparts(fileparts(mfilename('fullpath')));
originalDirectory = pwd;
originalPath = path;
cleanup = onCleanup(@() restore(originalDirectory, originalPath)); %#ok<NASGU>
cd(root); addpath(root);
assets = fullfile(root, 'docs', 'assets');
if ~isfolder(assets), mkdir(assets); end

addpath(fullfile(root, 'THA3', 'HW3-PA1', 'code'));
points = [0 1 0 0 1; 0 0 1 0 2; 0 0 0 1 3];
R = [0 -1 0; 1 0 0; 0 0 1];
p = [2;3;4];
T = maximum_likelihood(points, R*points+p);
assert(norm(T-[R p;0 0 0 1], 'fro') < 1e-10, 'Known-transform registration failed');
[data, ~] = loadDebugData(1);
assert(data.calbody.Nd > 0, 'Dataset loading failed from repository root');
rmpath(fullfile(root, 'THA3', 'HW3-PA1', 'code'));

run_asbr('tha3-registration');
assert(strcmp(pwd, root), 'Launcher failed to restore working directory');
run_asbr('tha3-unknown');
run_asbr('tha3-hand-eye');
saveCurrentFigure(assets, 'tha3-hand-eye.png');
run_asbr('tha4-tubular');
saveCurrentFigure(assets, 'tha4-tubular.png');
run_asbr('tha4-conical');
saveCurrentFigure(assets, 'tha4-conical.png');

export_kuka_preview;
run_asbr('tha2-ik');
fprintf('\nASBR MATLAB verification completed. Constrained IK requires Optimization Toolbox.\n');
end

function saveCurrentFigure(assets, name)
f = get(groot, 'CurrentFigure');
assert(~isempty(f), 'Demo produced no figure');
axesList = findall(f, 'Type', 'axes');
for k = 1:numel(axesList)
    if ~isempty(axesList(k).Toolbar), axesList(k).Toolbar.Visible = 'off'; end
end
drawnow;
exportgraphics(f, fullfile(assets,name), 'Resolution',150);
close all;
end

function restore(folder, oldPath)
cd(folder); path(oldPath);
end
