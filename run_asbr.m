function run_asbr(demo)
%RUN_ASBR Run one course demo with its own MATLAB path and output directory.
%   run_asbr('tha2-ik')
%   run_asbr('tha3-registration')
%   run_asbr('tha3-hand-eye')
%   run_asbr('tha4-tubular')
%   Call without arguments to list all available demos.
root = fileparts(mfilename('fullpath'));
catalog = {
    'tha2-ik', 'THA2/matlab_converted', 'demo_pose_ik.m';
    'tha2-transpose', 'THA2/matlab_converted', 'demo_jacobian_transpose.m';
    'tha2-redundancy', 'THA2/matlab_converted', 'demo_redundancy_resolution.m';
    'tha3-registration', 'THA3/HW3-PA1/code', 'demo_registration_debug.m';
    'tha3-unknown', 'THA3/HW3-PA1/code', 'demo_registration_unknown.m';
    'tha3-hand-eye', 'THA3/HW3-PA2', 'demo_hand_eye.m';
    'tha4-tubular', 'THA4/hw', 'demo_tubular_impedance.m';
    'tha4-conical', 'THA4/hw', 'demo_conical_impedance.m';
    'tha4-position', 'THA4/pa/codes', 'demo_position_ik.m';
    'tha4-pose', 'THA4/pa/codes', 'demo_pose_ik.m';
    'tha4-position-sweep', 'THA4/pa/codes', 'demo_position_sweep.m';
    'tha4-pose-sweep', 'THA4/pa/codes', 'demo_pose_sweep.m'
};
if nargin == 0
    fprintf('Available demos:\n');
    fprintf('  %s\n', catalog{:,1});
    return;
end
index = find(strcmp(demo, catalog(:,1)), 1);
assert(~isempty(index), 'ASBR:UnknownDemo', 'Unknown demo. Call run_asbr to list options.');
if startsWith(demo, 'tha2') || startsWith(demo, 'tha4-p')
    assert(exist('importrobot', 'file') ~= 0, 'ASBR:MissingRobotics', ...
        'This demo requires Robotics System Toolbox.');
end
if startsWith(demo, 'tha4-p')
    assert(exist('lsqlin', 'file') ~= 0, 'ASBR:MissingOptimization', ...
        'Constrained IK requires Optimization Toolbox (lsqlin).');
end
oldPath = path;
oldDirectory = pwd;
cleanup = onCleanup(@() restoreEnvironment(oldPath, oldDirectory)); %#ok<NASGU>
codeDirectory = fullfile(root, catalog{index,2});
addpath(codeDirectory, '-begin');
outputDirectory = fullfile(root, 'outputs', demo);
if ~isfolder(outputDirectory), mkdir(outputDirectory); end
cd(outputDirectory);
fprintf('Running %s. Outputs: %s\n', demo, outputDirectory);
executeDemo(fullfile(codeDirectory, catalog{index,3}));
end

function executeDemo(scriptPath)
% A separate workspace lets original scripts use clear without destroying
% the caller's cleanup object. eval preserves the outputs working directory.
eval(fileread(scriptPath));
end

function restoreEnvironment(oldPath, oldDirectory)
path(oldPath);
cd(oldDirectory);
end
