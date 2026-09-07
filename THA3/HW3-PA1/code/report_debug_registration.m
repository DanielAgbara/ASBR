%% report_debug_registration.m
% Full debug evaluation script
%
% Runs:
%   - Part 3 registration
%   - Part 4 EM pivot calibration
%   - Part 5 optical pivot calibration
%
% For selected datasets, computes debug errors when ground truth is available,
% generates visualizations, and saves summary artifacts.

clc;
clear;
close all;

%% ============================================================
% Configuration
% ============================================================

cfg = struct();

% Dataset selection
cfg.mode = 'debug';              % 'debug' or 'unknown'
cfg.test_ids = 1:7;              % debug a-g

% Registration settings
cfg.max_iter = 100;
cfg.tol = 1e-3;

% Plotting / export
cfg.make_registration_plots = true;
cfg.make_em_pivot_plots = true;
cfg.make_opt_pivot_plots = true;

cfg.save_figures = true;
cfg.close_figures_after_save = true;
cfg.fig_dir = 'debug_results_figures';

% Summary export
cfg.save_summary_csv = true;
cfg.summary_csv = 'debug_full_summary.csv';

% MAT export
cfg.save_mat = true;
cfg.mat_name = 'debug_full_results.mat';

% Console behavior
cfg.verbose = true;

%% ============================================================
% Validate configuration
% ============================================================

validate_cfg(cfg);

if cfg.save_figures && ~exist(cfg.fig_dir, 'dir')
    mkdir(cfg.fig_dir);
end

if cfg.verbose
    fprintf('\n====================================================\n');
    fprintf('Running full debug evaluation\n');
    fprintf('Mode                   : %s\n', cfg.mode);
    fprintf('Datasets               : %s\n', mat2str(cfg.test_ids));
    fprintf('Max iterations         : %d\n', cfg.max_iter);
    fprintf('Tolerance              : %.3e\n', cfg.tol);
    fprintf('Registration plots     : %d\n', cfg.make_registration_plots);
    fprintf('EM pivot plots         : %d\n', cfg.make_em_pivot_plots);
    fprintf('Optical pivot plots    : %d\n', cfg.make_opt_pivot_plots);
    fprintf('Save figures           : %d\n', cfg.save_figures);
    fprintf('Save CSV               : %d\n', cfg.save_summary_csv);
    fprintf('Save MAT               : %d\n', cfg.save_mat);
    fprintf('====================================================\n');
end

%% ============================================================
% Run report
% ============================================================

tStart = tic;
results = run_full_debug_report(cfg);
elapsed_sec = toc(tStart);

%% ============================================================
% Save outputs
% ============================================================

if cfg.save_summary_csv
    writetable(results.summary_table, cfg.summary_csv);
    if cfg.verbose
        fprintf('Saved summary CSV: %s\n', cfg.summary_csv);
    end
end

if cfg.save_mat
    save(cfg.mat_name, 'results', 'cfg');
    if cfg.verbose
        fprintf('Saved MAT file   : %s\n', cfg.mat_name);
    end
end

%% ============================================================
% Final message
% ============================================================

if cfg.verbose
    fprintf('Finished in %.2f seconds.\n', elapsed_sec);
end
disp('Done.');

%% ============================================================
% Local validation helper
% ============================================================

function validate_cfg(cfg)

    if ~isfield(cfg, 'mode') || ~ismember(lower(cfg.mode), {'debug', 'unknown'})
        error('cfg.mode must be ''debug'' or ''unknown''.');
    end

    if ~isfield(cfg, 'test_ids') || isempty(cfg.test_ids)
        error('cfg.test_ids must be a non-empty vector.');
    end

    if any(cfg.test_ids < 1) || any(cfg.test_ids > 11) || any(mod(cfg.test_ids,1) ~= 0)
        error('cfg.test_ids must contain integer dataset IDs between 1 and 11.');
    end

    if ~isfield(cfg, 'max_iter') || cfg.max_iter <= 0
        error('cfg.max_iter must be positive.');
    end

    if ~isfield(cfg, 'tol') || cfg.tol <= 0
        error('cfg.tol must be positive.');
    end

    required_flags = { ...
        'make_registration_plots', ...
        'make_em_pivot_plots', ...
        'make_opt_pivot_plots', ...
        'save_figures', ...
        'close_figures_after_save', ...
        'save_summary_csv', ...
        'save_mat', ...
        'verbose'};

    for i = 1:numel(required_flags)
        f = required_flags{i};
        if ~isfield(cfg, f)
            error('Missing cfg.%s', f);
        end
    end

    if cfg.save_figures
        if ~isfield(cfg, 'fig_dir') || isempty(cfg.fig_dir)
            error('cfg.fig_dir must be provided when cfg.save_figures is true.');
        end
    end

    if cfg.save_summary_csv
        if ~isfield(cfg, 'summary_csv') || isempty(cfg.summary_csv)
            error('cfg.summary_csv must be provided when cfg.save_summary_csv is true.');
        end
    end

    if cfg.save_mat
        if ~isfield(cfg, 'mat_name') || isempty(cfg.mat_name)
            error('cfg.mat_name must be provided when cfg.save_mat is true.');
        end
    end
end