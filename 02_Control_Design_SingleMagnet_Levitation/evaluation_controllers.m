% #########################################################################
%
%  FILE      : evaluation_controllers.m
%  PROJECT   : Single Magnet Levitation System (Magnetic Levitation)
%  CONTENT   : Automated simulation & comparison of all controllers
%
%  CONTROLLERS : PID | SSC | FBL+SSC | Backstepping | SMC | MPC
%
%  PROCESS   : 1) Load common parameters (setup_sim_params.m)
%              2) Load controller parameters (.mat)
%              3) Simulate Simulink models headless (sim())
%              4) Extract signals (logsout)
%              5) Compare & plot results
%              6) Calculate performance criteria & output as table
%
%  NOTE      : Simulink models do NOT need to be open.
%              All models must be fully configured
%              (Solver, Signal logging: 'mass_pos', 'meas_air_gap').
%
% #########################################################################

clc; clear; close all;

% #########################################################################
%% === CONFIGURATION — Adjust model names & paths here ===
% #########################################################################
% --- Common path to the Controller_Params file ---
PARAMS_PATH = '.\Controller_Params\';

% --- Simulink Model Definitions ---
%  Fields:
%    folder — Folder where the .slx model is located (relative to this script)
%    model  — Filename of the Simulink model (without .slx)
%    label  — Display name in plots & tables
%    params — Name of the .mat file containing controller parameters
%
%  Project Structure:
%    01_Linear_Controller\          → PID, SSC
%    02_Nonlinear_Controller_Analytical\ → FBL+SSC, Backstepping, SMC
LIN  = '.\01_Linear_Controller\';
NL   = '.\02_Nonlinear_Controller_Analytical\';
MPC  = '.\03_Nonlinear_MPC';

models = {
    struct('folder', LIN, 'model', 'Maglev_PID',          'label', 'PID',          'params', 'LinearControllerData');
    struct('folder', LIN, 'model', 'Maglev_SSC',          'label', 'SSC',          'params', 'LinearControllerData');
    struct('folder', NL,  'model', 'Maglev_FBL_SSC',      'label', 'FBL + SSC',    'params', 'NonlinearControllerData');
    struct('folder', NL,  'model', 'Maglev_Backstepping', 'label', 'Backstepping', 'params', 'NonlinearControllerData');
    struct('folder', NL,  'model', 'Maglev_SMC_mfunc',    'label', 'SMC',          'params', 'NonlinearControllerData');
};

% --- Names of logged signals (from Simulink model) ---
SIG_AIRG  = 'meas air gap';    % Measured air gap (logged in each model)
SIG_MASSP = 'mass pos';        % Mass position

% --- Plot Colors (one per controller) ---
COLORS = [
    0.00, 0.45, 0.74;   % Blue   — PID
    0.85, 0.33, 0.10;   % Orange — SSC
    0.47, 0.67, 0.19;   % Green  — FBL+SSC
    0.49, 0.18, 0.56;   % Purple — Backstepping
    0.80, 0.07, 0.07;   % Red    — SMC
];
LINEWIDTH = 1.6;

% #########################################################################
%% === STEP 1: Load Common Simulation Parameters ===
% #########################################################################
fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('  Starting automated overall simulation\n');
fprintf('%s\n', repmat('=', 1, 70));

run('setup_sim_params.m');
n_models = numel(models);

% #########################################################################
%% === STEP 2: Execute Simulations ===
% #########################################################################
results(n_models) = struct('t', [], 'ag', [], 'label', '', 'success', false);

for k = 1:n_models
    mdl   = models{k}.model;
    label = models{k}.label;
    fprintf('\n[%d/%d] Simulating: %-20s ...', k, n_models, label);
    
    %% --- Load controller parameters ---
    param_file = fullfile(PARAMS_PATH, [models{k}.params, '.mat']);
    if exist(param_file, 'file')
        load(param_file);
    else
        fprintf(' ERROR: Parameter file not found: %s\n', param_file);
        continue
    end
    
    %% --- Simulate model (headless, no opening required) ---
    try
        model_path = fullfile(models{k}.folder, mdl);
        % Load model (without opening UI)
        load_system(model_path);
        
        % Temporarily deactivate PreLoadFcn — prevents path conflicts
        orig_preload = get_param(mdl, 'PreLoadFcn');
        set_param(mdl, 'PreLoadFcn', '');
        
        % Load parameters into Base Workspace (model accesses these directly)
        run('setup_sim_params.m');
        load(param_file);
        if exist(fullfile(PARAMS_PATH, 'EKFData.mat'), 'file')
            load(fullfile(PARAMS_PATH, 'EKFData.mat'));
        end
        
        % Run simulation — uses Base Workspace directly
        simOut = sim(mdl, ...
                    'StopTime',          num2str(T_sim), ...
                    'SaveOutput',        'on', ...
                    'SignalLogging',     'on', ...
                    'SignalLoggingName', 'logsout');
                
        % Restore PreLoadFcn & close model (without saving)
        set_param(mdl, 'PreLoadFcn', orig_preload);
        close_system(mdl, 0);
        
        %% --- Extract signals ---
        if isprop(simOut, 'logsout')
            logs = simOut.logsout;
        else
            logs = simOut.get('logsout');
        end
        
        % Measured air gap
        sig_ag          = logs.get(SIG_AIRG);
        results(k).t    = sig_ag.Values.Time(:);
        results(k).ag   = sig_ag.Values.Data(:);   % (:) → always flat column vector
        
        % Mass position (optional — for future expansion)
        sig_list = logs.getElementNames();
        if any(strcmp(sig_list, SIG_MASSP))
            sig_mp         = logs.get(SIG_MASSP);
            results(k).pos = sig_mp.Values.Data(:);
        end
        
        results(k).label   = label;
        results(k).success = true;
        fprintf(' OK\n');
        
    catch ME
        % Ensure model is closed and PreLoadFcn restored on error
        if bdIsLoaded(mdl)
            try
                set_param(mdl, 'PreLoadFcn', orig_preload);
            catch; end
            close_system(mdl, 0);
        end
        fprintf(' ERROR: %s\n', ME.message);
        results(k).label   = label;
        results(k).success = false;
    end
end

% #########################################################################
%% === STEP 3: Calculate Performance Criteria ===
% #########################################################################
%  Calculates for each successful simulation:
%    - Overshoot
%    - Rise time      (10 % → 90 % of target value)
%    - Settling time  (2 % criterion)
%    - Steady-state error (Average of last 10 % of simulation)

target   = airgap_soll;              % Target air gap [mm]
band_pct = 0.02;                     % Tolerance band for settling time [±2 %]
band     = band_pct * target;

fprintf('\n%s\n', repmat('-', 1, 70));
fprintf('  PERFORMANCE CRITERIA\n');
fprintf('%s\n', repmat('-', 1, 70));
fprintf('  %-16s %12s %12s %12s %12s\n', ...
        'Controller', 'Overshoot[%]', 'Rise[ms]', 'Settling[ms]', 'Error[µm]');
fprintf('%s\n', repmat('-', 1, 70));

for k = 1:n_models
    if ~results(k).success; continue; end
    t  = results(k).t;
    ag = results(k).ag;
    
    % --- Overshoot ---
    %  Air gap drops from airgap_init to airgap_soll (decrease).
    %  Undershoot = Minimum below the target value.
    ag_min   = min(ag);
    overshoot_pct = max(0, (target - ag_min) / (airgap_init - target) * 100);
    
    % --- Rise Time (10 % → 90 % of target value) ---
    delta      = airgap_init - target;
    lvl_10     = airgap_init - 0.10 * delta;
    lvl_90     = airgap_init - 0.90 * delta;
    idx_10     = find(ag <= lvl_10, 1, 'first');
    idx_90     = find(ag <= lvl_90, 1, 'first');
    
    if ~isempty(idx_10) && ~isempty(idx_90)
        t_rise = (t(idx_90) - t(idx_10)) * 1e3;   % [ms]
    else
        t_rise = NaN;
    end
    
    % --- Settling Time (2 % criterion) ---
    in_band    = abs(ag - target) <= band;
    last_out   = find(~in_band, 1, 'last');
    if isempty(last_out)
        t_settle = 0;
    else
        t_settle = t(last_out) * 1e3;              % [ms]
    end
    
    % --- Steady-state Error (last 10 % of simulation) ---
    % n_ss: Number of samples in steady-state window (last 10%).
    % Clamped to [1, length(ag)-1] to prevent 'Array indices must be positive integers' error.
    n_total     = length(ag);
    n_ss        = max(1, min(round(0.10 * n_total), n_total - 1));
    ss_error_um = abs(mean(ag(end-n_ss:end)) - target) * 1e3; % [µm]
    
    % Save results
    results(k).overshoot  = overshoot_pct;
    results(k).t_rise     = t_rise;
    results(k).t_settle   = t_settle;
    results(k).ss_error   = ss_error_um;
    
    fprintf('  %-16s %12.1f %12.1f %12.1f %12.2f\n', ...
            results(k).label, overshoot_pct, t_rise, t_settle, ss_error_um);
end
fprintf('%s\n', repmat('-', 1, 70));

% #########################################################################
%% === STEP 4: Comparison Plots ===
% #########################################################################

%% --- Figure 1: Air Gap Time Response (Overall View) ---
fig1 = figure('Name', 'Controller Comparison: Air Gap', ...
              'Units', 'normalized', 'Position', [0.05 0.1 0.9 0.75]);

% --- Subplot 1: Total Time Response ---
ax1 = subplot(2, 2, [1 2]);
hold on; grid on; box on;
yline(airgap_soll, 'k--', 'LineWidth', 1.2, 'Label', ...
      sprintf('Target: %.1f mm', airgap_soll), 'HandleVisibility', 'off');
for k = 1:n_models
    if ~results(k).success; continue; end
    plot(results(k).t, results(k).ag, ...
         'Color', COLORS(k,:), 'LineWidth', LINEWIDTH, ...
         'DisplayName', results(k).label);
end
xlabel('Time [s]');
ylabel('Air Gap [mm]');
title('Air Gap Time Response — All Controllers');
legend('Location', 'best', 'FontSize', 9);
ylim([max(0, airgap_soll - 0.8), airgap_init + 0.2]);
xlim([0, T_sim]);

% --- Subplot 2: Settling Process (Transient, first 0.3 s) ---
ax2 = subplot(2, 2, 3);
hold on; grid on; box on;
yline(airgap_soll, 'k--', 'LineWidth', 1.2,       'HandleVisibility', 'off');
yline(airgap_soll + band, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
yline(airgap_soll - band, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
for k = 1:n_models
    if ~results(k).success; continue; end
    plot(results(k).t, results(k).ag, ...
         'Color', COLORS(k,:), 'LineWidth', LINEWIDTH, ...
         'DisplayName', results(k).label);
end
xlabel('Time [s]');
ylabel('Air Gap [mm]');
title('Settling Process (Transients)');
legend('Location', 'best', 'FontSize', 9);
xlim([0, min(0.3, T_sim)]);
ylim([max(0, airgap_soll - 0.5), airgap_init + 0.1]);

% --- Subplot 3: Steady-state Region (last 20 % of simulation) ---
ax3 = subplot(2, 2, 4);
hold on; grid on; box on;
t_ss_start = 0.8 * T_sim;
yline(airgap_soll,         'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(airgap_soll + band,  'k:',  'LineWidth', 0.8, 'HandleVisibility', 'off');
yline(airgap_soll - band,  'k:',  'LineWidth', 0.8, 'HandleVisibility', 'off');
for k = 1:n_models
    if ~results(k).success; continue; end
    mask = results(k).t >= t_ss_start;
    plot(results(k).t(mask), results(k).ag(mask), ...
         'Color', COLORS(k,:), 'LineWidth', LINEWIDTH, ...
         'DisplayName', results(k).label);
end
xlabel('Time [s]');
ylabel('Air Gap [mm]');
title(sprintf('Steady-state Region (t > %.1f s)', t_ss_start));
legend('Location', 'best', 'FontSize', 9);
xlim([t_ss_start, T_sim]);

%% --- Figure 2: Performance Criteria Bar Chart ---
fig2 = figure('Name', 'Performance Criteria Comparison', ...
              'Units', 'normalized', 'Position', [0.05 0.1 0.9 0.5]);

% Only successful simulations
valid   = find([results.success]);
labels  = {results(valid).label};
colors_v = COLORS(valid, :);
metrics = [
    [results(valid).overshoot];
    [results(valid).t_rise];
    [results(valid).t_settle];
    [results(valid).ss_error];
];
metric_names  = {'Overshoot [%]', 'Rise Time [ms]', ...
                 'Settling Time [ms]',  'Steady-State Error [µm]'};

for m = 1:4
    ax = subplot(1, 4, m);
    b  = bar(metrics(m, :), 'FaceColor', 'flat');
    for i = 1:numel(valid)
        b.CData(i, :) = colors_v(i, :);
    end
    set(ax, 'XTickLabel', labels, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel(metric_names{m});
    title(metric_names{m}, 'FontSize', 9);
    grid on; box on;
end
sgtitle('Comparison of Performance Criteria', 'FontWeight', 'bold');

% #########################################################################
%% === STEP 5: Save Plots ===
% #########################################################################
%  Figures are saved in the 'Results' folder.
%  Filenames include noise status and timestamp to prevent overwriting.

% --- Determine & create target folder ---
RESULTS_PATH = fullfile(pwd, 'Results');
if ~exist(RESULTS_PATH, 'dir')
    mkdir(RESULTS_PATH);
    fprintf('  Folder created: %s\n', RESULTS_PATH);
end

% --- Filename suffix based on noise status ---
if isNoiseActive
    noise_tag = 'Noise_active';
else
    noise_tag = 'Noise_disabled';
end

% --- Timestamp (for unique filenames during multiple runs) ---
timestamp = datetime('now', 'Format', 'yyyyMMdd_HHmm');

% --- Figure 1: Time Response ---
fname1 = sprintf('TimeResponse_%s', noise_tag);
exportgraphics(fig1, fullfile(RESULTS_PATH, [fname1 '.png']), ...
    'Resolution', 300);
exportgraphics(fig1, fullfile(RESULTS_PATH, [fname1 '.pdf']), ...
    'ContentType', 'vector');

% --- Figure 2: Performance Criteria ---
fname2 = sprintf('PerformanceCriteria_%s', noise_tag);
exportgraphics(fig2, fullfile(RESULTS_PATH, [fname2 '.png']), ...
    'Resolution', 300);
exportgraphics(fig2, fullfile(RESULTS_PATH, [fname2 '.pdf']), ...
    'ContentType', 'vector');

fprintf('  Plots saved in: %s\n', RESULTS_PATH);
fprintf('    %s.png/.pdf\n', fname1);
fprintf('    %s.png/.pdf\n', fname2);

%% --- Completion Message ---
fprintf('\n%s\n', repmat('=', 1, 70));
n_ok = sum([results.success]);
fprintf('  Simulation complete: %d/%d models successful.\n', n_ok, n_models);
fprintf('%s\n\n', repmat('=', 1, 70));

%% --- Cleanup: Delete Simulink Cache Files ---
folders = [{pwd}, {LIN}, {NL}];
for f = 1:numel(folders)
    if exist("slprj",'dir')
        rmdir('slprj', 's')
    end
    slxc = dir(fullfile(folders{f}, '*.slxc'));
    for i = 1:numel(slxc)
        delete(fullfile(slxc(i).folder, slxc(i).name));
    end
end
fprintf('  Cache files cleared.\n');