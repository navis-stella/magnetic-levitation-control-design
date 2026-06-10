% #########################################################################
%
%  FILE      : evaluation_controllers.m
%  PROJECT   : Single-Magnet Levitation System (Magnetic Levitation)
%  CONTENT   : Automated simulation & comparison of all controllers
%
%  CONTROLLERS: PID | SSC | Backstepping | FeedbackLin | SMC | NMPC
%
%  WORKFLOW  : 1) Load common parameters (setup_sim_params.m)
%              2) Load controller parameters (.mat)
%              3) Run Simulink models headless (sim())
%              4) Extract signals (logsout)
%              5) Compute performance criteria (transient + disturbance)
%              6) Compute current quality (peak, RMS, TV, slew rate)
%              7) Compare & plot results (5 figures)
%              8) Save plots & results
%
%  SIGNALS   : 'meas_airgap'    — Measured air gap
%              'mass_pos'       — Mass position
%              'cmd_current'    — Command current (controller output)
%
%  NOTE      : Simulink models do NOT need to be open.
%              All signals must be logged in logsout.
%
% #########################################################################

clc; clear; close all;

% Suppress exportgraphics warning for vector graphics with many data points
% — affects performance only, not correctness
warning('off', 'MATLAB:exportgraphics:vectorContentMightBeUnexpected');


% #########################################################################
%% === CONFIGURATION ===
% #########################################################################

% --- Paths ---
PARAMS_PATH = '.\Controller_Params\';
LIN         = '.\01_Linear_Controller\';
NLC         = '.\02_Nonlinear_Controller_Analytical\';
MPC         = '.\03_Nonlinear_MPC\';

% --- Model Definitions ---
models = {
    struct('folder', LIN, 'model', 'Maglev_PID',             'label', 'PID',          'params', 'LinearControllerData');
    struct('folder', LIN, 'model', 'Maglev_SSC',             'label', 'SSC',          'params', 'LinearControllerData');
    struct('folder', NLC, 'model', 'Maglev_Backstepping',    'label', 'Backstepping', 'params', 'NonlinearControllerData');
    struct('folder', NLC, 'model', 'Maglev_FeedbackLin',     'label', 'FeedbackLin',  'params', 'NonlinearControllerData');
    struct('folder', NLC, 'model', 'Maglev_ISMC',            'label', 'SMC',          'params', 'NonlinearControllerData');
    struct('folder', MPC, 'model', 'Maglev_NMPC_OffsetFree', 'label', 'NMPC',         'params', 'OffsetFreeNMPCData');
};
% --- Signal Names ---
SIG_AIRG  = 'meas_airgap';
SIG_MASSP = 'mass_pos';
SIG_ICMD  = 'cmd_current';

% --- Colors (one per controller) ---
COLORS = [
    0.56, 0.00, 1.00;   % Violet     
    0.85, 0.33, 0.10;   % Orange     
    0.00, 0.47, 0.44;   % Pine green 
    0.82, 0.71, 0.55;   % Tan        
    0.00, 0.45, 0.74;   % Blue       
    0.44, 0.26, 0.08;   % Sepia      
];

LINEWIDTH = 1.6;


% #########################################################################
%% === STEP 1: Start ===
% #########################################################################

fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('    Starting automated full simulation\n');
fprintf('%s\n', repmat('=', 1, 70));

n_models = numel(models);


% #########################################################################
%% === STEP 2: Run Simulations & Extract Signals ===
% #########################################################################

results(n_models) = struct( ...
    't',    [], 'ag',    [], ...
    't_i',  [], 'i_cmd', [], ...
    'pos',  [], 'label', '', 'success', false);

for k = 1:n_models

    mdl   = models{k}.model;
    label = models{k}.label;

    fprintf('\n[%d/%d] Simulating: %-20s ...', k, n_models, label);

    % --- Load parameter file ---
    param_file = fullfile(PARAMS_PATH, [models{k}.params, '.mat']);
    if ~exist(param_file, 'file')
        fprintf(' ERROR: Parameter file not found: %s\n', param_file);
        results(k).label = label;
        continue
    end

    try
        % Load model (without opening it)
        model_path   = fullfile(models{k}.folder, mdl);
        load_system(model_path);

        % Temporarily disable PreLoadFcn — prevents path conflicts
        orig_preload = get_param(mdl, 'PreLoadFcn');
        set_param(mdl, 'PreLoadFcn', '');

        % Load parameters into base workspace
        % run('setup_sim_params.m');
        load(param_file);
        if exist(fullfile(PARAMS_PATH, 'EKFData.mat'), 'file')
            load(fullfile(PARAMS_PATH, 'EKFData.mat'));
        end

        % Run simulation
        simOut = sim(mdl, ...
                    'StopTime',          num2str(T_sim), ...
                    'SaveOutput',        'on', ...
                    'SignalLogging',     'on', ...
                    'SignalLoggingName', 'logsout');

        % Restore PreLoadFcn & close model
        set_param(mdl, 'PreLoadFcn', orig_preload);
        close_system(mdl, 0);

        % --- Extract signals ---
        if isprop(simOut, 'logsout')
            logs = simOut.logsout;
        else
            logs = simOut.get('logsout');
        end
        sig_list = logs.getElementNames();

        % ── Air gap ───────────────────────────────────────────────────────
        sig_ag        = logs.get(SIG_AIRG);
        results(k).t  = sig_ag.Values.Time(:);
        results(k).ag = sig_ag.Values.Data(:);

        % ── Mass position (optional) ───────────────────────────────────────
        if any(strcmp(sig_list, SIG_MASSP))
            sig_mp         = logs.get(SIG_MASSP);
            results(k).pos = sig_mp.Values.Data(:);
        end

        % ── Command current (controller output) ───────────────────────────
        if any(strcmp(sig_list, SIG_ICMD))
            sig_ic           = logs.get(SIG_ICMD);
            results(k).i_cmd = sig_ic.Values.Data(:);
            results(k).t_i   = sig_ic.Values.Time(:);
        end

        results(k).label   = label;
        results(k).success = true;
        fprintf(' OK\n');

    catch ME
        if bdIsLoaded(mdl)
            try set_param(mdl, 'PreLoadFcn', orig_preload); catch; end
            close_system(mdl, 0);
        end
        fprintf(' ERROR: %s\n', ME.message);
        results(k).label   = label;
        results(k).success = false;
    end
end


% #########################################################################
%% === STEP 3a: Compute Performance Criteria ===
% #########################################################################
%  Time windows:
%    Transient              : 0                              <= t < T_stp     -> settling
%    SS before disturbance  : 0.70*T_stp                    <= t < T_stp     -> unforced SS
%    Disturbance window     : T_stp                          <= t <= T_sim    -> disturbance metrics
%    SS after disturbance   : T_stp + 0.90*(T_sim - T_stp)  <= t <= T_sim    -> final SS

target           = airgap_soll;
band_pct         = 0.02;              % +-2%
band             = band_pct * target;
t_dist_start     = T_stp;
t_pre_ss_start   = 0.70 * t_dist_start;                           % last 30% before disturbance
t_final_ss_start = t_dist_start + 0.90 * (T_sim - t_dist_start);  % last 10% of disturbance window

fprintf('\n%s\n', repmat('-', 1, 86));
fprintf('  PERFORMANCE CRITERIA — INITIAL TRANSIENT  (0 <= t < %.2f s)\n', t_dist_start);
fprintf('  Unforced SS window: %.2f s <= t < %.2f s\n', t_pre_ss_start, t_dist_start);
fprintf('%s\n', repmat('-', 1, 86));
fprintf('  %-16s | %12s %12s %14s %14s\n', ...
        'Controller', 'Overshoot[%]', 'Rise[ms]', 'Settling[ms]', 'SS-Error[um]');
fprintf('%s\n', repmat('-', 1, 86));

for k = 1:n_models
    if ~results(k).success; continue; end

    t  = results(k).t;
    ag = results(k).ag;

    % Time ranges
    idx_pre  = t <  t_dist_start;
    idx_dist = t >= t_dist_start;
    t_pre    = t(idx_pre);   ag_pre  = ag(idx_pre);
    t_dwin   = t(idx_dist);  ag_dwin = ag(idx_dist);

    % -- 1. Overshoot ---------------------------------------------------------
    ag_min        = min(ag_pre);
    overshoot_pct = max(0, (target - ag_min) / (airgap_init - target) * 100);

    % -- 2. Rise time 10% -> 90% ----------------------------------------------
    delta  = airgap_init - target;
    lvl_10 = airgap_init - 0.10 * delta;
    lvl_90 = airgap_init - 0.90 * delta;
    idx_10 = find(ag_pre <= lvl_10, 1, 'first');
    idx_90 = find(ag_pre <= lvl_90, 1, 'first');
    if ~isempty(idx_10) && ~isempty(idx_90)
        t_rise = (t_pre(idx_90) - t_pre(idx_10)) * 1e3;
    else
        t_rise = NaN;
    end

    % -- 3. Settling time +-2% (initial transient only) -----------------------
    in_band_pre = abs(ag_pre - target) <= band;
    last_out    = find(~in_band_pre, 1, 'last');
    if isempty(last_out)
        t_settle = 0;
    else
        t_settle = t_pre(last_out) * 1e3;
    end

    % -- 4. Steady-state error WITHOUT disturbance (NEW) ----------------------
    %  Time-window based, NOT sample-count -> robust against variable step sizes.
    mask_pre_ss = (t >= t_pre_ss_start) & (t < t_dist_start);
    if any(mask_pre_ss)
        ss_err_unforced = abs(mean(ag(mask_pre_ss)) - target) * 1e3;
    else
        ss_err_unforced = NaN;
    end

    results(k).overshoot       = overshoot_pct;
    results(k).t_rise          = t_rise;
    results(k).t_settle        = t_settle;
    results(k).ss_err_unforced = ss_err_unforced;

    fprintf('  %-16s | %12.1f %12.1f %14.1f %14.3f\n', ...
            results(k).label, overshoot_pct, t_rise, t_settle, ss_err_unforced);
end
fprintf('%s\n', repmat('-', 1, 86));


% --- Second table: Disturbance rejection ---------------------------------
fprintf('\n%s\n', repmat('-', 1, 104));
fprintf('  PERFORMANCE CRITERIA — DISTURBANCE REJECTION  (t >= %.2f s, Step = %.1f N)\n', ...
        t_dist_start, Ext_Fina_Val);
fprintf('  Final SS window: %.2f s <= t <= %.2f s\n', t_final_ss_start, T_sim);
fprintf('%s\n', repmat('-', 1, 104));
fprintf('  %-16s | %12s %12s %12s %14s %14s\n', ...
        'Controller', 'PeakDev.[um]', 'Recovery[ms]', 'IAE[um*s]', 'ITAE[um*s2]', 'finalSS[um]');
fprintf('%s\n', repmat('-', 1, 104));

for k = 1:n_models
    if ~results(k).success; continue; end

    t  = results(k).t;
    ag = results(k).ag;

    idx_dist = t >= t_dist_start;
    t_dwin   = t(idx_dist);
    ag_dwin  = ag(idx_dist);

    % -- 5. Peak deviation in disturbance window ------------------------------
    if ~isempty(ag_dwin)
        dist_peak_um = max(abs(ag_dwin - target)) * 1e3;
    else
        dist_peak_um = NaN;
    end

    % -- 6. Recovery time -----------------------------------------------------
    t_recovery = NaN;
    if ~isempty(ag_dwin)
        in_band_d  = abs(ag_dwin - target) <= band;
        last_out_d = find(~in_band_d, 1, 'last');
        if isempty(last_out_d)
            t_recovery = 0;
        else
            t_recovery = (t_dwin(last_out_d) - t_dist_start) * 1e3;
        end
    end

    % -- 7. IAE in disturbance window -----------------------------------------
    IAE_dist = NaN;
    if length(t_dwin) > 1
        err_um   = abs(ag_dwin - target) * 1e3;  % um
        IAE_dist = trapz(t_dwin, err_um);
    end

    % -- 8. ITAE in disturbance window (NEW) ----------------------------------
    %  Time measured from disturbance onset -> penalizes slow recovery more heavily.
    %  Unit: [um*s^2]
    ITAE_dist = NaN;
    if length(t_dwin) > 1
        err_um    = abs(ag_dwin - target) * 1e3;
        ITAE_dist = trapz(t_dwin, (t_dwin - t_dist_start) .* err_um);
    end

    % -- 9. Steady-state error AFTER sustained disturbance (RENAMED) ----------
    mask_final_ss = t >= t_final_ss_start;
    if any(mask_final_ss)
        ss_err_final = abs(mean(ag(mask_final_ss)) - target) * 1e3;
    else
        ss_err_final = NaN;
    end

    results(k).dist_peak       = dist_peak_um;
    results(k).t_recovery      = t_recovery;
    results(k).IAE_dist        = IAE_dist;
    results(k).ITAE_dist       = ITAE_dist;
    results(k).ss_err_final    = ss_err_final;

    fprintf('  %-16s | %12.1f %12.1f %12.4f %14.4f %14.3f\n', ...
            results(k).label, dist_peak_um, t_recovery, IAE_dist, ITAE_dist, ss_err_final);
end
fprintf('%s\n', repmat('-', 1, 104));


% #########################################################################
%% === STEP 3b: Compute Current Quality ===
% #########################################################################
%  Peak / RMS : entire simulation           (actuator sizing & thermal)
%  TV         : disturbance window (200 ms) (controller activity under excitation)
%  Energy     : entire simulation           (int i^2 dt ~ heat dissipation)

t_tv_end = min(t_dist_start + 0.20, T_sim);  % 200 ms from disturbance onset
has_current = false;

fprintf('\n%s\n', repmat('-', 1, 90));
fprintf('  CURRENT QUALITY\n');
fprintf('  TV window: %.2f s <= t <= %.2f s   |   Energy: entire simulation\n', ...
        t_dist_start, t_tv_end);
fprintf('%s\n', repmat('-', 1, 90));
fprintf('  %-16s | %10s %10s %16s | %14s %14s\n', ...
        'Controller', 'Peak [A]', 'RMS [A]', 'TV-Dist [A]', ...
        'W_T [A^2*s]', 'W_D [A^2*s]');
fprintf('%s\n', repmat('-', 1, 90));

for k = 1:n_models
    if ~results(k).success; continue; end
    if isempty(results(k).i_cmd)
        fprintf('  %-16s  (no current signal logged)\n', results(k).label);
        continue
    end
    has_current = true;

    t_i   = results(k).t_i;
    i_cmd = results(k).i_cmd;

    % -- 1. Peak — entire simulation ------------------------------------------
    i_peak = max(abs(i_cmd));

    % -- 2. RMS — entire simulation -------------------------------------------
    %  Note: time-weighted is more accurate than mean(i^2) for variable step sizes.
    if length(t_i) > 1
        i_rms = sqrt(trapz(t_i, i_cmd .^ 2) / (t_i(end) - t_i(1)));
    else
        i_rms = abs(i_cmd);
    end

    % -- 3. Total Variation in disturbance window (CHANGED) -------------------
    %  Captures true controller activity under excitation — not pure sensor noise.
    idx_tv = (t_i >= t_dist_start) & (t_i <= t_tv_end);
    i_tv_win = i_cmd(idx_tv);
    if length(i_tv_win) > 1
        i_tv = sum(abs(diff(i_tv_win)));
    else
        i_tv = NaN;
    end

   % -- 4. Excess control energy SPLIT ---------------------------------------
    %  Instead of a single total energy, two separate contributions:
    %    W_excess_T : startup cost      (0 <= t < T_stp)
    %    W_excess_D : disturbance cost  (T_stp <= t <= T_sim)
    %  Both relative to the controller equilibrium current i_eq.

    mask_eq = (t_i >= t_pre_ss_start) & (t_i < t_dist_start);
    if any(mask_eq)
        i_eq_k = mean(i_cmd(mask_eq)); % Estimated from pre-disturbance plateau
    else
        i_eq_k = mean(i_cmd);
    end

    mask_T = t_i <  t_dist_start;
    mask_D = t_i >= t_dist_start;

    if sum(mask_T) > 1
        W_excess_T = trapz(t_i(mask_T), (i_cmd(mask_T) - i_eq_k).^2);
    else
        W_excess_T = NaN;
    end

    if sum(mask_D) > 1
        W_excess_D = trapz(t_i(mask_D), (i_cmd(mask_D) - i_eq_k).^2);
    else
        W_excess_D = NaN;
    end

    results(k).i_eq_used   = i_eq_k;
    results(k).W_excess_T  = W_excess_T;
    results(k).W_excess_D  = W_excess_D;

    results(k).i_peak  = i_peak;
    results(k).i_rms   = i_rms;
    results(k).i_tv    = i_tv;
    % results(k).W_ctrl  = W_ctrl;

    fprintf('  %-16s | %10.3f %10.3f %16.2f | %14.4f %14.4f\n', ...
            results(k).label, i_peak, i_rms, i_tv, W_excess_T, W_excess_D);

end
fprintf('%s\n', repmat('-', 1, 90));


for k = 1:n_models
    fprintf('%-16s : i_eq_used = %.4f A\n', results(k).label, results(k).i_eq_used);
end

% #########################################################################
%% === STEP 4: Comparison Plots ===
% #########################################################################

valid    = find([results.success]);
labels_v = {results(valid).label};
colors_v = COLORS(valid, :);

% Controllers with logged current
valid_i  = valid(arrayfun(@(k) ~isempty(results(k).i_cmd), valid));
labels_i = {results(valid_i).label};
colors_i = COLORS(valid_i, :);


% =========================================================================
%% --- Figure 1: Air Gap Time Response — 4 Views ---
% =========================================================================
fig1 = figure('Name', 'Controller Comparison: Air Gap', ...
              'Units', 'normalized', 'Position', [0.02 0.08 0.96 0.82]);

% -- Subplot 1: Full time response --------------------------------------------
subplot(2, 3, [1 2 3]);
hold on; grid on; box on;
xline(t_dist_start, 'b--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(airgap_soll, 'k--', 'LineWidth', 1.2, ...
      'Label', sprintf('Target: %.1f mm', airgap_soll), ...
      'HandleVisibility', 'off');
for k = valid
    plot(results(k).t, results(k).ag, ...
         'Color', COLORS(k,:), 'LineWidth', LINEWIDTH, ...
         'DisplayName', results(k).label);
end
text(t_dist_start + 0.01*T_sim, airgap_init + 0.05, ...
     sprintf('Disturbance\nt=%.2fs / %.1fN', t_dist_start, Ext_Fina_Val), ...
     'FontSize', 8, 'Color', [0 0 0.8]);
xlabel('Time [s]'); ylabel('Air Gap [mm]');
title('Air Gap Time Response — All Controllers');
legend('Location', 'best', 'FontSize', 9);
ylim([max(0, airgap_soll - 0.8), airgap_init + 0.2]);
xlim([0, T_sim]);

% -- Subplot 2: Initial transient ---------------------------------------------
subplot(2, 3, 4);
hold on; grid on; box on;
yline(airgap_soll,        'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(airgap_soll + band, 'k:',  'LineWidth', 0.8, 'HandleVisibility', 'off');
yline(airgap_soll - band, 'k:',  'LineWidth', 0.8, 'HandleVisibility', 'off');
for k = valid
    plot(results(k).t, results(k).ag, ...
         'Color', COLORS(k,:), 'LineWidth', LINEWIDTH, ...
         'DisplayName', results(k).label);
end
xlabel('Time [s]'); ylabel('Air Gap [mm]');
title('Settling Process (Transients)');
legend('Location', 'best', 'FontSize', 8);
xlim([0, min(0.30, t_dist_start)]);
ylim([max(0, airgap_soll - 0.5), airgap_init + 0.1]);

% -- Subplot 3: Disturbance window (zoom) -------------------------------------
subplot(2, 3, 5);
hold on; grid on; box on;
xline(t_dist_start, 'b--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(airgap_soll,        'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
yline(airgap_soll + band, 'k:',  'LineWidth', 0.8, 'HandleVisibility', 'off');
yline(airgap_soll - band, 'k:',  'LineWidth', 0.8, 'HandleVisibility', 'off');
for k = valid
    plot(results(k).t, results(k).ag, ...
         'Color', COLORS(k,:), 'LineWidth', LINEWIDTH, ...
         'DisplayName', results(k).label);
end
xlabel('Time [s]'); ylabel('Air Gap [mm]');
title(sprintf('Disturbance Response (t \\geq %.2f s)', t_dist_start));
legend('Location', 'best', 'FontSize', 8);
xlim([t_dist_start - 0.02, min(t_dist_start + 0.30, T_sim)]);
peak_all = max([results(valid).dist_peak]) / 1e3;
y_margin = max(0.03, 1.3 * peak_all);
ylim([airgap_soll - y_margin, airgap_soll + y_margin]);

% -- Subplot 4: Steady-state region -------------------------------------------
subplot(2, 3, 6);
hold on; grid on; box on;
yline(airgap_soll,        'k--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(airgap_soll + band, 'k:',  'LineWidth', 0.8, 'HandleVisibility', 'off');
yline(airgap_soll - band, 'k:',  'LineWidth', 0.8, 'HandleVisibility', 'off');
for k = valid
    mask = results(k).t >= t_final_ss_start;
    plot(results(k).t(mask), results(k).ag(mask), ...
         'Color', COLORS(k,:), 'LineWidth', LINEWIDTH, ...
         'DisplayName', results(k).label);
end
xlabel('Time [s]'); ylabel('Air Gap [mm]');
title(sprintf('Steady-State Region (t > %.2f s)', t_final_ss_start));
legend('Location', 'best', 'FontSize', 8);
xlim([t_final_ss_start, T_sim]);


% =========================================================================
%% --- Figure 2: Performance Criteria — Initial Transient ---
% =========================================================================
fig2 = figure('Name', 'Performance Criteria — Initial Transient', ...
              'Units', 'normalized', 'Position', [0.02 0.55 0.96 0.38]);

transient_data  = [[results(valid).overshoot];
                   [results(valid).t_rise];
                   [results(valid).t_settle];
                   [results(valid).ss_err_unforced];
                   [results(valid).W_excess_T]];
transient_names = {
    'Overshoot [%]', ...
    'Rise Time [ms]', ...
    'Settling Time [ms]', ...
    'Unforced SS-Error [µm]', ...
    sprintf('Transient Energy [A²·s]\n(\\int(i - i_{eq})^2 dt, t < %.2f s)', t_dist_start)
};

for m = 1:5
    ax = subplot(1, 5, m);
    b  = bar(transient_data(m,:), 'FaceColor', 'flat');
    for i = 1:numel(valid)
        b.CData(i,:) = colors_v(i,:);
    end
    set(ax, 'XTickLabel', labels_v, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel(transient_names{m});
    title(transient_names{m}, 'FontSize', 9);
    grid on; box on;
end
sgtitle('Comparison of Performance Criteria — Initial Transient', ...
        'FontWeight', 'bold');


% =========================================================================
%% --- Figure 3: Performance Criteria — Disturbance Rejection ---
% =========================================================================
fig3 = figure('Name', 'Performance Criteria — Disturbance Rejection', ...
              'Units', 'normalized', 'Position', [0.05 0.10 0.85 0.78]);

dist_data  = [[results(valid).dist_peak];
              [results(valid).t_recovery];
              [results(valid).ss_err_final];
              [results(valid).IAE_dist];
              [results(valid).ITAE_dist];
              [results(valid).W_excess_D]];
dist_names = {
    sprintf('Peak Deviation [µm]\n(t \\geq %.2f s, step = %.1f N)', ...
            t_dist_start, Ext_Fina_Val), ...
    sprintf('Recovery Time [ms]\n(step onset → last ±2%% exit)'), ...
    sprintf('Final SS-Error [µm]\n(last 10%% of disturbance window)'), ...
    sprintf('IAE [µm·s]\n(\\int|e|dt, disturbance window)'), ...
    sprintf('ITAE [µm·s²]\n(\\int t|e|dt, disturbance window)'), ...
    sprintf('Disturbance Energy [A²·s]\n(\\int(i - i_{eq})^2 dt, t \\geq %.2f s)', t_dist_start)
};

for m = 1:6
    ax = subplot(2, 3, m);
    b  = bar(dist_data(m,:), 'FaceColor', 'flat');
    for i = 1:numel(valid)
        b.CData(i,:) = colors_v(i,:);
    end
    set(ax, 'XTickLabel', labels_v, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel(dist_names{m});
    title(dist_names{m}, 'FontSize', 9);
    grid on; box on;

    % "Never left band" annotation only on Recovery Time (m == 2)
    if m == 2
        hold on;
        for ii = 1:numel(valid)
            if results(valid(ii)).t_recovery == 0
                text(ii, max(dist_data(m,:)) * 0.25, 'Never left band', ...
                     'HorizontalAlignment', 'center', 'FontSize', 7, ...
                     'Color', colors_v(ii,:), 'FontWeight', 'bold', ...
                     'Rotation', 90);
            end
        end
    end
end
sgtitle(sprintf('Disturbance Rejection — Step Force %.1f N (gravity direction)', ...
        abs(Ext_Fina_Val)), 'FontWeight', 'bold');

% =========================================================================
%% --- Figures 4-5: Current Analysis ---
% =========================================================================

if has_current

%% --- Figure 4: Current Response — All Controllers ---
fig4 = figure('Name', 'Current Response — All Controllers', ...
              'Units', 'normalized', 'Position', [0.02 0.08 0.96 0.82]);

% -- Subplot 1: Full current response -----------------------------------------
subplot(2, 3, [1 2 3]);
hold on; grid on; box on;
xline(t_dist_start, 'b--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
for k = valid_i
    plot(results(k).t_i, results(k).i_cmd, ...
         'Color', COLORS(k,:), 'LineWidth', LINEWIDTH, ...
         'DisplayName', results(k).label);
end
text(t_dist_start + 0.01*T_sim, max([results(valid_i).i_peak]) * 0.95, ...
     sprintf('Disturbance\nt=%.2fs / %.1fN', t_dist_start, Ext_Fina_Val), ...
     'FontSize', 8, 'Color', [0 0 0.8]);
xlabel('Time [s]'); ylabel('Current [A]');
title('Command Current — All Controllers');
legend('Location', 'best', 'FontSize', 9);
xlim([0, T_sim]);

% -- Subplot 2: Initial transient ---------------------------------------------
subplot(2, 3, 4);
hold on; grid on; box on;
for k = valid_i
    plot(results(k).t_i, results(k).i_cmd, ...
         'Color', COLORS(k,:), 'LineWidth', LINEWIDTH, ...
         'DisplayName', results(k).label);
end
xlabel('Time [s]'); ylabel('Current [A]');
title('Transient — Command Current');
legend('Location', 'best', 'FontSize', 8);
xlim([0, min(0.30, t_dist_start)]);

% -- Subplot 3: Disturbance window --------------------------------------------
subplot(2, 3, 5);
hold on; grid on; box on;
xline(t_dist_start, 'b--', 'LineWidth', 1.0, 'HandleVisibility', 'off');
for k = valid_i
    plot(results(k).t_i, results(k).i_cmd, ...
         'Color', COLORS(k,:), 'LineWidth', LINEWIDTH, ...
         'DisplayName', results(k).label);
end
xlabel('Time [s]'); ylabel('Current [A]');
title(sprintf('Disturbance Window (t \\geq %.2f s)', t_dist_start));
legend('Location', 'best', 'FontSize', 8);
xlim([t_dist_start - 0.02, min(t_dist_start + 0.30, T_sim)]);

% -- Subplot 4: Steady-state --------------------------------------------------
subplot(2, 3, 6);
hold on; grid on; box on;
for k = valid_i
    mask = results(k).t_i >= t_final_ss_start;
    plot(results(k).t_i(mask), results(k).i_cmd(mask), ...
         'Color', COLORS(k,:), 'LineWidth', LINEWIDTH, ...
         'DisplayName', results(k).label);
end
xlabel('Time [s]'); ylabel('Current [A]');
title(sprintf('Steady-State Command Current\nt > %.2f s', t_final_ss_start));
legend('Location', 'best', 'FontSize', 7);
xlim([t_final_ss_start, T_sim]);


%% --- Figure 5: Current Quality — Bar Charts ---
fig5 = figure('Name', 'Current Quality — Comparison', ...
              'Units', 'normalized', 'Position', [0.10 0.15 0.80 0.40]);

current_data  = [[results(valid_i).i_peak];
                 [results(valid_i).i_rms];
                 [results(valid_i).i_tv]];
current_names = {
    'Peak Current [A]', ...
    'RMS Current [A]', ...
    sprintf('Total Variation [A]\n(disturbance window, 200 ms)')
};

for m = 1:3
    ax = subplot(1, 3, m);
    b  = bar(current_data(m,:), 'FaceColor', 'flat');
    for i = 1:numel(valid_i)
        b.CData(i,:) = colors_i(i,:);
    end
    set(ax, 'XTickLabel', labels_i, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel(current_names{m});
    title(current_names{m}, 'FontSize', 9);
    grid on; box on;
end
sgtitle('Control Input Quality — Command Current Signal', 'FontWeight', 'bold');


end  % has_current


% #########################################################################
%% === STEP 5: Save Plots and Results ===
% #########################################################################

RESULTS_PATH = fullfile(pwd, 'Results');
if ~exist(RESULTS_PATH, 'dir')
    mkdir(RESULTS_PATH);
    fprintf('\n  Folder created: %s\n', RESULTS_PATH);
end

if isNoiseActive
    noise_tag = 'noise_active';
else
    noise_tag = 'noise_deactive';
end

fprintf('\n  Saving results to: %s\n', RESULTS_PATH);

% --- Fig 1: Time response (time series -> image for performance) ---
fname1 = sprintf('TimeResponse_%s', noise_tag);
exportgraphics(fig1, fullfile(RESULTS_PATH, [fname1 '.png']), ...
    'Resolution', 600);
fprintf('    %s.png\n', fname1);

% --- Fig 2: Transient performance criteria (bar chart -> vector) ---
fname2 = sprintf('PerformanceCriteria_%s', noise_tag);
exportgraphics(fig2, fullfile(RESULTS_PATH, [fname2 '.png']), ...
    'Resolution', 600);
fprintf('    %s.png\n', fname2);

% --- Fig 3: Disturbance rejection (bar chart -> vector) ---
fname3 = sprintf('DisturbanceRejection_%s', noise_tag);
exportgraphics(fig3, fullfile(RESULTS_PATH, [fname3 '.png']), ...
    'Resolution', 300);
fprintf('    %s.png\n', fname3);

% --- Figs 4-5: Current analysis (only if current was logged) ---
if has_current
    fname4 = sprintf('CurrentResponse_%s', noise_tag);
    exportgraphics(fig4, fullfile(RESULTS_PATH, [fname4 '.png']), ...
        'Resolution', 600);
    fprintf('    %s.png\n', fname4);

    fname5 = sprintf('CurrentQuality_%s', noise_tag);
    exportgraphics(fig5, fullfile(RESULTS_PATH, [fname5 '.png']), ...
        'Resolution', 600);
    fprintf('    %s.png\n', fname5);
end

% --- Save results as .mat ---
save(fullfile(RESULTS_PATH, sprintf('simulation_results_%s.mat', noise_tag)), ...
     'results');
fprintf('    simulation_results_%s.mat\n', noise_tag);


% #########################################################################
%% === COMPLETION MESSAGE ===
% #########################################################################

fprintf('\n%s\n', repmat('=', 1, 70));
n_ok = sum([results.success]);
fprintf('  Simulation complete: %d/%d models successful.\n', ...
        n_ok, n_models);
if has_current
    n_i = numel(valid_i);
    fprintf('  Current analysis:   %d/%d models with current signal.\n', ...
            n_i, n_ok);
end
fprintf('%s\n\n', repmat('=', 1, 70));


% #########################################################################
%% === CLEANUP: Delete Simulink Cache Files ===
% #########################################################################

folders = [{pwd}, {LIN}, {NLC}, {MPC}];
for f = 1:numel(folders)
    if exist(fullfile(folders{f}, 'slprj'), 'dir')
        rmdir(fullfile(folders{f}, 'slprj'), 's');
    end
    slxc = dir(fullfile(folders{f}, '*.slxc'));
    for i = 1:numel(slxc)
        delete(fullfile(slxc(i).folder, slxc(i).name));
    end
end
fprintf('  Cache files cleaned up.\n\n');