%#########################################################################%
%#                                                                       #%
%#  FILE:    evaluation_sled_levitation.m                                #%
%#                                                                       #%
%#  DESCRIPTION:                                                         #%
%#  Main script for evaluating the magnetic levitation simulation        #%
%#  without the Simulink GUI.                                            #%
%#                                                                       #%
%#  DISTURBANCE:                                                         #%
%#    At t = 0.5*T_sim a CONSTANT disturbance force along x and y       #%
%#    is applied (F_x, F_y).                                             #%
%#                                                                       #%
%#  PLOT STRATEGY:                                                       #%
%#    • Air gap and command current: one figure per signal group with    #%
%#      the full profile as the main plot and two embedded zoom          #%
%#      insets (init transient and disturbance response). The zoom       #%
%#      regions are marked on the main plot with red rectangles;         #%
%#      arrows connect each rectangle to its corresponding inset.        #%
%#    • Sled pose: three separate figures (full / init / dist),          #%
%#      since five DOFs are displayed side by side. Units are mm         #%
%#      (translational) and mrad (rotational).                           #%
%#                                                                       #%
%#                                                                       #%
%#  OUTPUTS (saved to Results/):                                         #%
%#    Time series:                                                        #%
%#      - airgap_noise_*.png             main plot + 4 insets            #%
%#      - current_noise_*.png            main plot + 2 insets            #%
%#      - sled_pose_{full,init,dist}_noise_*.png  (mm/mrad units)        #%
%#    Metrics:                                                            #%
%#      - pose_metrics_trans_noise_*.png                                 #%
%#      - pose_metrics_rot_noise_*.png                                   #%
%#      - airgap_metrics_noise_*.png                                     #%
%#      - current_metrics_noise_*.png                                    #%
%#    Data:                                                               #%
%#      - simulation_results_noise_*.mat                                 #%
%#                                                                       #%
%#########################################################################%

clc; clear; close all;


%% 1 — Simulation Configuration
setup_sim_params;

% --- Time windows for metric evaluation ---------------------------------
t_dist_start     = 0.5 * T_sim;                                   % disturbance onset
t_pre_ss_start   = 0.70 * t_dist_start;                           % unforced SS window
t_final_ss_start = t_dist_start + 0.90*(T_sim - t_dist_start);    % final SS window
t_tv_end         = min(t_dist_start + 0.20, T_sim);               % TV window (200 ms)

% --- Zoom windows for figures -------------------------------------------
t_init_end       = 0.10;                                          % init zoom up to 100 ms
t_dist_zoom_pre  = 0.005;                                         % 5 ms before disturbance
t_dist_zoom_post = 0.080;                                         % 80 ms after disturbance
t_dist_win_start = max(t_dist_start - t_dist_zoom_pre, 0);
t_dist_win_end   = min(t_dist_start + t_dist_zoom_post, T_sim);

% --- Tolerance bands ----------------------------------------------------
band_pct_ag       = 0.02;          % ±2 % of setpoint for settling time
band_pct_rec      = 0.05;          % 5 % of peak for recovery time
band_um_rec_floor = 0.3;           % absolute lower bound 0.3 µm

% --- Output -------------------------------------------------------------
results_dir = 'Results';
if ~exist(results_dir, 'dir'), mkdir(results_dir); end
timestamp      = char(datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss'));
fig_format     = '-dpng';
fig_dpi        = '-r300';
fig_position   = [50, 50, 1400, 800];
zmbox_color    = [1.00, 0.27, 0.00]; 
zmtitle_color  = [0.00, 0.00, 1.00];
zmplot_color   = [0, 0, 0];

%% 2 — Run Simulation (Headless, no GUI)
noise_labels = {"off", "on"};
fprintf('Starting simulation (T = %.1f s, Noise = %s) ...\n', ...
        T_sim, noise_labels{noise_switch + 1});
noise_tag = 'noise_' + noise_labels{noise_switch + 1};

load_system('SledUnit_Levitation_Control');

hws = get_param('SledUnit_Levitation_Control', 'ModelWorkspace');
evalin(hws, "run('.\Setup_Machine_Model\setup_structure_params.m')");

simOut = sim('SledUnit_Levitation_Control.slx', ...
             'StopTime',                num2str(T_sim), ...
             'SignalLoggingName',       'logsout', ...
             'ReturnWorkspaceOutputs',  'on');
fprintf('Simulation completed successfully.\n\n');


%% 3 — Extract Signals from Logging Data
time = simOut.tout;
N    = numel(time);
to_NxC = @(X) reshape(X, N, []);

Is_cmd      = to_NxC(simOut.logsout.get("u_cmd").Values.Data);
I_act_raw   = to_NxC(simOut.logsout.get("act_curVals").Values.Data);
meas_ag_raw = to_NxC(simOut.logsout.get("meas_airgap").Values.Data);

slu_x     = to_NxC(simOut.logsout.get("px").Values.Data);
slu_y     = to_NxC(simOut.logsout.get("py").Values.Data);
slu_roll  = to_NxC(simOut.logsout.get("roll").Values.Data);
slu_pitch = to_NxC(simOut.logsout.get("pitch").Values.Data);
slu_yaw   = to_NxC(simOut.logsout.get("yaw").Values.Data);


%% 4 — Reorder Sensor Signals to elMaglabels Convention
% Sensor order:   SLO=1, SRO=2, ELO=3, ERO=4, SLU=5, SRU=6, ELU=7, ERU=8
% Target order:   ELO=1, ELU=2, ERO=3, ERU=4, SLO=5, SLU=6, SRO=7, SRU=8
reorder = [3, 7, 4, 8, 1, 5, 2, 6];
I_act   = I_act_raw(:, reorder);
meas_ag = meas_ag_raw(:, reorder);

elMaglabels = {'ELO','ELU','ERO','ERU','SLO','SLU','SRO','SRU'};
idx_up = [1, 3, 5, 7];   % upper magnets: ELO, ERO, SLO, SRO
idx_lw = [2, 4, 6, 8];   % lower magnets: ELU, ERU, SLU, SRU


%% 5 — Equilibrium Current per Magnet (for Excess Energy)
mask_eq = (time >= t_pre_ss_start) & (time < t_dist_start);
if isfield(ssc_params, 'nom_curvec') && numel(ssc_params.nom_curvec) == 8
    i_eq_vec    = ssc_params.nom_curvec(:).';
    i_eq_source = 'parameter';
else
    i_eq_vec    = mean(Is_cmd(mask_eq, :), 1);
    i_eq_source = 'measured';
end
fprintf('Equilibrium currents (%s): [%s]\n', i_eq_source, sprintf('%.3f ', i_eq_vec));

i_eq_measured = mean(Is_cmd(mask_eq, :), 1);
mismatch      = abs(i_eq_vec - i_eq_measured);
if max(mismatch) > 0.05
    warning('i_eq vs measured mean differs by up to %.3f A — check ordering!', ...
            max(mismatch));
    fprintf('  Magnet  | i_eq_param  i_eq_measured  diff\n');
    for m = 1:8
        fprintf('  %-6s  | %10.3f %14.3f %6.3f\n', elMaglabels{m}, ...
                i_eq_vec(m), i_eq_measured(m), i_eq_vec(m)-i_eq_measured(m));
    end
end


%% 6 — Performance Criteria: Pose (5 DOF)
%  Scaling for display: m → µm, rad → µrad.
pose_data           = {slu_x, slu_y, slu_roll, slu_pitch, slu_yaw};
pose_labels         = {'x', 'y', 'Roll', 'Pitch', 'Yaw'};
pose_metric_units   = {'µm', 'µm', 'µrad', 'µrad', 'µrad'};
pose_metric_scale   = [1e6, 1e6, 1e6, 1e6, 1e6];  % m → µm, rad → µrad
idx_trans           = [1, 2];
idx_rot             = [3, 4, 5];
n_dof               = numel(pose_data);

pose = struct('label', {pose_labels}, 'unit', {pose_metric_units});

mask_pre      = time <  t_dist_start;
mask_dwin     = time >= t_dist_start;
mask_pre_ss   = (time >= t_pre_ss_start) & (time < t_dist_start);
mask_final_ss = time >= t_final_ss_start;
t_dwin        = time(mask_dwin);

fprintf('\n%s\n', repmat('-', 1, 90));
fprintf('  POSE PERFORMANCE CRITERIA  (target value 0 for all DOFs)\n');
fprintf('%s\n', repmat('-', 1, 90));
fprintf('  %-10s │ %12s %14s %12s %14s %12s\n', ...
        'DOF', 'PeakTrans', 'unfSS', 'PeakDist', 'IAE', 'finalSS');
fprintf('%s\n', repmat('-', 1, 90));

for d = 1:n_dof
    sig      = pose_data{d} * pose_metric_scale(d);          % display-scaled (µm / µrad)
    err_pre  = sig(mask_pre);
    err_dwin = sig(mask_dwin);

    pose.peak_transient(d)  = max(abs(err_pre));
    pose.ss_err_unforced(d) = abs(mean(sig(mask_pre_ss)));
    pose.peak_dist(d)       = max(abs(err_dwin));
    if length(t_dwin) > 1
        pose.IAE(d)  = trapz(t_dwin, abs(err_dwin));
        pose.ITAE(d) = trapz(t_dwin, (t_dwin - t_dist_start) .* abs(err_dwin));
    else
        pose.IAE(d) = NaN; pose.ITAE(d) = NaN;
    end
    pose.ss_err_final(d) = abs(mean(sig(mask_final_ss)));

    fprintf('  %-10s │ %10.3f %s %12.4f %s %10.3f %s %12.4f %s %10.4f %s\n', ...
            pose_labels{d}, ...
            pose.peak_transient(d), pose_metric_units{d}, ...
            pose.ss_err_unforced(d), pose_metric_units{d}, ...
            pose.peak_dist(d),      pose_metric_units{d}, ...
            pose.IAE(d),            [pose_metric_units{d} '·s'], ...
            pose.ss_err_final(d),   pose_metric_units{d});
end
fprintf('%s\n', repmat('-', 1, 90));

% --- Diagnostic: unexcited DOFs in pre-disturbance phase ----------------
% A symmetric initialization does not excite certain DOFs (typically x,
% Pitch, Yaw) — peak_transient then sits at numerical noise (~1e-4 µm).
% This is physically correct, not an algorithm error.
noise_floor_thr = 1e-3;       % below 1 nm or 1 nrad => numerical noise
fprintf('\n  DIAGNOSTIC Pre-Disturbance Excitation:\n');
any_quiet = false;
for d = 1:n_dof
    if pose.peak_transient(d) < noise_floor_thr
        any_quiet = true;
        fprintf('    %-5s: peak_transient = %.2e %s  → no pre-disturbance excitation\n', ...
                pose_labels{d}, pose.peak_transient(d), pose_metric_units{d});
        fprintf('           peak_dist      = %.3f %s  → excited only by disturbance\n', ...
                pose.peak_dist(d), pose_metric_units{d});
    end
end
if ~any_quiet
    fprintf('    all DOFs above numerical noise floor.\n');
end


%% 7 — Performance Criteria: Air Gaps (8 Magnets)
target_ag = ssc_params.nom_airgap;                  % [mm]
band_ag   = band_pct_ag * target_ag;                % [mm]  — settling band

ag_perf = struct('label', {elMaglabels});
fprintf('\n%s\n', repmat('-', 1, 120));
fprintf('  AIR GAP PERFORMANCE CRITERIA  (setpoint %.3f mm)\n', target_ag);
fprintf('    Settling band  (for tSettle):  ±%.0f %% × setpoint = ±%.3f mm\n', ...
        band_pct_ag*100, band_ag);
fprintf('    Recovery band  (for tRec):     max(%.0f %% × peak_dist, %.1f µm) — adaptive\n', ...
        band_pct_rec*100, band_um_rec_floor);
fprintf('%s\n', repmat('-', 1, 120));
fprintf('  %-6s │ %10s %12s %10s %10s %10s %12s %12s %12s\n', ...
        'Magnet', 'PeakT[µm]', 'tSettle[ms]', 'unfSS[µm]', ...
        'PeakD[µm]', 'recBd[µm]', 'tRec[ms]', 'IAE[µm·s]', 'finalSS[µm]');
fprintf('%s\n', repmat('-', 1, 120));

for m = 1:8
    sig      = meas_ag(:, m);                       % [mm]
    err      = (sig - target_ag);                   % [mm], signed
    err_pre  = err(mask_pre);
    err_dwin = err(mask_dwin);

    % Transient peak (pre-disturbance region)
    ag_perf.peak_transient(m) = max(abs(err_pre)) * 1e3;        % µm

    % Settling time (±2 % of setpoint)
    in_band_pre = abs(err_pre) <= band_ag;
    last_out    = find(~in_band_pre, 1, 'last');
    if isempty(last_out)
        ag_perf.t_settle(m) = 0;
    else
        t_pre = time(mask_pre);
        ag_perf.t_settle(m) = t_pre(last_out) * 1e3;            % ms
    end

    ag_perf.ss_err_unforced(m) = abs(mean(err(mask_pre_ss))) * 1e3;   % µm

    % Disturbance peak
    ag_perf.peak_dist(m) = max(abs(err_dwin)) * 1e3;                 % µm

    % Recovery time with adaptive band -----------------------------------
    % BUGFIX vs. original version: the previous band
    % (±2 % of setpoint = 10 µm) was larger than every peak (< 10 µm),
    % causing t_rec to always be 0. Adaptive band resolves this.
    err_um_dwin   = abs(err_dwin) * 1e3;                              % µm
    rec_band_um   = max(band_pct_rec * ag_perf.peak_dist(m), band_um_rec_floor);
    ag_perf.rec_band(m) = rec_band_um;
    in_band_r     = err_um_dwin <= rec_band_um;
    last_out_r    = find(~in_band_r, 1, 'last');
    if isempty(last_out_r)
        ag_perf.t_recovery(m) = 0;
    else
        ag_perf.t_recovery(m) = (t_dwin(last_out_r) - t_dist_start) * 1e3;  % ms
    end

    % IAE + ITAE in disturbance window
    if length(t_dwin) > 1
        ag_perf.IAE(m)  = trapz(t_dwin, err_um_dwin);                % µm·s
        ag_perf.ITAE(m) = trapz(t_dwin, (t_dwin - t_dist_start) .* err_um_dwin);
    else
        ag_perf.IAE(m) = NaN; ag_perf.ITAE(m) = NaN;
    end

    ag_perf.ss_err_final(m) = abs(mean(err(mask_final_ss))) * 1e3;   % µm

    fprintf('  %-6s │ %10.2f %12.1f %10.3f %10.2f %10.3f %12.2f %12.4f %12.3f\n', ...
            elMaglabels{m}, ag_perf.peak_transient(m), ag_perf.t_settle(m), ...
            ag_perf.ss_err_unforced(m), ag_perf.peak_dist(m), ...
            ag_perf.rec_band(m), ag_perf.t_recovery(m), ...
            ag_perf.IAE(m), ag_perf.ss_err_final(m));
end
fprintf('%s\n', repmat('-', 1, 120));


%% 8 — Performance Criteria: Current Quality (8 Magnets)
cur_perf = struct('label', {elMaglabels});

fprintf('\n%s\n', repmat('-', 1, 100));
fprintf('  CURRENT QUALITY  (excess w.r.t. i_eq, %s)\n', i_eq_source);
fprintf('%s\n', repmat('-', 1, 100));
fprintf('  %-6s │ %10s %10s %10s %14s %14s %14s\n', ...
        'Magnet', 'Peak[A]', 'RMS[A]', 'i_eq[A]', ...
        'TV-Dist[A]', 'W_T[A²·s]', 'W_D[A²·s]');
fprintf('%s\n', repmat('-', 1, 100));

for m = 1:8
    i_cmd = Is_cmd(:, m);
    i_eq  = i_eq_vec(m);

    cur_perf.i_peak(m) = max(abs(i_cmd));
    if length(time) > 1
        cur_perf.i_rms(m) = sqrt(trapz(time, i_cmd.^2) / (time(end) - time(1)));
    else
        cur_perf.i_rms(m) = abs(i_cmd);
    end

    idx_tv   = (time >= t_dist_start) & (time <= t_tv_end);
    i_tv_win = i_cmd(idx_tv);
    if length(i_tv_win) > 1
        cur_perf.i_tv(m) = sum(abs(diff(i_tv_win)));
    else
        cur_perf.i_tv(m) = NaN;
    end

    if sum(mask_pre) > 1
        cur_perf.W_excess_T(m) = trapz(time(mask_pre), (i_cmd(mask_pre) - i_eq).^2);
    else
        cur_perf.W_excess_T(m) = NaN;
    end
    if sum(mask_dwin) > 1
        cur_perf.W_excess_D(m) = trapz(time(mask_dwin), (i_cmd(mask_dwin) - i_eq).^2);
    else
        cur_perf.W_excess_D(m) = NaN;
    end

    cur_perf.i_eq(m) = i_eq;

    fprintf('  %-6s │ %10.3f %10.3f %10.3f %14.2f %14.4f %14.4f\n', ...
            elMaglabels{m}, cur_perf.i_peak(m), cur_perf.i_rms(m), ...
            i_eq, cur_perf.i_tv(m), cur_perf.W_excess_T(m), cur_perf.W_excess_D(m));
end
fprintf('%s\n', repmat('-', 1, 100));


%% 9 — Auxiliary Variables for Figures
dist_label = sprintf('Const. F_x, F_y @ %.2f s', t_dist_start);

% Plot views for the pose (three separate figures: full / init / dist).
% Air gap and command current use main plot + insets instead.
views(1).tag       = 'full';
views(1).xlim      = [];
views(1).label     = 'Full Profile';
views(1).show_dist = true;

views(2).tag       = 'init';
views(2).xlim      = [0, t_init_end];
views(2).label     = sprintf('Initialization Transient (0 \\rightarrow %.2f s)', t_init_end);
views(2).show_dist = false;

views(3).tag       = 'dist';
views(3).xlim      = [t_dist_win_start, t_dist_win_end];
views(3).label     = sprintf('Disturbance Response (%.3f \\rightarrow %.3f s)', ...
                             t_dist_win_start, t_dist_win_end);
views(3).show_dist = true;


%% 10 — Figure: Air Gap Profiles with Zoom Insets
%  Layout: four rows × two columns
%    Row 1 : upper main plot  (spanning both columns)
%    Row 2 : init inset (left) | dist inset (right) for upper magnets
%    Row 3 : lower main plot  (spanning both columns)
%    Row 4 : init inset (left) | dist inset (right) for lower magnets

fig_airgap = figure('Name', 'Air Gap with Insets', 'Position', [80, 80, 1500, 1100]);
tlay_ag = tiledlayout(fig_airgap, 4, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

magnet_groups = {idx_up, idx_lw};
group_styles  = {'-', '--'};
group_titles  = {'Upper Magnets (ELO, ERO, SLO, SRO)', ...
                 'Lower Magnets (ELU, ERU, SLU, SRU)'};

ax_main_ag = cell(2, 1);
ax_init_ag = cell(2, 1);
ax_dist_ag = cell(2, 1);

for g = 1:2
    % --- Main plot (full width) ---
    ax_main_ag{g} = nexttile(tlay_ag, [1 2]);
    hold on;
    plot(time, meas_ag(:, magnet_groups{g}), group_styles{g}, 'LineWidth', 1.2);
    yline(target_ag, 'k-.', 'Setpoint', 'LineWidth', 0.8, ...
          'LabelHorizontalAlignment', 'right');
    xline(t_dist_start, 'b--', 'Label', dist_label, ...
          'LineWidth', 1.0, 'LabelOrientation', 'horizontal', ...
          'LabelHorizontalAlignment', 'right', ...
          'LabelVerticalAlignment', 'top', ...
          'HandleVisibility', 'off');
    hold off;
    grid on;
    ylabel('Air Gap [mm]');
    if g == 2, xlabel('Time [s]'); end
    legend(elMaglabels(magnet_groups{g}), 'Location', 'east');
    title(group_titles{g});

    % --- Inset: Init Transient ---
    ax_init_ag{g} = nexttile(tlay_ag);
    hold on;
    plot(time, meas_ag(:, magnet_groups{g}), group_styles{g}, 'LineWidth', 1.0);
    yline(target_ag, 'k-.', 'LineWidth', 0.5);
    hold off;
    xlim([0, t_init_end]);
    grid on; box on;
    set(gca, 'XColor', zmplot_color, 'YColor', zmplot_color, 'LineWidth', 1.2);
    title('Init Transient', 'FontSize', 9, 'Color', zmtitle_color);
    ylabel('Air Gap [mm]', 'FontSize', 9);
    if g == 2, xlabel('Time [s]', 'FontSize', 9); end

    % --- Inset: Disturbance Response ---
    ax_dist_ag{g} = nexttile(tlay_ag);
    hold on;
    plot(time, meas_ag(:, magnet_groups{g}), group_styles{g}, 'LineWidth', 1.0);
    yline(target_ag, 'k-.', 'LineWidth', 0.5);
    xline(t_dist_start, 'b--', 'LineWidth', 0.5);
    hold off;
    xlim([t_dist_win_start, t_dist_win_end]);
    grid on; box on;
    set(gca, 'XColor', zmplot_color, 'YColor', zmplot_color, 'LineWidth', 1.2);
    title('Disturbance Response', 'FontSize', 9, 'Color', zmtitle_color);
    if g == 2, xlabel('Time [s]', 'FontSize', 9); end
end

title(tlay_ag, 'Air Gap Profiles with Zoom Insets', 'FontWeight', 'bold');

% --- Draw rectangles on main plots + arrows to insets -------------------
drawnow;   % let the layout engine resolve positions

for g = 1:2
    ax = ax_main_ag{g};
    yl = ax.YLim;

    % Zoom-region rectangles (data coordinates)
    rectangle(ax, 'Position', [0, yl(1), t_init_end, diff(yl)], ...
              'EdgeColor', zmbox_color, 'LineWidth', 1.5, 'LineStyle','-');
    rectangle(ax, 'Position', [t_dist_win_start, yl(1), ...
                                t_dist_win_end - t_dist_win_start, diff(yl)], ...
              'EdgeColor', zmbox_color, 'LineWidth', 1.5, 'LineStyle','-');

    % Arrows: from the bottom edge of the rectangle to the top edge of the inset
    pos_main = ax.Position;
    pos_init = ax_init_ag{g}.Position;
    pos_dist = ax_dist_ag{g}.Position;

    % --- Init arrow ---
    xl_main = ax.XLim;
    x_src_n = pos_main(1) + (t_init_end/2 - xl_main(1)) / diff(xl_main) * pos_main(3);
    y_src_n = pos_main(2);                              % bottom edge of main plot
    x_dst_n = pos_init(1) + 0.5 * pos_init(3);
    y_dst_n = pos_init(2) + pos_init(4);                % top edge of inset
    annotation(fig_airgap, 'arrow', [x_src_n x_dst_n], [y_src_n y_dst_n], ...
               'Color', zmbox_color, 'LineWidth', 1.2, 'HeadWidth', 8);

    % --- Dist arrow ---
    x_src_n = pos_main(1) + ((t_dist_win_start + t_dist_win_end)/2 - xl_main(1)) ...
              / diff(xl_main) * pos_main(3);
    y_src_n = pos_main(2);
    x_dst_n = pos_dist(1) + 0.5 * pos_dist(3);
    y_dst_n = pos_dist(2) + pos_dist(4);
    annotation(fig_airgap, 'arrow', [x_src_n x_dst_n], [y_src_n y_dst_n], ...
               'Color', zmbox_color, 'LineWidth', 1.2, 'HeadWidth', 8);
end

print(fig_airgap, fullfile(results_dir, sprintf('airgap_%s', noise_tag)), ...
      fig_format, fig_dpi);


%% 11 — Figure: Sled Unit Pose (3 views, mm / mrad)
%  Sled pose is displayed in mm (x, y) and mrad (Roll, Pitch, Yaw).
%  Scaling: position m → mm × 1e3, angle rad → mrad × 1e3.
pose_signals_plot = {slu_x*1e3, slu_y*1e3, slu_roll*1e3, slu_pitch*1e3, slu_yaw*1e3};
pose_titles_plot  = {'x [mm]', 'y [mm]', 'Roll [mrad]', 'Pitch [mrad]', 'Yaw [mrad]'};
pose_colors       = {'b', '#A2142F', 'm', '#0072BD', '#7E2F8E'};

for v = 1:numel(views)
    fig = figure('Name', sprintf('Sled Pose — %s', views(v).tag), 'Position', fig_position);

    for d = 1:5
        subplot(5, 1, d);
        hold on;
        plot(time, pose_signals_plot{d}, 'Color', pose_colors{d}, 'LineWidth', 1.2);
        if views(v).show_dist
            xline(t_dist_start, 'b--', 'Label', dist_label, ...
                  'LineWidth', 1.0, 'LabelOrientation', 'horizontal', ...
                  'LabelHorizontalAlignment', 'right', ...
                  'LabelVerticalAlignment', 'top', ...
                  'HandleVisibility', 'off');
        end
        hold off;
        ylabel(pose_titles_plot{d}); grid on;
        if ~isempty(views(v).xlim), xlim(views(v).xlim); end
        if d == 1
            title(sprintf('Sled Unit Pose — %s', views(v).label));
        end
    end
    xlabel('Time [s]');

    print(fig, fullfile(results_dir, sprintf('sled_pose_%s_%s', views(v).tag, noise_tag)), ...
          fig_format, fig_dpi);
end


%% 12 — Figure: Command Current Profiles with Zoom Insets
%  Layout: two rows × two columns
%    Row 1 : main plot (all 8 magnets, spanning both columns)
%    Row 2 : init inset (left) | dist inset (right)

fig_current = figure('Name', 'Command Current with Insets', 'Position', [50, 50, 1500, 900]);
tlay_cur = tiledlayout(fig_current, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- Main plot (full width) ---------------------------------------------
ax_main_cur = nexttile(tlay_cur, [1 2]);
hold on;
plot(time, Is_cmd(:, idx_up), '-',  'LineWidth', 1.2);
plot(time, Is_cmd(:, idx_lw), '--', 'LineWidth', 1.2);
xline(t_dist_start, 'k:', 'Label', dist_label, ...
      'LineWidth', 1.0, 'LabelOrientation', 'horizontal', ...
      'LabelHorizontalAlignment', 'right', ...
      'LabelVerticalAlignment', 'top', ...
      'HandleVisibility', 'off');
hold off;
ylabel('Command Current I_{cmd} [A]'); xlabel('Time [s]'); grid on;
legend([elMaglabels(idx_up), elMaglabels(idx_lw)], ...
       'Location', 'east', 'NumColumns', 2);
title('Command Current — All 8 Magnets');

% --- Inset: Init Transient ----------------------------------------------
ax_init_cur = nexttile(tlay_cur);
hold on;
plot(time, Is_cmd(:, idx_up), '-',  'LineWidth', 1.0);
plot(time, Is_cmd(:, idx_lw), '--', 'LineWidth', 1.0);
hold off;
xlim([0, t_init_end]);
grid on; box on;
set(gca, 'XColor', zmplot_color, 'YColor', zmplot_color, 'LineWidth', 1.2);
title('Init Transient', 'FontSize', 9, 'Color', zmtitle_color);
ylabel('I_{cmd} [A]', 'FontSize', 9);
xlabel('Time [s]', 'FontSize', 9);

% --- Inset: Disturbance Response ----------------------------------------
ax_dist_cur = nexttile(tlay_cur);
hold on;
plot(time, Is_cmd(:, idx_up), '-',  'LineWidth', 1.0);
plot(time, Is_cmd(:, idx_lw), '--', 'LineWidth', 1.0);
xline(t_dist_start, 'k:', 'LineWidth', 0.5);
hold off;
xlim([t_dist_win_start, t_dist_win_end]);
grid on; box on;
set(gca, 'XColor', zmplot_color, 'YColor', zmplot_color, 'LineWidth', 1.2);
title('Disturbance Response', 'FontSize', 9, 'Color', zmtitle_color);
xlabel('Time [s]', 'FontSize', 9);

title(tlay_cur, 'Command Current Profiles with Zoom Insets', 'FontWeight', 'bold');

% --- Rectangles + arrows ------------------------------------------------
drawnow;
yl_cur = ax_main_cur.YLim;
xl_cur = ax_main_cur.XLim;

rectangle(ax_main_cur, 'Position', [0, yl_cur(1), t_init_end, diff(yl_cur)], ...
          'EdgeColor', zmbox_color, 'LineWidth', 1.5);
rectangle(ax_main_cur, 'Position', [t_dist_win_start, yl_cur(1), ...
                                     t_dist_win_end - t_dist_win_start, diff(yl_cur)], ...
          'EdgeColor', zmbox_color, 'LineWidth', 1.5);

pos_main = ax_main_cur.Position;
pos_init = ax_init_cur.Position;
pos_dist = ax_dist_cur.Position;

% --- Init arrow ---
x_src_n = pos_main(1) + (t_init_end/2 - xl_cur(1)) / diff(xl_cur) * pos_main(3);
y_src_n = pos_main(2);
x_dst_n = pos_init(1) + 0.5 * pos_init(3);
y_dst_n = pos_init(2) + pos_init(4);
annotation(fig_current, 'arrow', [x_src_n x_dst_n], [y_src_n y_dst_n], ...
           'Color', zmbox_color, 'LineWidth', 1.2, 'HeadWidth', 8);

% --- Dist arrow ---
x_src_n = pos_main(1) + ((t_dist_win_start + t_dist_win_end)/2 - xl_cur(1)) ...
          / diff(xl_cur) * pos_main(3);
y_src_n = pos_main(2);
x_dst_n = pos_dist(1) + 0.5 * pos_dist(3);
y_dst_n = pos_dist(2) + pos_dist(4);
annotation(fig_current, 'arrow', [x_src_n x_dst_n], [y_src_n y_dst_n], ...
           'Color', zmbox_color, 'LineWidth', 1.2, 'HeadWidth', 8);

print(fig_current, fullfile(results_dir, sprintf('current_%s', noise_tag)), ...
      fig_format, fig_dpi);


%% 13 — Bar Chart: Pose Performance Metrics (Trans. + Rot. in one figure)
%  Row 1: translational DOFs (x, y) in µm
%  Row 2: rotational DOFs (Roll, Pitch, Yaw) in µrad
%  Metrics: Peak Transient + Unforced SS   → init transient phase
%           Peak Dist + IAE + ITAE + finSS → disturbance rejection phase
fig_pose = figure('Name', 'Pose Performance Metrics', 'Position', [50 50 1700 640]);

trans_data = [pose.peak_transient(idx_trans);
              pose.ss_err_unforced(idx_trans);
              pose.peak_dist(idx_trans);
              pose.IAE(idx_trans);
              pose.ITAE(idx_trans);
              pose.ss_err_final(idx_trans)];
trans_names = {'Peak Transient [µm]', ...
               'Unforced SS-Error [µm]', ...
               'Peak Disturbance [µm]', ...
               'IAE [µm·s]', ...
               'ITAE [µm·s²]', ...
               'Final SS-Error [µm]'};

rot_data = [pose.peak_transient(idx_rot);
            pose.ss_err_unforced(idx_rot);
            pose.peak_dist(idx_rot);
            pose.IAE(idx_rot);
            pose.ITAE(idx_rot);
            pose.ss_err_final(idx_rot)];
rot_names = {'Peak Transient [µrad]', ...
             'Unforced SS-Error [µrad]', ...
             'Peak Disturbance [µrad]', ...
             'IAE [µrad·s]', ...
             'ITAE [µrad·s²]', ...
             'Final SS-Error [µrad]'};

% --- Row 1: translational ---
for k = 1:6
    subplot(2, 6, k);
    bar(trans_data(k, :)); grid on; box on;
    set(gca, 'XTickLabel', pose_labels(idx_trans), 'FontSize', 9);
    title(trans_names{k}, 'FontSize', 10);
    if k == 1
        ylabel('Translational DOFs', 'FontWeight', 'bold');
    end
end

% --- Row 2: rotational ---
for k = 1:6
    subplot(2, 6, 6 + k);
    bar(rot_data(k, :)); grid on; box on;
    set(gca, 'XTickLabel', pose_labels(idx_rot), 'XTickLabelRotation', 30, 'FontSize', 9);
    title(rot_names{k}, 'FontSize', 10);
    if k == 1
        ylabel('Rotational DOFs', 'FontWeight', 'bold');
    end
end

sgtitle('Pose Performance Criteria — Translational (top) / Rotational (bottom)', ...
        'FontWeight', 'bold');

print(fig_pose, fullfile(results_dir, sprintf('pose_metrics_%s', noise_tag)), ...
      fig_format, fig_dpi);


%% 14 — Bar Chart: Air Gap Performance Criteria (8 Magnets)
fig_ag = figure('Name', 'Air Gap Metrics', 'Position', [50 50 1600 700]);

ag_data = {ag_perf.peak_transient,  'Peak Transient [µm]';
           ag_perf.peak_dist,       'Peak Disturbance [µm]';
           ag_perf.t_recovery,      sprintf('Recovery Time [ms]\n(adaptive band: max(5%%\\cdotpeak,0.3µm))');
           ag_perf.IAE,             'IAE [µm·s]';
           ag_perf.ITAE,            'ITAE [µm·s²]';
           ag_perf.ss_err_final,    'Final SS-Error [µm]'};

colors_ag = repmat([0.30 0.55 0.85], 8, 1);
colors_ag(idx_lw, :) = repmat([0.85 0.40 0.30], numel(idx_lw), 1);

for k = 1:6
    subplot(2, 3, k);
    b = bar(ag_data{k, 1}, 'FaceColor', 'flat');
    for mm = 1:8, b.CData(mm,:) = colors_ag(mm,:); end
    set(gca, 'XTickLabel', elMaglabels, 'XTickLabelRotation', 30, 'FontSize', 9);
    title(ag_data{k, 2}, 'FontSize', 10);
    grid on; box on;
end
sgtitle(sprintf('Air Gap Performance Criteria — 8 Magnets  (Setpoint %.3f mm)', ...
        target_ag), 'FontWeight', 'bold');

print(fig_ag, fullfile(results_dir, sprintf('airgap_metrics_%s', noise_tag)), ...
      fig_format, fig_dpi);


%% 15 — Bar Chart: Current Quality (8 Magnets)
fig_cur = figure('Name', 'Current Quality', 'Position', [50 50 1600 700]);

cur_data = {cur_perf.i_peak,       'Peak Current [A]';
            cur_perf.i_rms,        'RMS Current [A]';
            cur_perf.i_tv,         sprintf('Total Variation [A]\n(disturbance window, 200 ms)');
            cur_perf.W_excess_T,   sprintf('Transient Excess Energy W_T [A^2 \\cdot s]\n(\\int(i-i_{eq})^2 dt, t < %.2f s)', t_dist_start);
            cur_perf.W_excess_D,   sprintf('Disturbance Excess Energy W_D [A^2 \\cdot s]\n(\\int(i-i_{eq})^2 dt, t \\geq %.2f s)', t_dist_start);
            cur_perf.i_eq,         'Equilibrium Current i_{eq} [A]'};

for k = 1:6
    subplot(2, 3, k);
    b = bar(cur_data{k, 1}, 'FaceColor', 'flat');
    for mm = 1:8, b.CData(mm,:) = colors_ag(mm,:); end
    set(gca, 'XTickLabel', elMaglabels, 'XTickLabelRotation', 30, 'FontSize', 9);
    title(cur_data{k, 2}, 'FontSize', 10);
    grid on; box on;
end
sgtitle('Current Quality — Command Current per Magnet', 'FontWeight', 'bold');

print(fig_cur, fullfile(results_dir, sprintf('current_metrics_%s', noise_tag)), ...
      fig_format, fig_dpi);


%% 17 — Save Results
results.time         = time;
results.Is_cmd       = Is_cmd;
results.I_act        = I_act;
results.meas_ag      = meas_ag;
results.pose         = [slu_x, slu_y, slu_roll, slu_pitch, slu_yaw];
results.labels       = elMaglabels;
results.nom_airgap   = target_ag;
results.i_eq_vec     = i_eq_vec;
results.pose_metrics = pose;
results.ag_metrics   = ag_perf;
results.cur_metrics  = cur_perf;
results.config       = struct('T_sim',           T_sim, ...
                              't_dist_start',    t_dist_start, ...
                              't_init_end',      t_init_end, ...
                              't_dist_win',      [t_dist_win_start, t_dist_win_end], ...
                              'band_pct_ag',     band_pct_ag, ...
                              'band_pct_rec',    band_pct_rec, ...
                              'band_um_rec_floor', band_um_rec_floor, ...
                              'noise',           noise_switch, ...
                              'timestamp',       timestamp);

save(fullfile(results_dir, sprintf('simulation_results_%s.mat', noise_tag)), 'results');

fprintf('\n=== Results saved to %s/ ===\n', results_dir);
fprintf('  Time series:\n');
fprintf('    airgap_%s.png             (main plot + 4 insets)\n', noise_tag);
fprintf('    current_%s.png            (main plot + 2 insets)\n', noise_tag);
for v = 1:numel(views)
    fprintf('    sled_pose_%s_%s.png\n', views(v).tag, noise_tag);
end
fprintf('  Metrics:\n');
fprintf('    pose_metrics_%s.png       (trans top + rot bottom)\n', noise_tag);
fprintf('    airgap_metrics_%s.png\n',     noise_tag);
fprintf('    current_metrics_%s.png\n',    noise_tag);
fprintf('  Data:      simulation_results_%s.mat\n', noise_tag);


%% 18 — Cleanup: Remove Simulink Cache Files
if exist('slprj', 'dir'), rmdir('slprj', 's'); end
slxc = dir(fullfile(pwd, '*.slxc'));
for k = 1:numel(slxc)
    delete(fullfile(slxc(k).folder, slxc(k).name));
end
fprintf('Cache files removed successfully.\n');