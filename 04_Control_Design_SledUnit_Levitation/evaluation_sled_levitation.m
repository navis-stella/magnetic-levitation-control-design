%#########################################################################%
%#                                                                       #%
%#  FILE:    evaluation_sled_levitation.m                                #%
%#                                                                       #%
%#  DESCRIPTION:                                                         #%
%#  Main script for evaluating the magnetic bearing simulation without   #%
%#  the Simulink GUI. The model is executed, logged signals are          #%
%#  extracted, diagnostic plots are created, and all results are saved.  #%
%#                                                                       #%
%#  PARAMETER STRUCTURE:                                                 #%
%#    - setup_sled_dyn_params.m   Kinematic & Dynamic Parameters         #%
%#                                (Mass, Inertia Tensor, Geometry)       #%
%#    - setup_sim_params.m        Simulation Parameters                  #%
%#                                (Duration, Disturbances, Noise switch) #%
%#    - Controller_Params         Saved Controller Parameters            #%
%#                                (Reinforcements, Observer Matrices)    #%
%#                                                                       #%
%#  PREREQUISITES:                                                       #%
%#    Standard Case: Run this script directly.                           #%
%#          After making changes to setup_sled_dyn_params.m:             #%
%#              → Re-run state_space_control_design.m so that            #%
%#                Controller_Params are updated.                         #%
%#                                                                       #%
%#  OUTPUTS (saved in Results/):                                         #%
%#    - airgap_profiles.png         Air gap profiles per magnet          #%
%#    - current_profiles.png        Target and actual currents           #%
%#    - sled_unit_pose.png          Position and angle profiles          #%
%#    - simulation_results.mat      Complete time-series data            #%
%#                                                                       #%
%#########################################################################%

clc; clear; close all;

% Helper function as a substitute for the ternary operator
ternary = @(cond, a, b) subsref({b, a}, struct('type','{}','subs',{{cond+1}}));

%% 1 — Simulation Configuration
% Load controller and environmental parameters
setup_sim_params;

% Simulation duration [s]
T_sim = 2;

% Sensor noise: 0 = disabled, 1 = enabled
noise_switch = 1;
observer_method = 'kalman filter';

% Create output directory if it doesn't exist
results_dir = 'Results';
if ~exist(results_dir, 'dir'), mkdir(results_dir); end

% Timestamp for unique filenames
timestamp = char(datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss'));

% Export settings for figures
fig_format   = '-dpng';
fig_dpi      = '-r300';
fig_position = [50, 50, 1400, 800];  % [left, bottom, width, height] in pixels

%% 2 — Run Simulation (Headless, without GUI)
noise_labels = {"off", "on"};
fprintf('Starting simulation (T = %.1f s, Noise = %s) ...\n', ...
        T_sim, noise_labels{noise_switch + 1});

noise_tag = 'noise_' + noise_labels{noise_switch + 1};

simOut = sim('SledUnit_Levitation_Control.slx', ...
             'StopTime',                num2str(T_sim), ...
             'SignalLoggingName',       'logsout', ...
             'ReturnWorkspaceOutputs',  'on');

fprintf('Simulation completed successfully.\n\n');

%% 3 — Extract Signals from Logging Data
time = simOut.tout;   % Time vector [N×1]
N    = numel(time);

% Helper function: bring signal array to [N × channels]
fix = @(X) reshape(X, N, []);

% Target currents (order according to elMaglabels)
Is_cmd = fix(simOut.logsout.get("u_cmd").Values.Data);

% Actual currents and measured air gaps (sensor order; will be reordered in Section 4)
I_act_raw   = fix(simOut.logsout.get("act_curVals").Values.Data);
meas_ag_raw = fix(simOut.logsout.get("meas_airgap").Values.Data);

% Pose of the sled unit (translational and rotational)
slu_x     = fix(simOut.logsout.get("x").Values.Data);
slu_y     = fix(simOut.logsout.get("y").Values.Data);
slu_roll  = fix(simOut.logsout.get("roll").Values.Data);
slu_pitch = fix(simOut.logsout.get("pitch").Values.Data);
slu_yaw   = fix(simOut.logsout.get("yaw").Values.Data);

% Velocities of the sled unit
slu_x_dot     = fix(simOut.logsout.get("x_dot").Values.Data);
slu_y_dot     = fix(simOut.logsout.get("y_dot").Values.Data);
slu_roll_dot  = fix(simOut.logsout.get("roll_dot").Values.Data);
slu_pitch_dot = fix(simOut.logsout.get("pitch_dot").Values.Data);
slu_yaw_dot   = fix(simOut.logsout.get("yaw_dot").Values.Data);

%% 4 — Reorder Sensor Signals to elMaglabels Convention
% Sensor output order:     SLO=1, SRO=2, ELO=3, ERO=4, SLU=5, SRU=6, ELU=7, ERU=8
% Target order (elMaglabels): ELO=1, ELU=2, ERO=3, ERU=4, SLO=5, SLU=6, SRO=7, SRU=8
reorder = [3, 7, 4, 8, 1, 5, 2, 6];

I_act   = I_act_raw(:, reorder);     % Reordered actual currents [N×8]
meas_ag = meas_ag_raw(:, reorder);   % Reordered air gaps [N×8]

elMaglabels = {'ELO','ELU','ERO','ERU','SLO','SLU','SRO','SRU'};
idx_up = [1, 3, 5, 7];   % Indices of upper magnets: ELO, ERO, SLO, SRO
idx_lw = [2, 4, 6, 8];   % Indices of lower magnets: ELU, ERU, SLU, SRU

%% 5 — Calculate Steady-State Key Figures
% Evaluation over the last 10% of the simulation duration
idx_ss = round(0.9*N):N;

ag_ss_mean = mean(meas_ag(idx_ss, :), 1);          % Mean air gap per magnet [mm]
ag_ss_err  = ag_ss_mean - ssc_params.nom_airgap;   % Deviation from target air gap [mm]

pose_ss = [mean(slu_x(idx_ss)),     mean(slu_y(idx_ss)), ...
           mean(slu_roll(idx_ss)),   mean(slu_pitch(idx_ss)), ...
           mean(slu_yaw(idx_ss))];

fprintf('=== Steady-State Key Figures (Last 10%% of Simulation) ===\n');
fprintf('  Air gap deviation from target value [mm]:\n');
for k = 1:8
    fprintf('    %-4s: %+.4f mm\n', elMaglabels{k}, ag_ss_err(k));
end
fprintf('  Maximum air gap deviation (magnitude): %.4f mm\n', max(abs(ag_ss_err)));
fprintf('  Pose [x, y, Roll, Pitch, Yaw]:\n');
fprintf('    [%.4e, %.4e, %.4e, %.4e, %.4e]\n', pose_ss);

%% 6 — Diagram: Air Gap Profiles
fig1 = figure('Name', 'Air Gap Profiles', 'Position', fig_position);
subplot(2,1,1);
hold on;
plot(time, meas_ag(:, idx_up), '-', 'LineWidth', 1.2);
yline(ssc_params.nom_airgap, 'k-.', 'Target', 'LineWidth', 0.8);
hold off;
ylabel('Air Gap [mm]'); grid on;
legend(elMaglabels(idx_up), 'Location', 'east');
title('Upper Magnets (ELO, ERO, SLO, SRO)');

subplot(2,1,2);
hold on;
plot(time, meas_ag(:, idx_lw), '--', 'LineWidth', 1.2);
yline(ssc_params.nom_airgap, 'k-.', 'Target', 'LineWidth', 0.8);
hold off;
ylabel('Air Gap [mm]'); xlabel('Time [s]'); grid on;
legend(elMaglabels(idx_lw), 'Location', 'east');
title('Lower Magnets (ELU, ERU, SLU, SRU)');

sgtitle('Air Gap Profiles');
print(fig1, fullfile(results_dir, sprintf('airgap_profiles_%s', noise_tag)), fig_format, fig_dpi);

%% 7 — Diagram: Current Profiles
fig2 = figure('Name', 'Current Profiles', 'Position', fig_position);
subplot(2,1,1);
hold on;
plot(time, Is_cmd(:, idx_up), '-',  'LineWidth', 1.2);
plot(time, Is_cmd(:, idx_lw), '--', 'LineWidth', 1.2);
hold off;
ylabel('Target Current [A]'); grid on;
legend([elMaglabels(idx_up), elMaglabels(idx_lw)], ...
       'Location', 'east', 'NumColumns', 2);
title('Target Currents (I_{cmd})');

subplot(2,1,2);
hold on;
plot(time, I_act(:, idx_up), '-',  'LineWidth', 1.2);
plot(time, I_act(:, idx_lw), '--', 'LineWidth', 1.2);
% yline([I_min, I_max], 'r:', 'LineWidth', 0.8);  % Current limits (show if needed)
hold off;
ylabel('Actual Current [A]'); xlabel('Time [s]'); grid on;
legend([elMaglabels(idx_up), elMaglabels(idx_lw)], ...
       'Location', 'east', 'NumColumns', 2);
title('Actual Currents (I_{act})');

sgtitle('Current Profiles');
print(fig2, fullfile(results_dir, sprintf('current_profiles_%s', noise_tag)), fig_format, fig_dpi);

%% 8 — Diagram: Sled Unit Pose
fig3 = figure('Name', 'Sled Pose', 'Position', fig_position);
subplot(5,1,1);
plot(time, slu_x, 'b', 'LineWidth', 1.2);
ylabel('x [mm]'); grid on;
title('Sled Unit Pose');
ylim([min(slu_x) - max(slu_x)*0.1, max(slu_x)*1.1]);

subplot(5,1,2);
plot(time, slu_y, 'Color', '#A2142F', 'LineWidth', 1.2);
ylabel('y [mm]'); grid on;
ylim([min(slu_y) - max(slu_y)*0.1, max(slu_y)*1.1]);

subplot(5,1,3);
plot(time, slu_roll, 'm', 'LineWidth', 1.2);
ylabel('Roll [rad]'); grid on;
ylim([min(slu_roll) - max(slu_roll)*0.1, max(slu_roll)*1.1]);

subplot(5,1,4);
plot(time, slu_pitch, 'Color', '#0072BD', 'LineWidth', 1.2);
ylabel('Pitch [rad]'); grid on;
ylim([min(slu_pitch) - max(slu_pitch)*0.1, max(slu_pitch)*1.1]);

subplot(5,1,5);
plot(time, slu_yaw, 'Color', '#7E2F8E', 'LineWidth', 1.2);
ylabel('Yaw [rad]'); xlabel('Time [s]'); grid on;
ylim([min(slu_yaw) - max(slu_yaw)*0.1, max(slu_yaw)*1.1]);

sgtitle('Sled Unit Pose — Position and Angle Profiles');
print(fig3, fullfile(results_dir, sprintf('sled_unit_pose_%s', noise_tag)), fig_format, fig_dpi);

%% 9 — Save Results
results.time       = time;
results.Is_cmd     = Is_cmd;
results.I_act      = I_act;
results.meas_ag    = meas_ag;
results.pose       = [slu_x, slu_y, slu_roll, slu_pitch, slu_yaw];
results.velocity   = [slu_x_dot, slu_y_dot, slu_roll_dot, slu_pitch_dot, slu_yaw_dot];
results.labels     = elMaglabels;
results.nom_airgap = ssc_params.nom_airgap;
results.config     = struct('T_sim', T_sim, 'noise', noise_switch, ...
                            'observer', observer_method, ...
                            'timestamp', timestamp);

save(fullfile(results_dir, sprintf('simulation_results_%s_%s.mat', noise_tag)), 'results');

fprintf('\n=== Results saved in %s/ ===\n', results_dir);
fprintf('  Diagrams:   airgap_profiles_%s.png, current_profiles_%s.png, sled_unit_pose_%s.png\n', ...
        noise_tag, noise_tag, noise_tag);
fprintf('  Data:       simulation_results_%s_%s.mat\n', noise_tag);

%% 10 — Cleanup: Remove Simulink Cache Files
if exist('slprj', 'dir')
    rmdir('slprj', 's');
end
slxc = dir(fullfile(pwd, '*.slxc'));
if ~isempty(slxc)
    delete(slxc.name);
end
fprintf('Cache files successfully removed.\n');