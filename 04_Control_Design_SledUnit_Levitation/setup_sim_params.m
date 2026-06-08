%#########################################################################%
%#                                                                       #%
%#   SCRIPT: setup_sim_params.m                                          #%
%#                                                                       #%
%#   PURPOSE:                                                            #%
%#   Configuration of the simulation environment, definition of          #%
%#   disturbance inputs, and loading of controller and observer          #%
%#   parameters.                                                         #%
%#                                                                       #%
%#   PARAMETERS:                                                         #%
%#   - Simulation duration and sample rate                               #%
%#   - Noise modelling for eddy-current sensors                          #%
%#   - External disturbance force profiles                               #%
%#                                                                       #%
%#########################################################################%
% clc; clear; close all;

%% Simulation Configuration
% Simulation duration [s]
T_sim = 1;
% Sensor noise switch (0 -> off; 1 -> on)
% Noise specifications are defined in
% '.\Setup_Machine_Model\init_system_const.m'.
noise_switch = 1;

%% Disturbance Configuration
%-----------------------------------------------------------------
% Timestamps
T_dist = 0.5 * T_sim;   % Common disturbance onset time          [s]
% Amplitudes
Fx_amp = -1200;          % Disturbance force X                   [N]
Fy_amp = 800;            % Disturbance force Y                   [N]
Tz_amp = 1000;           % Disturbance torque Z                  [N·m]

%-----------------------------------------------------------------
% Format: [time, value] — duplicate timestamp → ideal step
%-----------------------------------------------------------------
% Fx — step at T_dist
ext_Fx = [0,        0;
          T_dist,   0;
          T_dist,   Fx_amp;
          T_sim,    Fx_amp];
% Fy — step at T_dist
ext_Fy = [0,        0;
          T_dist,   0;
          T_dist,   Fy_amp;
          T_sim,    Fy_amp];
% Tz — step at T_dist
ext_Tz = [0,        0;
          T_dist,   0;
          T_dist,   Tz_amp;
          T_sim,    Tz_amp];

%% Load Parameters and Constraints
% Load design data for controller and observer (Kalman filter).
% These files are located in the subfolder 'Controller_Params'.
load(".\Controller_Params\SSC_Params");
load(".\Controller_Params\Obs_Params");

% Load bus
load(".\Controller_Params\obs_params_Bus");

% ========================================================================
fprintf('✅ Simulation parameters loaded successfully.\n');