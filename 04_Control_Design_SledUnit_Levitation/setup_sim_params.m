%#########################################################################%
%#                                                                       #%
%#   SCRIPT: setup_sim_params.m                                          #%
%#                                                                       #%
%#   TASK:                                                               #%
%#   Configuration of the simulation environment, definition of          #%
%#   disturbances, and loading of controller and observer parameters.    #%
%#                                                                       #%
%#   PARAMETERS:                                                         #%
%#   - Simulation duration and sampling rate                              #%
%#   - Noise modeling for eddy current sensor technology                 #%
%#   - External disturbance force profiles                               #%
%#                                                                       #%
%#########################################################################%

% clc; clear; close all;

%% Simulation Configuration
% Total simulation duration [s]
T_sim = 2; 

% Toggle switch for sensor noise (0 -> OFF; 1 -> ON)
% Die Rauschspezifikationen wurden im Skript
% '.\Setup_Machine_Model\init_system_const.m' definiert.
noise_switch = 0; 

%% Disturbance Configuration
%-----------------------------------------------------------------
% Timestamps (Event Scheduling)
t_imp  = 0.3 * T_sim;   % Impulse event (Fy)              [s]
t_Fx   = 0.5 * T_sim;   % Force step in X-direction       [s]
t_Fy   = 0.6 * T_sim;   % Force step in Y-direction       [s]
t_Tz   = 0.7 * T_sim;   % Torque step around Z-axis       [s]
t_dur  = 0.02;          % Duration of the impulse         [s]

% Amplitudes
Fx_amp =  60;           % Step force X                    [N]
Fy_amp = -60;           % Step force Y                    [N]
Tz_amp =  200;          % Step torque Z                   [N·m]
Fy_imp = 3 * Fy_amp;    % Impulse amplitude (3x Step)     [N]

%-----------------------------------------------------------------
% Data Format: [Time, Value] 
% Note: Using double timestamps at the same time creates an ideal step.
%-----------------------------------------------------------------

% Fx — Step at t_Fx
ext_Fx = [0,           0;
          t_Fx,        0;
          t_Fx,        Fx_amp;
          T_sim,       Fx_amp];

% Fy — Impulse at t_imp, followed by a Step at t_Fy
ext_Fy = [0,           0;
          t_imp,        0;
          t_imp,        Fy_imp;    % Start impulse
          t_imp+t_dur,  Fy_imp;    % Hold impulse
          t_imp+t_dur,  0;         % End impulse
          t_Fy,         0;
          t_Fy,         Fy_amp;    % Start step
          T_sim,        Fy_amp];

% Tz — Torque step at t_Tz
ext_Tz = [0,           0;
          t_Tz,        0;
          t_Tz,        Tz_amp;
          T_sim,       Tz_amp];

%% Load Parameters and Constraints
% Loading design data for the controller and observer (Kalman Filter)
% These files are located in the 'Controller_Params' subfolder
load(".\Controller_Params\SSC_Params");
load(".\Controller_Params\Obs_Params");

% Load Signal Bus definitions
load(".\Controller_Params\obs_params_Bus");

% ========================================================================
fprintf('✅ Simulation parameters successfully loaded.\n');