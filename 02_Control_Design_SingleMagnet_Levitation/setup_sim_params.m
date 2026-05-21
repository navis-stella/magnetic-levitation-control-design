% #########################################################################
%
%  FILE      : setup_sim_params.m
%  PROJECT   : Single Magnet Levitation System (Magnetic Levitation)
%  CONTENT   : Central initialization of all simulation parameters
%
%  PURPOSE   : This script defines all common parameters required by the
%              Simulink models of all controllers.
%              It is executed automatically when opening a model or manually
%              before starting a simulation.
%
%  USAGE     : Can be executed directly or called from design scripts 
%              via: run('../setup_sim_params.m')
%
%  CONTAINS  : - Dynamic plant parameters (Mass, Air gap)
%              - Sensor configuration (Sampling time, Noise)
%              - Control effort constraints (Current limits)
%              - External disturbance step (Load simulation)
%              - Electromagnet parameters (from ElectromagnetConfig class)
%
%  DEPENDENT : ElectromagnetConfig.m  — Electromagnet parameter class
%              00_Shared_Library\  — Common utility functions
%
% #########################################################################
%
%  ╔══════════════════════════════════════════════════════════════════════╗
%  ║  IMPORTANT NOTE — CONTROLLER DESIGN AFTER PARAMETER CHANGE           ║
%  ║                                                                      ║
%  ║  If 'airgap_soll' or 'mass' are changed, BOTH design scripts         ║
%  ║  must be re-executed:                                                ║
%  ║    >> design_linear_controller.m                                     ║
%  ║    >> design_nonlinear_controller.m                                  ║
%  ║                                                                      ║
%  ║  Otherwise, the controller parameters will not be designed for the   ║
%  ║  new operating point → performance loss or instability.              ║
%  ╚══════════════════════════════════════════════════════════════════════╝
%
% #########################################################################
% clc; clear; close all;

%% --- Path Configuration ---
% Add common library (utility functions, classes) to the MATLAB path.
% Prerequisite: Script is started from a subfolder of the project directory.
addpath("..\00_Shared_Library");

% #########################################################################
%% === Dynamic System Parameters ===
% #########################################################################
%  These parameters describe the physical operating point of the system.
%  They determine both the linearized plant model and the equilibrium 
%  current calculation for all nonlinear controllers.

% --- Simulation Duration ---
T_sim = 1;                           % Total simulation time                [s]

% --- Air Gap Configuration ---
airgap_init = 2.5;                   % Initial air gap (start position)     [mm]
airgap_soll = 2.0;                   % Target air gap / Operating point     [mm]
airgap_eq   = airgap_soll * 1e-3;    % Target air gap in SI units           [m]

% --- Mechanical Parameters ---
mass = 10;                           % Mass of the suspended body           [kg]
grav = 9.80665;                      % Gravitational acceleration           [m/s²]

% #########################################################################
%% === Sensor Configuration (Noise) ===
% #########################################################################
%  Models the measurement noise of the position sensor (e.g., eddy current sensor).
%  Used as band-limited white noise in Simulink.

Ts = 1e-4;                           % Sensor sampling time                 [s]
                                     % (corresponds to 10 kHz sampling frequency)
isNoiseActive = 1;                   % Noise activation: 1 = active, 0 = disabled

% Noise standard deviation of the position sensor
sigma_n = 5e-3;                      % Sensor RMS noise ≈ 5 µm              [mm]

% Noise power spectral density for the Simulink block "Band-Limited White Noise":
%   NoisePower = sigma^2 / (1/Ts) = sigma^2 * Ts
%   Background: Simulink scales noise internally with 1/Ts so that
%   the effective RMS amplitude remains independent of the sampling time.
NoisePower = sigma_n^2 * Ts;         % Noise power density                  [mm²·s]

% #########################################################################
%% === External Disturbance Step ===
% #########################################################################
%  Simulates a sudden load on the suspended body (e.g., adding a weight) 
%  to test the disturbance rejection of all controllers.
%
%  Time profile:
%    0  ≤ t < T_stp : no disturbance (Ext_Init_Val = 0)
%    T_stp ≤ t ≤ T_sim: constant force (Ext_Fina_Val)
%
%  Magnitude: 20% of the weight force → 0.2 * m * g ≈ 19.6 N

T_stp        = 0.5 * T_sim;          % Time of step start                   [s]
Ext_Init_Val = 0;                    % Disturbance force before step        [N]
Ext_Fina_Val = 0.2 * mass * grav;    % Disturbance force after step         [N]

% #########################################################################
%% === Control Effort Constraints ===
% #########################################################################
%  Physical limits of the power amplifier (current controller).
%  These values are used as saturation in all Simulink models.

I_min = 0;                           % Minimum coil current (unidirectional) [A]
I_max = 15;                          % Maximum coil current                  [A]

% #########################################################################
%% === Electromagnet Parameters ===
% #########################################################################
%  Physical properties of the electromagnet are loaded from the central
%  configuration class 'ElectromagnetConfig'.

% Magnetic force constant Km           
Km = ElectromagnetConfig.MagConst;  % [N·m²/A²]

% #########################################################################
%% --- Completion Message ---
% #########################################################################
fprintf('\n%s\n', repmat('-', 1, 80));
fprintf('  Simulation parameters successfully loaded.\n');
fprintf('  Operating Point : x_eq = %.1f mm  |  Mass = %.1f kg\n', airgap_soll, mass);

statusNoise = ["disabled", "active"];
fprintf('  Noise           : %s  |  sigma_n = %.2g mm\n', ...
        statusNoise(isNoiseActive + 1), sigma_n);
fprintf('  Disturbance Step: %.1f N at t = %.2f s\n', Ext_Fina_Val, T_stp);
% fprintf('  Km              : %.4e N·m²/A²\n', Km);
fprintf('%s\n\n', repmat('-', 1, 80));