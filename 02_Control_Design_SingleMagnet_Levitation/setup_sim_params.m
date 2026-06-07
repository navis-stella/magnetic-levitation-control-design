% #########################################################################
%
%  FILE      : setup_sim_params.m
%  PROJECT   : Single-Magnet Levitation System (Magnetic Levitation)
%  CONTENT   : Central initialization of all simulation parameters
%
%  PURPOSE   : This script defines all common parameters required by the
%              Simulink models of all controllers.
%              It is executed automatically when a model is opened, or
%              manually before running a simulation.
%
%  USAGE     : Either run directly, or called from design scripts via
%              run('../setup_sim_params.m')
%
%  CONTAINS  : - Dynamic plant parameters (mass, air gap)
%              - Sensor configuration (sampling time, noise)
%              - Control input limits (current saturation)
%              - External disturbance step (load simulation)
%              - Electromagnet parameters (from ElectromagnetConfig class)
%
%  DEPENDS ON: ElectromagnetConfig.m  — Electromagnet parameter class
%              00_Gemeinsame_Bibliothek\  — Shared utility functions
%
% #########################################################################
%
%  ╔══════════════════════════════════════════════════════════════════════╗
%  ║  IMPORTANT NOTE — CONTROLLER REDESIGN AFTER PARAMETER CHANGE         ║
%  ║                                                                      ║
%  ║  If  airgap_soll  or  mass  are changed, BOTH design scripts         ║
%  ║  must be re-run:                                                     ║
%  ║    >> design_linear_controller.m                                     ║
%  ║    >> design_nonlinear_controller.m                                  ║
%  ║    >> design_nmpc/Maglev_nmpc_solver_creator.m                       ║
%  ║    >> design_nmpc_offsetfree/MaglevAug_nmpc_solver_creator.m         ║
%  ║                                                                      ║
%  ║  Otherwise the controller parameters are not designed for the new    ║
%  ║  operating point → performance degradation or instability.           ║
%  ╚══════════════════════════════════════════════════════════════════════╝
%
% #########################################################################

% clc; clear; close all;

%% --- Path Configuration ---
% Add shared library (utility functions, classes) to the MATLAB path.
% Prerequisite: script is started from a subfolder of the project directory.
addpath("..\00_Shared_Library");


% #########################################################################
%% === Dynamic System Parameters ===
% #########################################################################
%  These parameters describe the physical operating point of the system.
%  They determine both the linearized plant model and the equilibrium
%  current calculation for all nonlinear controllers.

% --- Simulation duration ---
T_sim = 2;                           % Total simulation time                [s]

% --- Air gap configuration ---
% If the initial position is changed, "setup_nmpc_params" in the "design_nmpc_*" folder
% must be run again to create the initial state sequence of MPC solver .
airgap_init = 2.5;                   % Initial air gap (start position)     [mm]
airgap_soll = 2.0;                   % Target air gap / operating point     [mm]
airgap_eq   = airgap_soll * 1e-3;    % Target air gap in SI units           [m]

% --- Mechanical parameters ---
mass = 10;                           % Mass of the levitated body           [kg]
grav = 9.80665;                      % Gravitational acceleration (ISO 80000-3) [m/s²]


% #########################################################################
%% === Sensor Configuration (Noise) ===
% #########################################################################
%  Models the measurement noise of the position sensor (e.g. eddy-current sensor).
%  Used as band-limited white noise in Simulink.

Ts = 1e-4;                           % Sensor sampling time                [s]
                                     % (corresponds to 10 kHz sampling frequency)

isNoiseActive = 1;                   % Noise activation: 1 = active, 0 = disabled

% Noise standard deviation of the position sensor
sigma_n = 2e-3;                      % Sensor RMS noise ≈ 2 µm             [mm]

% Noise power density for the Simulink "Band-Limited White Noise" block:
%   NoisePower = sigma^2 / (1/Ts) = sigma^2 * Ts
%   Background: Simulink scales noise by 1/Ts internally, so that the
%   effective RMS amplitude remains independent of the sampling time.
NoisePower = sigma_n^2 * Ts;         % Noise power density                 [mm²·s]


% #########################################################################
%% === External Disturbance Step ===
% #########################################################################
%  Simulates a sudden load on the levitated body (e.g. placing a weight)
%  to test disturbance rejection of all controllers.
%
%  Time profile:
%    0  ≤ t < T_stp : no disturbance  (Ext_Init_Val = 0)
%    T_stp ≤ t ≤ T_sim: constant force (Ext_Fina_Val)
%
%  Magnitude: 20% of gravitational force  →  0.2 * m * g ≈ 19.6 N

T_stp        = 0.5 * T_sim;          % Time of step onset                  [s]
Ext_Init_Val = 0;                    % Disturbance force before step        [N]
Ext_Fina_Val = -0.2 * mass * grav;   % Disturbance force after step (20% mg) [N]


% #########################################################################
%% === Control Input Limits ===
% #########################################################################
%  Physical limits of the power amplifier (current driver).
%  These values are used as saturation bounds in all Simulink models.

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
fprintf('  Operating point : ag_eq = %.3f mm  |  Mass = %.1f kg\n', airgap_soll, mass);
fprintf('  Initial point   : ag_init = %.3f mm \n', airgap_init);
noiseStatus = ["disabled", "active"];
fprintf('  Noise           : %s  |  sigma_n = %.2g mm\n', ...
        noiseStatus(isNoiseActive + 1), sigma_n);
fprintf('  Disturbance step: %.1f N at t = %.2f s\n', Ext_Fina_Val, T_stp);
% fprintf('  Km           : %.4e N*m^2/A^2\n', Km);
fprintf('%s\n\n', repmat('-', 1, 80));