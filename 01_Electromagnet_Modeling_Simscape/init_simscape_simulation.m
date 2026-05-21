% =========================================================================
% DESCRIPTION: Initialization of the simulation environment and parameters
% FUNCTION:    Test model configuration
% =========================================================================
clc; clear; close all;

% --- Path Configuration ---
% Adding referenced subsystems and auxiliary functions
addpath("..\00_Shared_Library\");

% --- Dynamic System Parameters ---
airgap_init = 2.5;    % Initial air gap [mm]
airgap_goal = 2;      % Operating point / target air gap [mm]
Mass = 10;            % Mass of the suspended body [kg]
grav = 9.81;          % Gravitational field / acceleration [m/s^2]

% --- Simulation Parameters ---
T_sim = 0.2;          % Total simulation duration [s]

% --- Constraints ---
I_min = 0;            % Minimum current [A]
I_max = 15;           % Maximum current [A]

% --- Load External Parameters ---
% Importing specific parameters for the electromagnet
ElectromagnetConfig.assign_to_Workspace();

fprintf('Initialization complete. System is ready for simulation.\n');