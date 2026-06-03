% #########################################################################
%
%   FILE:       setup_dualMaglev_system.m
%   PROJECT:    Double Electromagnet Levitation System (Push-Pull Configuration)
%
%   FUNCTION:   Initialization, Kinematics for Simscape Multibody (MBS), 
%               Operating Point Calculation, and Linear Controller Design
%
%   NOTE:       This script prepares the workspace variables for the 
%               associated Simulink model.
%
% #########################################################################
clc; clear; close all;

% --- Add project paths ---
% Contains ElectromagnetConfig and utility functions
if exist("..\00_Shared_Library", "dir")
    addpath("..\00_Shared_Library");
end

%% 1. Global Simulation Parameters
T_sim    = 2;              % Simulation duration [s]
grav     = 9.80665;        % Gravitational acceleration [m/s^2]
SlideMass = 50;            % Total mass of the levitation unit [kg]

% --- System limits (Constraints) ---
I_min = 0;                 % Minimum coil current [A]
I_max = 15;                % Maximum coil current [A]

% --- External Load ---
T_step = 0.5 * T_sim;            % Step time [s]
FinVal = 0.5 * SlideMass * grav; % Final force value [N]

%% 2. Kinematic Parameters for Multibody Simulation (MBS)
% Definition of geometry and initial positions for Simscape Multibody.
% The Z-axis is defined as the vertical axis.

% --- Motion limits ---
PJ_max_route  = 4;      % Maximum travel path (Prismatic Joint) [mm]

% --- Initial air gap ---
AirGapUpper_init = 4;       % Upper side [mm]
AirGapLower_init = 0;       % Lower side [mm]

% --- Setpoints for operating point ---
% IMPORTANT: If the target air gap or Slide mass is changed, the controller 
% design must be re-executed to guarantee performance.
% See: "design_state_space_control.m"
AirGapUpper_soll = 2;  % Desired static air gap at the top [mm]
AirGapLower_soll = PJ_max_route - AirGapUpper_soll;

% --- Dimensions (Length, Width, Height) [mm] ---
elMagnetSize = ElectromagnetConfig.MagnetSize;
ArmatureSize = [300 250  80];                   % Armature (solid & homogeneous)
SlideUnitSize = [250 200 150];                  % Movable Slide unit

% --- Material properties ---
ArmaturDensity = 1000000;       % Density [kg/m^3]

%% 2.1 Coordinate Transformations (Frame Definitions)
% GS = Geometric Center, WD = World Coordinate System, PJ = Prismatic Joint
% WD-axes: z upwards, x to the right, y inwards

% Lower armature plate (fixed)
lwArmaGS2WD = [0, 0, ArmatureSize(3)/2];   

% Slide (movable) - Initial position
SdMGS2WD    = [0, 0, ArmatureSize(3) + elMagnetSize(3) + AirGapLower_init + SlideUnitSize(3)/2];

% Upper armature plate (fixed)
upArmaGS2WD = [0, 0, SdMGS2WD(3) + SlideUnitSize(3)/2 + elMagnetSize(3) + AirGapUpper_init + ArmatureSize(3)/2];

% --- Positions relative to the Prismatic Joint (PJ) ---
lwMagGS2PJ  = [0, 0, -(SlideUnitSize(3) + elMagnetSize(3))/2];
upMagGS2PJ  = [0, 0,  (SlideUnitSize(3) + elMagnetSize(3))/2]; 

% Initial state for the simulation
SlidePOS_init = SdMGS2WD(3);

%% 2.2 Contact Surface Reference Frames (Force Application Points)
% Convention:
%   Upper pair: UpMagSf Z → upwards (+Z_world), UpArmSf Z → downwards (into armature)
%   Lower pair: LwMagSf Z → downwards (-Z_world), LwArmSf Z → upwards (+Z_world, into armature)

% Rotation matrices:
R_identity = eye(3);            % World-aligned (no change)
R_x180     = diag([1, -1, -1]); % 180° around X: flips Z and Y axes

% --- Translation vectors (unchanged) ---
UpArmSfGS2UpArmGS = [0, 0, -ArmatureSize(3)/2];   % Underside of upper armature
LwArmSfGS2LwArmGS = [0, 0,  ArmatureSize(3)/2];   % Top side of lower armature
UpMagSfGS2UpMagGS = [0, 0,  elMagnetSize(3)/2];   % Top side of upper magnet
LwMagSfGS2LwMagGS = [0, 0, -elMagnetSize(3)/2];   % Underside of lower magnet

% --- Rotation matrices for respective surface frames ---
R_UpArmSf = R_identity;   % Z points down (already -Z_world via geometry)
R_LwArmSf = R_x180;       % Z-flip: now points UP (+Z_world = into the lower armature)
R_UpMagSf = R_identity;   % Z points up (+Z_world, outwards), unchanged
R_LwMagSf = R_x180;       % Z-flip: now points DOWN (-Z_world = out of the lower magnet)

% --- Euler angles [deg] for Simscape Multibody "Rigid Body Transform" blocks ---
% Sequence: ZYX (Simscape Default)
euler_identity = [0,   0, 0];   % deg
euler_x180     = [0, 180, 0];   % 180° around X (corresponds in ZYX: first [0,0,180] then [180,0,0])

% NOTE: For Simscape "Rotation Type: Standard Axis"
% Axis: [1 0 0], Angle: 180 deg  ← simplest parameterization

% --- Kinematic Mapping --- 
% Determination of unit Slide motion relative to the local air gap.
% Measurement in the local frame relative to the world frame.
% x_dev_up = airgap_eq_up - airgap_meas_up;
% x_dev_lw = airgap_eq_lw - airgap_meas_lw;
MotVec_up = [0, 0,  1];  % Top: angle 0°, upward motion decreases the gap
MotVec_lw = [0, 0, -1];  % Bottom: angle 180°, upward motion increases the gap
G = [MotVec_up; MotVec_lw]; 
kinMap = pinv(G);

% --- Safety check for air gap ---
% Prevents division by zero in force calculation.
% Note: The air gap must always be greater than 0 (simplified model 
% without leakage flux and saturation).
airgap_up_init = max(AirGapUpper_init, ElectromagnetConfig.minSafeAirGap);
airgap_lw_init = max(AirGapLower_init, ElectromagnetConfig.minSafeAirGap);

%% 3. Electromagnetic Constants
% Simplified reluctance model (without leakage flux and iron saturation).
% The magnetic force is calculated as: F_m = K_m * (i_s^2 / x_s^2)
mu_vacu = ElectromagnetConfig.mu_vacu;
A_eff   = ElectromagnetConfig.PoleArea_all;
N_coil  = ElectromagnetConfig.NumTurns; 
Km      = (mu_vacu * A_eff * N_coil^2) / 2; % Magnetic force constant

%% 4. Import Controller Parameters 
load("DualMagnetControllerData.mat");