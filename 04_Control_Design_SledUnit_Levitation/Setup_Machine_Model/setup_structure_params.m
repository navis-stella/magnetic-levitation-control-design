%#########################################################################%
%#                                                                       #%
%#  Main Script:                                                         #%
%#     Structural parameters for creating a Multibody Model              #%
%#                                                                       #%
%#########################################################################%
%
%  Naming Convention of Coordinate Systems (CS)
%  -------------------------------------------
%  MDRCS     Machine Design Reference Coordinate System
%            Design reference of the machine bed – fixed base CS
%
%  GCRLW     Geometric Center – Rail Linear Way
%            Geometric center of the guide rails on the machine bed
%
%  SlGCS     Sled Unit Geometric Center System
%            Geometric center of the sled unit in the equilibrium state
%
%  MagSfGCS  Magnet Surface Geometric Center System
%            Geometric center of each electromagnet surface relative to SlGCS
%
%  Transformation Notation:  B2A  →  B relative to A
%  Fields:           .TV  Translation Vector [mm]
%                    .RA  Rotation Angles [rad], ZYX convention
%
%  Electromagnet Identification (8 Magnets):
%    E / S   End / Spindle   (along the Z-axis)
%    L / R   Left / Right    (along the Y-axis)
%    O / U   Top / Bottom    (along the X-axis) [German: Oben / Unten]

% Definition of magnet labels based on their mounting positions
elMaglabels = {'ELO', 'ELU', 'ERO', 'ERU', 'SLO', 'SLU', 'SRO', 'SRU'};

% Add path
thisDir     = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));   
LibaryPath  = fullfile(projectRoot,'00_Shared_Library');
addpath(LibaryPath);

% Load physical constants and limit values
init_system_const;

%% ========================================================================
%  Machine Bed (Stationary Components)
%  ========================================================================
% Geometric center of the guide rails (GCRLW) relative to MDRCS
GCRLW2MDRCS.TV = [0;  0; 1391];   % [mm]
GCRLW2MDRCS.RA = [0;  0;    0];   % [rad], ZYX convention

% ----------------------------------------------------------------------
% Reference coordinate systems of the armatures relative to GCRLW
% Translation vectors [mm]
% ----------------------------------------------------------------------
ArmaGCF2GCRLW.ELO.TV = [-145.46;  174.375;  300];
ArmaGCF2GCRLW.ELU.TV = [-145.46; -174.375;  300];
ArmaGCF2GCRLW.ERO.TV = [ 145.46;  174.375;  300];
ArmaGCF2GCRLW.ERU.TV = [ 145.46; -174.375;  300];
ArmaGCF2GCRLW.SLO.TV = [-145.46;  174.375; -300];
ArmaGCF2GCRLW.SLU.TV = [-145.46; -174.375; -300];
ArmaGCF2GCRLW.SRO.TV = [ 145.46;  174.375; -300];
ArmaGCF2GCRLW.SRU.TV = [ 145.46; -174.375; -300];

% Rotation angles [rad], ZYX convention
ArmaGCF2GCRLW.ELO.RA = deg2rad([ 135;  0;   0]);
ArmaGCF2GCRLW.ELU.RA = deg2rad([-135;  0; 180]);
ArmaGCF2GCRLW.ERO.RA = deg2rad([  45;  0; 180]);
ArmaGCF2GCRLW.ERU.RA = deg2rad([ -45;  0;   0]);
ArmaGCF2GCRLW.SLO.RA = deg2rad([ 135;  0;   0]);
ArmaGCF2GCRLW.SLU.RA = deg2rad([-135;  0; 180]);
ArmaGCF2GCRLW.SRO.RA = deg2rad([  45;  0; 180]);
ArmaGCF2GCRLW.SRU.RA = deg2rad([ -45;  0;   0]);

% ------------------------------------------------------------------------
% Local coordinate systems of the armature surfaces (ArmaSfCS)
% Goal: Align local axes with the magnet surface CS to uniquely 
% define the air gap axis (normal to the armature surface).
% Transformation from armature geometric center to surface center.
% Method: Offset along local standard Y-axis.
% ------------------------------------------------------------------------
ArmaSize = [600; 174; 40];          % Armature dimensions: Length, Width, Height [mm]
ArmaSfCS_Y_Off = ArmaSize(3)/2;     % Offset from armature center to surface in Y [mm]

% Rotation angles of the surface coordinate systems [rad], ZYX convention
% Each surface CS transforms from armature center of gravity to surface center
ArmaSfCS_RA.ELO = deg2rad([-90; -90; -180]);
ArmaSfCS_RA.ELU = deg2rad([-90;  90;    0]);
ArmaSfCS_RA.ERO = deg2rad([-90;  90;    0]);
ArmaSfCS_RA.ERU = deg2rad([-90; -90;  180]);
ArmaSfCS_RA.SLO = deg2rad([ 90;  90;  180]);
ArmaSfCS_RA.SLU = deg2rad([ 90; -90;    0]);
ArmaSfCS_RA.SRO = deg2rad([ 90; -90;    0]);
ArmaSfCS_RA.SRU = deg2rad([-90;  90;    0]);

%% ========================================================================
%  Sled Unit (Moving Components)
%  ========================================================================
% Sled Unit Reference Coordinate System (SURCS) relative to MDRCS
SURCS2MDRCS.TV = [0; 0; 0];   % [mm]
SURCS2MDRCS.RA = [0; 0; 0];   % [rad], ZYX convention

% Geometric center of the sled unit (SlGCS) relative to SURCS
% Equilibrium state: Sled unit rests aligned on the machine bed
SlGCS2SURCS.TV = [0; 0;  1591];   % [mm]
SlGCS2SURCS.RA = [0;  0;    0];   % [rad], ZYX convention

% Load kinematic and dynamic parameters of the sled unit
setup_sled_dyn_params;

% Structural parameters of the spindle tip
% The spindle tip serves as the application point for external forces/moments
SpdTip2SlGCS.TV = [0; 0; -1050];      % Translation vector from SlGCS center [mm]
SpdTip2SlGCS.RA = [0; 0; 0];          % Rotation angles [rad], ZYX convention


%% ========================================================================
%  Air Gap Calculation for all 8 Cartesian joints
%  =======================================================================
calculate_airgap_equilibrium_and_rest

%% ========================================================================
%  Initial State & Air Gap Configuration
%  ========================================================================
% Option 1: Equilibrium state (Levitating)
% CartJointInit  = CartJointInit_eq;
% SUInitPosAtt.TV = SlGCS2SURCS.TV;
% SUInitPosAtt.RA = SlGCS2SURCS.RA;

% Option 2: Resting state (Landed/Static)
CartJointInit  = CartJointInit_Rest;
SUInitPosAtt.TV = SlGCS2SURCS_rest.TV;
SUInitPosAtt.RA = SlGCS2SURCS_rest.RA;

% --- Calculation of initial pose and sled state ---
% Calculate relative displacement of the initial pose
% Generate rotation matrices from ZYX Euler angles
R_ref  = eul2rotm(SlGCS2SURCS.RA',   'ZYX');   % 3×3
R_init = eul2rotm(SUInitPosAtt.RA',  'ZYX');   % 3×3

% Relative rotation: expressed in the reference system
R_rel  = R_ref' * R_init;
RA_rel = rotm2eul(R_rel, 'XYZ')';              % [3×1] column

% Relative translation: transform initial translation back to reference system
TV_rel = R_ref' * (SUInitPosAtt.TV - SlGCS2SURCS.TV);  % [3×1]

% Correct relative pose (general, accounts for rotation)
SUInitPosAtt2SlGCS = [TV_rel; RA_rel];

% Create initial state vector for the controller
su_pos_tmp = SUInitPosAtt2SlGCS;
su_pos_tmp(3) = []; % Remove Z-component (vertical) for specific DOF control

% Combine position and zero-velocity vectors
sled_init_state = vertcat(su_pos_tmp(1:2)*1e-3,su_pos_tmp(3:5), zeros(numel(su_pos_tmp), 1));

% Initial state of the augmented observer
obs_init_state = vertcat(su_pos_tmp(1:2)*1e-3,su_pos_tmp(3:5), ...
                         zeros(numel(su_pos_tmp)*2, 1));


% --- Air Gap Safety Limits & Nominal Values ---
% Initial safety air gaps (ensures values do not drop below hardware threshold)
init_airgap_lw = max(CartJointInit.ELU(3), ElectromagnetConfig.minSafeAirGap);
init_airgap_up = max(CartJointInit.ELO(3), ElectromagnetConfig.minSafeAirGap);

% Calculate nominal air gap (mean of upper and lower air gaps)
nom_airgap = mean([CartJointInit_eq.ELU(3), CartJointInit_eq.ELO(3)]);

%% Display Results
fprintf('========================================================================\n');
fprintf('Initial air gap (bottom):   %0.4f mm \n', init_airgap_lw)
fprintf('Initial air gap (top):      %0.4f mm \n', init_airgap_up)
fprintf('Desired nominal air gap:    %0.4f mm \n', nom_airgap)
fprintf('========================================================================\n');