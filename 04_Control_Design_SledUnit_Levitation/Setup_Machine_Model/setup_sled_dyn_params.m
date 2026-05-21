%#########################################################################%
%#                                                                       #%                  
%#  Subscript:                                                           #%
%#      Kinematic and Dynamic Parameters of the Sled Unit                #%
%#                                                                       #%
%#########################################################################%
%
% Note: Definition of magnet labels based on their installation position
% elMaglabels = {'ELO', 'ELU', 'ERO', 'ERU', 'SLO', 'SLU', 'SRO', 'SRU'};

%% Kinematic Parameters of the Sled Unit
% The Geometric Center of the Sled (SlGCS) serves as the primary 
% dynamic reference coordinate system.

% Center of Mass (CoM) of the sled relative to SlGCS in equilibrium [mm]
CoM_TV2SlGCS = [0; 0; -65.31]; 

% Center of Mass (CoM) of the sled relative to SURCS (Surface Reference CS)
% DERIVED PARAMETER: CoM_TV2SURCS = SlGCS2SURCS.TV + CoM_TV2SlGCS
%                                 = [0; 0; 1591] + [0; 0; -65.31] 
%                                 = [0; 0; 1525.69]
% CoM_TV2SURCS = [0; 0; 1525.69];  % [mm]

% Inclination angles of the electromagnet surfaces (mounting angles) in radians
% Convention: α = 0° corresponds to a horizontal surface with normal vector nv = (0, 1, 0) along +Y.
% Normal vector formula: nv = [-sin(α), cos(α), 0]
MagSlantAngle.LO = deg2rad(-45);
MagSlantAngle.LU = deg2rad(-135);
MagSlantAngle.RO = deg2rad( 45);
MagSlantAngle.RU = deg2rad( 135);

% Normal vectors of the electromagnet surfaces relative to SlGCS
% Describes the direction of force application (reluctance force) for each magnet.
% Note: E-Side and S-Side share the same cross-section → identical normal vectors.
MagSfNVec.ELO = normal_vector(MagSlantAngle.LO);
MagSfNVec.ELU = normal_vector(MagSlantAngle.LU);
MagSfNVec.ERO = normal_vector(MagSlantAngle.RO);
MagSfNVec.ERU = normal_vector(MagSlantAngle.RU);
MagSfNVec.SLO = normal_vector(MagSlantAngle.LO);
MagSfNVec.SLU = normal_vector(MagSlantAngle.LU);
MagSfNVec.SRO = normal_vector(MagSlantAngle.RO);
MagSfNVec.SRU = normal_vector(MagSlantAngle.RU);

% Local Coordinate Systems of the Magnet Surfaces (MagSfGCS) relative to SlGCS
% Used to describe air gap deviations in the local coordinate frame.

% Translation Vectors: Geometric center of each magnet surface relative to SlGCS [mm]
% Note: |x| = 160.142 mm, |y| = 160.066 mm – slight asymmetry as per CAD model.
MagSfGCS2SlGCS.ELO.TV = [-160.142;  160.066;  336];
MagSfGCS2SlGCS.ELU.TV = [-160.142; -160.066;  336];
MagSfGCS2SlGCS.ERO.TV = [ 160.142;  160.066;  336];
MagSfGCS2SlGCS.ERU.TV = [ 160.142; -160.066;  336];
MagSfGCS2SlGCS.SLO.TV = [-160.142;  160.066; -336];
MagSfGCS2SlGCS.SLU.TV = [-160.142; -160.066; -336];
MagSfGCS2SlGCS.SRO.TV = [ 160.142;  160.066; -336];
MagSfGCS2SlGCS.SRU.TV = [ 160.142; -160.066; -336];

% Rotation angles of the Magnet Surface CS relative to SlGCS
% ZYX Convention: [psi, phi, theta] in radians
MagSfGCS2SlGCS.ELO.RA = [MagSlantAngle.LO; -pi/2; -pi/2];
MagSfGCS2SlGCS.ELU.RA = [MagSlantAngle.LU; -pi/2; -pi/2];
MagSfGCS2SlGCS.ERO.RA = [MagSlantAngle.RO; -pi/2; -pi/2];
MagSfGCS2SlGCS.ERU.RA = [MagSlantAngle.RU; -pi/2; -pi/2];
MagSfGCS2SlGCS.SLO.RA = [MagSlantAngle.LO;  pi/2; -pi/2];
MagSfGCS2SlGCS.SLU.RA = [MagSlantAngle.LU;  pi/2; -pi/2];
MagSfGCS2SlGCS.SRO.RA = [MagSlantAngle.RO;  pi/2; -pi/2];
MagSfGCS2SlGCS.SRU.RA = [MagSlantAngle.RU;  pi/2; -pi/2];

%% Dynamic Parameters of the Sled Unit
% Total mass of the simplified sled unit including electromagnets
SledUnitMass = 521.497;  % [kg]

% Moments of inertia relative to the Center of Mass (CoM) [kg·m²]
SledI_MC.Ixx = 66.551;   
SledI_MC.Iyy = 73.218;
SledI_MC.Izz = 14.429;
SledI_MC.Ixy =  0.000;
SledI_MC.Ixz =  0.000;
SledI_MC.Iyz =  0.031;

% Moments of inertia relative to the Geometric Center (SlGCS) [kg·m²]
% Calculated using the Steiner component (Parallel Axis Theorem / Huygens-Steiner Theorem):
% Formula: I_GC = I_MC + Mass * distance^2
SledI_GC.Ixx = 68.775;   
SledI_GC.Iyy = 75.442;
SledI_GC.Izz = 14.429;
SledI_GC.Ixy =  0.000;
SledI_GC.Ixz =  0.000;
SledI_GC.Iyz =  0.031;

% Configuration for the Multibody model in Simscape Multibody
InerMt_MC = [SledI_MC.Ixx, SledI_MC.Iyy, SledI_MC.Izz];  % Principal moments of inertia
InerPt_MC = [SledI_MC.Ixy, SledI_MC.Ixz, SledI_MC.Iyz];  % Products of inertia / Deviation moments

% Inertia tensors for the dynamic state-space formulation
% MC: Mass Center | GC: Geometric Center
InertiaTensor_MC = [SledI_MC.Ixx, SledI_MC.Ixy, SledI_MC.Ixz; ...
                    SledI_MC.Ixy, SledI_MC.Iyy, SledI_MC.Iyz; ...
                    SledI_MC.Ixz, SledI_MC.Iyz, SledI_MC.Izz];

InertiaTensor_GC = [SledI_GC.Ixx, SledI_GC.Ixy, SledI_GC.Ixz; ...
                    SledI_GC.Ixy, SledI_GC.Iyy, SledI_GC.Iyz; ...
                    SledI_GC.Ixz, SledI_GC.Iyz, SledI_GC.Izz];

%% Helper Function
function nv = normal_vector(alpha)
    % Calculates the normal vector for an inclined electromagnet surface
    % alpha: Inclination angle [rad]
    % Returns: Unit normal vector [3x1]
    nv = [-sin(alpha); cos(alpha);  0];    
end