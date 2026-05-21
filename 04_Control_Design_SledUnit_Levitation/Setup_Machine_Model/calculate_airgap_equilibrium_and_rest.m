%#########################################################################%
%#                                                                       #%
%#   SUBSCRIPT: calculate_airgap_equilibrium_and_rest.m                  #%
%#                                                                       #%
%#   TASK:                                                               #%
%#   Calculation of air gap values for all 8 magnet units                #%
%#   based on kinematic and structural parameters.                       #%
%#                                                                       #%
%#   CALCULATED STATES:                                                  #%
%#   1. Equilibrium State (Levitated): Sled floats at nominal position.   #%
%#      Sled levitates at nominal height.                                #%
%#   2. Resting State: Sled rests mechanically on the guide rails        #%
%#      (bottom air gap = 0).                                            #%
%#                                                                       #%
%#########################################################################%

%% SECTION 1: Air gap calculation in Equilibrium State
fprintf('\n');
fprintf('========================================================================\n');
fprintf('  SECTION 1: AIR GAP CALCULATION (EQUILIBRIUM)\n');
fprintf('========================================================================\n');
fprintf('\n');

% Offset of SlGCS in GCRLW coordinate system
SlGCS2GCRLW_eq = SlGCS2SURCS.TV + SURCS2MDRCS.TV - GCRLW2MDRCS.TV;  % Expected: [0; 0; 200] mm

% Nominal target air gap
nominal_gap = 0.5;       % [mm]
gap_tolerance = 0.03 * nominal_gap;   % 3% tolerance [mm]

fprintf('Sled position (Equilibrium):\n');
fprintf('  SlGCS2SURCS.TV = [%.4f; %.4f; %.4f] mm\n', ...
        SlGCS2SURCS.TV(1), SlGCS2SURCS.TV(2), SlGCS2SURCS.TV(3));
fprintf('  SlGCS2GCRLW    = [%.4f; %.4f; %.4f] mm\n\n', ...
        SlGCS2GCRLW_eq(1), SlGCS2GCRLW_eq(2), SlGCS2GCRLW_eq(3));

fprintf('Expected nominal air gap: %.3f mm (Tolerance: ±%.3f mm)\n\n', ...
        nominal_gap, gap_tolerance);

fprintf('Cartesian joint values in EQUILIBRIUM:\n');
fprintf('-----------------------------------------------------------------\n');
fprintf('Magnet | Type    |    X [mm]  |    Y [mm]  |    Z [mm]  | Dev.[mm] \n');
fprintf('-----------------------------------------------------------------\n');

for i = 1:numel(elMaglabels)
    lbl = elMaglabels{i};
    
    % Position of armature surface (ArmaSfCS) in GCRLW
    R_ArmaRTF = eul2rotm(ArmaGCF2GCRLW.(lbl).RA', 'ZYX');
    ArmaSf_GCRLW = ArmaGCF2GCRLW.(lbl).TV + ArmaSfCS_Y_Off * R_ArmaRTF(:,2);
    
    % Position of magnet surface (MagSfGCS) in GCRLW
    MagSf_GCRLW = SlGCS2GCRLW_eq + MagSfGCS2SlGCS.(lbl).TV;
    
    % Air gap vector in GCRLW
    gap_GCRLW = ArmaSf_GCRLW - MagSf_GCRLW;
    
    % Transformation into the local MagSfGCS coordinate system
    R_MagSf = eul2rotm(MagSfGCS2SlGCS.(lbl).RA', 'ZYX');
    gap_local = R_MagSf' * gap_GCRLW;
    
    % Store result
    CartJointInit_eq.(lbl) = gap_local;
    
    % Deviation from target air gap (local Z-axis)
    gap_deviation = abs(gap_local(3) - nominal_gap);
    
    % Validation: Critical error on penetration (negative air gap)
    if gap_local(3) < 0
        error('CRITICAL: Negative air gap for %s: Z = %.4f mm', lbl, gap_local(3));
    end
    
    % Warning if tolerance is exceeded
    if gap_deviation > gap_tolerance
        warning('Air gap for %s deviates: Z = %.4f mm (Target: %.3f ± %.3f mm)', ...
                lbl, gap_local(3), nominal_gap, gap_tolerance);
    end
    
    % Magnet type for display
    if contains(lbl, 'U')
        mag_type = 'Bottom';
    else
        mag_type = 'Top   ';
    end
    
    % Tabular output
    fprintf('%-6s | %s | %10.4f | %10.4f | %10.4f | %.4f\n', ...
            lbl, mag_type, gap_local(1), gap_local(2), gap_local(3), gap_deviation);
end
fprintf('-----------------------------------------------------------------\n');
fprintf('\n');

%% SECTION 2: Calculation of initial position (Resting State)
fprintf('\n');
fprintf('========================================================================\n');
fprintf('  SECTION 2: CALCULATION OF INITIAL POSE (RESTING STATE)\n');
fprintf('========================================================================\n');
fprintf('\n');

fprintf('Boundary conditions:\n');
fprintf('  - Sled rests mechanically on the machine bed\n');
fprintf('  - All 4 bottom magnets (ELU, ERU, SLU, SRU) have an air gap = 0 mm\n');
fprintf('  - Sled descends purely vertically (-Y direction)\n\n');

% Calculation of the required vertical descent
% The bottom magnets have a normal component in Y of -cos(45°)
% To close the gap: vertical_descent = air_gap / cos(45°)
ref_magnet = 'ELU';
actual_eq_gap = CartJointInit_eq.(ref_magnet)(3);

% Vertical descent to set the bottom air gap exactly to 0 mm
vertical_drop_y = actual_eq_gap / cos(deg2rad(45));

fprintf('Calculation of vertical descent:\n');
fprintf('  Required gap closure: %.3f mm\n', actual_eq_gap);
fprintf('  Magnet inclination angle: 45°\n');
fprintf('  Vertical descent = %.3f / cos(45°) = %.6f mm\n\n', ...
        actual_eq_gap, vertical_drop_y);

% New SlGCS position in Resting State (shifted in -Y)
delta_SlGCS = [0; -vertical_drop_y; 0];  % [mm]
SlGCS2GCRLW_rest = SlGCS2GCRLW_eq + delta_SlGCS;
SlGCS2SURCS_rest.TV = SlGCS2GCRLW_rest + GCRLW2MDRCS.TV;
SlGCS2SURCS_rest.RA = [0; 0; 0];  % [rad]

fprintf('Sled position in RESTING STATE:\n');
fprintf('  SlGCS2SURCS.TV = [%.6f; %.6f; %.6f] mm\n', ...
        SlGCS2SURCS_rest.TV(1), SlGCS2SURCS_rest.TV(2), SlGCS2SURCS_rest.TV(3));
fprintf('  SlGCS2SURCS.RA = [%.6f; %.6f; %.6f] rad\n\n', ...
        SlGCS2SURCS_rest.RA(1), SlGCS2SURCS_rest.RA(2), SlGCS2SURCS_rest.RA(3));

fprintf('Comparison: Equilibrium vs. Resting State:\n');
fprintf('  Equilibrium: [%.4f; %.4f; %.4f] mm\n', ...
        SlGCS2SURCS.TV(1), SlGCS2SURCS.TV(2), SlGCS2SURCS.TV(3));
fprintf('  Resting:     [%.4f; %.4f; %.4f] mm\n', ...
        SlGCS2SURCS_rest.TV(1), SlGCS2SURCS_rest.TV(2), SlGCS2SURCS_rest.TV(3));
fprintf('  Difference:  [%.4f; %.4f; %.4f] mm\n\n', ...
        delta_SlGCS(1), delta_SlGCS(2), delta_SlGCS(3));

% Calculation of air gaps in Resting State
fprintf('Cartesian joint values in RESTING STATE:\n');
fprintf('---------------------------------------------------------------\n');
fprintf('Magnet | Type    |    X [mm]  |    Y [mm]  |    Z [mm]  | ΔZ\n');
fprintf('---------------------------------------------------------------\n');

for i = 1:numel(elMaglabels)
    lbl = elMaglabels{i};
    
    % Armature position (remains fixed on the machine bed)
    R_ArmaRTF = eul2rotm(ArmaGCF2GCRLW.(lbl).RA', 'ZYX');
    ArmaSf_GCRLW = ArmaGCF2GCRLW.(lbl).TV + ArmaSfCS_Y_Off * R_ArmaRTF(:,2);
    
    % New magnet position (shifted due to sled descent)
    MagSf_GCRLW = SlGCS2GCRLW_rest + MagSfGCS2SlGCS.(lbl).TV;
    
    % Air gap vector
    gap_GCRLW = ArmaSf_GCRLW - MagSf_GCRLW;
    
    % Transformation into the local coordinate system
    R_MagSf = eul2rotm(MagSfGCS2SlGCS.(lbl).RA', 'ZYX');
    gap_local = R_MagSf' * gap_GCRLW;
    
    % Store results
    CartJointInit_Rest.(lbl) = gap_local;
    
    % Change in air gap compared to equilibrium
    gap_change = gap_local(3) - CartJointInit_eq.(lbl)(3);
    
    if contains(lbl, 'U')
        mag_type = 'Bottom';
    else
        mag_type = 'Top   ';
    end
    
    fprintf('%-6s | %s | %10.4f | %10.4f | %10.4f | %+6.4f\n', ...
            lbl, mag_type, gap_local(1), gap_local(2), gap_local(3), gap_change);
end
fprintf('---------------------------------------------------------------\n');
fprintf('\n');