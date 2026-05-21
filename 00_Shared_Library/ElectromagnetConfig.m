classdef ElectromagnetConfig < handle
% =========================================================================
%  ElectromagnetConfig  —  Physical constants for the E-shaped electromagnet
%
%  All properties are Constant — values are fixed for this specific magnet
%  and do not change between simulations or controller designs.
%
%  USAGE:
%    Km      = ElectromagnetConfig.MagConst;           
%    N       = ElectromagnetConfig.NumTurns;
%    ElectromagnetConfig.assign_to_Workspace();  % push all to base workspace
%    ElectromagnetConfig.assign_to_blocks(gcb);  % configure Simscape blocks
% =========================================================================

    % ---------------------------------------------------------------------
    %  COIL PARAMETERS
    % ---------------------------------------------------------------------
    properties (Constant)
        NumTurns = 100;       % Number of coil turns                 [-]
        Res_coil = 0.2;       % Coil DC resistance                   [Ω]
        Ind_coil = 100;       % Coil inductance                      [mH]
    end

    % ---------------------------------------------------------------------
    %  GEOMETRY — E-shaped iron core (3 poles: 1 middle + 2 outer)
    % ---------------------------------------------------------------------
    properties (Constant)
        % Pole face areas
        PoleArea_mid = 120e-3 * 60e-3;   % Middle pole face area           [m²]
        PoleArea_out = 120e-3 * 30e-3;   % One outer pole face area        [m²]
        PoleArea_all = 0.0144;           % Total effective pole area       [m²]
        %  = PoleArea_mid + 2*PoleArea_out = 7.2e-3 + 2*3.6e-3 = 0.0144 m²
        

        % Magnetic path lengths  (used by Simscape reluctance blocks)
        MagPath_core = 50 + 70 + 50;  % Core path: shared + branch  [mm]
        MagPath_arma = 70;            % Armature return path  [mm]
        
        % Overall dimensions  [mm]  — for documentation only, not used in calculations
        CoreSize   = [170, 120, 65];    % Magnet core  L × W × H            [mm]
        MagnetSize = [178, 170, 65];    % Full assembly L × W × H           [mm]
    end

    % ---------------------------------------------------------------------
    %  MATERIAL & MAGNETIC PROPERTIES
    % ---------------------------------------------------------------------
    properties (Constant)
        % Permeabilities
        mu_vacu = 4*pi*1e-7;   % Permeability of free space  µ0            [H/m]
        mu_core = 4000;        % Relative permeability — iron core          [-]
        mu_arma = 5000;        % Relative permeability — armature           [-]
        mu_air  = 1;           % Relative permeability — air gap            [-]

        % Contact model parameters (Simscape collision / end-stop)
        minSafeAirGap     = 1e-9;   % Minimum air gap before contact model  [m]
        Contact_stiffness = 1e5;    % Contact spring stiffness              [N/m]
        Contact_damping   = 5000;   % Contact damping coefficient           [N·s/m]
    end

    % ---------------------------------------------------------------------
    %  DERIVED CONSTANTS
    %  Computed from the properties above; placed here so the rest of the
    %  project can call ElectromagnetConfig.Km directly without repeating
    %  the derivation in every script.
    % ---------------------------------------------------------------------
    properties (Constant)
        % Magnetic force constant  Km  [N·m²/A²]
        %
        %  From the Maxwell reluctance force (air-gap force law):
        %    F = Km * i² / x²
        %
        %  Derivation:
        %    F  = (µ0 * A_eff * N²) / 2  *  (i / x)²
        %    Km = (µ0 * A_eff * N²) / 2
        %
        MagConst = (ElectromagnetConfig.mu_vacu * ...
                    ElectromagnetConfig.PoleArea_all * ...
                    ElectromagnetConfig.NumTurns^2) / 2;
    end

    % =====================================================================
    methods (Static)
    % =====================================================================

        % -----------------------------------------------------------------
        function assign_to_Workspace()
        % ASSIGN_TO_WORKSPACE  Push all parameters to the MATLAB base workspace.
        %
        %  Convenience method for Simulink PreLoadFcn callbacks.
        %  After calling this, all variables are available as plain workspace
        %  variables (e.g., N_coil, mu_vacu, Km, ...).
        % -----------------------------------------------------------------
            assignin('base', 'mu_air',    ElectromagnetConfig.mu_air);
            assignin('base', 'mu_arma',   ElectromagnetConfig.mu_arma);
            assignin('base', 'mu_core',   ElectromagnetConfig.mu_core);
            assignin('base', 'mu_vacu',   ElectromagnetConfig.mu_vacu);

            assignin('base', 'l_arma',    ElectromagnetConfig.MagPath_arma);
            assignin('base', 'l_core',    ElectromagnetConfig.MagPath_core);
            assignin('base', 'R_coil',    ElectromagnetConfig.Res_coil);
            assignin('base', 'N_coil',    ElectromagnetConfig.NumTurns);
            assignin('base', 'L_coil',    ElectromagnetConfig.Ind_coil);
            assignin('base', 'airgap_min',ElectromagnetConfig.minSafeAirGap);
            assignin('base', 'A_out',     ElectromagnetConfig.PoleArea_out);
            assignin('base', 'A_mid',     ElectromagnetConfig.PoleArea_mid);
            assignin('base', 'A_all',     ElectromagnetConfig.PoleArea_all);
            assignin('base', 'Km',        ElectromagnetConfig.MagConst);

            disp('ElectromagnetConfig: all parameters assigned to base workspace.');
        end

        % -----------------------------------------------------------------
        function assign_to_blocks(block_path)
        % ASSIGN_TO_BLOCKS  Write magnet parameters directly into Simscape blocks.
        %
        %  INPUT:
        %    block_path — Simulink path to the electromagnet subsystem.
        %                 Defaults to the currently selected block (gcb).
        %
        %  Useful when block parameters are not driven by workspace variables
        %  (e.g., during initial model setup or after block replacement).
        % -----------------------------------------------------------------
            if nargin < 1 || isempty(block_path)
                block_path = gcb;
            end

            try
                % Coil
                set_param([block_path '/Resistor'], ...
                    'R', num2str(ElectromagnetConfig.Res_coil));
                set_param([block_path '/Electromagnetic Converter'], ...
                    'Nw', num2str(ElectromagnetConfig.NumTurns));

                % Iron core reluctance
                set_param([block_path '/Magnetic Core'], ...
                    'CSA', num2str(ElectromagnetConfig.PoleArea_mid), ...
                    'g',   num2str(ElectromagnetConfig.MagPath_core), ...
                    'mur', num2str(ElectromagnetConfig.mu_core));

                % Armature reluctance
                set_param([block_path '/Armature'], ...
                    'CSA', num2str(ElectromagnetConfig.PoleArea_out), ...
                    'g',   num2str(ElectromagnetConfig.MagPath_arma), ...
                    'mur', num2str(ElectromagnetConfig.mu_arma));

                % Air-gap reluctance force actuator
                set_param([block_path '/Reluctance Force Actuator'], ...
                    'CSA',           num2str(ElectromagnetConfig.PoleArea_all), ...
                    'mur',           num2str(ElectromagnetConfig.mu_air), ...
                    'K_contact',     num2str(ElectromagnetConfig.Contact_stiffness), ...
                    'D_contact',     num2str(ElectromagnetConfig.Contact_damping), ...
                    'xmin',          num2str(ElectromagnetConfig.minSafeAirGap), ...
                    'K_contact_unit','N/m', ...
                    'D_contact_unit','N*s/m', ...
                    'xmin_unit',     'mm');

            catch err
                warning('ElectromagnetConfig.assign_to_blocks: %s', E.message);
            end
        end

    end % methods
end % classdef