%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FILE:     setup_nmpc_params.m  
% Project:  Single-Magnet Levitation
% Module:   Configuration of MPC Solver Parameters
% Description: 
%           Initialization and configuration of MPC parameters for the 
%           Acados solver. The parameters are prepared and exported for 
%           use as an S-Function in Simulink.
%
% See also: Controller_Design/design_nmpc/Maglev_nmpc_solver_creator.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Variable Check and Initialization
% Check simulation parameters; if they do not exist, load them.
if ~exist('mass', 'var') || ~exist('airgap_soll', 'var')
    run('../../setup_sim_params.m'); 
end
%% Configuration of the MPC Solver
% Ensure consistency with the Solver Creator
nmpc.Ts = 0.005;  % Sampling time (seconds)
nmpc.N  = 20;     % Prediction horizon (number of steps)
nmpc.nx = 2;      % Dimension of state space (nx)
nmpc.nu = 1;      % Dimension of control input (nu)

% Definition of reference values and physical parameters (S-Function input)
i_eq   = airgap_eq * sqrt(mass * grav / Km); % Equilibrium current
p_val  = [Km; mass; airgap_eq; grav];        % Parameter vector

%% Weighting Matrices (Cost Function)
W_x = diag([1000, 10]);      % State weighting (e.g., position, velocity)
W_u = 0.01;                  % Control input weighting (energy consumption)

%% Constraints and Port Configuration
% Constraints and reference trajectories for the solver
nmpc.dyn_pval   = p_val;
nmpc.dyn_ptraj  = repmat(p_val, nmpc.N+1, 1);

% Reference values (y_ref) for initial step, horizon, and terminal state
nmpc.y_ref_0    = [0; 0];
nmpc.y_ref      = repmat([0; 0; i_eq], nmpc.N-1, 1);
nmpc.y_ref_e    = [0; 0];

% Current limits (Constraints)
nmpc.lh         = repmat(I_min, nmpc.N-1, 1); % Lower bound
nmpc.uh         = repmat(I_max, nmpc.N-1, 1); % Upper bound
nmpc.lh_0       = I_min;
nmpc.uh_0       = I_max;

% Reshaping weighting matrices for export (column vectors)
nmpc.cost_W_0 = reshape(W_x, [], 1);
nmpc.cost_W   = reshape(blkdiag(W_x, W_u), [], 1);
nmpc.cost_W_e = reshape(W_x, [], 1);

%% Initialization of System State
nmpc.reset = 0;
% Trajectory initialization based on the air gap difference
nmpc.x_init = repmat([(airgap_soll-airgap_init)*1e-3; 0], nmpc.N+1, 1);
nmpc.x_init_con = [(airgap_soll-airgap_init)*1e-3; 0];

%% Data Storage
% Save MPC parameters in the "Controller_Params" folder
thisDir     = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));          
paramDir    = fullfile(projectRoot, 'Controller_Params');
    
if ~exist(paramDir, 'dir')
    mkdir(paramDir);
end
    
savePath = fullfile(paramDir, 'StandardNMPCData.mat');
save(savePath, 'nmpc');
cd (paramDir)

fprintf('Success: The MPC parameters have been saved in the "Controller_Params" folder.\n');

if exist('genFlag','var')
    cd (thisDir)
end