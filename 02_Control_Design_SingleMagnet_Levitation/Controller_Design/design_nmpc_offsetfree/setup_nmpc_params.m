%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% FILE:     setup_nmpc_params.m
% Project:  OffsetFreeNMPC-SingleMagnet-Levitation
% Module:   MPC Solver Parameter Configuration
% Description:
%           Initialization and configuration of the MPC parameters for the
%           Acados solver. The parameters are prepared and exported for use
%           as an S-Function in Simulink.
%
% See also: Controller_Design/design_nmpc/MaglevAug_nmpc_solver_creator.m
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Variable Check and Initialization
% Check simulation parameters; load them if not present.
if ~exist('mass', 'var') || ~exist('airgap_soll', 'var')
    run('../../setup_sim_params.m');
end

%% MPC Solver Configuration
% Ensure consistency with the solver creator
nmpc.Ts = 0.001;  % Sampling time (seconds)
nmpc.N  = 20;     % Prediction horizon (number of steps)
nmpc.nx = 3;      % Augmented state space dimension (nx)
nmpc.nu = 1;      % Control input dimension (nu)

% Definition of reference values and physical parameters (S-Function input)
i_eq   = airgap_eq * sqrt(mass * grav / Km); % Equilibrium current
p_val  = [Km; mass; airgap_eq; grav];        % Parameter vector

%% Weighting Matrices (Cost Function)
W_x = diag([10000, 100]);      % State weighting (e.g. position, velocity)
W_u = 0.01;                    % Control input weighting (energy penalty)

%% Constraints and Port Configuration
% Constraints and reference trajectories for the solver
nmpc.dyn_pval   = p_val;
nmpc.dyn_ptraj  = repmat(p_val, nmpc.N+1, 1);

% Reference values (y_ref) for initial step, horizon, and terminal state
% nmpc.y_ref_0    = [0; 0];
% nmpc.y_ref      = repmat([0; 0; i_eq], nmpc.N-1, 1);
% nmpc.y_ref_e    = [0; 0];

% Current bounds (constraints)
nmpc.lh         = repmat(I_min, nmpc.N-1, 1); % Lower bound
nmpc.uh         = repmat(I_max, nmpc.N-1, 1); % Upper bound
nmpc.lh_0       = I_min;
nmpc.uh_0       = I_max;

% Reshape weighting matrices for export (column vectors)
nmpc.cost_W_0 = reshape(W_x, [], 1);
nmpc.cost_W   = reshape(blkdiag(W_x, W_u), [], 1);
nmpc.cost_W_e = reshape(W_x, [], 1);

%% System State Initialization
nmpc.reset = 0;
% Initialize trajectory based on the air gap deviation
nmpc.x_init = repmat([(airgap_soll-airgap_init)*1e-3; 0; 0], nmpc.N+1, 1);
nmpc.x_init_con = [(airgap_soll-airgap_init)*1e-3; 0];

%% Data Storage
% Save MPC parameters to the "Controller_Params" folder
thisDir     = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));
paramDir    = fullfile(projectRoot, 'Controller_Params');

if ~exist(paramDir, 'dir')
    mkdir(paramDir);
end

savePath = fullfile(paramDir, 'OffsetFreeNMPCData.mat');
save(savePath, 'nmpc');
cd (paramDir)

fprintf('Success: OffsetFree NMPC parameters saved to the "Controller_Params" folder.\n');

if exist('genFlag','var')
    cd (thisDir)
end