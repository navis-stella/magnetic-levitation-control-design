%%#########################################################################
%
%  FILE:         Maglev_nmpc_solver_creator.m
%  PROJECT:      Model Predictive Control (MPC) for Single-Magnet Levitation
%  CREATED WITH: acados MATLAB Interface
%  DESCRIPTION:  This file configures the dynamic model, defines the
%                OCP (Optimal Control Problem), and generates the
%                S-Functions for integration into Simulink.
%  SYSTEM: Magnetic Levitation Controller (Single-Magnet)
%
%##########################################################################

% Set acados environment variables for Windows
acados_env_variables_windows()
import casadi.*
% Check Simulink options and initialize as empty if not found
if ~exist('simulink_opts', 'var')
    disp('simulink_opts not found – Solver will be generated without Simulink block.')
    simulink_opts = [];
end

% Check system requirements for acados
check_acados_requirements()
%---- Creation of the dynamic model and the MPC solver
%---- Classes used: AcadosModel(), AcadosOcp()

%% System Dynamics (Model Description)
% Define symbolic state variables
sym_x    = SX.sym('x',    [2,1]);  % State vector: [Air gap z; Velocity v]
sym_u    = SX.sym('u',    [1,1]);  % Control input: Coil current i_s
sym_p    = SX.sym('p',    [4,1]);  % Runtime parameters: [K; m; s0; g]
sym_xdot = SX.sym('xdot', [2,1]);  % Time derivative of states (for implicit form)

% Extract system parameters from the parameter vector
K  = sym_p(1);  % Magnetic force constant
m  = sym_p(2);  % Mass of the levitating magnet [kg]
s0 = sym_p(3);  % Nominal air gap (operating point) [m]
g  = sym_p(4);  % Gravitational acceleration [m/s²]

% Explicit system dynamics: dx/dt = f(x, u, p)
% Equation of motion: v̇ = (K/m) * i² / (s0 - z)² - g
f_expl_expr = [sym_x(2); ...
               (K/m) * (sym_u^2) / (s0 - sym_x(1))^2 - g];
% Implicit form of system dynamics: f(x, ẋ, u, p) = 0
f_impl_expr = f_expl_expr - sym_xdot;

% Populate the acados model object
dyn_model             = AcadosModel();
dyn_model.name        = 'singleMaglev';
dyn_model.x           = sym_x;
dyn_model.xdot        = sym_xdot;
dyn_model.u           = sym_u;
dyn_model.p           = sym_p;
dyn_model.f_expl_expr = f_expl_expr;
dyn_model.f_impl_expr = f_impl_expr;


%% OCP Formulation (Optimal Control Problem)
% Load MPC parameters from external configuration file
setup_nmpc_params;
Ts = nmpc.Ts;  % Sampling time [s]
N  = nmpc.N;   % Prediction horizon length [steps]

% Create OCP object
ocp = AcadosOcp();

% Weighting matrices for states and control input
W_x = diag([1000, 10]);  % State weighting matrix
W_u = 0.01;              % Control input weighting scalar

ocp.model = dyn_model;
nx = length(ocp.model.x);  % Number of state variables
nu = length(ocp.model.u);  % Number of control inputs

% --- Cost Function ---
% Initial cost term (Time step t = 0)
ocp.cost.cost_type_0          = 'NONLINEAR_LS';
ocp.cost.W_0                  = blkdiag(W_x, W_u);
ocp.cost.yref_0               = zeros(nx+nu, 1);
ocp.model.cost_y_expr_0       = vertcat(dyn_model.x, dyn_model.u);
% Path cost – Lagrange term (Time steps t = 1 … N-1)
ocp.cost.cost_type            = 'NONLINEAR_LS';
ocp.cost.W                    = blkdiag(W_x, W_u);
ocp.cost.yref                 = zeros(nx+nu, 1);
ocp.model.cost_y_expr         = vertcat(dyn_model.x, dyn_model.u);
% Terminal cost – Mayer term (Time step t = N)
ocp.cost.cost_type_e          = 'NONLINEAR_LS';
ocp.cost.W_e                  = W_x;
ocp.cost.yref_e               = zeros(nx, 1);
ocp.model.cost_y_expr_e       = dyn_model.x;

% --- Constraints ---
% Initial state: System starts at rest (no position and velocity error)
ocp.constraints.x0 = [0; 0];
% Control input constraint: Current i_s must be between I_min and I_max
ocp.constraints.lh_0          = I_min;   % Lower current bound (t = 0)
ocp.constraints.lh            = I_min;   % Lower current bound (path)
ocp.constraints.uh_0          = I_max;   % Upper current bound (t = 0)
ocp.constraints.uh            = I_max;   % Upper current bound (path)
ocp.model.con_h_expr_0        = dyn_model.u;
ocp.model.con_h_expr          = dyn_model.u;

% --- Solver Options ---
ocp.solver_options.tf                    = N * Ts;                     % Total time horizon [s]
ocp.solver_options.N_horizon             = N;                          % Number of prediction steps
ocp.solver_options.qp_solver             = 'PARTIAL_CONDENSING_HPIPM'; % QP solver (fast, suitable for real-time)
ocp.solver_options.nlp_solver_type       = 'SQP';                      % Nonlinear solver: Sequential Quadratic Programming
ocp.solver_options.nlp_solver_max_iter   = 50;                         % Maximum SQP iterations per sampling step
ocp.solver_options.hessian_approx        = 'GAUSS_NEWTON';             % Hessian approximation (efficient for LS costs)
ocp.solver_options.integrator_type       = 'ERK';                      % Explicit Runge-Kutta method
ocp.solver_options.sim_method_num_stages = 4;                          % RK stages (RK4)
ocp.solver_options.sim_method_num_steps  = 3;                          % Integration steps per interval
ocp.solver_options.print_level           = 0;                          % No console output from the solver
ocp.solver_options.ext_fun_compile_flags = '-O2';                      % Compiler optimization level
ocp.solver_options.qp_solver_mu0         = 1e3;                        % Initial barrier parameter for HPIPM
ocp.solver_options.qp_solver_cond_N      = 5;                          % Condensing steps for QP
ocp.solver_options.globalization         = 'MERIT_BACKTRACKING';       % Globalization via merit function with backtracking
ocp.solver_options.qp_solver_iter_max    = 1000;                       % Maximum QP iterations
ocp.solver_options.nlp_solver_tol_stat   = 1e-6;                       % Tolerance: Stationarity condition
ocp.solver_options.nlp_solver_tol_eq     = 1e-6;                       % Tolerance: Equality constraints
ocp.solver_options.nlp_solver_tol_ineq   = 1e-6;                       % Tolerance: Inequality constraints
ocp.solver_options.nlp_solver_tol_comp   = 1e-6;                       % Tolerance: Complementarity condition


%% Simulink Interface Configuration
simulink_opts = get_acados_simulink_opts();

% --- Input ports of the generated Simulink block ---
simulink_opts.inputs.parameter_traj = 1;  % Runtime parameters along the horizon
simulink_opts.inputs.y_ref          = 1;  % Reference for path costs
simulink_opts.inputs.y_ref_0        = 1;  % Reference for initial cost term
simulink_opts.inputs.y_ref_e        = 1;  % Reference for terminal costs
simulink_opts.inputs.x_init         = 1;  % Warm start: Initialization of the state trajectory
simulink_opts.inputs.cost_W_0       = 1;  % Weighting matrix for t = 0 (time-varying)
simulink_opts.inputs.cost_W         = 1;  % Weighting matrix for the path (time-varying)
simulink_opts.inputs.cost_W_e       = 1;  % Weighting matrix for the terminal state (time-varying)
simulink_opts.inputs.reset_solver   = 1;  % Solver reset if required (e.g., after controller switching)

% --- Output ports of the generated Simulink block ---
simulink_opts.outputs.utraj         = 1;  % Optimal control input trajectory
simulink_opts.outputs.xtraj         = 1;  % Predicted state trajectory
simulink_opts.outputs.cost_value    = 1;  % Total cost of the optimal solution
simulink_opts.outputs.KKT_residual  = 0;  % KKT residual (scalar value, disabled)
simulink_opts.outputs.KKT_residuals = 1;  % KKT residuals (vectorial, enabled)
simulink_opts.samplingtime   = '-1';  % Inherit sampling time from Simulink model
simulink_opts.show_port_info =  1;    % Show port labels in the generated block

ocp.simulink_opts = simulink_opts;

%% Acados OCP Solver: Initialization and Compilation
% Step 1: Remove all acados MEX functions from MATLAB memory
clear mex

% Step 2: Remove old code path from MATLAB search path (releases file locks)
if contains(path, 'c_generated_code')
    rmpath(genpath('c_generated_code'));
end

% Step 3: Safely delete the old generated code folder
if isfolder('c_generated_code')
    % Note: Lift write protection on Windows if necessary:
    % system('attrib -R "c_generated_code\*.*" /S /D');
    rmdir('c_generated_code', 's');
end

% Recreate acados solver and generate C code
nmpc_solver = AcadosOcpSolver(ocp);
disp('Acados solver has been successfully created.')


%% Compile S-Functions and generate Simulink block
if ~(exist('sim_flag', 'var') && sim_flag)
    cd c_generated_code
    make_sfun;      % Compile OCP solver as an S-Function
    make_sfun_sim;  % Compile integrator as an S-Function

    % Save port names for later use in Simulink
    
    save('sfun_port_names.mat', 'sfun_input_names', 'sfun_output_names');
    cd ..
    disp('✅ Acados solver and S-Functions successfully generated!');
end