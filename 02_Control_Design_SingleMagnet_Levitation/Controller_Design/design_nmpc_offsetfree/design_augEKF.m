%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%#                                                                       #%
%# FILE:         design_augEKF.m                                         #%
%# PROJECT:      Single-Magnet Levitation System — Offset-Free MPC       #%
%# DESCRIPTION:  Design of an Extended Kalman Filter (EKF) with          #%
%#               augmented state for estimating position,                #%
%#               velocity, and integrating disturbance d.               #%
%#                                                                       #%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% AUGMENTED STATE VECTOR:
%   x = [x1; x2; d]
%     x1 : Position deviation from operating point   [m]
%     x2 : Velocity                                  [m/s]
%     d  : Integrating acceleration disturbance      [m/s²]
%
% WORKFLOW:
%   1. Plant parameters & operating point
%   2. Symbolic derivation (3 states, d in x2 equation)
%   3. Jacobian matrix & observability check
%   4. Sampling time
%   5. Noise covariances (Q now 3x3, R unchanged)
%   6. Initialization
%   7. EKF structure
%   8. Simulink.Bus
%   9. Save as augEKFData.mat
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc;

%% --- 0. Check Base Parameters ---
if ~exist('mass', 'var') || ~exist('airgap_soll', 'var')
    fprintf('>> Warning: Base parameters not found. Running "setup_sim_params.m"...\n');
    run('../../setup_sim_params.m'); 
end

%% --- 1. Plant Parameters & Operating Point ---
x_eq = airgap_soll * 1e-3;          % Nominal air gap [m]

% Equilibrium current (magnetic force = gravitational force, d = 0)
i_eq = sqrt(mass * grav * x_eq^2 / Km);
fprintf('Equilibrium current i_eq = %.4f A\n', i_eq);

%% --- 2. Symbolic Nonlinear Dynamics (Augmented) ---
% States:  x = [x1; x2; d]
% Input:   u = i_s
% Output:  y = x1
%
% Continuous-time augmented model:
%   dx1/dt = x2
%   dx2/dt = (Km/m) * u^2 / (x_gap - x1)^2 - g + d
%   dd/dt  = 0                                       ← integrating disturbance
syms x1 x2 d u Ts_sym real
f_cont = [ x2;
           (Km/mass) * u^2 / (x_eq - x1)^2 - grav + d;
           0 ];

% Discretization using forward Euler method
f_disc = [x1; x2; d] + Ts_sym * f_cont;

fprintf('\nDiscretized augmented state equation f(x,u,Ts):\n');
disp(f_disc);

%% --- 3. Jacobian Matrix & Observability Check ---
% Jacobian matrix F = df/dx (3x3)
F_sym = jacobian(f_disc, [x1; x2; d]);
fprintf('Symbolic Jacobian matrix F = df/dx (3x3):\n');
disp(F_sym);

% Evaluation at operating point (x1=0, x2=0, d=0, u=i_eq)
F_eq = subs(F_sym, [x1, x2, d, u], [0, 0, 0, i_eq]);
fprintf('F at operating point (with symbolic Ts):\n');
disp(F_eq);

% Comparison with continuous system matrix A_aug
A_cont_sym = jacobian(f_cont, [x1; x2; d]);
A_eq = double(subs(A_cont_sym, [x1, x2, d, u], [0, 0, 0, i_eq]));
instabiler_pol = sqrt(A_eq(2,1));

fprintf('\nLinearized augmented system matrix A (continuous-time):\n');
disp(A_eq);
fprintf('Unstable pole = %.2f rad/s', instabiler_pol);
fprintf('   (Expected: ~99 rad/s from sqrt(2g/x0))\n');

% --- Observability Check of the AUGMENTED System ---
% Only position is measured: H = [1, 0, 0]
% Observability of d from y is CRITICAL for offset-free MPC.
H_obs = [1, 0, 0];
Obs_mat = [H_obs;
           H_obs * A_eq;
           H_obs * A_eq^2];
obs_rank = rank(Obs_mat);

fprintf('\nObservability matrix O = [H; H*A; H*A^2]:\n');
disp(Obs_mat);
fprintf('rank(O) = %d / 3  →  ', obs_rank);
if obs_rank == 3
    fprintf('Fully observable ✓ (d is reconstructable from y)\n');
else
    error('Augmented system NOT observable! EKF design not possible.');
end

%% --- 4. Sampling Time ---
Ts = 1e-4;             % 10 kHz — same as in the 2-state EKF

%% --- 5. Noise Covariance Design (Q, R) ---
% R: unchanged — sensor noise is a physical quantity
sigma_y = sigma_n * 1e-3;   
R = sigma_y^2;              

% Q: now 3x3 (new component q_d for disturbance state)
% q_pos: position — very small (integration from velocity is exact)
% q_vel: velocity — moderate (model error)
% q_d  : disturbance — PRIMARY TUNING KNOB for offset elimination
%        ↑ larger  → d_hat reacts faster, more noise
%        ↓ smaller → d_hat smoother, slower convergence
%        Rule of thumb: q_d / R ≈ 100 ... 10000
q_pos = 1e-16;
q_vel = 1e-8;
q_d   = 1e-2;               % Tuning parameter for disturbance estimation speed
Q = diag([q_pos, q_vel, q_d]);

fprintf('\nFilter Design Parameters:\n');
fprintf('  sigma_y       = %.2e m\n', sigma_y);
fprintf('  R             = %.2e m^2\n', R);
fprintf('  Q             = diag([%.1e, %.1e, %.1e])\n', q_pos, q_vel, q_d);
fprintf('  Ratio q_d / R = %.1e  (rule of thumb: 1e2 ... 1e4)\n', q_d/R);

%% --- 6. Initial State and Covariance Estimate ---
% Augmented initial state: [position; velocity; disturbance]
x_hat_0 = [(airgap_soll - airgap_init)*1e-3;   % from initial condition
           0;                                  % at rest
           0];                                 % no initial disturbance

% P0 for 3 states
P0 = diag([(1e-4)^2, ...   % sigma_x1 = 0.1 mm
           (1e-2)^2, ...   % sigma_x2 = 1 cm/s
           (1e-1)^2 ]);    % sigma_d  = 0.1 m/s² (generous — d is unknown)

%% --- 7. Creation of the augEKF Structure ---
augekf_params.Ts    = Ts;
augekf_params.Q     = Q;
augekf_params.R     = R;
augekf_params.P0    = P0;
augekf_params.x0    = x_hat_0;
augekf_params.u0    = i_eq;
augekf_params.H     = [1, 0, 0];        % Measurement matrix (position only)

% Plant parameters for prediction
augekf_params.Km    = Km;
augekf_params.m     = mass;
augekf_params.g     = grav;
augekf_params.s0    = x_eq;

fprintf('\nAugmented EKF structure successfully created.\n');

%% --- 8. Creation of the Simulink.Bus Object ---
clear elems
augekf_Bus = Simulink.Bus;
elemSpecs = {
%   Name        Dimension   Data type
    'Ts',       [1 1],      'double';
    'Q',        [3 3],      'double';     % 3x3 instead of 2x2
    'R',        [1 1],      'double';
    'P0',       [3 3],      'double';     % 3x3
    'x0',       [3 1],      'double';     % 3x1
    'u0',       [1 1],      'double';
    'H',        [1 3],      'double';     % 1x3
    'Km',       [1 1],      'double';
    'm',        [1 1],      'double';
    'g',        [1 1],      'double';
    's0',       [1 1],      'double';
};

for k = 1:size(elemSpecs,1)
    elems            = Simulink.BusElement;
    elems.Name       = elemSpecs{k,1};
    elems.Dimensions = elemSpecs{k,2};
    elems.DataType   = elemSpecs{k,3};
    elems.Complexity = 'real';
    elems.SamplingMode = 'Sample based';
    augekf_Bus.Elements(k) = elems;
end

augekf_Bus.Description = 'Parameter bus for the augmented EKF (offset-free)';

fprintf('Simulink.Bus "augekf_Bus" created with %d elements.\n', size(elemSpecs,1));

%% --- 9. Save Data (.mat) ---
thisDir     = fileparts(mfilename('fullpath'));
projectRoot = fileparts(fileparts(thisDir));          
paramDir    = fullfile(projectRoot, 'Controller_Params');

if ~exist(paramDir, 'dir')
    mkdir(paramDir);
end

savePath = fullfile(paramDir, 'augEKFData.mat');
save(savePath, 'augekf_params', 'augekf_Bus');
cd (paramDir)

fprintf('Data successfully saved to %s.\n', savePath);