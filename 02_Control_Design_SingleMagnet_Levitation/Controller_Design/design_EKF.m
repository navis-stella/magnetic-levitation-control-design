%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%#                                                                       #%
%# FILE:         design_EKF.m                                            #%
%# PROJECT:      Single Magnet Levitation System                         #%
%# DESCRIPTION:  Design of an Extended Kalman Filter (EKF) for           #%
%#               estimating position and velocity.                       #%
%#                                                                       #%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% WORKFLOW:
%   1. Definition of plant parameters and operating point
%   2. Symbolic derivation of the nonlinear dynamics
%   3. Calculation of the Jacobian matrix & plausibility check
%   4. Determination of the sample time
%   5. Design of noise covariances (Q, R)
%   6. Initialization of state and covariance (x0, P0)
%   7. Packaging into the EKF structure
%   8. Creation of a Simulink.Bus object for the MATLAB Function Block
%   9. Saving data to Controller_Params/EKFData.mat
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc; clear; close all;

%% --- 0.0 Check of Base Parameters ---
% Check if physical parameters exist in the workspace.
if ~exist('mass', 'var') || ~exist('airgap_soll', 'var')
    fprintf('>> Warning: Base parameters not found. Running "setup_sim_params.m"...\n');
    run('../setup_sim_params.m'); 
end

%% --- 1. Plant Parameters & Operating Point ---
% Operating point (Nominal air gap x0) in [m]
x_gap = airgap_soll * 1e-3;          
% Equilibrium current (Magnetic force = Gravitational force)
i_eq = sqrt(mass * grav * x_gap^2 / Km);
fprintf('Equilibrium current i0 = %.4f A\n', i_eq);

%% --- 2. Symbolic Nonlinear Dynamics ---
% States: x = [x1; x2]  -> x1: Position deviation [m]
%                          x2: Velocity [m/s]
% Input:  u = i_s       -> Coil current [A]
% Output: y = x1        -> Noisy position measurement
%
% Continuous model:
% dx1/dt = x2
% dx2/dt = (Km/m) * u^2 / (x_gap - x1)^2 - g
syms x1 x2 u Ts_sym real
f_cont = [ x2;
           (Km/mass) * u^2 / (x_gap - x1)^2 - grav];

% Discretization using Euler Forward method:
% $$x_{k+1} = x_k + T_s \cdot f(x_k, u_k)$$
f_disc = [x1; x2] + Ts_sym * f_cont;

fprintf('\nDiscretized state equation f(x,u,Ts):\n');
disp(f_disc);

%% --- 3. Jacobian Matrix & Linearization Check ---
% Calculation of the Jacobian matrix (df/dx) for the EKF
F_sym = jacobian(f_disc, [x1; x2]);
fprintf('Symbolic Jacobian matrix F = df/dx:\n');
disp(F_sym);

% Evaluation at the operating point (x1=0, x2=0, u=i0)
F_eq = subs(F_sym, [x1, x2, u], [0, 0, i_eq]);
fprintf('F at operating point (with symbolic Ts):\n');
disp(F_eq);

% Comparison with continuous system matrix A for verification
A_cont_sym = jacobian(f_cont, [x1; x2]);
A_eq = double(subs(A_cont_sym, [x1, x2, u], [0, 0, i_eq]));
instabiler_pol = sqrt(A_eq(2,1));

fprintf('\nLinearized system matrix A (continuous):\n');   disp(A_eq);
fprintf('Unstable pole = %.2f rad/s', instabiler_pol);
fprintf('   (Expected: ~99 rad/s from sqrt(2g/x0))\n');

%% --- 4. Defining the Sampling Time ---
% Rule of thumb: Ts should be at least 10x faster than the fastest dynamics.
% 10 kHz (100 µs) provides sufficient margin for the control loop.
Ts = 1e-4;             

%% --- 5. Design of Noise Covariances (Q, R) ---
% R: Measurement noise variance – Based on sensor datasheet.
% Typical value for eddy current sensors: ~1 µm standard deviation.
sigma_y = sigma_n * 1e-3;   % Position sensor noise [m]
R = sigma_y^2;              % Measurement noise covariance [m^2]

% Q: Process noise covariance – "Tuning parameter" for model uncertainty.
% q_pos: Near zero, as the integration of velocity is exact.
% q_vel: Higher value due to model errors (Km nonlinearity, eddy currents).
q_pos = 1e-14;
q_vel = 1e-6;
Q = diag([q_pos, q_vel]);

fprintf('\nFilter Design Parameters:\n');
fprintf('  sigma_y     = %.2e m  (Sensor standard deviation)\n', sigma_y);
fprintf('  R           = %.2e m^2\n', R);
fprintf('  Q           = diag([%.1e, %.1e])\n', q_pos, q_vel);
fprintf('  Ratio q_vel / R = %.1e (Filter aggressiveness)\n', q_vel/R);

%% --- 6. Initial State and Covariance Estimation ---
% Starting state based on initial air gap
x_hat_0 = [(airgap_soll - airgap_init)*1e-3; 0];                    
% P0: Initial estimation uncertainty
P0      = diag([(1e-4)^2, (1e-2)^2]); % sigma_x1 = 0.1 mm, sigma_x2 = 1 cm/s

%% --- 7. Creation of the EKF Structure ---
ekf_params.Ts    = Ts;
ekf_params.Q     = Q;
ekf_params.R     = R;
ekf_params.P0    = P0;
ekf_params.x0    = x_hat_0;
ekf_params.u0    = i_eq;
ekf_params.H     = [1, 0];        % Measurement matrix (only position measurable)

% Plant parameters for the prediction step in the EKF block
ekf_params.Km    = Km;
ekf_params.m     = mass;
ekf_params.g     = grav;
ekf_params.x_gap = x_gap;

fprintf('\nEKF structure created successfully.\n');

%% --- 8. Creation of the Simulink.Bus Object ---
% The bus allows passing the entire structure to a 
% MATLAB Function Block via a single port.
clear elems
ekf_Bus = Simulink.Bus;
elemSpecs = {
%   Name        Dimension   DataType
    'Ts',       [1 1],      'double';
    'Q',        [2 2],      'double';
    'R',        [1 1],      'double';
    'P0',       [2 2],      'double';
    'x0',       [2 1],      'double';
    'u0',       [1 1],      'double';
    'H',        [1 2],      'double';
    'Km',       [1 1],      'double';
    'm',        [1 1],      'double';
    'g',        [1 1],      'double';
    'x_gap',    [1 1],      'double';
};

for k = 1:size(elemSpecs,1)
    elems            = Simulink.BusElement;
    elems.Name       = elemSpecs{k,1};
    elems.Dimensions = elemSpecs{k,2};
    elems.DataType   = elemSpecs{k,3};
    elems.Complexity = 'real';
    elems.SamplingMode = 'Sample based';
    ekf_Bus.Elements(k) = elems;
end
ekf_Bus.Description = 'Parameter bus for the magnet system EKF block';
fprintf('Simulink.Bus "ekf_Bus" with %d elements created.\n', numel(ekf_Bus.Elements));

%% --- 9. Saving Data (.mat) ---
thisDir     = fileparts(mfilename('fullpath'));
projectRoot = fileparts(thisDir);          
paramDir    = fullfile(projectRoot, 'Controller_Params');

if ~exist(paramDir, 'dir')
    mkdir(paramDir);
end

savePath = fullfile(paramDir, 'EKFData.mat');
save(savePath, 'ekf_params', 'ekf_Bus');
fprintf('Data successfully saved at %s\n', savePath);