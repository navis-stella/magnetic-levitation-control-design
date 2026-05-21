% #########################################################################
%
%  FILE      : design_linear_controller.m
%  PROJECT   : Single Magnet Levitation System (Magnetic Levitation)
%  CONTENT   : Linearization of the plant & design of linear controllers
%
%  CONTROLLERS: 1) PD Controller  — Method A: Stiffness & Damping (PD_SD)
%               2) PD Controller  — Method B: 2. Order Pole Placement (PD_PV)
%               3) PID Controller — 3. Order Pole Placement (PID)
%               4) State Controller — Pole Placement & LQR (SSC)
%               5) Kalman Filter  — Discrete State Observer (KF)
%
%  PLANT     : Single Magnet Levitation System (1 magnet, 1 mass)
%              Equilibrium point: Air gap x0 = airgap_soll [mm]
%
%  METHOD    : Symbolic linearization (Taylor, 1st order) around the
%              stationary operating point (x_eq, i_eq).
%              Linearized transfer behavior:
%                G(s) = ki / (m*s² - kx)   [unstable 2nd order system]
%
%  DEPENDENT : setup_sim_params.m        — System parameters (mass, Km, grav, ...)
%             
%  OUTPUT    : LinearControllerData.mat
%              Contains: Plant, PD_SD, PD_PV, PID, SSC, KF
%
% #########################################################################
%
%  SYSTEM EQUATION (nonlinear):
%    m * x_ddot = Km * i_s^2 / x_s^2  -  m * g
%
%  LINEARIZATION around the operating point (x_eq, i_eq):
%    m * x_ddot = kx * x  +  ki * i
%
%    kx     = 2 * Km * i_eq^2 / x_eq^3  =  2 * m*g / x_eq   > 0  (destabilizing)
%    ki = 2 * Km * i_eq  / x_eq^2                         > 0  (input gain)
%
%    State-Space Model (deviation coordinates):
%      A = [0       1   ]     B = [0      ]
%          [kx/m    0   ]         [ki/m]
%      C = [1  0],  D = 0
%
%  NOTE: The system has an unstable pole at s = +sqrt(kx/m) ≈ +99 rad/s.
%        All controllers must actively move this pole to the left half-plane.
%
% --- Kinematic Mapping ---
%  Calculates position deviation from the target air gap and actual air gap measurement.
%  Application: x_dev = kinMapping * [ag_eq; ag_meas]
%                     = 1*ag_eq + (-1)*ag_meas
%                     = ag_eq - ag_meas
%  Sign: x_dev > 0 → Mass too far down (air gap too large)
%        x_dev < 0 → Mass too far up (air gap too small)
%  Unit conversion occurs in the Simulink model ([mm] → [m]).
%
% #########################################################################

clc; clear; close all;

%% --- Workspace Check ---
% System parameters are loaded from setup_sim_params.m if not already 
% present in the workspace. Expected: mass, Km, grav, airgap_soll.
if ~exist('mass', 'var') || ~exist('airgap_soll', 'var')
    fprintf('>> Warning: Base parameters not found. Loading setup_sim_params.m ...\n');
    run('../setup_sim_params.m');
end


% #########################################################################
%% === STEP 1: Symbolic Derivation & Linearization ===
% #########################################################################

%% --- 1.1 Magnetic Force (symbolic) ---
%  The magnetic force on the suspended mass is:
%    Fm(xs, is) = Km * is^2 / xs^2
%  where:
%    xs = physical air gap     [m]
%    is = coil current         [A]
%    Km = magnetic constant    [N·m²/A²]
syms xs is
Fm = symfun(Km * is^2 / xs^2, [xs, is]);


%% --- 1.2 Calculation of Stationary Operating Point ---
%  In equilibrium, forces are balanced:
%    Fm(x_eq, i_eq) = m * g
%    => Km * i_eq^2 / x_eq^2 = m * g
%    => i_eq = sqrt(m * g * x_eq^2 / Km)
x_eq = airgap_soll * 1e-3;         % Target air gap: [mm] → [m]
syms i
i_sol = solve(Fm(x_eq, i) == mass * grav, i);
i_eq  = double(i_sol(i_sol > 0));   % Only positive current is physically meaningful
fprintf('>> INFO: Operating Point:  x_eq = %.2f mm  |  i_eq = %.4f A\n', x_eq*1e3, i_eq);


%% --- 1.3 Linearization of Magnetic Force around the Operating Point ---
%  1st order Taylor series around (x=0, i=0) in deviation coordinates:
%    xs = x_eq - x     (Air gap decreases when mass moves up)
%    is = i_eq + i
%
%  Linearized force:
%    Fm_lin = Fm(x_eq, i_eq)  +  kx * x  +  ki * i
%           = m*g             +  kx * x  +  ki * i
%
%  Equation of motion for deviation:
%    m * x_ddot = kx * x  +  ki * i
%
%  Partial derivatives (linearization gradients):
syms x i
Fm_xi  = Km * (i + i_eq)^2 / (x_eq - x)^2;
dFm_dx = diff(Fm_xi, x);            % Position-force gradient [N/m]
dFm_di = diff(Fm_xi, i);            % Current-force gradient  [N/A]
kx = double(subs(dFm_dx, [x, i], [0, 0]));  % > 0: destabilizing force gradient
ki = double(subs(dFm_di, [x, i], [0, 0]));  % > 0: control gain
% Analytical expressions (for verification):
%   kx = 2 * Km * i_eq^2 / x_eq^3 = 2 * m * g / x_eq
%   ki = 2 * Km * i_eq / x_eq^2


%% --- 1.4 System Matrices of Linearized Model ---
%  State vector: [x; v] = [Position deviation; Velocity]
%  Transfer function: G(s) = ki / (m*s^2 - kx)
A = [0,        1; ...
     kx/mass,  0];          % Unstable pole at s = +sqrt(kx/mass) ≈ +99 rad/s
B = [0; ki/mass];
C = [1, 0];
D = 0;


%% --- 1.5 Save Plant Parameters ---
Plant.kx    = kx;            % Position-force gradient [N/m]
Plant.ki    = ki;            % Current-force gradient  [N/A]
Plant.i_eq  = i_eq;          % Equilibrium current    [A]
Plant.ag_eq = airgap_soll;   % Target air gap         [mm]
Plant.mass  = mass;          % Mass                   [kg]
Plant.kinM  = [1, -1];       % Kinematic mapping matrix


% #########################################################################
%% === CONTROLLER 1 & 2: PD Controllers ===
% #########################################################################
%
%  BASIC STRUCTURE:
%    u(t) = Kp * x(t)  +  Kd * v(t)
%
%  Closed-Loop System (without I-component):
%    m * x_ddot = kx * x  +  ki * (Kp*x + Kd*x_dot)
%    => m * x_ddot - (kx + ki*Kp)*x - ki*Kd*x_dot = 0
%
%  Characteristic Polynomial (2nd order):
%    p(s) = m*s^2  -  ki*Kd*s  -  (kx + ki*Kp)  =  0
%  Normalized: s^2 + 2*zeta*wn*s + wn^2 = 0
% -------------------------------------------------------------------------

%% --- Method A: Stiffness & Damping (PD_SD) ---
% -------------------------------------------------------------------------
%  Design based on physical parameters:
%    c_stiff: Virtual spring stiffness (chosen as 5x gravity stiffness)
%             Since magnetic stiffness kx = 2*m*g/x_eq is already 
%             destabilizing, the controller must overcompensate for it.
%    d_dampf: Critical damping d = sqrt(m * c_stiff)
%             Ensures aperiodic settling without overshoot.
%
%  Feedback gains (from coefficient comparison with target polynomial):
%    Kp = -(c_stiff + kx) / ki
%    Kd = -d_dampf        / ki

c_stiff = 5 * (mass * grav / x_eq);  % Virtual spring stiffness    [N/m]
d_dampf = 2 * sqrt(mass * c_stiff);  % Critical damping constant   [N·s/m]
PD_SD.Kp = -(c_stiff + kx) / ki;     % Proportional gain           [A/m]
PD_SD.Kd = -d_dampf        / ki;     % Differential gain           [A·s/m]

% -------------------------------------------------------------------------
%% --- Method B: 2nd Order Pole Placement (PD_PV) ---
% -------------------------------------------------------------------------
%  Direct design by specifying natural frequency and damping ratio.
%  Target characteristic polynomial:
%    p(s) = s^2 + 2*zeta*wn*s + wn^2 = 0
%
%  Coefficient comparison with closed-loop polynomial yields:
%    Kp = -(m * wn^2 + kx) / ki
%    Kd = -(2 * m * zeta * wn) / ki
%
%  Chosen poles: wn = 120 rad/s, zeta = 0.7
%    => Poles at s = -140 ± 143j (damped oscillation)

omega_n_pd  = 120;        % Target natural frequency [rad/s]
zeta_pd     = 0.7;        % Target damping ratio     [-]
PD_PV.Kp    = -(mass * omega_n_pd^2 + kx)            / ki;  % [A/m]
PD_PV.Kd    = -(2 * mass * zeta_pd * omega_n_pd)     / ki;  % [A·s/m]


% #########################################################################
%% === CONTROLLER 3: PID Controller (3rd Order Pole Placement) ===
% #########################################################################
%
%  Extending PD controller with an integral component:
%    u(t) = Kp*x + Kd*x_dot + Ki * integral(x dt)
%
%  The I-component eliminates steady-state error and compensates for 
%  constant disturbances (e.g., weight force error due to parameter inaccuracy).
%
%  CLOSED-LOOP CHARACTERISTIC POLYNOMIAL (3rd order):
%    p(s) = (s^2 + 2*zeta*wn*s + wn^2) * (s + alpha)
%         = s^3  +  (2*zeta*wn + alpha)*s^2
%                +  (wn^2 + 2*zeta*wn*alpha)*s
%                +  alpha*wn^2
%
%  The additional real pole alpha (integrator dynamics) is chosen significantly 
%  slower than the main poles: alpha = 0.2 * omega_n.
%
%  Coefficient comparison with system polynomial yields:
%    Kp = -(m * poly_b + kx) / ki
%    Kd = -(m * poly_a)      / ki
%    Ki = -(m * poly_c)      / ki

zeta         = 0.7;                     % Damping ratio               [-]
omega_n      = 100;                     % Natural frequency           [rad/s]
alpha_pole   = 0.2 * omega_n;           % Real pole of the integrator [rad/s]

% Polynomial coefficients of desired characteristic
poly_a = 2 * zeta * omega_n + alpha_pole;            % Coefficient s^2
poly_b = omega_n^2 + 2 * zeta * omega_n * alpha_pole; % Coefficient s^1
poly_c = alpha_pole * omega_n^2;                     % Coefficient s^0

PID.Kp = -(mass * poly_b + kx) / ki;    % Proportional gain [A/m]
PID.Kd = -(mass * poly_a)      / ki;    % Derivative gain   [A·s/m]
PID.Ki = -(mass * poly_c)      / ki;    % Integral gain     [A/(m·s)]


% #########################################################################
%% === CONTROLLER 4: State-Space Controller (LQR & Pole Placement) ===
% #########################################################################
%
%  BASIC STRUCTURE:
%    u = -K_aug * z_aug
%    z_aug = [x; v; integral(x dt)]   (augmented state vector)
%
%  The integral state ensures steady-state accuracy and allows rejection 
%  of constant disturbances (Anti-Windup capable).
%
%  AUGMENTED SYSTEM MODEL:
%    z_aug_dot = A_aug * z_aug + B_aug * u
%
%    A_aug = [0,      1,   0]    (Integral state grows with x)
%            [kx/m,  0,    0]    (Acceleration from position error)
%            [1,     0,    0]    (Integrated position error)
%
%    B_aug = [0;  ki/m;  0]

A_aug = [0,         1,    0; ...
         kx/mass,   0,    0; ...
         1,         0,    0];
B_aug = [0;  ki/mass;  0];
C_aug = [1,  0,  0];

% -------------------------------------------------------------------------
%% --- Method 1: Pole Placement ---
% -------------------------------------------------------------------------
%  Chosen poles (similar dynamics as PID):
%    s = [-50, -80+80j, -80-80j]
%      s = -50      : slow integrator dynamics (stationary accuracy)
%      s = -80±80j  : damped pole pair, ζ = 1/√2 ≈ 0.71, ωn = 113 rad/s
s_poles = [-50, -80+80i, -80-80i];
K_aug   = place(A_aug, B_aug, s_poles);  % State feedback vector [1×3]

% -------------------------------------------------------------------------
%% --- Method 2: Linear-Quadratic Regulator (LQR) ---
% -------------------------------------------------------------------------
%  Optimal state controller by minimizing the cost function:
%    J = integral( z' * Q * z  +  u' * R * u ) dt
%
%  Weighting matrices Q and R (Tuning parameters):
%    Q = diag([q_pos, q_vel, q_int])
%      q_pos = 1e-6   — very low weighting on position error
%      q_vel = 10000  — high weighting on velocity (high damping)
%      q_int = 2      — moderate weighting on integral error
%    R = 2e-6          — low control effort penalty
%
%  Return values:
%    K_lqr : optimal feedback matrix
%    S_lqr : solution of Algebraic Riccati Equation
%    P_lqr : eigenvalues of optimal system dynamics

Q_lqr = diag([1e-6, 10000, 2]);          % State weighting        [3×3]
R_lqr = 2e-6;                            % Control weighting      [-]
[K_lqr, S_lqr, P_lqr] = lqr(A_aug, B_aug, Q_lqr, R_lqr);

% -------------------------------------------------------------------------
%  Save State Controller structures
% -------------------------------------------------------------------------
SSC.A_aug = A_aug;          % System matrix of augmented model [3×3]
SSC.K_aug = K_aug;          % Feedback matrix (Pole placement) [1×3]
SSC.Q_lqr = Q_lqr;          % LQR state weighting              [3×3]
SSC.R_lqr = R_lqr;          % LQR control weighting            [-]
SSC.K_lqr = K_lqr;          % Feedback matrix (LQR optimal)    [1×3]


% #########################################################################
%% === CONTROLLER 5: Kalman Filter (Discrete State Observer) ===
% #########################################################################
%
%  BASIC IDEA:
%  The Kalman Filter estimates the full state vector [x; v] from the 
%  noisy position measurement y = x + measurement noise.
%  It minimizes the error covariance and provides the optimal linear estimator.
%
%  DISCRETIZATION:
%  The continuous model is discretized using Zero-Order-Hold (ZOH) at Ts.
%  ZOH matches the physical behavior of a digital controller.
%
%  NOISE MODEL:
%    Process noise Q_k: Uncertainty in system model (forces, model errors)
%    Measurement noise R_k: Uncertainty in position measurement (sensor, quantization)
%
%  KALMAN GAIN (time-invariant for LTI system):
%    K_f = P * C' * (C * P * C' + R_k)^-1
%  where P is the steady-state solution of the Riccati equation.

KF.Ts = 1e-4;                              % Sampling time [s]
% Discretization of linearized plant model (ZOH)
sys_c  = ss(A, B, C, 0);                   % Continuous state-space model
sys_d  = c2d(sys_c, KF.Ts, 'zoh');         % Discretization (Zero-Order Hold)
KF.A_d = sys_d.A;                          % Discrete system matrix  [2×2]
KF.B_d = sys_d.B;                          % Discrete input matrix   [2×1]
KF.C_d = sys_d.C;                          % Output matrix (position) [1×2]

% Noise Covariance Matrices
%   Q_k: Process noise — diagonal, assuming position and velocity are independent.
%     q_x = 1e-6 [m^2]   : Low position uncertainty (good model)
%     q_v = 1e-3 [m^2/s^2]: Higher velocity uncertainty
%   R_k: Measurement noise — scalar variance of position measurement
%     r_x = 1e-10 [m^2]  : Very precise position measurement (Eddy current sensor)
KF.Q_k = diag([1e-6, 1e-3]);               % Process noise covariance [2×2]
KF.R_k = 1e-10;                            % Measurement noise variance [m^2]

% Initial error covariance
%   P0 expresses initial state uncertainty.
KF.P0  = diag([1e-3, 1e-1]);            % Initial error covariance [2×2]

% Initial state of the estimator
%   Initialization with known starting deviation (airgap_init → airgap_soll).
KF.init_x = [(airgap_soll - airgap_init) * 1e-3; 0];  % [x_init [m]; v_init [m/s]]


% #########################################################################
%% --- Saving Linear Controller Parameters ---
% #########################################################################
%  All structures are saved in a common .mat file for Simulink models.
%
%  File Content:
%    Plant — Plant parameters & linearization values (kx, ki, i_eq, ...)
%    PD_SD — PD controller via Stiffness & Damping
%    PD_PV — PD controller via 2nd Order Pole Placement
%    PID   — PID controller via 3rd Order Pole Placement
%    SSC   — State controller (Pole Placement & LQR, with integral state)
%    KF    — Discrete Kalman Filter (Ts, system matrices, noise parameters)

save('../Controller_Params/LinearControllerData.mat', ...
     'Plant', 'PD_SD', 'PD_PV', 'PID', 'SSC', 'KF');

% cd ..\01_Linear_Controller\

%% --- Completion Message ---
fprintf('\n%s\n', repmat('=', 1, 80));
fprintf('  Linear controller design completed successfully.\n');
fprintf('  Timestamp : %s\n', datetime('now', 'Format', 'dd.MM.yyyy HH:mm:ss'));
fprintf('  Output    : Controller_Params/LinearControllerData.mat\n');
fprintf('  Controllers: PD_SD | PD_PV | PID | SSC (LQR + Pole Placement) | KF\n');
fprintf('%s\n\n', repmat('=', 1, 80));