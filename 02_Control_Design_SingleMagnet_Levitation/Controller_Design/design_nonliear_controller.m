% #########################################################################
%
%  FILE      : design_nonlinear_controller.m
%  PROJECT   : Single Magnet Levitation System (Magnetic Levitation)
%  CONTENT   : Design and parameterization of nonlinear controllers
%
%  CONTROLLERS: 1) Feedback Linearization + PID/State-Space Control
%               2) Integrator Backstepping with Integral Component
%               3) Sliding Mode Control (SMC) with Integral Sliding Surface
%
%  PLANT     : Single Magnet Levitation System (1 Magnet, 1 Mass)
%              Equilibrium point: Air gap x0 = airgap_soll [mm]
%              Simscape network model as simulation environment
%
%  DEPENDENT : setup_sim_params.m        — System parameters (mass, Km, grav, ...)
%
%  OUTPUT    : NonlinearControllerData.mat
%              Contains: Plant, fbklin, bksp, smc, smc_Bus
%
% #########################################################################
%
%  SYSTEM EQUATION:
%    m * x_ddot = Km * (i_s^2 / x_s^2) - m * g
%
%  STATE-SPACE REPRESENTATION (affine form):
%    State vector:  x = [x1; x2]
%      x1 = Position deviation from equilibrium  [m]
%      x2 = Velocity                             [m/s]
%    Virtual input:
%      u  = i_s^2                                [A^2]
%    Physical air gap:
%      x_s = x0 - x1                             [m]
%
%    Dynamics:  x_dot = f(x) + g(x) * u
%      f(x) = [x2;  -grav]
%      g(x) = [0;   Km / (mass * (x0 - x1)^2)]
%
% --- Kinematic Mapping ---
%  Calculates the position deviation from target air gap and actual measurement.
%  Application:  x_dev = kinMapping * [ag_eq; ag_meas]
%                    = 1*ag_eq + (-1)*ag_meas
%                    = ag_eq - ag_meas
%  Sign: x_dev > 0 → Mass too far down (air gap too large)
%        x_dev < 0 → Mass too far up (air gap too small)
%  Unit conversion occurs in the Simulink model ([mm] → [m]).
%
% #########################################################################

clc; clear; close all;

%% --- Workspace Environment Check ---
% System parameters are loaded from setup_sim_params.m if not already 
% present in the workspace. Expected: mass, Km, grav, airgap_soll, kinMapping.
if ~exist('mass', 'var') || ~exist('airgap_soll', 'var')
    run('../setup_sim_params.m');
end

%% --- Plant Parameters ---
Plant.ag_eq = airgap_soll;   % Target air gap [mm], passed to Simulink model
Plant.kinM  =  [1, -1];      % Kinematic mapping matrix
Plant.x_init = [(airgap_soll-airgap_init)*1e-3; 0]; % Initial state


% #########################################################################
%% === CONTROLLER 1: Feedback Linearization + State-Space Control ===
% #########################################################################
%
%  BASIC IDEA:
%  The nonlinear system is transformed into a linear chain of integrators 
%  via exact input-output linearization. The virtual input 'v' of the 
%  linearized system is then determined by a linear controller.
%
%  LINEARIZED VIRTUAL INPUT:
%    v = u_virtual = x2_dot = (Km/m) * u / x_s^2 - g
%
%  For the linear controller design:
%    x1_ddot ≡ v   (double integrator after extension with integral component)
%
%  AUGMENTED STATE VECTOR (with integral component for steady-state accuracy):
%    z = [integral(x1 dt);  x1;  x2]
%
%  SYSTEM MATRICES OF LINEARIZED SYSTEM (Triple-Integrator):
%    A_fbl = [0 1 0]     B_fbl = [0]
%            [0 0 1]             [0]
%            [0 0 0]             [1]

A_fbl = [0 1 0;
         0 0 1;
         0 0 0];
B_fbl = [0; 0; 1];

% -------------------------------------------------------------------------
%% --- 1.1 PID Controller Coefficients (Classic Approach after FBL) ---
% -------------------------------------------------------------------------
%  After feedback linearization, the control law is:
%    v = -(kd * x2 + kp * x1 + ki * integral(x1 dt))
%
%  Closed-loop characteristic polynomial:
%    p(s) = s^3 + kd*s^2 + kp*s + ki = 0
%
%  Chosen poles:  s = [-50,  -80+80j,  -80-80j]
%    Pole at s=-50       : Real pole, time constant τ = 1/50 = 20 ms
%    Pole pair at -80±80j: Damped oscillation, ζ ≈ 0.71, ωn = 113 rad/s
%
%  Coefficient comparison: kd = 210, kp = 20800, ki = 640000
%  (Negative signs since feedback: v = -K * z)

fbklin.kd = -210;       % D-gain: Coefficient of s^2 (damping)          [1/s]
fbklin.kp = -20800;     % P-gain: Coefficient of s^1 (stiffness)        [1/s^2]
fbklin.ki = -640000;    % I-gain: Coefficient of s^0 (stat. accuracy)   [1/s^3]

% -------------------------------------------------------------------------
%% --- 1.2 Pole Placement ---
% -------------------------------------------------------------------------
%  Design of a state controller K via direct pole placement.
%  Control law: v = -K_pole * z
%
%  Chosen poles (Left Half Plane, stable dynamics):
%    s_p = [-30, -70, -160]
%      s = -30  : Slow integrator dynamics (steady-state accuracy)
%      s = -70  : Medium dynamics          (position error)
%      s = -160 : Fast dynamics            (velocity control)

s_p      = [-30, -70, -160];
K_pole   = place(A_fbl, B_fbl, s_p);
fbklin.K_pole = K_pole;   % State feedback vector (Pole Placement)   [1×3]

% -------------------------------------------------------------------------
%% --- 1.3 Linear-Quadratic Regulator (LQR) ---
% -------------------------------------------------------------------------
%  Optimal state controller by minimizing the quadratic cost function:
%    J = integral( z' * Q * z  +  v' * R * v ) dt
%
%  Weighting matrices:
%    Q = diag([q_sigma, q_x1, q_x2])
%      q_sigma = 4e11  — High penalty on integral error (accuracy focus)
%      q_x1    = 5e7   — Medium penalty on position deviation
%      q_x2    = 5e4   — Lower penalty on velocity
%    R = 1             — No penalty on control effort 
%                        (input is uncritical in the virtualized system)

Q_fbl        = diag([4e11, 5e7, 5e4]);
R_fbl        = 1;
[K_lqr, S, P] = lqr(A_fbl, B_fbl, Q_fbl, R_fbl);
fbklin.K_lqr = K_lqr;    % State feedback vector (LQR optimization)  [1×3]

% #########################################################################
%% === CONTROLLER 2: Integrator Backstepping with Integral Component ===
% #########################################################################
%
%  BASIC IDEA:
%  Recursive Lyapunov-based design procedure. At each stage, a virtual 
%  control variable (alpha_i) is chosen so that a Lyapunov function 
%  decreases strictly monotonically. The integral state sigma ensures 
%  steady-state accuracy and disturbance rejection.
%
%  AUGMENTED STATE VECTOR:
%    [sigma; x1; x2]  where  sigma = integral(x1 dt)
%
%  LYAPUNOV CANDIDATE:
%    $$V = \frac{1}{2}\sigma^2 + \frac{1}{2}z_1^2 + \frac{1}{2}z_2^2 > 0$$
%    $$\dot{V} < 0$$ (guarantees global asymptotic stability)
%
%  CONTROL LAW (analytically derived):
%    i_s = (x0 - x1) * sqrt( (mass/Km) * max(grav - Kv*x2 - Kp*x1 - Ki*sigma, 0) )
%
%  COMBINED FEEDBACK COEFFICIENTS (from backstepping derivation):
%    Kv = c0 + c1 + c2              (Velocity damping)
%    Kp = 2 + c0*c1 + c0*c2 + c1*c2 (Position stiffness)
%    Ki = c0 + c2 + c0*c1*c2        (Integral gain)
%
%  DESIGN PARAMETERS c0, c1, c2:
%  These parameters define the desired closed-loop eigenvalues of the 
%  three backstepping stages. The overall system is stable if all ci > 0.
%
%  STABILITY RULE:
%  The unstable pole of the open-loop system is at:
%    p_unstable = sqrt(2*g / x0) ≈ 99 rad/s
%  c1 and c2 must dominate this (factor of 1.2 to 2 is usually sufficient).
%  c0 (Integral dynamics) should be chosen significantly smaller (factor 5–10):
%    Too high c0 → Integrator windup, overshoot due to interaction 
%    with fast control loops — the nonlinearity of the magnetic bearing 
%    further amplifies this effect.

bksp.c1 = 120;   % Backstepping coefficient Stage 2 (Velocity loop) [rad/s]
bksp.c2 = 120;   % Backstepping coefficient Stage 3 (Force loop)    [rad/s]
bksp.c0 = 20;    % Backstepping coefficient Stage 1 (Integral loop) [rad/s]


% #########################################################################
%% === CONTROLLER 3: Sliding Mode Control (SMC) with Integral Sliding Surface ===
% #########################################################################
%
%  BASIC IDEA:
%  The state is driven onto a sliding surface s(x) = 0 (reaching phase), 
%  on which a stable error dynamic is prescribed (sliding phase). 
%  The switching signal ensures robustness against disturbances and model 
%  uncertainties.
%
%  SLIDING SURFACE WITH INTEGRAL COMPONENT:
%    s = lambda*x1 + x2 + ki * integral(x1 dt)
%
%  SYSTEM TERMS ALONG THE SLIDING SURFACE:
%    C^T * f(x) = lambda*x2 - g + ki*x1     (Nonlinear component)
%    C^T * g(x) = (Km/m) / x_gap^2          (Input term, c = Km/m)
%
%  REACHING LAW (Exponential + Saturation Boundary Layer):
%    h(s) = -eta * sat(s / phi) - K * s
%
%  FULL CONTROL LAW:
%    u   = ( -C^T*f(x) + h(s) ) / ( C^T*g(x) )
%    u   = max(u, 0)    — physical constraint: i_s^2 >= 0
%    i_s = sqrt(u)      — commanded current
%
%  STATE VARIABLES: Estimated by the Extended Kalman Filter (EKF).
%  The integral component 'ki' compensates for EKF-related estimation offsets 
%  (measurement dead-time delay) and constant disturbances not modeled in EKF.
%
% -------------------------------------------------------------------------
%% --- 3.1 Equilibrium Values ---
% -------------------------------------------------------------------------
% Target air gap: conversion from mm to meters
x0     = airgap_soll * 1e-3;        % Target air gap x0                [m]
smc.x0 = x0;

% Equilibrium current from force balance at hover point:
%   Km * i0^2 / x0^2 = m * g   =>   i0 = sqrt(m * g * x0^2 / Km)
i0 = sqrt(mass * grav * x0^2 / Km); % Equilibrium current i0           [A]
u0 = i0^2;                          % Equilibrium control input u0     [A^2]

% -------------------------------------------------------------------------
%% --- 3.2 SMC Design Parameters ---
% -------------------------------------------------------------------------
% Sliding surface coefficient lambda
%   Defines the error dynamics on the sliding surface:
%     x1_dot + lambda * x1 = 0   =>   x1(t) = x1(0) * exp(-lambda * t)
%   Stability condition (necessary):
%     lambda > |unstable pole| = sqrt(2*g / x0) ≈ 99 rad/s
smc.lambda = 150;                    % Sliding surface coefficient      [rad/s]

% Exponential reaching gain K
%   Controls the convergence rate to the sliding surface during reaching phase:
%     s_dot = -K * s   =>   s(t) = s(0) * exp(-K * t)
%   Higher K: faster reaching, higher noise sensitivity.
smc.K = 50;                          % Exponential reaching gain        [-]

% Switching gain eta
%   Ensures reaching condition s * s_dot < 0 despite disturbances:
%     eta > || d(x,t) ||_max   (upper disturbance bound in s-space)
%   Scaling with u0 adjusts eta physically to the control input range.
%   Too high eta: increased chattering (remedy: increase phi).
smc.eta = 3;                   % Switching gain                   [A^2]

% Boundary layer thickness phi
%   Replaces sign(s) with the continuous saturation function sat(s/phi):
%
%              | s / phi,   if |s| <  phi   (linear transition zone)
%   sat(s) =  |
%              | sign(s),   if |s| >= phi   (switching zone)
%
%   phi too small (phi → 0): hard switching → chattering, noise sensitive
%   phi too large (phi > s_typ): permanent residence in boundary layer
%                  → steady-state error (without ki compensation)
%   Unit of phi matches unit of s [m/s]:
%     s = lambda [rad/s] * x1 [m] + x2 [m/s] → [m/s]
smc.phi = 0.02;                     % Boundary layer thickness         [m/s]

% Integral sliding surface component ki
%   Extends the sliding surface by: + ki * integral(x1 dt)
%   Purpose (two aspects):
%     1. Eliminating steady-state offsets due to EKF estimation lag
%     2. Compensating for constant disturbances not modeled in EKF
%   Too high ki: Integrator windup → Overshoot.
%   Anti-Windup: Freeze integrator once i_cmd saturates (|i| = i_max).
smc.ki = 5000;                       % Integral sliding surface gain    [1/s]

% Sliding vector C
%   Defines the sliding surface in compact vector form:
%     s = C^T * x = lambda * x1 + x2
%   C is passed as an input parameter to the Simulink SMC block.
smc.C = [smc.lambda; 1];             % Sliding vector C                 [2×1]

% -------------------------------------------------------------------------
%% --- 3.3 SMC Parameter Bus (Simulink Bus Object) ---
% -------------------------------------------------------------------------
%  All SMC parameters are bundled as a typed Simulink Bus.
%  Advantages: structured signal management, compile-time type checking, 
%  unified interface between Workspace and Simulink blocks.

smc_Bus             = Simulink.Bus;
smc_Bus.Description = 'Sliding Mode Controller Parameters Bus';

% Bus structure: {FieldName, Dimension}
elems = {
    'x0',     1;       % Target air gap                           [m]
    'lambda', 1;       % Sliding surface coefficient              [rad/s]
    'K',      1;       % Exponential reaching gain                [-]
    'eta',    1;       % Switching gain                           [A^2]
    'phi',    1;       % Boundary layer thickness                 [m/s]
    'ki',     1;       % Integral sliding surface gain            [1/s]
    'C',      [2 1];   % Sliding vector                           [2×1]
};

% Programmatically create bus elements
for k = 1:size(elems, 1)
    e            = Simulink.BusElement;
    e.Name       = elems{k, 1};
    e.Dimensions = elems{k, 2};
    e.DataType   = 'double';
    e.SampleTime = -1;               % Sample time: inherited from model
    e.Complexity = 'real';
    smc_Bus.Elements(k) = e;
end

% #########################################################################
%% --- Saving Controller Parameters ---
% #########################################################################
%  All structures are saved in a common .mat file and are available 
%  to all Simulink models via the Workspace.
save('../Controller_Params/NonlinearControllerData.mat', ...
     'Plant', 'fbklin', 'bksp', 'smc', 'smc_Bus');

% cd ..\02_Nonlinear_Controller_Analytical\

%% --- Completion Message ---
fprintf('\n%s\n', repmat('=', 1, 80));
fprintf('  Nonlinear controller design completed successfully.\n');
fprintf('  Timestamp : %s\n', datetime('now', 'Format', 'dd.MM.yyyy HH:mm:ss'));
fprintf('  Output    : Controller_Params/NonlinearControllerData.mat\n');
fprintf('  Controllers: FeedbackLin  |  Backstepping  |  SMC\n');
fprintf('%s\n\n', repmat('=', 1, 80));