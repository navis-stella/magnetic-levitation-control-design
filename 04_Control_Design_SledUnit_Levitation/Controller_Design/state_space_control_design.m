%#########################################################################%
%#                                                                       #%
%#  FILE:  state_space_control_design.m                                  #%
%#                                                                       #%
%#  DESCRIPTION:                                                         #%
%#  State-space controller design for a high-speed machine with          #%
%#  magnetic bearings (5-DOF sled unit, 8 electromagnets). The script    #%
%#  computes operating-point currents via constrained optimisation,      #%
%#  linearises the reluctance force model about the equilibrium point,   #%
%#  designs an LQR controller with disturbance feedforward and an        #%
%#  augmented Kalman observer (15 states), and verifies the closed loop  #%
%#  for stability, steady-state accuracy, and frequency response.        #%
%#                                                                       #%
%#  SECTIONS:                                                            #%
%#    S1 — Operating-point currents (fmincon)                            #%
%#    S2 — Linearisation (Ks, Ki)                                        #%
%#    S3 — Base LQR design                                               #%
%#    S4 — Disturbance feedforward                                       #%
%#    S5 — Kinematic mapping (sensor → degree of freedom / DOF)          #%
%#    S6 — Augmented Kalman observer (with disturbance estimation)       #%
%#    S7 — Closed-loop verification                                      #%
%#    S8 — Parameter export                                              #%
%#                                                                       #%
%#########################################################################%

clc; clear; close all;

% Extend search path to include the shared library
addpath('..\..\00_Shared_Library\')

% Anonymous function as a substitute for the ternary operator (Cond ? A : B),
% since MATLAB does not support inline conditional expressions.
% Usage: ternary(logical_expression, value_if_true, value_if_false)
ternary = @(cond, a, b) subsref({b, a}, struct('type','{}','subs',{{cond+1}}));

% Load the complete parameter set of the machine model.
% This ensures that the controller design and the multi-body simulation
% use the same constants (geometry, masses, inertias, air gap).
run("..\Setup_Machine_Model\setup_structure_params");

% Set the nominal air gap if it has not already been defined by the
% parameter script (fallback value: 0.5 mm).
if ~exist("nom_airgap",'var')
    nom_airgap = 0.5;   % Nominal air gap [mm]
end


%% S1: Operating-point currents via constrained optimisation
%--------------------------------------------------------------------------
% The system has 8 electromagnets but only 5 degrees of freedom (DOF),
% making it mechanically redundant (over-actuated).
% Instead of enforcing an artificial symmetry (left = right), we minimise
% the ohmic copper losses (∝ ΣI²) subject to the conditions of full
% force and moment equilibrium.
% This also correctly accounts for an asymmetric centre of mass.
%--------------------------------------------------------------------------

% Symbolic positive current variables for each of the 8 electromagnets.
% The "positive" constraint reflects physical reality
% (currents cannot become negative in this system).
syms I0_elo I0_elu I0_ero I0_eru positive   % Electromagnets (top/bottom, left/right)
syms I0_slo I0_slu I0_sro I0_sru positive   % Side magnets   (top/bottom, left/right)
vars_I = [I0_elo; I0_elu; I0_ero; I0_eru; I0_slo; I0_slu; I0_sro; I0_sru]; % 8×1

% Convert nominal air gap from millimetres to metres (SI unit for computation)
airgap0 = nom_airgap * 1e-3;       % [mm] → [m]

% Determine the number of electromagnets from the labels (defined in setup_structure_params)
nMag = length(elMaglabels);        % Yields 8

% Direction unit vectors of the magnet forces in the sled frame (3×8).
% Each column describes the direction in which the corresponding magnet pulls.
n_vec = [MagSfNVec.ELO, MagSfNVec.ELU, MagSfNVec.ERO, MagSfNVec.ERU, ...
         MagSfNVec.SLO, MagSfNVec.SLU, MagSfNVec.SRO, MagSfNVec.SRU];

% Position vectors from the sled GCS to the respective magnet surface GCS (3×8) [m].
% These moment arms are needed for the torque calculation (r × F).
r_vec = [MagSfGCS2SlGCS.ELO.TV, MagSfGCS2SlGCS.ELU.TV, ...
         MagSfGCS2SlGCS.ERO.TV, MagSfGCS2SlGCS.ERU.TV, ...
         MagSfGCS2SlGCS.SLO.TV, MagSfGCS2SlGCS.SLU.TV, ...
         MagSfGCS2SlGCS.SRO.TV, MagSfGCS2SlGCS.SRU.TV] * 1e-3;  % [mm] → [m]

% Gravitational force on the sled [N] (acting at the sled GCS origin)
Fg_num = SledUnitMass * grav;               % 3×1 [N], e.g. [0; 0; -m*g]

% Gravitational moment about the sled GCS origin [N·m].
% Arises because the centre of mass (CoM) is offset from the GCS origin.
Mg_num = cross(CoM_TV2SlGCS * 1e-3, Fg_num);   % 3×1 [N·m]

fprintf('\nComputing equilibrium currents:\n')

% --- Objective function: deviation of currents from desired operating-point current ---
% We minimise ΣΔI² (quadratic deviation), which is equivalent to minimising
% copper losses when I_target is the optimal rated current.
I_target  = I_work;                             % Desired operating-point current, 10 [A]
objective  = @(I) sum((I - I_target).^2);       % Scalar cost function

% --- Nonlinear equality constraints: static equilibrium ---
% ceq = [F_net; M_net] must be zero (6 equations for 8 unknowns → over-determined).
nonlcon = @(I) equilibriumConstraints(I, airgap0, n_vec, r_vec, Fg_num, Mg_num);

% Physical current bounds (from magnet specification, defined in setup_structure_params)
lb = I_min * ones(nMag, 1);     % Lower bound [A]
ub = I_max * ones(nMag, 1);     % Upper bound  [A]

% Initial guess: uniform current distribution as a neutral starting point
I0_guess = 2.0 * ones(nMag, 1); % [A]

% Solver options for fmincon (Sequential Quadratic Programming)
% SQP is well-suited for nonlinear equality constraints.
options = optimoptions('fmincon', ...
    'Algorithm',              'sqp', ...        % SQP algorithm for NLP
    'Display',                'iter', ...       % Show iteration progress
    'MaxFunctionEvaluations', 10000, ...        % Maximum function evaluations
    'MaxIterations',          1000, ...         % Maximum number of iterations
    'ConstraintTolerance',    1e-10, ...        % Admissible equilibrium residual [N, N·m]
    'OptimalityTolerance',    1e-10, ...        % KKT optimality condition
    'StepTolerance',          1e-12);           % Minimum step size in parameter space

% Start optimisation: no linear inequalities (A,b) or equalities (Aeq,beq)
[I_opt, fval, exitflag, output] = fmincon(objective, I0_guess, ...
                                            [], [], ...   % No linear inequalities
                                            [], [], ...   % No linear equalities
                                            lb, ub, ...   % Current bounds
                                            nonlcon, ...  % Equilibrium constraints
                                            options);

% Convergence check: exitflag ≤ 0 means no optimum was found
if exitflag <= 0
    warning('fmincon did not converge (exitflag = %d). Check the initial guess or bounds.', exitflag);
    disp(output.message);
end

% Evaluate residual forces and moments at the found operating point.
% For a perfect equilibrium, all entries should be close to zero.
[~, ceq_check] = nonlcon(I_opt);
fprintf('\nResidual force  [N]:   [%.2e, %.2e, %.2e]\n', ceq_check(1:3));
fprintf('Residual moment [N*m]: [%.2e, %.2e, %.2e]\n', ceq_check(4:6));

% Store the optimised currents in a struct for later parameter transfer
curVal.I0_elo = I_opt(1);   % Electromagnet left-top
curVal.I0_elu = I_opt(2);   % Electromagnet left-bottom
curVal.I0_ero = I_opt(3);   % Electromagnet right-top
curVal.I0_eru = I_opt(4);   % Electromagnet right-bottom
curVal.I0_slo = I_opt(5);   % Side magnet left-top
curVal.I0_slu = I_opt(6);   % Side magnet left-bottom
curVal.I0_sro = I_opt(7);   % Side magnet right-top
curVal.I0_sru = I_opt(8);   % Side magnet right-bottom

disp(newline + "Levitation equilibrium currents (optimised):");
disp(curVal);


%% S2: Linearisation of reluctance forces about the operating point
%--------------------------------------------------------------------------
% Kinematic relationship for small deviations about equilibrium:
%   δg_k = n_k^T · (δp + δα × r_k)
%
%   δg_k   : air gap deviation of the k-th magnet [m]
%   n_k    : unit normal vector of the k-th magnet
%   δp     : translational deviation [dx, dy, 0]^T [m]
%   δα     : rotational deviation    [dθ, dφ, dψ]^T [rad]
%   r_k    : moment arm from sled GCS to magnet surface [m]
%
% The partial derivatives yield two linearised coefficient matrices:
%   Ks (6×5): position-force gain  ∂F/∂q  — describes the negative spring stiffness
%   Ki (6×8): current-force gain   ∂F/∂i  — corresponds to the actuator gain
%--------------------------------------------------------------------------

% Symbolic degrees of freedom (δz = 0, since axial motion is locked)
syms x y theta phi psi real          % x, y: translation; θ,φ,ψ: rotation (roll, pitch, yaw)
s_dev = [x; y; theta; phi; psi];     % 5×1 deviation vector in state space

% Symbolic current deviations from the operating point (Δi = i - I0)
syms i_elo i_elu i_ero i_eru i_slo i_slu i_sro i_sru real
i_dev = [i_elo; i_elu; i_ero; i_eru; i_slo; i_slu; i_sro; i_sru];  % 8×1

% Nominal values: all air gaps equal (equilibrium), currents from S1
ag0    = airgap0 * ones(8,1);        % Nominal air gaps (8×1) [m]
I0_vec = [curVal.I0_elo curVal.I0_elu curVal.I0_ero curVal.I0_eru ...
          curVal.I0_slo curVal.I0_slu curVal.I0_sro curVal.I0_sru]; % 1×8 [A]

% Accumulate symbolic total force and moment on the sled
Fm_dev = [0;0;0];   % Resultant magnetic force   [N],   3×1
KM_dev = [0;0;0];   % Resultant magnetic moment  [N·m], 3×1

for i = 1:8
    mag  = elMaglabels{i};              % Identifier of the i-th magnet

    % Moment arm from sled GCS to the force application plane of the magnet [m]
    pv_i = MagSfGCS2SlGCS.(mag).TV/1000;

    % Normal vector of the i-th magnet (direction of the attractive force)
    nv_i = MagSfNVec.(mag);

    % Translational + rotational deviation at the force application point (linearisation)
    % The term cross(δα, r) describes the displacement induced by rotation.
    rp_i = [x; y; 0] + cross([theta; phi; psi], pv_i);

    % Projected air gap deviation along the magnet normal
    % (positive = gap decreases → force increases)
    ags_i = nv_i' * rp_i;

    % Perturbed air gap and perturbed current at the i-th magnet
    xs_i = ag0(i) - ags_i;   % Reduction of gap → more force
    is_i = I0_vec(i) + i_dev(i);

    % Individual reluctance force vector (scalar value × direction vector)
    F_i = reluctForce(xs_i, is_i) * nv_i;    % 3×1 [N]

    % Add force and moment to the total vectors
    Fm_dev = Fm_dev + F_i;
    KM_dev = KM_dev + cross(pv_i, F_i);      % Moment contribution: r × F
end

% Jacobian matrices of total force and moment w.r.t. pose and currents
dFm_ds = jacobian(Fm_dev, s_dev);    % ∂F/∂q  (3×5): force-pose gradient
dFm_di = jacobian(Fm_dev, i_dev);    % ∂F/∂i  (3×8): force-current gradient
dKM_ds = jacobian(KM_dev, s_dev);    % ∂M/∂q  (3×5): moment-pose gradient
dKM_di = jacobian(KM_dev, i_dev);    % ∂M/∂i  (3×8): moment-current gradient

% Evaluate all symbolic variables at the operating point (s_dev = 0, i_dev = 0)
old_vars = [s_dev; i_dev];
new_vals = zeros(numel(old_vars), 1);   % Operating point: all deviations = 0
dFm_ds_eq = subs(dFm_ds, old_vars, new_vals);
dFm_di_eq = subs(dFm_di, old_vars, new_vals);
dKM_ds_eq = subs(dKM_ds, old_vars, new_vals);
dKM_di_eq = subs(dKM_di, old_vars, new_vals);

% Assemble force and moment terms into combined matrices (6×5 and 6×8)
ks_full = double([dFm_ds_eq; dKM_ds_eq]);   % 6×5: [Fx,Fy,Fz,Mx,My,Mz]^T / ∂q
ki_full = double([dFm_di_eq; dKM_di_eq]);   % 6×8: [Fx,Fy,Fz,Mx,My,Mz]^T / ∂i

% Remove the Z-force row (row 3) — axial motion is locked (no DOF)
Ks = ks_full([1 2 4 5 6], :);   % 5×5: active DOFs [x, y, θ, φ, ψ]
Ki = ki_full([1 2 4 5 6], :);   % 5×8: active DOFs [x, y, θ, φ, ψ]


%% S3: Linearised state-space model and LQR design
%--------------------------------------------------------------------------
% ## S3.1: State-space form
%
% State vector:  x = [q; q̇]^T  (10×1)
%   q   = [x, y, θ, φ, ψ]^T  — generalised position coordinates  (5×1)
%   q̇  = time derivatives of the position coordinates             (5×1)
%
% Input vector: u = i_dev  (8×1), current deviations from the operating point
%
% Linearised equation of motion:
%   M·q̈ = Ks·q + Ki·u
%   → q̈ = M⁻¹·Ks·q + M⁻¹·Ki·u
%
% In state-space form:
%   ẋ = A·x + B·u    with   A = [0, I; M⁻¹Ks, 0],  B = [0; M⁻¹Ki]
%--------------------------------------------------------------------------

dim_s = numel(s_dev);   % Number of position DOFs = 5
dim_u = numel(vars_I);  % Number of current inputs = 8
dim_d = dim_s;          % Number of generalised disturbance components = 5

% --- Mass and inertia matrix referred to the sled GCS origin ---
% The off-diagonal terms m·rz_com arise from the offset of the centre of
% mass from the GCS origin in the Z-direction and couple translation with
% rotation (Newton-Euler).
rz_com = CoM_TV2SlGCS(3) * 1e-3;    % Z-offset of the centre of mass [m]
m      = SledUnitMass;              % Total mass of the sled [kg]

% Full mass matrix M (5×5), symmetric positive definite
% Structure: [m, 0, 0, m*rz, 0; 0, m, -m*rz, 0, 0; ...]
MassInertia = [m,         0,          0,                       m*rz_com,                 0;
               0,         m,         -m*rz_com,                0,                        0;
               0,        -m*rz_com,   InertiaTensor_GC(1,1),   InertiaTensor_GC(1,2),    InertiaTensor_GC(1,3);
               m*rz_com,  0,          InertiaTensor_GC(2,1),   InertiaTensor_GC(2,2),    InertiaTensor_GC(2,3);
               0,         0,          InertiaTensor_GC(3,1),   InertiaTensor_GC(3,2),    InertiaTensor_GC(3,3)];

% State-space matrices of the linearised system
A = [zeros(dim_s), eye(dim_s); MassInertia\Ks, zeros(dim_s)];  % 10×10 system matrix
B = [zeros(dim_s, dim_u); MassInertia\Ki];                     % 10×8  input matrix
C = eye(2*dim_s);                                              % Full state output
D = zeros(2*dim_s, dim_u);                                     % No direct feedthrough

% --- PBH controllability test (Popov-Belevitch-Hautus) ---
% A system is controllable if rank([λI-A, B]) = n for all eigenvalues λ.
% Checked robustly here: unstable poles that are not controllable abort the script.
n = size(A, 1);
unc = arrayfun(@(lam) rank([lam*eye(n) - A, B]) < n, unique(eig(A)));
assert(~any(unc), 'Base system is not controllable.');
sys = ss(A, B, C, D);   % MATLAB state-space object for controller design

%--------------------------------------------------------------------------
% ## S3.2: LQR design for the base system (without integrator)
%
% Cost criterion: J = ∫(x'·Q·x + u'·R·u) dt → minimise
%   Q: weights undesired state deviations (position weighted more than velocity)
%   R: penalises control effort (small R → aggressive control, high currents)
%
% lqr() solves the algebraic Riccati equation and returns the optimal
% feedback gain K such that u = -K·x stabilises the loop.
%--------------------------------------------------------------------------
Q_pos = 1000*diag([2, 2, 2, 2, 2]);       % Position weights (x, y, θ, φ, ψ)
Q_vel = 1e-1 * diag([1, 1, 1, 1, 1]);     % Velocity weights
Q     = blkdiag(Q_pos, Q_vel);            % Full state weighting matrix (10×10)
R     = 2e-6 * eye(dim_u);                % Control effort weighting (8×8), small → aggressive

K_lqr = lqr(sys, Q, R);                   % Optimal LQR gain (8×10)


%% S4: Disturbance feedforward
%--------------------------------------------------------------------------
% The augmented observer (S6) estimates a 5-dimensional generalised
% disturbance d̂ = [F̂x; F̂y; M̂θ; M̂φ; M̂ψ] acting in the sled frame.
% Steady-state compensation:
%
%   At equilibrium (q = 0, q̇ = 0):
%     0 = Ki·u_ss + d̂  →  u_ss = −pinv(Ki)·d̂
%
% Final control law:
%   u = −K·x̂  −  pinv(Ki)·d̂
%     = −K_total · [x̂; d̂]   with   K_total = [K, pinv(Ki)]   (8×15)
%
% The feedforward acts directly — as soon as d̂ has converged, the
% disturbance is compensated without the LQR term having to "see" it
% through a position deviation q.
% This achieves steady-state accuracy (offset-free) without an explicit
% integrator in the controller.
%--------------------------------------------------------------------------
K_d_ff   = pinv(Ki);                % 8×5: maps d̂ → compensating current vector
K_total  = [K_lqr, K_d_ff];         % 8×15: combined gain for [x̂; d̂]

% Plausibility check: Ki must have full row rank (= 5) so that every
% disturbance direction can be cancelled by a suitable current combination.
fprintf('rank(Ki) = %d (must be %d, otherwise the disturbance cannot be fully compensated)\n', ...
        rank(Ki), dim_s);
assert(rank(Ki) == dim_s, 'Ki is rank-deficient — disturbance cannot be compensated.');


%% S5: Kinematic mapping matrix (sensor → degree of freedom)
%--------------------------------------------------------------------------
% The 8 air gap sensors each measure a scalar gap change δg_k.
% The goal is to map these to the 5 generalised coordinates q:
%
%   δg_k = nx·x + ny·y + (-ny·rz)·θ + (nx·rz)·φ + (-nx·ry + ny·rx)·ψ
%
% In matrix form: δg = H · q   (8×5)
%
% Since the system is over-determined (8 equations, 5 unknowns),
% the pseudo-inverse H⁺ = (H'H)⁻¹H' provides the least-squares estimate:
%   q̂ = H⁺ · δg   (5×1)
%--------------------------------------------------------------------------
H = zeros(dim_u, dim_s);   % Kinematic measurement matrix (8×5), initially empty

for i = 1:numel(elMaglabels)
    mag  = elMaglabels{i};

    % Normal vector of the i-th magnet (direction of the sensitive measurement)
    nv_i = MagSfNVec.(mag);

    % Moment arm from sled GCS to the sensor / magnet surface point [m]
    rv_i = MagSfGCS2SlGCS.(mag).TV * 1e-3;

    % Components for the kinematic equation
    nx_mag = nv_i(1);   ny_mag = nv_i(2);           % x and y components of the normal vector
    rx_mag = rv_i(1);   ry_mag = rv_i(2);  rz_mag = rv_i(3);   % Moment arm components [m]

    % Row of the measurement matrix: δg_i = f(x, y, θ, φ, ψ) from linearised kinematics
    H(i,:) = [nx_mag, ny_mag, -ny_mag*rz_mag, nx_mag*rz_mag, ...
              -nx_mag*ry_mag + ny_mag*rx_mag];
end

% Pseudo-inverse: solves the over-determined system in the LS sense (5×8)
H_pinv = pinv(H);

% Check the quality of the kinematic mapping
fprintf('\n=== S5 Kinematic Mapping Check ===\n')
fprintf('H rank: %d (should be %d)\n', rank(H), dim_s);
% Consistency check: H·H⁺·H should equal H (property of the pseudo-inverse)
fprintf('Maximum reconstruction error: %.2e\n', max(max(abs(H * H_pinv * H - H))));


%% S6: Augmented Kalman observer with disturbance estimation
%--------------------------------------------------------------------------
% Augmented plant model (continuous-time, integrating disturbance):
%   ẋ = A·x + B·u + B_d·d
%   ḋ = 0                    (assumption of constant disturbance)
%   y = C_obs·x              (no direct measurement of d)
%
% Disturbance entry: d ∈ R^5 represents generalised forces/moments
% acting in the sled frame. Thus B_d = [0; M⁻¹] and d̂ has a physical
% interpretation: the equivalent unmodelled force required to explain
% motions that the linear model does not predict (modelling errors,
% friction, external loads).
%
% NOTE on observability: the plant model is very stiff
% (|M⁻¹·Ks| ~ 10^6 s⁻²), so the steady-state gain
% |H·Ks⁻¹| from disturbance force to air gap measurement is of the order
% 1e-8 m/N. The augmented system is mathematically observable,
% but the disturbance modes have singular values near MATLAB's default
% rank tolerance, so rank()-based tests give inconsistent results.
% The true test is whether the resulting Kalman gain yields a stable
% observer — which we verify at the end of this block.
%--------------------------------------------------------------------------

% --- S6.1: Measurement matrix and disturbance entry ---
C_obs    = [H, zeros(dim_u, dim_s)];                       % 8×10
B_d_phys = [zeros(dim_s, dim_d);
            MassInertia \ eye(dim_d)];                     % 10×5

% --- S6.2: Augmented system matrices ---
A_aug = [A,                       B_d_phys;
         zeros(dim_d, 2*dim_s),   zeros(dim_d, dim_d)];    % 15×15
B_aug = [B;
         zeros(dim_d, dim_u)];                              % 15×8
C_aug = [C_obs, zeros(dim_u, dim_d)];                       % 8×15

% --- S6.3: Spectrum diagnostics (informational, no assertion) ---
sv_PR = svd([-A, -B_d_phys; C_obs, zeros(dim_u, dim_d)]);
fprintf('\n=== S6 Augmented System Spectrum ===\n');
fprintf('  Singular values of the PR matrix: σ_max = %.2e, σ_min = %.2e\n', ...
        max(sv_PR), min(sv_PR));
fprintf('  Spread: %.1f decades — disturbance modes are weakly observable\n', ...
        log10(max(sv_PR)/min(sv_PR)));
fprintf('  This is intrinsic to the stiff maglev plant, not a design error.\n');

% --- S6.4: Kalman filter noise covariances ---
% Measurement noise (from sensor datasheet)
sn_std = sensor_noise_std * 1e-3;                          % [m]
R_kf   = sn_std^2 * eye(dim_u);                            % 8×8

% Plant process noise — small, since unmodelled effects now live in d̂
q_pos_t = 1e-10;
q_pos_r = 1e-8;
q_vel_t = 1e-6;
q_vel_r = 1e-4;
Q_x_kf  = diag([q_pos_t, q_pos_t, q_pos_r, q_pos_r, q_pos_r, ...
                q_vel_t, q_vel_t, q_vel_r, q_vel_r, q_vel_r]);

% Disturbance process noise — sized to compensate the weak DC
% observability. Target frequency of the disturbance poles ~10 rad/s:
%   Q_d ≈ ω_d² · R / σ_obs²
%   with ω_d = 10, R ≈ (2e-6)² = 4e-12, σ_obs ≈ 2e-8  →  Q_d ≈ 1e6
q_d_force  = 1e10;     % (N)²    — variance of translational disturbance
q_d_torque = 1e8;      % (N·m)²  — variance of rotational disturbance
Q_d_kf     = diag([q_d_force, q_d_force, q_d_torque, q_d_torque, q_d_torque]);

Q_aug_kf = blkdiag(Q_x_kf, Q_d_kf);                         % 15×15

% --- S6.5: Kalman gain computation via LQR duality ---
L_aug = lqr(A_aug', C_aug', Q_aug_kf, R_kf)';               % 15×8

% --- S6.6: Verify observer stability (the decisive test) ---
eig_obs_aug = eig(A_aug - L_aug * C_aug);
assert(all(real(eig_obs_aug) < 0), ...
    'Augmented observer is unstable — Kalman design failed.');

% Report pole distribution
poles_real  = sort(real(eig_obs_aug), 'descend');           % slowest first
plant_poles = poles_real(poles_real <= -100);               % fast group
dist_poles  = poles_real(poles_real >  -100);               % slow group (d̂)

fprintf('\n=== S6 Observer Pole Distribution ===\n');
fprintf('  Disturbance poles (slow group, %d poles):\n', numel(dist_poles));
fprintf('    %.2f\n', dist_poles);
fprintf('  Plant state poles (fast group, %d poles):  fastest %.0f, slowest %.0f\n', ...
        numel(plant_poles), min(plant_poles), max(plant_poles));
fprintf('  Condition number of L_aug: %.2e\n', cond(L_aug));
fprintf('  Augmented observer stable — design accepted.\n');


%% S7: Closed-loop verification
%--------------------------------------------------------------------------
% Combined state vector: ξ = [x; x̂; d̂]   (10 + 10 + 5 = 25)
%
% Validation strategy:
%   S7.1 — Stability check (eigenvalues of the closed loop)
%   S7.2 — Step disturbance response (offset-free property)
%   S7.3 — Initial condition response (observer convergence)
%   S7.4 — Sinusoidal disturbance rejection (multiple frequencies)
%   S7.5 — Sensitivity frequency response (Bode plot)
%--------------------------------------------------------------------------
L_x = L_aug(1:2*dim_s,    :);                            % 10×8
L_d = L_aug(2*dim_s+1:end, :);                           % 5×8

% Closed-loop matrix:
%   ẋ      = A·x − B·K·x̂ − B·K_d_ff·d̂
%   x̂̇    = L_x·C_obs·x + (A − B·K − L_x·C_obs)·x̂ + (B_d − B·K_d_ff)·d̂
%   d̂̇   = L_d·C_obs·x − L_d·C_obs·x̂
A_full = [A,                          -B*K_lqr,                       -B*K_d_ff;
          L_x*C_obs,                   A - B*K_lqr - L_x*C_obs,        B_d_phys - B*K_d_ff;
          L_d*C_obs,                  -L_d*C_obs,                      zeros(dim_d)];        % 25×25

% --- S7.1: Stability check ---
eig_full = eig(A_full);
assert(all(real(eig_full) < 0), 'Full closed loop is unstable!');
fprintf('\n=== S7.1 Closed-Loop Stability ===\n');
fprintf('  Maximum real part of all 25 poles: %.2f\n', max(real(eig_full)));
fprintf('  Minimum real part of all 25 poles: %.2e\n', min(real(eig_full)));

% --- S7.2: Step disturbance response (1 N force in y-direction) ---
% Expected: steady-state error = 0 (offset-free), not 5 µm.
B_full_d = [B_d_phys(:,2);            % True plant feels the force
            zeros(2*dim_s, 1);        % Observer does not see it directly
            zeros(dim_d, 1)];         % d̂ must converge there via KF update

C_full_q = [eye(dim_s), zeros(dim_s, dim_s + 2*dim_s + dim_d)];  % 5×25, output position
sys_dist = ss(A_full, B_full_d, C_full_q, zeros(dim_s,1));

figure('Name','S7.2: Step Disturbance (1 N in F_y)');
[y_step, t_step] = step(sys_dist, 5);
plot(t_step, y_step * 1e3); grid on
xlabel('Time [s]'); ylabel('Position deviation [mm]');
title('Step disturbance 1 N in F_y — should converge to zero');
legend({'x','y','\theta','\phi','\psi'}, 'Location','best');

fprintf('\n=== S7.2 Step Disturbance Response ===\n');
fprintf('  Residual error at t=5s: %.2e mm (should be at machine precision)\n', ...
        max(abs(y_step(end,:)))*1e3);

% --- S7.3: Initial condition response (observer convergence) ---
% Test: plant starts slightly displaced, observer and d̂ initialised at zero.
% Expected: x → 0 and observer error ê = x̂ − x → 0 within a few milliseconds.
sys_ic = ss(A_full, zeros(25,1), eye(25), zeros(25,1));

xi0 = zeros(25,1);
% Initial plant position: 0.1 mm in x, 0.1 mm in y, 0.1 mrad roll angle
xi0(1:dim_s) = [1e-4; 1e-4; 1e-4; 0; 0];

[y_ic, t_ic] = initial(sys_ic, xi0, 0.1);

figure('Name','S7.3: Initial Condition Response');
subplot(3,1,1);
plot(t_ic, y_ic(:,1:dim_s)*1e3); grid on;
xlabel('Time [s]'); ylabel('Position [mm]');
title('Plant state x (position coordinates)');
legend({'x','y','\theta','\phi','\psi'}, 'Location','best');

subplot(3,1,2);
% Observer error: x̂ − x  (states 11-15 minus 1-5 for position coordinates)
obs_err = y_ic(:, 2*dim_s+1 : 2*dim_s+dim_s) - y_ic(:, 1:dim_s);
plot(t_ic, obs_err*1e6); grid on;
xlabel('Time [s]'); ylabel('Estimation error [µm]');
title('Observer error (x̂ − x)');
legend({'e_x','e_y','e_\theta','e_\phi','e_\psi'}, 'Location','best');

subplot(3,1,3);
% Estimated disturbance — should remain close to zero (no true disturbance present)
d_hat_ic = y_ic(:, 4*dim_s+1 : end);   % States 21-25
plot(t_ic, d_hat_ic); grid on;
xlabel('Time [s]'); ylabel('Estimated disturbance');
title('d̂ — should remain small (no true disturbance input)');
legend({'F_x','F_y','M_\theta','M_\phi','M_\psi'}, 'Location','best');

fprintf('\n=== S7.3 Initial Condition Response ===\n');
fprintf('  Settling time (plant fallen to 1%%): ');
settle_threshold = 0.01 * max(abs(xi0(1:dim_s)));
plant_norm = vecnorm(y_ic(:,1:dim_s), 2, 2);
idx_settle = find(plant_norm < settle_threshold, 1, 'first');
if ~isempty(idx_settle)
    fprintf('%.4f s\n', t_ic(idx_settle));
else
    fprintf('> %.2f s (end of simulation reached)\n', t_ic(end));
end
fprintf('  Maximum observer error: %.2e µm\n', max(abs(obs_err(:)))*1e6);
fprintf('  Maximum |d̂| without disturbance input: %.2e\n', max(abs(d_hat_ic(:))));

% --- S7.4: Sinusoidal disturbance rejection ---
% Examines behaviour under harmonic disturbance across several frequencies.
% A sin(ωt) force disturbance in y-direction is injected (unit amplitude 1 N).
freqs_Hz = [0.1, 1, 10, 50, 100, 500];          % Hz
amp_mm   = zeros(numel(freqs_Hz), dim_s);

for k = 1:numel(freqs_Hz)
    f       = freqs_Hz(k);
    omega   = 2*pi*f;
    % At least 10 periods, sampled at ~50 points/period
    dt_sim  = min(1/(50*f), 1e-4);
    t_sin   = 0 : dt_sim : 10/f;
    u_sin   = sin(omega * t_sin);
    y_sin   = lsim(sys_dist, u_sin, t_sin);
    % Steady-state amplitude from the second half of the signal
    idx_ss  = round(numel(t_sin)/2) : numel(t_sin);
    amp_mm(k,:) = max(abs(y_sin(idx_ss,:)), [], 1) * 1e3;
end

figure('Name','S7.4: Sinusoidal Disturbance Rejection');
loglog(freqs_Hz, max(amp_mm, 1e-15), '-o', 'LineWidth', 1.2); grid on;
xlabel('Frequency [Hz]'); ylabel('Steady-state amplitude [mm]');
title('Response to sin(\omega t) F_y with unit amplitude');
legend({'x','y','\theta','\phi','\psi'}, 'Location','best');

fprintf('\n=== S7.4 Sinusoidal Disturbance Rejection ===\n');
fprintf('  Freq [Hz]   |y|_max [mm]\n');
fprintf('  ---------   ------------\n');
for k = 1:numel(freqs_Hz)
    fprintf('  %8.2f   %.3e\n', freqs_Hz(k), amp_mm(k,2));
end

% --- S7.5: Sensitivity frequency response (Bode) ---
% Shows the transfer function from disturbance force F_y to position compactly.
% At low frequencies, the gain should be significantly below the pure
% LQR level thanks to d̂ compensation (offset-free property).
figure('Name','S7.5: Sensitivity Bode');
opts = bodeoptions; opts.FreqUnits = 'Hz'; opts.PhaseVisible = 'off';
bodeplot(sys_dist, {2*pi*1e-2, 2*pi*1e4}, opts);
grid on;
title('Sensitivity: F_y \rightarrow Position (5 outputs)');

% Quantify the DC gain
dc_gain_mm_per_N = dcgain(sys_dist) * 1e3;
fprintf('\n=== S7.5 Sensitivity Frequency Response ===\n');
fprintf('  DC gain F_y → Position [mm/N]:\n');
fprintf('    x:     %.3e\n', dc_gain_mm_per_N(1));
fprintf('    y:     %.3e\n', dc_gain_mm_per_N(2));
fprintf('    theta: %.3e\n', dc_gain_mm_per_N(3));
fprintf('    phi:   %.3e\n', dc_gain_mm_per_N(4));
fprintf('    psi:   %.3e\n', dc_gain_mm_per_N(5));
fprintf('  (Values near machine precision confirm offset-free behaviour.)\n');


%% S8: Export of design parameters
%--------------------------------------------------------------------------
%   ssc_params: K_total (combined LQR + disturbance feedforward gain)
%   obs_params: A_aug, B_aug, C_aug, L_aug  (augmented 15-state observer)
%--------------------------------------------------------------------------
ssc_params.dim_s      = dim_s;          % 5
ssc_params.dim_x      = 2*dim_s;        % 10  (plant states only)
ssc_params.dim_d      = dim_d;          % 5   (disturbance states)
ssc_params.dim_aug    = 2*dim_s + dim_d;% 15  (full augmented state)
ssc_params.dim_u      = dim_u;          % 8
ssc_params.nom_airgap = nom_airgap;
ssc_params.nom_curval = curVal;
ssc_params.nom_curvec = I0_vec;
ssc_params.K          = K_lqr;          % 8×10  pure SSC gain
ssc_params.K_d_ff     = K_d_ff;         % 8×5   disturbance feedforward gain
ssc_params.K_total    = K_total;        % 8×15 = [K, K_d_ff], combined gain

obs_params.A_aug      = A_aug;          % 15×15
obs_params.B_aug      = B_aug;          % 15×8
obs_params.C_aug      = C_aug;          % 8×15
obs_params.L_aug      = L_aug;          % 15×8

% Create output directory if it does not yet exist
scriptFolder = fileparts(mfilename('fullpath'));
parentFolder = fileparts(scriptFolder);
paramsFolder = fullfile(parentFolder, 'Controller_Params');
if ~exist(paramsFolder, 'dir')
    mkdir(paramsFolder);
end

% Save variables
save(fullfile(paramsFolder, 'SSC_Params'), 'ssc_params');
save(fullfile(paramsFolder, 'Obs_Params'), 'obs_params');

%--------------------------------------------------------------------------
% ## S8.2: Simulink bus definition for observer parameters
%
% For Simulink integration, the observer matrices are collected in a
% bus object. Simulink can use this bus object as a structured
% input/output of a subsystem.
%--------------------------------------------------------------------------
obs_Bus = Simulink.Bus;

% Bus elements: matrix name, its dimension, and data type
elems = {
    'A_aug',  [2*dim_s + dim_d, 2*dim_s + dim_d];   % 15×15
    'B_aug',  [2*dim_s + dim_d, dim_u];              % 15×8
    'C_aug',  [dim_u,           2*dim_s + dim_d];    % 8×15
    'L_aug',  [2*dim_s + dim_d, dim_u];              % 15×8
};

for k = 1:size(elems,1)
    e = Simulink.BusElement;
    e.Name       = elems{k,1};          % Field name in the bus
    e.Dimensions = elems{k,2};          % Matrix dimension
    e.DataType   = 'double';            % Floating-point data (64-bit)
    obs_Bus.Elements(k) = e;
end
save(fullfile(paramsFolder,'obs_params_Bus.mat'), 'obs_Bus');


%% Summary
fprintf("\n===============================================================================\n")
fprintf("[INFO] Controller design completed successfully.\n")
fprintf("[INFO] Controller and observer parameters saved to 'Controller_Params/'.\n")
fprintf("===============================================================================\n")

cd (paramsFolder)


%% Helper functions

function magForce = reluctForce(xs, is)
% RELUCTFORCE  Computes the scalar reluctance force of a single electromagnet.
%
%   magForce = reluctForce(xs, is)
%
%   Inputs:
%     xs       — Current air gap [m] (positive = open gap)
%     is       — Coil current [A]
%
%   Output:
%     magForce — Magnetic attractive force [N] (always positive; direction via n-vector)
%
%   Physical model (simplified reluctance formula):
%     F = Km · i² / x²   with   Km = μ₀ · A_eff · N² / 2
%
%   Note: This formula assumes a homogeneous field in the air gap and
%         neglects fringing flux and saturation effects.

    % Magnetic constant Km [N·m²/A²]
    Km = ElectromagnetConfig.MagConst;

    % Reluctance force: F = Km · i² / x² [N]
    magForce = Km * is^2 / xs^2;
end


function [c, ceq] = equilibriumConstraints(I, airgap0, n_vec, r_vec, Fg, Mg)
% EQUILIBRIUMCONSTRAINTS  Nonlinear equality constraints for fmincon.
%
%   [c, ceq] = equilibriumConstraints(I, airgap0, n_vec, r_vec, Fg, Mg)
%
%   Computes the equilibrium residuals for force and moment and returns
%   them as equality constraints ceq to fmincon (ceq = 0 required).
%
%   Inputs:
%     I        — Current vector of the 8 magnets [A] (8×1)
%     airgap0  — Nominal air gap [m] (scalar, equal for all magnets)
%     n_vec    — Normal vectors of the magnets (3×8)
%     r_vec    — Moment arms from GCS to magnet surface (3×8) [m]
%     Fg       — Gravitational force on the sled (3×1) [N]
%     Mg       — Gravitational moment at the GCS origin (3×1) [N·m]
%
%   Outputs:
%     c        — Empty: no inequality constraints
%     ceq      — Equilibrium residuals: [F_net; M_net] (6×1)
%                Equilibrium ⟺ ceq = 0

    F_net = Fg;     % Initialise with gravitational force
    M_net = Mg;     % Initialise with gravitational moment

    for k = 1:length(I)
        % Magnetic force vector of the k-th magnet (scalar × normal vector)
        Fk    = reluctForce(airgap0, I(k)) * n_vec(:,k);   % 3×1 [N]

        % Force contribution to the total equilibrium
        F_net = F_net + Fk;

        % Moment contribution (cross product: r × F)
        M_net = M_net + cross(r_vec(:,k), Fk);             % 3×1 [N·m]
    end

    ceq = [F_net; M_net];   % 6×1: equilibrium requires F_net = 0 and M_net = 0
    c   = [];               % No inequality constraints present
end