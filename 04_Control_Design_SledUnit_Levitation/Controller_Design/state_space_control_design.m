%#########################################################################%
%#                                                                       #%
%#  FILE:         state_space_control_design.m                           #%
%#                                                                       #%
%#  DESCRIPTION:                                                         #%
%#  State-space controller design for a high-speed machine with          #%
%#  magnetic bearings. The script calculates operating point currents    #%
%#  via constrained optimization, linearizes the reluctance force model, #%
%#  designs an integrally extended LQR controller with anti-windup, and  #%
%#  verifies closed-loop stability (pole placement, sensitivity, and     #%
%#  disturbance rejection).                                              #%
%#                                                                       #%
%#  SECTIONS:                                                            #%
%#    S1 — Operating Point Currents (fmincon)                            #%
%#    S2 — Linearization (Ks, Ki)                                        #%
%#    S3 — Baseline LQR Design                                           #%
%#    S4 — Integral Extension + Anti-Windup                              #%
%#    S5 — Kinematic Mapping (Sensor → Degree of Freedom/DOF)            #%
%#    S6 — State Observer (Pole Placement / Kalman Filter)               #%
%#    S7 — Closed-Loop Verification                                      #%
%#    S8 — Parameter Export                                              #%
%#                                                                       #%
%#########################################################################%

clc; clear; close all;

% Extend search path to include the shared library
addpath('..\..\00_Shared_Library\')

% Anonymous function serving as a ternary operator replacement (Cond ? A : B),
% since MATLAB does not feature native inline conditional expressions.
% Usage: ternary(logical_expression, value_if_true, value_if_false)
ternary = @(cond, a, b) subsref({b, a}, struct('type','{}','subs',{{cond+1}}));

% Load the complete parameter set of the machine model.
% This ensures that the controller design and multibody simulation
% utilize identical constants (geometry, masses, inertias, air gap).
run("..\Setup_Machine_Model\setup_structure_params");

% Set nominal air gap if it has not already been defined by the parameter script
% (Fallback value: 0.5 mm).
if ~exist("nom_airgap",'var')
    nom_airgap = 0.5;   % Nominal air gap [mm]
end


%% S1: Operating Point Currents via Constrained Optimization
%--------------------------------------------------------------------------
% The system features 8 electromagnets but only 5 degrees of freedom (DOF),
% rendering it mechanically redundant (over-actuated).
% Instead of enforcing artificial symmetry (left = right), we minimize
% ohmic copper losses (propto sum I^2) subject to full force and moment equilibrium.
% This approach also correctly accounts for an asymmetric center of mass.
%--------------------------------------------------------------------------

% Symbolic positive current variables for each of the 8 electromagnets.
% The "positive" constraint reflects physical reality (currents cannot become
% negative in this system configuration).

syms I0_elo I0_elu I0_ero I0_eru positive   % Electromagnets (top/bottom, left/right)
syms I0_slo I0_slu I0_sro I0_sru positive   % Side magnets  (top/bottom, left/right)
vars_I = [I0_elo; I0_elu; I0_ero; I0_eru; I0_slo; I0_slu; I0_sro; I0_sru]; % 8×1

% Convert nominal air gap from millimeters to meters (SI unit for calculation)
airgap0 = nom_airgap * 1e-3;       % [mm] → [m]
% Determine number of electromagnets from labels (defined in setup_structure_params)
nMag = length(elMaglabels);        % Results in 8

% Directional unit vectors of magnetic forces in sled CS (3×8).
% Each column describes the pull direction of the corresponding magnet.
n_vec = [MagSfNVec.ELO, MagSfNVec.ELU, MagSfNVec.ERO, MagSfNVec.ERU, ...
         MagSfNVec.SLO, MagSfNVec.SLU, MagSfNVec.SRO, MagSfNVec.SRU];
% Position vectors from sled GCS to the respective magnet surface GCS (3×8) [m].
% These lever arms are required for torque/moment calculation (r × F).

r_vec = [MagSfGCS2SlGCS.ELO.TV, MagSfGCS2SlGCS.ELU.TV, ...
         MagSfGCS2SlGCS.ERO.TV, MagSfGCS2SlGCS.ERU.TV, ...
         MagSfGCS2SlGCS.SLO.TV, MagSfGCS2SlGCS.SLU.TV, ...
         MagSfGCS2SlGCS.SRO.TV, MagSfGCS2SlGCS.SRU.TV] * 1e-3;  % [mm] → [m]

% Gravitational force acting on the sled [N] (acts at the sled GCS origin)
Fg_num = SledUnitMass * grav;               % 3×1 [N], e.g., [0; 0; -m*g]

% Gravitational torque/moment relative to the sled GCS origin [N·m].
% Arises because the center of mass (CoM) is offset from the GCS origin.
Mg_num = cross(CoM_TV2SlGCS * 1e-3, Fg_num);   % 3×1 [N·m]
fprintf('\nCalculating equilibrium current values:\n')

% --- Cost Function: Deviation of currents from desired operating point current ---
% We minimize sum(delta_I^2) (quadratic deviation), which is equivalent to
% minimizing copper losses when I_target is the optimal rated current.
I_target  = I_work;                             % Desired operating point current, 10 [A]
objective  = @(I) sum((I - I_target).^2);       % Scalar cost function

% --- Nonlinear Equality Constraints: Static Equilibrium ---
% ceq = [F_net; M_net] must be zero (6 equations for 8 unknowns → overdetermined).
nonlcon = @(I) equilibriumConstraints(I, airgap0, n_vec, r_vec, Fg_num, Mg_num);

% Physical current limits (from magnet specification, defined in setup_structure_params)
lb = I_min * ones(nMag, 1);     % Lower bound [A]
ub = I_max * ones(nMag, 1);     % Upper bound [A]

% Initial guess: uniform current distribution as a neutral starting point
I0_guess = 2.0 * ones(nMag, 1); % [A]

% Solver options for fmincon (Sequential Quadratic Programming)
% SQP is highly suitable for nonlinear equality constraints.
options = optimoptions('fmincon', ...
    'Algorithm',              'sqp', ...        % SQP algorithm for NLP
    'Display',                'iter', ...       % Display iteration progress
    'MaxFunctionEvaluations', 10000, ...        % Maximum function evaluations
    'MaxIterations',          1000, ...         % Maximum iterations
    'ConstraintTolerance',    1e-10, ...        % Allowed equilibrium deviation [N, N·m]
    'OptimalityTolerance',    1e-10, ...        % KKT optimality tolerance
    'StepTolerance',          1e-12);           % Minimum step size in parameter space

% Start optimization: no linear inequalities (A,b) or linear equalities (Aeq,beq)
[I_opt, fval, exitflag, output] = fmincon(objective, I0_guess, ...
                                            [], [], ...   % No linear inequalities
                                            [], [], ...   % No linear equalities
                                            lb, ub, ...   % Current limits
                                            nonlcon, ...  % Equilibrium conditions
                                            options);

% Convergence check: exitflag <= 0 indicates no optimum was found
if exitflag <= 0
    warning('fmincon did not converge (exitflag = %d). Check initial guess or boundaries.', exitflag);
    disp(output.message);
end

% Evaluate residual forces and torques at the found operating point.
% For perfect equilibrium, all entries should be close to zero.
[~, ceq_check] = nonlcon(I_opt);
fprintf('\nResidual force  [N]:   [%.2e, %.2e, %.2e]\n', ceq_check(1:3));
fprintf('Residual torque [N*m]:   [%.2e, %.2e, %.2e]\n', ceq_check(4:6));

% Store optimized currents in a structure for later parameter transfer
curVal.I0_elo = I_opt(1);   % Electromagnet top-left
curVal.I0_elu = I_opt(2);   % Electromagnet bottom-left
curVal.I0_ero = I_opt(3);   % Electromagnet top-right
curVal.I0_eru = I_opt(4);   % Electromagnet bottom-right
curVal.I0_slo = I_opt(5);   % Side magnet top-left
curVal.I0_slu = I_opt(6);   % Side magnet bottom-left
curVal.I0_sro = I_opt(7);   % Side magnet top-right
curVal.I0_sru = I_opt(8);   % Side magnet bottom-right

disp(newline + "Levitation state current values (optimized):");
disp(curVal);


%% S2: Linearization of Reluctance Forces around the Operating Point
%--------------------------------------------------------------------------
% Kinematic relationship for small deviations around equilibrium:
%   delta_g_k = n_k^T · (delta_p + delta_alpha × r_k)
%
%   delta_g_k   : Air gap deviation of the k-th magnet [m]
%   n_k        : Unit normal vector of the k-th magnet
%   delta_p     : Translational deviation [dx, dy, 0]^T [m]
%   delta_alpha : Rotational deviation    [dtheta, dphi, dpsi]^T [rad]
%   r_k        : Lever arm from sled GCS to magnet surface [m]
%
% Partial derivatives yield two linearized coefficient matrices:
%   Ks (6×5): Position-force coefficient matrix  ∂F/∂q  — describes negative spring stiffness
%   Ki (6×8): Current-force coefficient matrix   ∂F/∂i  — corresponds to actuator gain
%--------------------------------------------------------------------------

% Symbolic degrees of freedom (delta_z = 0, as axial motion is blocked)
syms x y theta phi psi real          % x, y: translation; theta, phi, psi: rotation (roll, pitch, yaw)
s_dev = [x; y; theta; phi; psi];     % 5×1 deviation vector in state-space

% Symbolic current deviations from operating point (Delta_i = i - I0)
syms i_elo i_elu i_ero i_eru i_slo i_slu i_sro i_sru real
i_dev = [i_elo; i_elu; i_ero; i_eru; i_slo; i_slu; i_sro; i_sru];  % 8×1

% Nominal values: all air gaps identical (equilibrium), currents from S1
ag0    = airgap0 * ones(8,1);        % Nominal air gaps (8×1) [m]
I0_vec = [curVal.I0_elo curVal.I0_elu curVal.I0_ero curVal.I0_eru ...
          curVal.I0_slo curVal.I0_slu curVal.I0_sro curVal.I0_sru]; % 1×8 [A]

% Accumulate symbolic net force and torque on the sled
Fm_dev = [0;0;0];   % Resultant magnetic force   [N],   3×1
KM_dev = [0;0;0];   % Resultant magnetic torque  [N·m], 3×1

for i = 1:8
    mag  = elMaglabels{i};              % Identifier of the i-th magnet

    % Lever arm from sled GCS to the force application plane of the magnet [m]
    pv_i = MagSfGCS2SlGCS.(mag).TV/1000;

    % Normal vector of the i-th magnet (direction of attractive force)
    nv_i = MagSfNVec.(mag);

    % Translational + rotational deviation at the force application point (linearization)
    % The term cross(delta_alpha, r) describes the displacement induced by rotation.
    rp_i = [x; y; 0] + cross([theta; phi; psi], pv_i);
    
    % Projected air gap deviation along the magnet normal
    % (positive = gap decreases → force increases)
    ags_i = nv_i' * rp_i;

    % Perturbed air gap and perturbed current at the i-th magnet
    xs_i = ag0(i) - ags_i;   % Decrease in gap → more force
    is_i = I0_vec(i) + i_dev(i);

    % Individual reluctance force vector (scalar value × direction vector)
    F_i = reluctForce(xs_i, is_i) * nv_i;    % 3×1 [N]

    % Add force and torque to the total vector
    Fm_dev = Fm_dev + F_i;
    KM_dev = KM_dev + cross(pv_i, F_i);      % Torque contribution: r × F
end

% Jacobian matrices of total force and torque w.r.t. position and currents
dFm_ds = jacobian(Fm_dev, s_dev);    % ∂F/∂q  (3×5): Force-position gradient
dFm_di = jacobian(Fm_dev, i_dev);    % ∂F/∂i  (3×8): Force-current gradient
dKM_ds = jacobian(KM_dev, s_dev);    % ∂M/∂q  (3×5): Torque-position gradient
dKM_di = jacobian(KM_dev, i_dev);    % ∂M/∂i  (3×8): Torque-current gradient

% Evaluate all symbolic variables at the operating point (s_dev = 0, i_dev = 0)
old_vars = [s_dev; i_dev];
new_vals = zeros(numel(old_vars), 1);   % Operating point: all deviations = 0
dFm_ds_eq = subs(dFm_ds, old_vars, new_vals);
dFm_di_eq = subs(dFm_di, old_vars, new_vals);
dKM_ds_eq = subs(dKM_ds, old_vars, new_vals);
dKM_di_eq = subs(dKM_di, old_vars, new_vals);

% Combine force and torque terms into total matrices (6×5 and 6×8)
ks_full = double([dFm_ds_eq; dKM_ds_eq]);   % 6×5: [Fx,Fy,Fz,Mx,My,Mz]^T / ∂q
ki_full = double([dFm_di_eq; dKM_di_eq]);   % 6×8: [Fx,Fy,Fz,Mx,My,Mz]^T / ∂i

% Remove z-force row (row 3) — axial motion is blocked (not a DOF)
Ks = ks_full([1 2 4 5 6], :);   % 5×5: active DOFs [x, y, theta, phi, psi]
Ki = ki_full([1 2 4 5 6], :);   % 5×8: active DOFs [x, y, theta, phi, psi]


%% S3: Baseline State-Space Model and LQR Design
%--------------------------------------------------------------------------
% ## S3.1: State-Space Form
%
% State vector:  x = [q; q̇]^T  (10×1)
%   q  = [x, y, θ, φ, ψ]^T  — generalized coordinates (position)  (5×1)
%   q̇  = time derivatives of the position coordinates             (5×1)
%
% Input vector: u = i_dev  (8×1), current deviations from operating point
%
% Linearized equation of motion:
%   M·q̈ = Ks·q + Ki·u
%   → q̈ = M⁻¹·Ks·q + M⁻¹·Ki·u
%
% In state-space form:
%   ẋ = A·x + B·u    with   A = [0, I; M⁻¹Ks, 0],  B = [0; M⁻¹Ki]
%--------------------------------------------------------------------------

dim_s = numel(s_dev);   % Number of position DOFs = 5
dim_u = numel(vars_I);  % Number of current inputs = 8

% --- Mass and Inertia Matrix relative to the sled GCS origin ---
% The off-diagonal terms m·rz_com arise due to the offset of the center of mass (CoM)
% from the GCS origin in the Z-direction, coupling translation and rotation (Newton-Euler).
rz_com = CoM_TV2SlGCS(3) * 1e-3;    % Z-offset of the center of mass [m]
m      = SledUnitMass;              % Total mass of the sled [kg]

% Full mass matrix M (5×5), symmetric positive definite
% Structure: [m, 0, 0, m*rz, 0; 0, m, -m*rz, 0, 0; ...]
MassInertia = [m,         0,          0,                       m*rz_com,                 0;
               0,         m,         -m*rz_com,                0,                        0;
               0,        -m*rz_com,   InertiaTensor_GC(1,1),   InertiaTensor_GC(1,2),    InertiaTensor_GC(1,3);
               m*rz_com,  0,          InertiaTensor_GC(2,1),   InertiaTensor_GC(2,2),    InertiaTensor_GC(2,3);
               0,         0,          InertiaTensor_GC(3,1),   InertiaTensor_GC(3,2),    InertiaTensor_GC(3,3)];

% State-space matrices of the linearized system
A = [zeros(dim_s), eye(dim_s); MassInertia\Ks, zeros(dim_s)];  % 10×10 System matrix
B = [zeros(dim_s, dim_u); MassInertia\Ki];                     % 10×8  Input matrix
C = eye(2*dim_s);                                              % Full state output
D = zeros(2*dim_s, dim_u);                                     % No direct feedthrough

% --- PBH Controllability Test (Popov-Belevitch-Hautus) ---
% A system is controllable if rank([λI-A, B]) = n holds true for all eigenvalues λ.
% Robustly verified here: any unstable poles that are uncontrollable will trigger an abort.
n = size(A, 1);
unc = arrayfun(@(lam) rank([lam*eye(n) - A, B]) < n, unique(eig(A)));
assert(~any(unc), 'Baseline system is not controllable.');
sys = ss(A, B, C, D);   % MATLAB State-Space object for controller design

%--------------------------------------------------------------------------
% ## S3.2: LQR Design for the Baseline System (without Integrator)
%
% Performance index: J = ∫(x'·Q·x + u'·R·u) dt → minimize
%   Q: Penalizes undesired state deviations (position heavier than velocity)
%   R: Penalizes control effort (small R → aggressive control, higher currents)
%
% lqr() solves the continuous-time algebraic Riccati equation and returns the optimal
% feedback gain matrix K, such that u = -K·x stabilizes the closed loop.
%--------------------------------------------------------------------------

Q_pos = diag([2, 2, 2, 2, 2]);            % Position weighting (x, y, θ, φ, ψ)
Q_vel = 1e0 * diag([1, 1, 1, 1, 1]);      % Velocity weighting
Q     = blkdiag(Q_pos, Q_vel);            % Total state weighting matrix (10×10)
R = 2e-6 * eye(dim_u);                    % Control effort weighting (8×8), small → aggressive
K_lqr = lqr(sys, Q, R);                   % Optimal LQR gain matrix (8×10)


%% S4: Integral Extension and Anti-Windup
%--------------------------------------------------------------------------
% ## S4.1: Augmented State Vector with Integrators
%
% Without an integrator, the LQR controller cannot entirely eliminate steady-state 
% tracking errors caused by constant disturbance forces. By appending 
% integrator states z = ∫q dt, the system is converted into a Type-1 controller:
%
%   ẋ_aug = A_aug · x_aug + B_aug · u
%
%   [ q̇   ]   [ 0      I    0 ] [ q   ]   [ 0    ]
%   [ q̈   ] = [ M⁻¹Ks  0    0 ] [ q̇  ] + [ M⁻¹Ki] u
%   [ ż   ]   [ I      0    0 ] [ z   ]   [ 0    ]
%
% Augmented state vector: x_aug = [q; q̇; z]^T (15×1)
%--------------------------------------------------------------------------

dim_z = dim_s;          % One integrator per position DOF (5 integrators)

% System matrix of the augmented system (15×15)
A_aug = [A,             zeros(2*dim_s, dim_z);                      % Physical dynamics
         eye(dim_z),    zeros(dim_z, dim_s), zeros(dim_z, dim_z)];  % Integrator dynamics: ż = q

% Input matrix of the augmented system (15×8)
B_aug = [B;
         zeros(dim_z, dim_u)];      % Inputs do not directly affect the integrators
C_aug = eye(2*dim_s + dim_z);       % Full state output (for design purposes)
D_aug = zeros(2*dim_s + dim_z, dim_u);

% --- PBH Test for the Augmented System ---
% The integrators append 5 poles at s=0, which must be controllable.
n_aug = size(A_aug, 1);
uncontrollable_modes = eig(A_aug);
uncontrollable_modes = uncontrollable_modes( ...
    arrayfun(@(lam) rank([lam*eye(n_aug) - A_aug, B_aug]) < n_aug, ...
             uncontrollable_modes));
assert(isempty(uncontrollable_modes), ...
    'Augmented system: %d uncontrollable mode(n) detected.', ...
    numel(uncontrollable_modes));
fprintf('PBH-Test: CONTROLLABLE — all %d modes are reachable\n', n_aug);
sys_aug = ss(A_aug, B_aug, C_aug, D_aug);


%--------------------------------------------------------------------------
% ## S4.2: LQR for the Augmented System
%
% In addition to Q_pos and Q_vel, Q_int is introduced for the integrator states.
% Higher Q_int values → faster disturbance rejection, but increased overshoot.
% Rotational channels (θ, φ, ψ) require significantly higher weights because
% angular deviations in [rad] are numerically much smaller than
% translational deviations in [m].
%--------------------------------------------------------------------------
Q_int = diag([1e8, 1e8, 1e14, 1e14, 1e14]);    % Integrator weighting (5×5)
Q_aug = blkdiag(Q_pos, Q_vel, Q_int);          % Total augmented system weighting matrix (15×15)
R_aug = R;                                     % Control effort weighting remains as before (8×8)
K_aug = lqr(A_aug, B_aug, Q_aug, R_aug);       % Optimal augmented gain matrix (8×15)
% Extract partial gains from K_aug (for separate downstream application)
K_q    = K_aug(:, 1:dim_s);                    % 8×5: Proportional / position gain
K_qdot = K_aug(:, dim_s+1:2*dim_s);            % 8×5: Derivative / velocity gain
K_z    = K_aug(:, 2*dim_s+1:2*dim_s+dim_z);    % 8×5: Integral gain


%--------------------------------------------------------------------------
% ## S4.3: Anti-Windup Gain (Back-Calculation Method)
%
% Problem: If the actuator constraints (e.g., maximum current saturation) clip
% the control input command, the integrator state z continues to wind up.
%
% Solution: The difference between the saturated and unsaturated control command
%   Δu = u_sat - u_cmd
% is fed back via K_aw into the integrator dynamics:
%   ż = q + K_aw · Δu
%
% K_aw = K_z⁺ (pseudo-inverse) maps the 8-dimensional saturation error back
% onto the 5 integrator states, slowing down or reversing their growth.
%--------------------------------------------------------------------------
K_aw = pinv(K_z);   % 5×8 Anti-windup gain matrix (pseudo-inverse of K_z)
fprintf('Anti-windup gain K_aw: %dx%d\n', size(K_aw));


%% S5: Kinematic Mapping Matrix (Sensor → Degree of Freedom)
%--------------------------------------------------------------------------
% The 8 air gap sensors each measure a scalar clearance change δg_k.
% We require the mapping function onto the 5 generalized coordinates q:
%
%   δg_k = nx·x + ny·y + (-ny·rz)·θ + (nx·rz)·φ + (-nx·ry + ny·rx)·ψ
%
% In matrix form: δg = H · q   (8×5)
%
% Since the system is overdetermined (8 equations, 5 unknowns),
% the pseudo-inverse H⁺ = (H'H)⁻¹H' provides the least-squares estimation:
%   q̂ = H⁺ · δg   (5×1)
%--------------------------------------------------------------------------

H = zeros(dim_u, dim_s);   % Kinematic measurement matrix (8×5), initially empty

for i = 1:numel(elMaglabels)
    mag  = elMaglabels{i};

    % Normal vector of the i-th magnet (direction of sensitive measurement)
    nv_i = MagSfNVec.(mag);

    % Lever arm from sled GCS to the sensor/magnet surface point [m]
    rv_i = MagSfGCS2SlGCS.(mag).TV * 1e-3;

    % Components for the kinematic equation
    nx_mag = nv_i(1);   ny_mag = nv_i(2);           % X and Y components of the normal vector
    rx_mag = rv_i(1);   ry_mag = rv_i(2);  rz_mag = rv_i(3);   % Lever arm components [m]

    % Measurement matrix row: δg_i = f(x, y, θ, φ, ψ) based on linearized kinematics
    H(i,:) = [nx_mag, ny_mag, -ny_mag*rz_mag, nx_mag*rz_mag, ...
              -nx_mag*ry_mag + ny_mag*rx_mag];
end

% Pseudo-inverse: solves the overdetermined system of equations in the least-squares sense (5×8)
H_pinv = pinv(H);

% Verify the accuracy of the kinematic mapping
fprintf('\n=== S5 Kinematic Mapping Verification ===\n')
fprintf('H-Rank: %d (expected %d)\n', rank(H), dim_s);

% Consistency check: H·H⁺·H should be identical to H (a fundamental property of the pseudo-inverse)
fprintf('Maximum reconstruction error: %.2e\n', max(max(abs(H * H_pinv * H - H))));


%% S6: State Observer Design
%--------------------------------------------------------------------------
% Since not all states are directly measurable (particularly the 
% generalized velocities q̇), a Luenberger observer (or Kalman Filter) 
% is deployed to reconstruct the full state vector x̂ = [q̂; q̂̇]^T 
% from the sensor measurements.
%
% Both methods utilize the identical observer structure in Simulink:
%   x̂̇ = A·x̂ + B·u + L·(y - C_obs·x̂)
%
% The sole distinction lies within the observer gain matrix L:
%   L_pp: Pole Placement — deterministic, no noise model
%   L_kf: Kalman Filter  — stochastically optimal, requires noise covariance matrices
%--------------------------------------------------------------------------


% ## S6.1: Observer Measurement Equation
% Sign convention: A smaller air gap → smaller sensor value.
%   y = C_obs · x   →   C_obs = [H, 0]  (position term active, velocity not directly measurable)
C_obs = [H, zeros(dim_u, dim_s)];    % 8×10: only position is observable
D_obs = zeros(dim_u, dim_u);
% Ensure system observability (Kalman Rank Test)
assert(rank(obsv(A, C_obs)) == 2*dim_s, 'System is not observable!');
% Controller poles as a reference for speed comparison with the observer
eig_ctrl     = eig(A - B * K_lqr);         % Baseline controller poles (10 poles)
eig_ctrl_aug = eig(A_aug - B_aug * K_aug); % Augmented controller poles (15 poles)
slowest_ctrl = max(real(eig_ctrl_aug));    % Slowest (least damped) controller pole


%--------------------------------------------------------------------------
% ## S6.2: Method 1 — Pole Placement (for noise-free simulation)
%
% Observer poles are defined by scaling the controller poles.
% Rule of thumb: The observer should be 3–10× faster than the controller 
%                so that the state estimation does not lag behind the control loop.
%
% Advantage:  Simple, predictable speed ratio
% Disadvantage: No noise model, no guarantee of optimality
%--------------------------------------------------------------------------
speed_ratio  = 6;                                   % Observer is 6× faster than the controller
obs_poles_pp = speed_ratio * eig_ctrl;              % Scaled observer poles
L_pp = place(A', C_obs', obs_poles_pp)';            % Pole placement via MATLAB place()


%--------------------------------------------------------------------------
% ## S6.3: Method 2 — Kalman Filter (for noisy simulation / hardware)
%
% The Kalman Filter minimizes the mean squared estimation error 
% by considering process noise w and measurement noise v:
%   ẋ = A·x + B·u + w,   w ~ N(0, Q_kf)
%   y  = C·x     + v,    v ~ N(0, R_kf)
%
% Tuning Guide:
%   R_kf   — directly from sensor data sheet (σ² per channel)
%   Q_kf   — process noise as a design parameter:
%             • q_pos small → position model well known
%             • q_vel large  → unmodeled disturbance accelerations possible
%             • speed_factor → accelerate observer further
%
% The Kalman gain is calculated via LQR duality:
%   L_kf = (lqr(A', C', Q_kf, R_kf))'
%--------------------------------------------------------------------------
% Measurement noise covariance from sensor specification (σ² per channel)
sn_std = sensor_noise_std * 1e-3;   % Sensor noise: e.g., 0.002 mm → [m]
R_kf = sn_std^2 * eye(dim_u);       % 8×8 Measurement noise covariance matrix [m²]

% Process noise covariance (model uncertainty and unmodeled forces)
q_pos        = 1e-4;    % Position well modeled → small process noise
q_vel        = 1e-1;    % Velocity: larger model uncertainty (disturbance forces)
speed_factor = 100;     % Additional gain to push observer further into the LHP

% Diagonal process noise covariance matrix (10×10), split for position and velocity
Q_kf = diag([q_pos * ones(1, dim_s), ...
             q_vel * speed_factor * ones(1, dim_s)]);
% Kalman gain via algebraic Riccati equation (dual LQR problem)
L_kf = lqr(A', C_obs', Q_kf, R_kf)';   % 10×8 Kalman gain matrix
fprintf('Observer Riccati condition number: %.2e\n', cond(L_kf));


%--------------------------------------------------------------------------
% ## S6.4: Comparison of Both Observer Methods and Selection
%--------------------------------------------------------------------------
eig_pp = eig(A - L_pp * C_obs);        % Observer poles (Pole Placement)
eig_kf = eig(A - L_kf * C_obs);        % Observer poles (Kalman Filter)
ratio_pp = max(real(eig_pp)) / slowest_ctrl;    % Speed ratio observer/controller
ratio_kf = max(real(eig_kf)) / slowest_ctrl;
fprintf('\n=== S6 Observer Comparison ===\n');
fprintf('  %-20s  Slowest pole = %8.1f,  Ratio = %.1fx\n', ...
        'Pole Placement:', max(real(eig_pp)), ratio_pp);
fprintf('  %-20s  Slowest pole = %8.1f,  Ratio = %.1fx\n', ...
        'Kalman Filter:',  max(real(eig_kf)), ratio_kf);

% Select method:
%   'pole_placement' → noise-free simulation (fast, deterministic)
%   'kalman'         → noisy simulation or hardware operation (noise-optimal)
observer_method = 'kalman';
switch observer_method
    case 'pole_placement'
        L_obs = L_pp;
    case 'kalman'
        L_obs = L_kf;
end
fprintf('  Selected: %s (Ratio = %.1fx)\n', observer_method, ...
        max(real(eig(A - L_obs * C_obs))) / slowest_ctrl);


%% S7: Closed-Loop Verification
%--------------------------------------------------------------------------
% Performed checks:
%   S7.1 — Pole Analysis (open loop, closed loop, observer)
%   S7.2 — Full Observer-Based Closed Loop (Separation Principle)
%   S7.3 — Open-Loop Transfer Function (Singular Values)
%   S7.4 — MIMO Stability Margins (Sensitivity-Based)
%   S7.5 — Pole-Zero Map
%   S7.6 — Step Disturbance Rejection (Time Domain)
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% ## S7.1: Pole Analysis
% Expectation: Open loop unstable (reluctance forces act like negative springs),
%              Closed loop stable (all poles in the left half-plane).
%--------------------------------------------------------------------------

% Open-loop poles (unstable poles expected since Ks < 0)
eig_OL = eig(A);
fprintf('\n=== Open-Loop Poles ===\n');
fprintf('  %+8.2f %+8.2fj   Stable: %d\n', ...
    [real(eig_OL), imag(eig_OL), real(eig_OL) < 0]');

% Closed-loop poles with integral controller (all must be negative)
eig_CL = eig(A_aug - B_aug * K_aug);
fprintf('\n=== Closed-Loop Poles (State Feedback, Augmented) ===\n');
fprintf('  %+10.4f %+10.4fj   Stable: %d\n', ...
    [real(eig_CL), imag(eig_CL), real(eig_CL) < 0]');
assert(all(real(eig_CL) < 0), 'UNSTABLE poles detected in the closed-loop system!');
fprintf('  All %d poles in the left half-plane (LHP).\n', length(eig_CL));

% Observer poles (must also lie in the LHP)
eig_obs = eig(A - L_obs * C_obs);
fprintf('\n=== Observer Poles ===\n');
fprintf('  %+10.4f %+10.4fj   Stable: %d\n', ...
    [real(eig_obs), imag(eig_obs), real(eig_obs) < 0]');
assert(all(real(eig_obs) < 0), 'UNSTABLE observer poles detected!');

% Speed comparison: Observer must be faster than the controller
bw_ctrl = max(real(eig_CL));    % Slowest controller pole (least negative time constant)
bw_obs  = max(real(eig_obs));   % Slowest observer pole
fprintf('\n  Slowest controller pole:    %.2f\n', bw_ctrl);
fprintf('  Slowest observer pole:       %.2f\n', bw_obs);
fprintf('  Observer/Controller ratio:   %.1fx\n', bw_obs / bw_ctrl);
assert(bw_obs < bw_ctrl, 'The observer is SLOWER than the controller!');

%--------------------------------------------------------------------------
% ## S7.2: Full Closed-Loop System with Observer
%
% The Separation Principle states: The poles of the complete system 
% (controller + observer) are the combined set of both subsystem poles, 
% provided the system is controllable and observable.
%
% Combined state vector: ξ = [x_plant; x̂; z]  (10 + 10 + 5 = 25)
%
% Full system matrix A_full (25×25):
%   [ A     -B·Kx        -B·Kz  ]   [physical plant     ]
%   [ L·C   A-B·Kx-L·C   -B·Kz  ]   [observer           ]
%   [ 0     C_pos           0   ]   [integrator states  ]
%--------------------------------------------------------------------------
K_x   = K_aug(:, 1:2*dim_s);                       % 8×10: Gain for physical states
C_pos = [eye(dim_s), zeros(dim_s, dim_s)];         % 5×10: Extracts q from [q; q̇]
% Full system matrix of the observer-based closed loop (25×25)
A_full = [A,                        -B*K_x,                -B*K_z;
          L_obs*C_obs,   A - B*K_x - L_obs*C_obs,          -B*K_z;
          zeros(dim_z, 2*dim_s),     C_pos,                zeros(dim_z)];
eig_full = eig(A_full);
fprintf('\n=== Full Closed-Loop Poles (Observer-Based) ===\n');
fprintf('  %+10.4f %+10.4fj\n', [real(eig_full), imag(eig_full)]');
assert(all(real(eig_full) < 0), 'UNSTABLE poles in the full closed loop!');
fprintf('  All %d poles stable.\n', length(eig_full));

% Numerically verify the Separation Principle:
% The sorted eigenvalues of A_full must match [eig_CL; eig_obs].
eig_expected = sort([eig_CL; eig_obs]);    % Expected poles from subsystems
eig_actual   = sort(eig_full);             % Actual poles of the total system
sep_error = norm(eig_expected - eig_actual);
fprintf('  Separation Principle error: %.2e (should tend toward ~eps)\n', sep_error);

%--------------------------------------------------------------------------
% ## S7.3: Open-Loop Transfer Function — Singular Value Plot
%
% For MIMO systems, the singular value plot replaces the Bode plot.
% The transfer function L(s) = C(s)·P(s) of the opened loop provides 
% insights into control bandwidth and robustness.
%
% Controller State-Space: Input y (8×1) → Output u (8×1)
%   State = [x̂; z]  (15×1, observer + integrators)
%--------------------------------------------------------------------------
A_ctrl = [A - L_obs*C_obs - B*K_x,   -B*K_z;
          C_pos,                     zeros(dim_z)];  % 15×15 Controller system matrix
B_ctrl = [L_obs;
          zeros(dim_z, dim_u)];                      % 15×8: Input = measurement error L·ỹ
C_ctrl = [-K_x,  -K_z];                              % 8×15: Output = target current deviation
D_ctrl = zeros(dim_u, dim_u);                        % No direct feedthrough in the controller
sys_ctrl  = ss(A_ctrl, B_ctrl, C_ctrl, D_ctrl);      % Controller transfer function
sys_plant = ss(A, B, C_obs, D_obs);                  % Plant transfer function

% Open loop: L(s) = C(s) · P(s) in the MIMO sense
sys_L = sys_ctrl * sys_plant;   % 8×8 Open loop transfer matrix
figure('Name', 'Singular Values');
sigma(sys_L);
grid on;
title('Open-Loop Transfer Function — Singular Values');
yline(0, 'r--', '0 dB');

%--------------------------------------------------------------------------
% ## S7.4: MIMO Stability Margins (Sensitivity-Based)
%
% For coupled MIMO systems, channel-specific SISO metrics 
% (gain margin, phase margin) are not sufficiently informative.
% Instead, the maximum sensitivity ||S||_∞ is utilized:
%
%   S(s) = (I + L(s))⁻¹   — Sensitivity function (disturbance transmission)
%   T(s) = L(s)·(I + L)⁻¹ — Complementary sensitivity (tracking transmission)
%
% From ||S||_∞ = Ms, guaranteed simultaneous stability margins can be derived:
%   GM ≥ Ms/(Ms−1),  PM ≥ 2·arcsin(1/(2·Ms))
%--------------------------------------------------------------------------
sys_I = ss(zeros(dim_u), zeros(dim_u), zeros(dim_u), eye(dim_u));  % Identity matrix I (8×8)
sys_S = feedback(sys_I, sys_L);           % S = (I + L)⁻¹
sys_T = feedback(sys_L, sys_I);           % T = L·(I + L)⁻¹
% Calculate the peak singular value of the sensitivity function
[sv_S, ~] = sigma(sys_S);
peak_S    = max(sv_S(1,:));               % ||S||_∞ (linear)
peak_S_dB = 20*log10(peak_S);             % ||S||_∞ in dB
fprintf('\n=== MIMO Robustness Metrics ===\n');
fprintf('  Maximum sensitivity ||S||_inf: %.2f dB (%.2f)\n', peak_S_dB, peak_S);
if peak_S_dB < 6
    fprintf('  Acceptable (< 6 dB)\n');
else
    fprintf('  Too high — reduce controller bandwidth or increase R\n');
end

% Guaranteed MIMO stability margins from the sensitivity peak condition
GM_mimo = peak_S / (peak_S - 1);          % Guaranteed gain margin [linear]
PM_mimo = 2 * asind(1/(2*peak_S));        % Guaranteed phase margin [degrees]
fprintf('\n  Guaranteed MIMO Gain Margin:  %.1f dB\n', 20*log10(GM_mimo));
fprintf('  Guaranteed MIMO Phase Margin: %.1f degrees\n', PM_mimo);
figure('Name', 'Sensitivity Analysis');
subplot(2,1,1);
sigma(sys_S);
hold on; yline(6, 'r--', '6 dB Boundary'); hold off;
grid on;
title('Sensitivity S(s) — Lower implies better disturbance rejection');
subplot(2,1,2);
sigma(sys_T);
grid on;
title('Complementary Sensitivity T(s) — Bandwidth and noise amplification');

% Return Difference: σ_min(I + L) > 1 is a necessary condition for stability
figure('Name', 'Return Difference');
sys_F = sys_I + sys_L;
sigma(sys_F);
hold on; yline(0, 'r--', '0 dB (Stability Margin)'); hold off;
grid on;
title('Return Difference I + L(s) — Minimum of \sigma must remain above 0 dB');

%--------------------------------------------------------------------------
% ## S7.5: Pole-Zero Map
% Visualizes the pole shift due to the controller and the observer poles.
%--------------------------------------------------------------------------
figure('Name', 'Pole-Zero Map');
plot(real(eig_CL), imag(eig_CL), 'bx', 'MarkerSize', 12, 'LineWidth', 2);  % Controller poles
hold on;
plot(real(eig_obs), imag(eig_obs), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % Observer poles
plot(real(eig_OL),  imag(eig_OL),  'ks', 'MarkerSize', 10, 'LineWidth', 1.5); % OL poles
xline(0, 'k--');    % Imaginary axis = stability boundary
grid on;
legend('Closed Loop (Controller)', 'Observer', 'Open Loop', 'Location', 'best');
title('Pole Distribution');
xlabel('Real Part'); ylabel('Imaginary Part');
hold off;

%--------------------------------------------------------------------------
% ## S7.6: Step Disturbance Rejection (Time-Domain Verification)
%
% Test Scenarios: Applying a unit step disturbance force (1 N) in the Y-direction.
% The integral action of the controller must regulate the steady-state tracking error to zero.
%
% Full system dynamics for disturbance analysis:
%   ξ̇ = A_full · ξ + B_full_d · d_disturb
%   y  = C_full_q · ξ   (position output only)
%--------------------------------------------------------------------------
Bd = zeros(2*dim_s, 1);
Bd(dim_s + 2) = 1/m;           % 1 N force acting on Y-acceleration: a_y = F/m [m/s²]

% Disturbance input for the full 25-dimensional state vector
B_full_d = [Bd; zeros(10, 1); zeros(dim_z, 1)];    % 25×1: acts strictly on plant states

% Output matrix: extracts only the 5 position coordinates from ξ
C_full_q = [C_pos, zeros(dim_s, 10), zeros(dim_s, dim_z)];   % 5×25

% Time-domain simulation using step() (unit step at the disturbance input)
sys_dist = ss(A_full, B_full_d, C_full_q, zeros(dim_s, 1));
figure('Name', 'Disturbance Rejection');
[y_step, t_step] = step(sys_dist, 2);   % Simulation over 2 seconds
plot(t_step, y_step * 1e3);             % Display in mm instead of m
grid on;
xlabel('Time [s]');
ylabel('Position Deviation [mm]');
title('Rejection of a Step Disturbance (1 N in Y-Direction)');
legend({'x','y','Roll','Pitch','Yaw'}, 'Location', 'best');

% Verify residual steady-state tracking error after settling (t → ∞)
fprintf('\n  Steady-state tracking error after disturbance: %.2e mm\n', ...
    max(abs(y_step(end,:))) * 1e3);


%% S8: Export of Design Parameters
%--------------------------------------------------------------------------
% ## S8.1: Save Controller and Observer Parameters as .mat Files
%
% ssc_params — all variables relevant to the controller
% obs_params — all variables required for the observer in Simulink
%--------------------------------------------------------------------------
ssc_params.dim_s       = dim_s;         % Number of rigid-body DOFs (5)
ssc_params.dim_x       = 2*dim_s;       % Total state dimension (10)
ssc_params.dim_u       = dim_u;         % Number of inputs (8)
ssc_params.nom_airgap  = nom_airgap;    % Nominal air gap [mm]
ssc_params.nom_curval  = curVal;        % Optimized operating point currents (struct)
ssc_params.nom_curvec  = I0_vec;        % Operating point currents as a vector [A] (1×8)
ssc_params.K_base      = K_lqr;         % LQR baseline gain matrix (8×10)
ssc_params.K_aug       = K_aug;         % Augmented LQR gain matrix with integral action (8×15)
ssc_params.K_aw        = K_aw;          % Anti-windup gain matrix (5×8)

obs_params.A_sys       = A;             % System matrix of the linearized model (10×10)
obs_params.B_sys       = B;             % Input matrix (10×8)
obs_params.C_obs       = C_obs;         % Observer measurement matrix (8×10)
obs_params.L_obs       = L_obs;         % Selected observer gain matrix (10×8)

% Create target directory if it does not exist yet
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
% ## S8.2: Simulink Bus Definition for Observer Parameters
%
% For Simulink integration, the observer matrices are grouped into a
% Bus object. Simulink can use this Bus object as a structured input/output 
% interface for a subsystem block.
%--------------------------------------------------------------------------
obs_Bus = Simulink.Bus;

% Bus elements: Name of the matrix, its dimension, and data type
elems = {
    'A_sys',  [2*dim_s, 2*dim_s];   % 10×10 System matrix
    'B_sys',  [2*dim_s,   dim_u];   % 10×8  Input matrix
    'C_obs',  [  dim_u, 2*dim_s];   % 8×10  Measurement matrix
    'L_obs',  [2*dim_s,   dim_u];   % 10×8  Observer gain matrix
};

for k = 1:size(elems,1)
    e = Simulink.BusElement;
    e.Name       = elems{k,1};          % Field name inside the bus
    e.Dimensions = elems{k,2};          % Matrix dimensions
    e.DataType   = 'double';            % Floating point data (64-bit double)
    obs_Bus.Elements(k) = e;
end

save(fullfile(paramsFolder,'obs_params_Bus.mat'), 'obs_Bus');


%% Summary
fprintf("\n===============================================================================\n")
fprintf("[INFO] Control design completed successfully.\n")
fprintf("[INFO] Controller and observer parameters have been saved under 'Controller_Params'.\n")
fprintf("===============================================================================\n")

cd (paramsFolder)

%% Helper Functions
function magForce = reluctForce(xs, is)
% RELUCTFORCE  Computes the scalar value of the reluctance force of an electromagnet.
%
%   magForce = reluctForce(xs, is)
%
%   Inputs:
%     xs       — Current air gap [m] (positive = open gap)
%     is       — Coil current [A]
%
%   Output:
%     magForce — Magnetic attraction force [N] (always positive, direction via normal vector)
%
%   Physical Model (simplified reluctance formula):
%     F = Km · i² / x²   with   Km = μ₀ · A_eff · N² / 2
%
%   Note: This formula assumes a homogeneous magnetic field inside the air gap and
%         neglects magnetic stray flux and saturation effects.

    % Magnetic constant Km [N·m²/A²]
    Km = ElectromagnetConfig.MagConst;
    
    % Reluctance force: F = Km · i² / x² [N]
    magForce = Km * is^2 / xs^2;
end



function [c, ceq] = equilibriumConstraints(I, airgap0, n_vec, r_vec, Fg, Mg)
% EQUILIBRIUMCONSTRAINTS  Nonlinear equality constraints for fmincon optimization.
%
%   [c, ceq] = equilibriumConstraints(I, airgap0, n_vec, r_vec, Fg, Mg)
%
%   Computes the force and torque equilibrium residuals and returns them
%   as equality constraints ceq to fmincon (requires ceq = 0 at convergence).
%
%   Inputs:
%     I        — Current vector of the 8 magnets [A] (8×1)
%     airgap0  — Nominal air gap [m] (scalar, identical for all magnets)
%     n_vec    — Normal vectors of the magnets (3×8)
%     r_vec    — Lever arms from GCS to the magnet face surface (3×8) [m]
%     Fg       — Gravitational force acting on the carriage (3×1) [N]
%     Mg       — Gravitational torque in the GCS origin (3×1) [N·m]
%
%   Outputs:
%     c        — Empty: no inequality constraints present
%     ceq      — Equilibrium residuals: [F_net; M_net] (6×1)
%                Static equilibrium ⟺ ceq = 0

    F_net = Fg;     % Initialize with gravitational force
    M_net = Mg;     % Initialize with gravitational torque
    
    for k = 1:length(I)
        % Magnetic force vector of the k-th magnet (Scalar × Normal vector)
        Fk    = reluctForce(airgap0, I(k)) * n_vec(:,k);   % 3×1 [N]
        
        % Force contribution to the net equilibrium
        F_net = F_net + Fk;
        
        % Torque contribution (cross product: r × F)
        M_net = M_net + cross(r_vec(:,k), Fk);             % 3×1 [N·m]
    end
    
    ceq = [F_net; M_net];   % 6×1: Equilibrium requires F_net = 0 and M_net = 0
    c   = [];               % No inequality constraints defined
end