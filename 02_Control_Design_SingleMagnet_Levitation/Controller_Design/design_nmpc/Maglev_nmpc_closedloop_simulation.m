%%#########################################################################
%
%  FILE:        Maglev_nmpc_closedloop_simulation.m   
%  PROJECT:     NMPC for an Electromagnet System
%  DESCRIPTION: Validation script for the acados OCP solver.
%               Calculates the optimal control input (current) to
%               stabilize the air gap at the reference value.
%
%%#########################################################################

clc; clear; close all;

%% Variable Check and Initialization
% Load simulation parameters if they do not exist in the workspace yet.
if ~exist('mass', 'var') || ~exist('airgap_soll', 'var')
    run('../../setup_sim_params.m');
end

%% Step 1 — Create Solver Instance
% Generates a new instance of the NMPC solver (without Simulink block generation).
sim_flag = true;
Maglev_nmpc_solver_creator;
clearvars sim_flag;

%% Step 2 — Set Runtime Parameters Across All Horizon Stages
% Parameter vector p = [Km; mass; equilibrium air gap; gravitational acceleration]
p_val = [Km; mass; airgap_eq; grav];
for stage = 0:N
    nmpc_solver.set('p', p_val, stage);
end
fprintf('Parameters successfully set.\n')

%% Step 3 — Define Reference Values for the Cost Function
% Equilibrium current i_eq: exactly compensates the gravitational force at air gap s0.
i_eq = airgap_eq * sqrt(mass * grav / Km);
fprintf('Equilibrium current i_eq = %.4f A\n', i_eq)
% Path cost reference: state deviation = 0, control input = i_eq
for k = 0:N-1
    nmpc_solver.set('cost_y_ref', [0; 0; i_eq], k);
end
% Terminal cost reference: states only (no control input in the terminal term)
nmpc_solver.set('cost_y_ref_e', [0; 0], N);
fprintf('References successfully set.\n')

%% Step 4 — Initialize Trajectory (Warm Start)
% Initial air gap from Simscape initialization (Conversion: mm → m)
xs_init = airgap_init * 1e-3;
x_dev_0 = airgap_eq - xs_init;  % Position deviation from equilibrium point [m]
x0      = [x_dev_0; 0];         % Initial state vector: [deviation; velocity]
% Physically meaningful warm start:
%   State trajectory: ball remains at the starting position
%   Control input trajectory: equilibrium current is applied constantly
x_traj_init = repmat(x0,   1, N+1);
u_traj_init = repmat(i_eq, 1, N);
nmpc_solver.set('init_x', x_traj_init);
nmpc_solver.set('init_u', u_traj_init);
fprintf('Trajectory initialized.\n')

%% Step 5 — Set Initial State as Hard State Constraint
% Fixes the measured state as the unalterable starting point of the OCP.
nmpc_solver.set('constr_x0', x0);

%% Step 6 — Single Optimizer Call (Offline Validation)
% Solves the OCP and outputs the first optimal control input and the predicted
% next state.
nmpc_solver.solve();
status = nmpc_solver.get('status');
fprintf('Solver status = %d\n', status)
if status == 0
    u_opt = nmpc_solver.get('u', 0);  % First optimal control input [A]
    x1    = nmpc_solver.get('x', 1);  % Predicted state at time step k+1
    fprintf('SUCCESS: Optimal solution found.\n')
    fprintf('  u_opt (Target Current) = %.4f A\n',       u_opt)
    fprintf('  x1    (Next State)     = [%.6f m, %.4f m/s]\n', x1(1), x1(2))
else
    fprintf('ERROR: Solver could not find a solution (Status %d).\n', status)
    disp(nmpc_solver.get('status'))
end

%% Step 7 — Closed-Loop Simulation
% Time parameters
Ts_mpc  = 0.005;             % MPC sampling time [s]
Ts_sim  = 0.0001;            % Integration step size for stiff dynamics [s]
n_inner = Ts_mpc / Ts_sim;   % Number of RK4 steps per MPC interval
t_end   = 3.0;               % Simulation end time [s]
n_steps = round(t_end / Ts_mpc);  % Total number of control steps
% Initialize logging vectors
log_t      = zeros(1, n_steps);  % Time vector
log_xs     = zeros(1, n_steps);  % Absolute air gap [m]
log_xdev   = zeros(1, n_steps);  % State deviation [m]
log_vel    = zeros(1, n_steps);  % Velocity [m/s]
log_u      = zeros(1, n_steps);  % Applied current [A]
log_status = zeros(1, n_steps);  % Solver status per step
% Initialize current state and control input
x_cur     = x0;
u_applied = 0;  % Conservative: initial current = 0 A (consistent with warm start)
for k = 1:n_steps
    %-- State feedback: pass measured state as hard constraint
    nmpc_solver.set('constr_x0', x_cur);
    %-- Solve OCP and extract optimal control input
    nmpc_solver.solve();
    status    = nmpc_solver.get('status');
    u_applied = nmpc_solver.get('u', 0);
    %-- Error handling: fall back to equilibrium current on solver error
    if status ~= 0
        fprintf('Solver error (Status %d) in step %d — x_dev=%.5f m, v=%.4f m/s\n', ...
                status, k, x_cur(1), x_cur(2))
        u_applied = i_eq;
    end
    %-- Control saturation: clamp current to allowable range [I_min, I_max]
    u_applied = max(I_min, min(I_max, u_applied));
    %-- Log state variables
    xs_cur      = airgap_eq - x_cur(1);  % Absolute air gap [m]
    log_t(k)      = (k-1) * Ts_mpc;
    log_xs(k)     = xs_cur;
    log_xdev(k)   = x_cur(1);
    log_vel(k)    = x_cur(2);
    log_u(k)      = u_applied;
    log_status(k) = status;
    %-- Print first 10 steps to console for diagnostics
    if k <= 10
        fprintf('Step %2d: x_dev=%8.5f m  v=%7.4f m/s  i=%6.4f A  xs=%.4f mm\n', ...
                k, x_cur(1), x_cur(2), u_applied, xs_cur*1e3)
    end
    %-- Integrate plant forward (RK4 with fine step size)
    x_sim = x_cur;
    for j = 1:n_inner
        x_sim = rk4_maglev(x_sim, u_applied, Ts_sim, Km, mass, airgap_eq, grav);
        % Safety abort: ball hits magnet during integration
        if airgap_eq - x_sim(1) <= 1e-5
            fprintf('WARNING: Ball hits magnet within RK4 (Step %d)!\n', k)
            break
        end
    end
    x_cur = x_sim;
    %-- Safety check after each control step
    xs_next = airgap_eq - x_cur(1);
    if xs_next <= 1e-5
        fprintf('ABORT: Ball hits magnet in step %d (xs = %.6f m).\n', k, xs_next)
        n_steps = k; break
    end
    if xs_next > 0.05
        fprintf('ABORT: Ball out of range in step %d (xs = %.4f m).\n', k, xs_next)
        n_steps = k; break
    end
end

%% Step 8 — Visualize Results
figure('Name', 'Closed-Loop NMPC — Validation')
subplot(3,1,1)
plot(log_t(1:n_steps), log_xs(1:n_steps)*1e3, 'b-', 'LineWidth', 1.5)
hold on
yline(airgap_eq*1e3,                        'r--', 'Setpoint',     'LineWidth', 1.2)
yline(ElectromagnetConfig.minSafeAirGap*1e3, 'k:',  'Min. Air Gap', 'LineWidth', 1.0)
ylabel('Air Gap [mm]'); grid on; title('Air Gap')
subplot(3,1,2)
plot(log_t(1:n_steps), log_u(1:n_steps), 'g-', 'LineWidth', 1.5)
hold on
yline(i_eq,  'r--', 'i_{eq}', 'LineWidth', 1.2)
yline(I_max, 'k:',  'I_{max}','LineWidth', 1.0)
ylabel('Current [A]'); grid on; title('Control Input (Current)')
subplot(3,1,3)
plot(log_t(1:n_steps), log_status(1:n_steps), 'r.', 'MarkerSize', 8)
ylabel('Status'); ylim([-0.5 5]); grid on
title('Solver Status (0 = Success)'); xlabel('Zeit [s]')
% Summary in the console
fprintf('\nFinal Air Gap:    %.4f mm  (Setpoint: %.4f mm)\n', ...
        log_xs(n_steps)*1e3, airgap_eq*1e3)
fprintf('Solver Errors:    %d / %d Steps\n', ...
        sum(log_status(1:n_steps) ~= 0), n_steps)



%% Helper Function: RK4 Integrator for the Maglev System
function x_next = rk4_maglev(x, u, dt, Km, mass, s0, g)
    % System dynamics: ẋ = [v; (Km/m)*u²/(s0-z)² - g]
    % Protection against division by zero at minimum air gap
    f  = @(xx) [xx(2); (Km/mass) * u^2 / max(s0 - xx(1), 1e-6)^2 - g];
    k1 = f(x);
    k2 = f(x + dt/2 * k1);
    k3 = f(x + dt/2 * k2);
    k4 = f(x + dt   * k3);
    x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end