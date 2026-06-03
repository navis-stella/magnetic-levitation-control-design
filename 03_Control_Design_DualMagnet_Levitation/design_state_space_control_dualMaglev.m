% #########################################################################
%
%   FILE:       design_state_space_control_dualMaglev.m
%   PROJECT:    Double Electromagnet Levitation System (Push-Pull Configuration)
%   FUNCTION:   Calculation of the operating point (fmincon), linearization of 
%               the net force, and design of the State-Space Control (SSC).
%
%   NOTE:       This script requires physical parameters from the 
%               base setup (setup_double_magnet_system.m).
%
% #########################################################################

%% --- Environment Check ---
if ~exist('AirGapUpper_soll', 'var') || ~exist('SlideMass', 'var')
    fprintf('>> [INFO]: Base parameters missing. Running setup script...\n');
    run('setup_dualMaglev_system.m'); 
end

%% 1. Configuration of Equilibrium Setpoints
% Definition of target air gaps [mm]
airgap_up_soll = AirGapUpper_soll;  % [mm]
airgap_lw_soll = PJ_max_route - airgap_up_soll;

% Conversion to SI units [m]
x_up0 = airgap_up_soll * 1e-3;  % [mm] --> [m]
x_lw0 = airgap_lw_soll * 1e-3;  % [mm] --> [m]

% Adjustable minimum current per coil [A]
minCurrent = 2;

%% 2. Operating Point Calculation via fmincon
% Objective: Minimize copper losses while maintaining force equilibrium.

% --- Numerical Optimization with fmincon ---
% Objective function: Minimize the sum of squared currents (proportional to power loss)
objFun = @(i) i(1)^2 + i(2)^2;

% Nonlinear constraint: F_net - F_gravity = 0
% F_net = F_upper - F_lower
nonlcon = @(i) deal([], Km*(i(1)^2 / x_up0^2) - Km*(i(2)^2 / x_lw0^2) - SlideMass*grav);

% Boundary conditions [i_upper; i_lower]
lb = [minCurrent; minCurrent]; % Avoiding unstable regions for i -> 0
ub = [I_max; I_max];

% --- Adaptive Initial Guess ---
% Calculate a plausible bias current to accelerate convergence
a_coeff = Km / x_up0^2;
b_coeff = Km / x_lw0^2;
gravity = SlideMass * grav;

% Adaptive Bias Current (works for both symmetric and asymmetric cases)
i_bias_approx = sqrt(gravity / (4 * min(a_coeff, b_coeff))) + minCurrent + 0.5; % +0.5 A safety margin

% Small differential current for the start (to approximate initial net force)
i_delta_approx = 1.0;

% Assemble start values and ensure they remain within bounds
iUpper_start = min(max(i_bias_approx + i_delta_approx, minCurrent), I_max - 1);
iLower_start = min(max(i_bias_approx - i_delta_approx, minCurrent), I_max - 1);
i0_start = [iUpper_start; iLower_start];

disp(['→ Adaptive start value chosen: i_start = [' num2str(i0_start(1), '%.2f') '; ' num2str(i0_start(2), '%.2f') '] A'])

% Optimization options (sqp algorithm = stable and fast)
options = optimoptions('fmincon', ...
                       'Display', 'none', ...
                       'Algorithm', 'sqp', ...
                       'OptimalityTolerance', 1e-9, ...
                       'ConstraintTolerance', 1e-8, ...
                       'StepTolerance', 1e-10);

% Call fmincon (requires Optimization Toolbox)
[iSol, powerVal, exitflag] = fmincon(objFun, i0_start, [], [], [], [], lb, ub, nonlcon, options);

% Check if a valid solution was found
if exitflag <= 0
    error('fmincon: No valid solution for equilibrium currents found. Please reduce minCurrent or change the target air gap.');
end

i_up0 = iSol(1);
i_lw0 = iSol(2);
i_bias   = (i_up0 + i_lw0)/2;    % Bias current (common-mode component)
i_delta0 = (i_up0 - i_lw0)/2;    % Differential current (for net force)

disp("iUpper0 = " + num2str(i_up0, '%.4f') + " A")
disp("iLower0 = " + num2str(i_lw0, '%.4f') + " A")
disp("Minimized power loss (proportional) = " + num2str(powerVal, '%.2f'))

%% 3. Linearization around the Operating Point
% The nonlinear model is linearized around the equilibrium:
% m*ddx = kx*x + ki*i_dev
% Deviations: x (Position, positive upwards), i (Current deviation)

syms xs is
Fm = symfun(Km * is^2/xs^2, [xs,is]);
syms x_up x_lw i_dev

% F_net = Km * ( i_dev + i_up0)^2 / (x_up0 - x_up)^2 - ...
%          Km * (-i_dev + i_lw0)^2 / (x_lw0 - x_lw)^2;
F_net_sym = Fm(x_up0 - x_up, i_dev + i_up0) - Fm(x_lw0 - x_lw, -i_dev + i_lw0);

% Partial derivatives with respect to position (x) and current deviation (i)
kx_up = double(subs(diff(F_net_sym, x_up), [x_up, x_lw, i_dev], [0, 0, 0]));
kx_lw = double(subs(diff(F_net_sym, x_lw), [x_up, x_lw, i_dev], [0, 0, 0]));
ki    = double(subs(diff(F_net_sym, i_dev), [x_up, x_lw, i_dev], [0, 0, 0]));

% Effective stiffness for 1D translation (x_lw = -x_up)
kx = kx_up - kx_lw;

fprintf('>> [INFO]: Linearization complete.\n   kx = %.2f N/m, ki = %.2f N/A\n', kx, ki);

%% 4. State-Space Controller Design (SSC)
% Augmented system [Position; Velocity; Integral Error]
A_aug = [0, 1, 0; ...
         kx / SlideMass, 0, 0; ...
         1, 0, 0];
B_aug = [0; ki / SlideMass; 0];
C_aug = [1, 0, 0];

% Check controllability
isCtrb = rank(ctrb(A_aug,B_aug)) == size(A_aug,1);
fprintf('Controllable: %s\n', string(rank(ctrb(A_aug,B_aug)) == size(A_aug,1)));

% --- Method 1: Pole Placement ---
s_poles = [-50, -80+80i, -80-80i];
K_aug = place(A_aug, B_aug, s_poles);

% --- Method 2: LQR (Optimal Controller) ---
% Weighting matrices (Adjust for desired behavior)
Q = diag([1e4, 1e2, 1e4]);  % High weighting on position and integral error
R = 100;                    % Penalty for control effort

% Calculate LQR gain (requires Control System Toolbox)
[K_lqr, ~, ~] = lqr(A_aug, B_aug, Q, R);

% Export parameters to SSC structure
SSC.K_aug = K_aug;
SSC.K_lqr = K_lqr;
SSC.x_up0 = x_up0;
SSC.x_lw0 = x_lw0;
SSC.i_up0 = i_up0;
SSC.i_lw0 = i_lw0;
SSC.kx    = kx;
SSC.ki    = ki;

%% DATA STORAGE & SUMMARY
save('DualMagnetControllerData.mat', 'SSC');

fprintf('\n%s\n', repmat('=', 1, 80));
fprintf('>> [SUCCESS]: State-space controller (Pole & LQR) successfully designed.\n');
fprintf('>> [INFO]: Controller parameters saved in "DoppelMagnetControllerData.mat".');
fprintf('\n%s\n', repmat('=', 1, 80));

%% Kalman Filter Design for State Estimation (Optional)
% A_red = [0 1; kx/Mass 0];
% B_red = [0; ki/Mass];
% C_red = [1 0];
% 
% Ts = 1e-4;  % Sampling time [s]
% sys_red_c = ss(A_red, B_red, C_red, 0);
% sys_red_d = c2d(sys_red_c, Ts, 'zoh');
% A_d = sys_red_d.A;
% B_d = sys_red_d.B;
% C_d = sys_red_d.C;
% 
% Q_k = diag([1e-3, 1e-2]);  % [x_m; v]
% R_k = 1e-1;
% P0 = diag([1e-3, 1e-1]);