# 7. Nonlinear Model Predictive Control

The controllers in [§2](02_pid.md)–[§6](06_sliding_mode_control.md) share a common design philosophy: derive a closed-form control law (via linearization, exact cancellation, Lyapunov recursion, or sliding-mode reasoning) and tune its parameters offline. **Nonlinear Model Predictive Control (NMPC)** is a structurally different approach: at every sampling instant, solve a small constrained optimal-control problem over a finite prediction horizon, apply only the first input, and re-solve at the next instant. Constraints and trajectory shaping enter the design directly through the optimization rather than being retrofitted via saturation or anti-windup.

NMPC is documented in its own chapter (rather than included in the [§10](10_results.md) comparison) because the standard formulation used here exhibits a residual steady-state error against the realistic Simscape plant, which would make a direct comparison against the integral-action controllers misleading. The natural remedy — offset-free MPC — is outside the current scope and discussed as future work.

## 7.1 Why NMPC for the Maglev System
The maglev plant has nonlinear dynamics, hard input constraints (amplifier current bounds), and physical state limits (the air gap cannot be negative). The closed-form controllers handle these constraints externally — via saturation, clamping, or anti-windup — meaning the controller is operating outside its design region whenever a constraint is hit. NMPC **incorporates the constraints directly into the optimization problem**, so the commanded input is admissible by construction and the controller's optimality guarantee survives at the constraint boundary. The trade-off is computational cost: instead of evaluating a closed-form expression, the controller solves a small nonlinear program at every sample.

## 7.2 Problem Formulation
At each sampling instant, the controller solves

$$
\min_{u_0, \ldots, u_{N-1}} \sum_{k=0}^{N-1}\Big(\mathbf{x}_k^\top Q  \mathbf{x}_k + u_k^\top R  u_k\Big) + \mathbf{x}_N^\top Q_N  \mathbf{x}_N
$$

subject to

$$
\mathbf{x}_{k+1} = f_d(\mathbf{x}_k, u_k),\qquad
\mathbf{x}_0 = \hat{\mathbf{x}}(t),\qquad
u_{\min} \leq u_k \leq u_{\max}
$$

where:
- $\mathbf{x}_k = [x_1, x_2]^\top$ — position-velocity state
- $u_k = i_s$ — physical current command
- $f_d$ — discretized nonlinear plant
- $N$ — prediction horizon
- $u_{\min} \leq u_k \leq u_{\max}$ — input constraint
- $Q$, $R$, $Q_N$ — stage- and terminal-cost weight matrices

The receding-horizon principle is standard: at each sample, solve the OCP, apply $u_0^\star$, then re-solve at the next sample with the updated initial state.

## 7.3 Implementation via acados
The OCP is solved with **acados**, a real-time-capable nonlinear MPC library with MATLAB and Simulink interfaces. acados supports symbolic definition of the nonlinear dynamics, structured QP and SQP-RTI solvers tuned for embedded real-time use, and direct compilation of the OCP solver to a C-MEX S-function consumable by Simulink.

The design pipeline in `Controller_Design/design_nmpc/` follows the standard acados workflow:

1. Define the symbolic plant model.
2. Construct an `AcadosOcp` object specifying the cost matrices, horizon, sampling time, constraints, and solver options.
3. Generate C code for the OCP solver.
4. Compile to S-functions using **`make_sfun`** (for the controller) and **`make_sfun_sim`** (for the integrator used as the prediction plant in the AcadosSim variant).

The numerical parameters (cost weights, horizon, sample time, constraint bounds) are saved to `Controller_Params/StdNMPCData.mat`. For new users, the official acados Simulink examples are a good starting point.

## 7.4 Two Implementation Variants
- **`Maglev_NMPC_AcadosSim.slx`** — uses acados's internal integrator (`acados_sim_solver`) as the simulation plant. Prediction model and plant model are *identical*: no model mismatch. The closed-loop state converges asymptotically to the origin.
- **`Maglev_NMPC_Simscape.slx`** — uses the full Simscape Magnetic plant from folder `00_Shared_Library`, with the same parametric uncertainties and external disturbances as the other controllers. Under this realistic mismatch, the closed-loop converges to a small **non-zero** steady-state offset.

The discrepancy between the two variants is exactly the plant–model mismatch that standard NMPC cannot absorb on its own.

## 7.5 Known Limitation and Recommended Extension
Standard NMPC implicitly assumes the prediction model matches the plant. When it does (AcadosSim), tracking is exact. When it does not (Simscape, and any real hardware deployment), a residual offset remains. The closed-form controllers of [§2](02_pid.md)–[§6](06_sliding_mode_control.md) carry an integral state that absorbs constant disturbances and steady model errors. NMPC needs an equivalent mechanism, and the standard remedy is **offset-free MPC**: augment the prediction model with a constant-disturbance state, estimate that state online with a Kalman filter on the augmented model, and feed it into both the cost and the constraint terms of the OCP.

The implementation effort is non-trivial — re-deriving the OCP, redesigning the disturbance observer, and recompiling the S-functions — and is documented here as the **recommended next step** rather than included in the current scope.

For the current scope, the standard NMPC is documented as a working but imperfect controller. The honest evaluation: it stabilizes the system, demonstrates the acados integration end-to-end, and produces a clearly identifiable residual error that motivates the offset-free extension.

## 7.6 Dependency: acados
NMPC requires **acados** and its MATLAB/Simulink interface to be installed. The S-functions in this project link against acados shared libraries at runtime; without acados present, neither NMPC Simulink model will load. Installation instructions are at [the acados documentation](https://docs.acados.org/installation/). The compiled S-functions are **not committed** to the repository (they are platform- and version-specific); running `design_nmpc.m` against a working acados installation regenerates them locally.
