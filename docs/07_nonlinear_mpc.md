# 7. Nonlinear Model Predictive Control

The controllers in [§2](02_pid.md)–[§6](06_sliding_mode_control.md) share a common design philosophy: derive a closed-form control law (via linearization, exact cancellation, Lyapunov recursion, or sliding-mode reasoning) and tune its parameters offline. **Nonlinear Model Predictive Control (NMPC)** is a structurally different approach: at every sampling instant, solve a small constrained optimal-control problem over a finite prediction horizon, apply only the first input, and re-solve at the next instant. Constraints and trajectory shaping enter the design directly through the optimization rather than being retrofitted via saturation or anti-windup.

This chapter documents two NMPC variants. The **standard NMPC** of [§7.2](07_nonlinear_mpc.md)–[§7.4](07_nonlinear_mpc.md) stabilizes the ideal model asymptotically with zero residual error, but exhibits a steady-state offset against the realistic Simscape plant and cannot reject external disturbances — exactly the failure mode the closed-form controllers absorb with an integral state. The **offset-free NMPC** of [§7.5](07_nonlinear_mpc.md) closes this gap by augmenting the prediction model with a disturbance state, estimating it with an augmented EKF, and recomputing the OCP tracking target online from the current disturbance estimate. All three NMPC models (AcadosSim, Simscape, OffsetFree) can be run separately to view and compare their behavior; only the offset-free NMPC enters the [§10](10_results.md) comparison, alongside the five closed-form controllers..

## 7.1 Why NMPC for the Maglev System

The maglev plant has nonlinear dynamics, hard input constraints (amplifier current bounds), and physical state limits (the air gap cannot be negative). The closed-form controllers handle these constraints externally — via saturation, clamping, or anti-windup — meaning the controller is operating outside its design region whenever a constraint is hit. NMPC **incorporates the constraints directly into the optimization problem**, so the commanded input is admissible by construction and the controller's optimality guarantee survives at the constraint boundary. The trade-off is computational cost: instead of evaluating a closed-form expression, the controller solves a small nonlinear program at every sample.

## 7.2 Standard NMPC Formulation

At each sampling instant, the controller solves

$$
\min_{u_0, \ldots, u_{N-1}} \sum_{k=0}^{N-1}\Big(\mathbf{x}_k^\top Q \mathbf{x}_k + u_k^\top R u_k\Big) + \mathbf{x}_N^\top Q_N \mathbf{x}_N
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

Both NMPC variants are solved with **acados**, a real-time-capable nonlinear MPC library with MATLAB and Simulink interfaces. acados supports symbolic definition of the nonlinear dynamics, structured QP and SQP-RTI solvers tuned for embedded real-time use, and direct compilation of the OCP solver to a C-MEX S-function consumable by Simulink.

The design pipelines in `Controller_Design/design_nmpc/` (standard) and `Controller_Design/design_nmpc_offsetfree/` (offset-free) follow the same acados workflow:

1. Define the symbolic plant model.
2. Construct an `AcadosOcp` object specifying the cost matrices, horizon, sampling time, constraints, and solver options.
3. Generate C code for the OCP solver.
4. Compile to S-functions using **`make_sfun`** (for the controller) and **`make_sfun_sim`** (for the integrator used as the prediction plant in the AcadosSim variant).

The numerical parameters are saved to `Controller_Params/StdNMPCData.mat` and `Controller_Params/OffsetFreeNMPCData.mat` respectively. For new users, the official acados Simulink examples are a good starting point.

## 7.4 Standard NMPC: AcadosSim vs Simscape — The Diagnostic Gap

Two variants of the standard NMPC reveal the structural limitation that motivates the offset-free design.

**`Maglev_NMPC_AcadosSim.slx`** — uses acados's internal integrator (`acados_sim_solver`) as the simulation plant. Prediction model and plant model are *identical*: no mismatch, no unmodelled disturbance, no measurement noise. Under these conditions the closed-loop state converges asymptotically to the origin with zero residual error, confirming that the OCP formulation and the solver pipeline are correct.

**`Maglev_NMPC_Simscape.slx`** — uses the full Simscape Magnetic plant from `00_Shared_Library/`, with the same parametric uncertainties, sensor noise, and external disturbance profile that the closed-form controllers face in [§10](10_results.md). Under these realistic conditions the closed-loop converges to a non-zero steady-state offset and fails to reject the applied step disturbance.

The discrepancy is exactly the plant–model mismatch that the closed-form controllers absorb with an integral state. Standard NMPC has no equivalent mechanism: the prediction model is the smooth analytical form, the plant is the high-fidelity Simscape model, and the offset between them — together with any external disturbance — accumulates as a constant force on the velocity equation that the OCP cannot account for. The receding-horizon optimization drives the state to the *prediction model's* optimum rather than the plant's. The remedy is to augment the model with an explicit disturbance state and let the observer estimate it.

## 7.5 Offset-Free NMPC

The offset-free formulation (see Pannocchia, 2015, for a tutorial review) extends the prediction model with a constant-disturbance state, estimates that state online, and recomputes the OCP tracking target at every sample so that the optimum is achievable in the presence of the estimated disturbance. The implementation lives entirely in `Controller_Design/design_nmpc_offsetfree/` — including the augmented observer, since it is used only here and would be out of place in [§8](08_ekf.md) alongside the universal observer for the closed-form controllers.

### 7.5.1 Augmented Dynamics

A scalar disturbance state $d$ is introduced on the velocity channel of the §1 dynamics:

$$
\dot{x}_1 = x_2,\qquad \dot{x}_2 = \frac{K_m}{m}\cdot\frac{i_s^2}{(x_0 - x_1)^2} - g + d,\qquad \dot{d} = 0
$$

The choice to absorb $d$ on the velocity equation reflects where mismatch physically enters the model: deviations in $K_m$ or $m$ scale the magnetic-force term, payload variations enter as a gravity-like force, and current-amplifier nonidealities perturb the input — all of these accumulate into a net force-per-unit-mass disturbance acting on velocity. The random-walk model $\dot{d}=0$ is the standard structural choice: it expresses that $d$ is not known a priori but is slowly time-varying compared to the closed-loop dynamics, leaving the observer's noise tuning to set how quickly $\hat{d}$ adapts.

This augmented model is used **both** as the OCP prediction model and as the observer model. The physical plant (Simscape) is unchanged.

### 7.5.2 Augmented EKF

The augmented state $\tilde{\mathbf{x}} = [x_1, x_2, d]^\top$ is estimated by an augmented EKF that reuses the structural pattern of [§8](08_ekf.md): Euler-forward discretization at $T_s = 100 \mu\text{s}$, recursive predict/update, position-only measurement $y = x_1 + n$. The continuous-time Jacobian becomes

$$
\tilde{F}_c(\hat{\tilde{\mathbf{x}}}, i_s) = \begin{bmatrix} 0 & 1 & 0 \\\\[6pt] \dfrac{2 K_m i_s^2}{m(x_0 - \hat{x}_1)^3} & 0 & 1 \\\\[6pt] 0 & 0 & 0 \end{bmatrix}
$$

The new third column reflects the additive entry of $d$ on the velocity equation; the new third row encodes the random-walk dynamics $\dot{d}=0$. The output Jacobian is the constant $\tilde{H} = [1\enspace 0\enspace 0]$ — only position is measured.

The process- and measurement-noise covariances follow the [§8](08_ekf.md) philosophy without modification: $R$ matches the eddy-current sensor specification, $Q$ is diagonal with near-zero $q_\text{pos}$, sizeable $q_\text{vel}$, and a small $q_d$ that sets how quickly the filter adapts to changes in the disturbance. No special tuning was required — the structural choices of [§8](08_ekf.md) carry over directly.

**Observability.** A rank check on the augmented pair $(\tilde{A}, \tilde{H})$ at the operating point confirms that the position measurement is sufficient to recover all three augmented states. This is the standard offset-free MPC observability requirement: the disturbance must be observable through the same measurement that drives the original state estimate.

This observer is used only inside the offset-free NMPC. The unaugmented EKF of [§8](08_ekf.md) remains the right observer for the five closed-form controllers, which carry their own integral states; the augmented EKF is the right observer for the offset-free NMPC, which carries none.

### 7.5.3 Target Steady-State Calculation

Under a nonzero $\hat{d}$, the origin is no longer an equilibrium of the augmented plant — driving the state to zero is no longer the right objective, because the disturbance estimate would persistently fight the controller. Instead, the controller must drive the state to the **achievable steady state** consistent with the current disturbance estimate and the reference set-point $z_\text{ref}$.

Setting $\dot{x}_2 = 0$ in the augmented dynamics at $x_1 = z_\text{ref}$ and solving for the steady-state input:

$$
\frac{K_m}{m}\cdot\frac{u_s^2}{(x_0 - z_\text{ref})^2} = g - \hat{d} \quad\Longrightarrow\quad u_s = (x_0 - z_\text{ref})\sqrt{\frac{m (g - \hat{d})}{K_m}}
$$

The corresponding state target is $x_s = [z_\text{ref}, 0]^\top$ — position at the reference, zero velocity.

The MATLAB function **`target_ss_calc.m`** implements this analytical solution with three guards before returning:

- **Physical-gap guard.** $x_0 - z_\text{ref}$ must be strictly positive; otherwise the reference is unreachable (the mass would have to pass through the magnet).
- **Effective-gravity guard.** $g - \hat{d}$ must be strictly positive; otherwise no real-valued $u_s$ exists. When this condition is briefly violated by a transient on $\hat{d}$, the function clips $g - \hat{d}$ to a small positive value and flags the result as infeasible, allowing the OCP to use the clipped reference for the current sample while the EKF recovers.
- **Actuator-limit guard.** The computed $u_s$ is clamped to $[I_\text{min}, I_\text{max}]$ and flagged infeasible if it falls outside that range.

The `feasible` flag is exposed for diagnostics, but the OCP runs unconditionally: under sustained infeasibility the optimizer's own constraint-handling absorbs the situation, and across the operating profile of this project the guards never trigger past the initial transient.

The reference $z_\text{ref}$ defaults to zero — track the linearization equilibrium — but can be set to any admissible deviation value to stabilize the system at a different operating point without redesigning the controller or the observer. This is a property the closed-form integral-action controllers do not share: changing the operating point there would require re-running the design script with new equilibrium parameters.

### 7.5.4 Modified OCP

The offset-free OCP is the standard NMPC of [§7.2](07_nonlinear_mpc.md) with the zero-reference replaced by the recomputed target $(x_s, u_s)$:

$$
\min_{u_0, \ldots, u_{N-1}} \sum_{k=0}^{N-1}\left[(\mathbf{x}_k - x_s)^\top Q (\mathbf{x}_k - x_s) + (u_k - u_s)^\top R (u_k - u_s)\right] + (\mathbf{x}_N - x_s)^\top Q_N (\mathbf{x}_N - x_s)
$$

subject to the same augmented plant dynamics, the same input bounds, and the same initial-state constraint $\mathbf{x}_0 = \hat{\mathbf{x}}(t)$ as the standard formulation. The target $(x_s, u_s)$ is recomputed at every sample from the latest $\hat{d}$ — it is a function of the disturbance estimate, not a fixed design parameter. As $\hat{d}$ converges to the true disturbance, $(x_s, u_s)$ converges to the true achievable steady state, and the OCP optimum coincides with the plant's true zero-error operating point. This is exactly the role that the integral state plays in the closed-form controllers, expressed in the optimization framework.

The cost matrices $Q$, $R$, $Q_N$, the horizon $N$, and the sampling time $T_s$ are inherited unchanged from the standard NMPC design — offset-free MPC does not require re-tuning the cost, only augmenting the model, adding the observer, and inserting the target calculation.

## 7.6 Implementation: `Maglev_NMPC_OffsetFree.slx`

The Simulink model wires together three components downstream of the Simscape plant:

1. **Augmented EKF** (MATLAB Function block) — receives the noisy position measurement and the applied current $i_s$, outputs $\hat{x}_1, \hat{x}_2, \hat{d}$ at each sample.
2. **Target calculation** (MATLAB Function `target_ss_calc`) — receives $\hat{d}$, $z_\text{ref}$, the dynamics parameter bus, and the input bounds, outputs the target pair $(x_s, u_s)$.
3. **acados OCP solver** (S-function) — receives $\hat{\mathbf{x}} = [\hat{x}_1, \hat{x}_2]^\top$ and the target pair, outputs the current command $i_s$.

All three components run at the same sample rate, and their parameter buses are loaded by the model's `PreLoadFcn` from `Controller_Params/OffsetFreeNMPCData.mat`. The S-functions are regenerated by running `design_nmpc_offsetfree.m` against a working acados installation.

## 7.7 Dependency: acados

As in [§7.3](07_nonlinear_mpc.md), both NMPC models link against acados shared libraries at runtime and require the acados MATLAB/Simulink interface to be installed. The compiled S-functions are platform- and version-specific and are **not committed** to the repository; running the corresponding design script regenerates them locally. Installation instructions are at [the acados documentation](https://docs.acados.org/installation/).