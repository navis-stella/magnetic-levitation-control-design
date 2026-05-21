# 8. State Estimation: Extended Kalman Filter

In simulation, velocity can be read directly from a Simscape sensor or reconstructed by a filtered derivative. On hardware, only the air-gap position is measured — and that measurement is noisy. To make the simulation environment match the planned hardware deployment, **all controllers in this project run on top of the same Extended Kalman Filter (EKF)**, embedded as a Simulink subsystem inside each model. The controller consumes $\hat{x}_1, \hat{x}_2$ from the filter rather than the ideal Simscape signals, so what is validated in simulation is exactly what will be deployed.

The EKF is designed in `Controller_Design/design_EKF.m`, the parameters are saved in `Controller_Params/EKFData.mat`, and the same filter subsystem is dropped into every EKF-based controller model.

## 8.1 Plant Model Used by the EKF
The EKF uses the **physical current $i_s$** as the input — not the squared current $u = i_s^2$ used in the controller derivations of [§4](04_backstepping.md)–[§6](06_sliding_mode_control.md) for control-affine convenience. The continuous-time model is therefore

$$
\dot{x}_1 = x_2,\qquad \dot{x}_2 = \frac{K_m}{m}\cdot\frac{i_s^2}{(x_0 - x_1)^2} - g
$$

with output $y = x_1 + n$, where $n$ is zero-mean Gaussian measurement noise from the eddy-current gap sensor.

## 8.2 Discretization and Jacobian
The model is discretized by **Euler-forward** at $T_s = 100  \mu\text{s}$ (10 kHz), roughly ten times faster than the dominant closed-loop dynamics:

$$
\mathbf{x}_{k+1} = \mathbf{x}_k + T_s  f(\mathbf{x}_k, i_{s,k})
$$

Differentiating with respect to $\mathbf{x}$ gives the discrete state Jacobian

$$
F_d(\hat{\mathbf{x}}, i_s) = I + T_s  F_c,\qquad F_c = \begin{bmatrix} 0 & 1 \\\\[6pt] \dfrac{2 K_m  i_s^2}{m  (x_0 - \hat{x}_1)^3} & 0 \end{bmatrix}
$$

evaluated symbolically once in `design_EKF.m` and computed at runtime at each prediction step. The output Jacobian is the constant $H = [1\ \ 0]$ since only position is measured.

**Plausibility check.** Before the filter is wired into any controller, `design_EKF.m` evaluates $F_c$ at the operating point $(0, 0, i_0)$, extracts the unstable real pole as $\sqrt{F_c(2,1)} = \sqrt{2g/x_0}$, and verifies that it matches the analytical $\approx 99$ rad/s from [§1.4](01_system_modeling.md). This is a useful guard against parameter typos: a wrong $K_m$, $m$, or $x_0$ entered in `setup_sim_params.m` will produce a visibly wrong pole here, long before any controller misbehaves.

## 8.3 Noise Covariances
- $R = \sigma_y^2$ — measurement noise variance, taken from the eddy-current sensor specification.
- $Q = \mathrm{diag}(q_\mathrm{pos}, q_\mathrm{vel})$ — process noise covariance, **deliberately asymmetric**:
  - $q_\mathrm{pos}$ is set near zero, because $x_1$ evolves as the exact integral of $x_2$ — there is no physical mechanism for an independent position disturbance.
  - $q_\mathrm{vel}$ is set substantially larger, absorbing unmodelled effects on the velocity equation: nonlinearity in $K_m$, eddy-current damping, payload variations, current-amplifier nonidealities.

The ratio $q_\mathrm{vel} / R$ is the practical aggressiveness handle — higher ratios trust the sensor more, lower ratios trust the model more.

## 8.4 Implementation in Simulink
The EKF is realized as a subsystem containing a single MATLAB Function block (`ekf_maglev`) that performs the recursive predict/update steps. Three input ports feed the block each cycle:

<div align="center">

| Port | Source | Role |
|---|---|---|
| `u` | physical current $i_s$, delayed by $1/z$ | input to the prediction step |
| `y` | noisy position measurement | input to the update step |
| `p` | `ekf_params` parameter bus | $T_s, Q, R, H, P_0, \hat{\mathbf{x}}_0, K_m, m, g, x_0$ |

</div>

Two further `1/z` delays close the recursion by feeding $\hat{x}_{k-1},\ P_{k-1}$ back into the block. The unit delay on $u$ preserves the standard discrete time-step alignment: the prediction at step $k$ is computed from $u_{k-1}$, then corrected by $y_k$.

**Why the input convention differs from the controllers.** The controller derivations in [§4](04_backstepping.md)–[§6](06_sliding_mode_control.md) work with the squared current $u = i_s^2$ for control-affine algebra. The EKF, however, uses the *physical* current $i_s$ as its input because that is what the signal lines and the actual amplifier carry. The reconciliation is done at every controller's output stage: each controller computes its $u$, takes $i_s = \sqrt{\max(u,0)}$ (followed by the current limiter), and the resulting $i_s$ is what reaches both the plant and the EKF. Both conventions therefore describe the same physical signal, and the controllers, the plant, and the EKF stay perfectly consistent.

**Parameter bundling via Simulink.Bus.** Rather than wiring every scalar (Q, R, Ts, Km, m, ...) into the Function block individually, `design_EKF.m` constructs a `Simulink.Bus` object `ekf_Bus` describing the parameter struct, and the block receives it through a single bus port `p`. This keeps the subsystem interface clean and lets the same EKF subsystem be copy-pasted into every controller model without rewiring.

## 8.5 Integration Across Controllers
The same EKF subsystem is dropped into `Maglev_PID.slx`, `Maglev_SSC.slx`, `Maglev_Backstepping.slx`, `Maglev_FBL_SSC.slx`, and `Maglev_SMC_mfunc.slx`. Each model loads `EKFData.mat` through its `PreLoadFcn`. PID, SSC, Backstepping, and FBL all run with negligible performance loss compared to their ideal-state baselines — the EKF effectively decouples the controller design from the sensing reality. The SMC behavior under the EKF, and the integral-action variant designed to restore zero steady-state error, are treated in [§6.7](06_sliding_mode_control.md)–[§6.8](06_sliding_mode_control.md).
