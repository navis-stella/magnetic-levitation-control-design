# 6. Sliding Mode Control

Feedback linearization ([§5](05_feedback_linearization.md)) cancels the nonlinearity exactly but assumes precise knowledge of $K_m$ and $m$. Parameter mismatch and unmodelled disturbances leak directly into the closed loop as a residual perturbation. **Sliding mode control** is designed to be insensitive to such *matched* uncertainty: a high-gain switching law forces the trajectory onto a designer-chosen invariant manifold, after which the closed-loop dynamics follows a reduced-order reference behavior regardless of disturbances within the design bound.

The same control-affine reformulation $u = i_s^2$ from [§5.1](05_feedback_linearization.md) is reused.

## 6.1 Sliding Surface
Choose the sliding variable as a linear combination of position and velocity error:

$$
s(\mathbf{x}) = C^\top\mathbf{x} = \lambda x_1 + x_2,\qquad \lambda > 0
$$

On the surface ($s = 0$), $x_2 = -\lambda x_1$ and hence $\dot{x}_1 = -\lambda x_1$ — position converges exponentially with rate $\lambda$. The time derivative of the sliding variable is

$$
\dot{s} = \lambda \dot{x}_1 + \dot{x}_2 = \lambda x_2 + \left(-g + \frac{K_m}{m(x_0 - x_1)^2}\bullet u\right)
$$

> **Practical lower bound on $\lambda$.** Classical SMC theory only requires $\lambda > 0$ for sliding-mode stability. For this plant, however, the open-loop has an unstable real pole at $+\sqrt{2g/x_0} \approx 99$ rad/s. The closed-loop bandwidth set by the sliding dynamics must exceed this open-loop instability rate, so $\lambda$ must be chosen well above $\sqrt{2g/x_0}$. Values below this threshold fail to stabilize the plant regardless of how $\eta, K, \phi$ are tuned — a non-obvious failure mode if the sliding-surface design is taken in isolation from the open-loop dynamics.

## 6.2 Equivalent Control
Setting $\dot{s} = 0$ and solving for $u$ gives the equivalent control:

$$
u_{eq} = \frac{m(x_0 - x_1)^2}{K_m} (g - \lambda x_2)
$$

## 6.3 Reaching Law (Exponential)
The exponential reaching law combines a switching term with a proportional term:

$$
\dot{s} = -\eta \bullet \theta(s) - K s
$$

A pure sign function produces chattering. Replacing it with a **saturation** of width $\phi$ smooths the switching:

$$
\theta(s) = \mathrm{sat}\left(\frac{s}{\phi}\right) =
\begin{cases}
1 & s > \phi \\\\
s/\phi & |s| \leq \phi \\\\
-1 & s < -\phi
\end{cases}
$$

Parameter roles:
- $\eta$ — switching amplitude; sets the size of matched disturbance the controller can reject
- $K$ — exponential decay rate inside the boundary layer
- $\phi$ — boundary-layer thickness; trades chattering reduction against sliding-surface precision

## 6.4 Full Control Law

$$
u = \frac{m(x_0 - x_1)^2}{K_m}\left(g - \lambda x_2 - \eta \bullet \mathrm{sat} \left(\frac{s}{\phi}\right) - K s\right)
$$

The physical current command (positive root with safe clamp):

$$
i_s = \sqrt{\max(u, 0)}
$$

followed by the current limiter.

## 6.5 Lyapunov Justification
With candidate $V = \tfrac{1}{2}s^2$, outside the boundary layer ($|s| > \phi$),

$$
\dot{V} = s \dot{s} = -\eta |s| - K s^2 \leq -\eta |s|
$$

guaranteeing finite-time reaching of the boundary layer.

## 6.6 Implementation Variants
- **`Maglev_SMC_block_ideal.slx`** — uses the built-in Sliding Mode Controller block from Simulink Control Design, with the ideal Simscape velocity. Reference baseline for cross-checking.
- **`Maglev_SMC_mfunc.slx`** — user-defined MATLAB Function implementing the full control law of §6.8 (integral-action variant), running on the EKF-estimated state. This is the version exercised in the [§9](09_evaluation_framework.md) evaluation.

## 6.7 Behavior Under EKF-Estimated Velocity
The SMC design above was validated against the ideal Simscape velocity. When the velocity is replaced by the EKF estimate $$\hat{x}_2$$ (as on hardware), the equivalent control $$u_{eq}$$ no longer exactly cancels gravity, because the small phase lag and bias of $$\hat{x}_2$$ enter the controller as an effective matched disturbance. The reaching law's switching term absorbs this disturbance into the boundary layer, but the trajectory settles at $$|s| \leq \phi$$ rather than at $$s = 0$$ — and on the standard surface $$s = \lambda x_1 + x_2$$, a non-zero $$s$$ implies a non-zero $$x_1$$ at steady state.

In practice this residual offset is very small, thanks to SMC's intrinsic robustness. But it is non-zero, which is inconsistent with the zero-steady-state-error behavior of every other controller in this project. To restore that property, an integral-action variant is implemented in §6.8.

## 6.8 SMC with Integral Action (Augmented Sliding Surface)
The standard surface of §6.1 is extended with an integral term, keeping $\lambda$ in its original role and adding a single integral gain $k_i$:

$$
s(\mathbf{x}, \sigma) = \lambda x_1 + x_2 + k_i \sigma,\qquad \dot{\sigma} = x_1
$$

where $\sigma$ is computed by an external Simulink integrator. Differentiating:

$$
\dot{s} = k_i x_1 + \lambda x_2 - g + \frac{K_m}{m(x_0 - x_1)^2}\bullet u
$$

Imposing the same exponential reaching law as in §6.3:

$$
u = \frac{m(x_0 - x_1)^2}{K_m} \left( g - k_i x_1 - \lambda x_2 - \eta \bullet \mathrm{sat} \left(\tfrac{s}{\phi}\right) - K s \right)
$$

**Sliding dynamics.** On $s = 0$, differentiation gives the homogeneous second-order ODE

$$
\ddot{x}_1 + \lambda \dot{x}_1 + k_i x_1 = 0
$$

with characteristic polynomial $p^2 + \lambda p + k_i = 0$. The two design parameters now play their familiar second-order roles: $\omega_n = \sqrt{k_i}$ and $\zeta = \lambda / (2\sqrt{k_i})$.

**Why this restores zero steady-state error.** Under a constant matched disturbance $d$, $\dot{s} = -\eta \mathrm{sat}(s/\phi) - K s + d$ settles at a non-zero $s_\text{ss}$ inside the boundary layer. With the augmented surface, this non-zero $s_\text{ss}$ is absorbed by the integral state ($\lambda x_1 + x_2 + k_i \sigma_\text{ss} = s_\text{ss}$), not by a non-zero $x_1$ — because $\dot{\sigma} = x_1 = 0$ is required at any true steady state. The mechanism is identical to PID, SSC, and Backstepping with integral action.

## 6.9 Observer Reuse
The same EKF from [§8](08_ekf.md) provides $\hat{x}_2$ from position-only measurements. No SMC-specific observer is required.

Numerical values for $(\lambda, k_i, \eta, K, \phi)$ are documented in `Controller_Design/design_nonlinear_controller.m`.
