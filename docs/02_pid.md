# 2. PID Controller

A pole-placement-based PID design on the linearized plant of [§1](01_system_modeling), with anti-windup.

## 2.1 PD via Second-Order Pole Placement
A PD law

$$
i = K_p\bullet x + K_d\bullet \dot{x}
$$

substituted into the linearized plant of §1.4 yields

$$
m\ddot{x} - k_i K_d \dot{x} - (k_x + k_i K_p) x = 0
$$

with characteristic polynomial

$$
s^2 - \frac{k_i K_d}{m}s - \frac{k_x + k_i K_p}{m} = 0
$$

Matching against the desired second-order target $s^2 + 2\zeta\omega_n s + \omega_n^2 = 0$ gives closed-form expressions for $K_p, K_d$.

**Why this is not enough.** PD stabilizes the linearized plant but cannot eliminate the constant residual error caused by:
- model mismatch in $K_M$, $m$, and $x_0$ (the cancellation of $mg$ relies on an exact linearization),
- constant external disturbances (e.g. payload variations),
- amplifier offsets and sensor bias.

The closed-loop transfer function has no integrator, so a step disturbance produces a non-zero steady-state offset. Integral action is required.

## 2.2 PID via Third-Order Pole Placement
Adding the integral term:

$$
i = K_p x + K_i \int x\ dt + K_d \dot{x}
$$

gives the third-order closed-loop polynomial

$$
m s^3 - k_i K_d s^2 - (k_i K_p + k_x) s - k_i K_i = 0
$$

Matched against the desired polynomial

$$
s^3 + (2\zeta\omega_n + \alpha) s^2 + (\omega_n^2 + 2\zeta\omega_n\alpha) s + \alpha\omega_n^2 = 0
$$

the gains follow in closed form:

$$
K_p = -\frac{m(\omega_n^2 + 2\zeta\omega_n\alpha) + k_x}{k_i},\quad K_d = -\frac{m(2\zeta\omega_n + \alpha)}{k_i},\quad K_i = -\frac{m\alpha\omega_n^2}{k_i}
$$

The three design knobs are the damping $\zeta$, the dominant natural frequency $\omega_n$, and the location $\alpha$ of the additional real pole.

## 2.3 Anti-Windup
Because the current command saturates at the amplifier limit, integral wind-up must be prevented. Both back-calculation and clamping are configured in the Simulink PID block.

## Implementation

The Simulink model `Maglev_PID.slx` runs against the EKF-estimated state (see [§8](08_ekf.md)). Numerical gains, $\zeta$, $\omega_n$, $\alpha$, and the anti-windup back-calculation coefficient are set in `Controller_Design/design_linear_controller.m`. PD is recovered by setting $K_i = 0$.
