# 4. Integrator Backstepping

Backstepping is a recursive Lyapunov-based design that constructs the stabilizing control law and its stability certificate **simultaneously**, step by step through the state hierarchy. Unlike feedback linearization ([§5](05_feedback_linearization.md)), which cancels the nonlinearity via an input transformation, backstepping shapes the closed loop directly so that a designer-chosen composite Lyapunov function is non-increasing at every step. The design knobs are the per-stage decay coefficients $c_0, c_1, c_2$, each directly bounding a Lyapunov component's exponential rate.

The plant is the control-affine form $u = i_s^2$ (see [§5.1](05_feedback_linearization.md) for the derivation of this reformulation).

## 4.1 Basic Backstepping (Without Integral Action)
Define the first error variable and choose its desired dynamics through a virtual control:

$$
z_1 = x_1,\qquad \alpha_1 = -c_1  z_1\enspace (c_1 > 0),\qquad z_2 = x_2 - \alpha_1 = x_2 + c_1  x_1
$$

With $V_1 = \tfrac{1}{2}z_1^2$, the derivative along trajectories is

$$
\dot{V}_1 = z_1  x_2 = z_1(\alpha_1 + z_2) = -c_1  z_1^2 + z_1 z_2
$$

The cross-term $z_1 z_2$ is cancelled in the next step. Extending the Lyapunov function to $V_2 = V_1 + \tfrac{1}{2}z_2^2$ and choosing the actual control so that $\dot{z}_2 = -z_1 - c_2  z_2$ ($c_2 > 0$) yields

$$
\dot{V}_2 = -c_1  z_1^2 - c_2  z_2^2 \leq 0
$$

and the current command:

$$
i_s = (x_0 - x_1)\sqrt{\dfrac{m}{K_m}  \bigl[  g - (1 + c_1 c_2)  x_1 - (c_1 + c_2)  x_2  \bigr]}
$$

**Why this isn't enough.** This design stabilizes the nominal plant asymptotically but contains no integral term. Modelling errors and constant disturbances produce a non-zero steady-state offset — exactly the failure mode of the PD design in [§2.1](02_pid.md#21-pd-via-second-order-pole-placement). The remedy is to extend the procedure with an integral state.

## 4.2 Backstepping with Augmented Integral State
Introduce the integral of position: $\dot{\sigma} = x_1$. The augmented system $(\sigma, x_1, x_2)$ is treated in three stages.

**Step 0 — integral state, virtual control is $x_1$.**

$$
z_0 = \sigma,\qquad \alpha_0 = -c_0  z_0,\qquad z_1 = x_1 - \alpha_0 = x_1 + c_0  \sigma
$$

**Step 1 — position, virtual control is $x_2$.**

$$
\alpha_1 = -(c_0 + c_1)  x_1 - (1 + c_0 c_1)  \sigma,\qquad z_2 = x_2 - \alpha_1
$$

**Step 2 — velocity, actual control is $u = i_s^2$.** Choose the control so that $\dot{z}_2 = -z_1 - c_2 z_2$. With the composite Lyapunov function

$$
V = \tfrac{1}{2}\bigl(z_0^2 + z_1^2 + z_2^2\bigr)
$$

this yields

$$
\dot{V} = -c_0  z_0^2 - c_1  z_1^2 - c_2  z_2^2 \leq 0,\qquad c_0, c_1, c_2 > 0
$$

Solving the bottom step gives the **nonlinear backstepping current law**:

$$
{\enspace i_s = (x_0 - x_1)\sqrt{\dfrac{m}{K_m}  \bigl[  g - K_v  x_2 - K_p  x_1 - K_i  \sigma  \bigr]}\enspace }
$$

with effective state-feedback gains expressed in terms of the backstepping coefficients:

$$
K_v = c_0 + c_1 + c_2,\qquad K_p = 2 + c_0 c_1 + c_0 c_2 + c_1 c_2,\qquad K_i = c_0 + c_2 + c_0 c_1 c_2
$$

## 4.3 Relationship to Feedback Linearization
The current command above structurally matches the Feedback Linearization form $i_s = (x_0 - x_1)\sqrt{(m/K_m)(v + g)}$ of [§5](05_feedback_linearization.md), with the virtual input identified as

$$
v = -K_v  x_2 - K_p  x_1 - K_i  \sigma
$$

The two designs reach a similar closed loop but expose different tuning knobs: Feedback Linearization chooses three gains $(K_1, K_2, K_3)$ independently via pole placement or LQR, while backstepping chooses $(c_0, c_1, c_2)$ — per-stage Lyapunov decay rates — and the gains $(K_v, K_p, K_i)$ are nonlinear functions of those choices. Backstepping has fewer degrees of freedom but provides a Lyapunov stability certificate by construction; Feedback Linearization has full gain freedom but offloads stability to the outer-loop design.

The empirical confirmation of this equivalence appears in [§10](10_results.md): the Feedback Linearization and Backstepping responses are indistinguishable on every performance metric.

## 4.4 Robustness Limitations and Implementation
Like FBL, the cancellation relies on accurate $K_m$ and $m$. Parameter uncertainty produces residual perturbations not handled by the design. When matched bounded uncertainty dominates, SMC ([§6](06_sliding_mode_control.md)) remains preferable.

Numerical values for $c_0, c_1, c_2$ are configured in `Controller_Design/design_nonlinear_controller.m`. The Simulink model is `Maglev_Backstepping.slx`, running on the EKF estimate (see [§8](08_ekf.md)).
