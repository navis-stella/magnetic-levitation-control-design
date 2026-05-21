# 5. Feedback Linearization with Outer State-Space Controller

The linearization in [§1.4](01_system_modeling.md) is only valid in a small neighborhood of $(x_0, i_0)$. The true stiffness coefficients $k_x(x), k_i(x, i)$ vary strongly with the air gap, so a linear controller designed at $x_0 = 2\,\text{mm}$ degrades quickly when the system is excited far from equilibrium. **Feedback linearization cancels the nonlinearity exactly** through an input transformation, yielding a closed-loop system that is linear by construction over the entire admissible operating region.

## 5.1 Affine-in-Control Reformulation
The nonlinear dynamics

$$

m\ddot{x} = K_m\,\frac{i_s^2}{(x_0 - x)^2} - mg

$$

are **not** affine in the physical input $i_s$, but they become affine in the squared current

$$

u := i_s^2

$$

With the state vector $\mathbf{x} = [x_1, x_2]^\top = [x, \dot{x}]^\top$, the system takes the standard control-affine form

$$

\dot{\mathbf{x}} = f(\mathbf{x}) + g(\mathbf{x})\,u

$$

with

$$

f(\mathbf{x}) = \begin{bmatrix} x_2 \\ -g \end{bmatrix},\qquad g(\mathbf{x}) = \begin{bmatrix} 0 \\ \dfrac{K_m}{m\,(x_0 - x_1)^2} \end{bmatrix}

$$

The actual current is recovered downstream as $i_s = \sqrt{u}$ (positive root), saturated to the amplifier range. This same $u = i_s^2$ substitution is reused by Backstepping ([§4](04_backstepping.md)) and SMC ([§6](06_sliding_mode_control.md)).

## 5.2 Input–Output Linearization via Lie Derivatives
Choose the output $y = h(\mathbf{x}) = x_1$. Differentiating along the trajectories:

$$

\dot{y} = L_f h\,(\mathbf{x}) = \frac{\partial h}{\partial \mathbf{x}}\,f(\mathbf{x}) = x_2

$$

$$

\ddot{y} = L_f^2 h + (L_g L_f h)\,u

$$

with

$$

L_f^2 h = -g,\qquad L_g L_f h = \frac{K_m}{m\,(x_0 - x_1)^2}

$$

The input $u$ first appears in $\ddot{y}$, so the **relative degree is $r = 2$**. Since $r$ equals the system order, the transformation achieves **full-state linearization** — there are no internal dynamics and no zero dynamics to verify.

## 5.3 Linearizing Control Law
Introduce the virtual input $v$ such that $\ddot{y} = v$:

$$

-g + \frac{K_m}{m\,(x_0 - x_1)^2}\,u = v \quad\Longrightarrow\quad u = \frac{m\,(x_0 - x_1)^2}{K_m}\,(v + g)

$$

and the physical current command is

$$

i_s = (x_0 - x_1)\sqrt{\dfrac{m}{K_m}\,(v + g)}

$$

well-defined whenever $v + g \geq 0$ and the gap is non-zero. In the linearizing coordinates $\boldsymbol{\xi} = [x_1, x_2]^\top$, the closed-loop system is exactly a **double integrator**:

$$

\dot{\boldsymbol{\xi}} = \begin{bmatrix} 0 & 1 \\ 0 & 0 \end{bmatrix}\boldsymbol{\xi} + \begin{bmatrix} 0 \\ 1 \end{bmatrix} v

$$

## 5.4 Outer-Loop Design on the Virtual Input
Two stabilizing strategies are documented; the second is the one shipped in the Simulink model.

**Option A — PID outer loop.** With

$$

v = -k_p\,x - k_d\,\dot{x} - k_i\int x\,d\tau

$$

the closed-loop characteristic polynomial is $s^3 + k_d s^2 + k_p s + k_i = 0$.

**Option B — State-space controller with integral action (implemented).** Augmenting the linearized double integrator with the integral state $q = \int x\,dt$ yields the same triple $\mathbf{z} = [x, v, q]^\top$ as in [§3](03_state_space_control.md), but with system matrix

$$

A_{\text{FBL}} = \begin{bmatrix} 0 & 1 & 0 \\ 0 & 0 & 0 \\ 1 & 0 & 0 \end{bmatrix},\qquad B_{\text{FBL}} = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}

$$

The crucial difference from [§3](03_state_space_control.md): $A_{\text{FBL}}$ contains no stiffness terms $k_x/m$, because the nonlinearity has been cancelled exactly. The feedback gain $K$ for $v = -K\mathbf{z}$ is obtained by pole placement or LQR on $(A_{\text{FBL}}, B_{\text{FBL}})$, and the resulting design is **independent of the operating-point gap** — the same gain works at any equilibrium where the model holds.

## 5.5 Practical Considerations
- **Model dependence.** The exact cancellation relies on accurate knowledge of $K_m$ and $m$. Parameter uncertainty leaks directly into a residual nonlinearity, which is the primary limitation of pure FBL and the motivation for the robust (SMC, [§6](06_sliding_mode_control.md)) and predictive (NMPC, [§7](07_nonlinear_mpc.md)) approaches.
- **Singularity.** The transformation is invertible as long as $x_0 - x_1 > 0$ — i.e., the mass has not contacted the magnet. A current limiter is applied after the square root to handle saturation gracefully.
- **Real-time cost.** Negligible: one squaring, one division, one square root per control step.

The outer-loop gain matrix $K$ is configured in `Controller_Design/design_nonlinear_controller.m`. The model is `Maglev_FBL_SSC.slx`, running on the EKF estimate (see [§8](08_ekf.md)).