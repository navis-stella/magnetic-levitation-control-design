# 3. State-Space Control with Integral Action

The PID design of §2 adds integral action by raising the order of a SISO controller. The state-space formulation does the same thing more transparently: the integral of the tracking error becomes an additional **state**, and a single feedback gain matrix handles regulation, damping, and steady-state error in one design step.

## 3.1 Augmented State
With reference $x_{\text{ref}} = 0$, define the integral state
$$q = \int x\,dt$$
and the augmented state vector
$$\mathbf{z} = \begin{bmatrix} x \\ v \\ q \end{bmatrix},\qquad v = \dot{x}$$

## 3.2 Augmented Plant
The linearized dynamics in state-space form:
$$\dot{\mathbf{z}} = A\,\mathbf{z} + B\,\Delta i,\qquad A = \begin{bmatrix} 0 & 1 & 0 \\ k_x/m & 0 & 0 \\ 1 & 0 & 0 \end{bmatrix},\qquad B = \begin{bmatrix} 0 \\ k_i/m \\ 0 \end{bmatrix}$$

## 3.3 Full-State Feedback
$$\Delta i = -K\,\mathbf{z} = -K_1\,x - K_2\,v - K_3\,q$$
The gain $K$ is obtained either by **pole placement** (`place`) or by **LQR** (`lqr`) on the augmented pair $(A, B)$. The integral state ensures zero steady-state error and rejection of constant input/output disturbances by construction — no feedforward or reference pre-filter is needed.

## 3.4 Implementation Variants

- **`Maglev_SSC_ideal.slx`** — assumes a clean position measurement and computes velocity from a derivative-with-filter block. Used to verify the gain design against the linearized plant.
- **`Maglev_SSC.slx`** — runs on the EKF estimate (see [§8](08_ekf.md)), the realistic configuration that mirrors hardware deployment.

The gain matrix $K$ is configured in `Controller_Design/design_linear_controller.m` and saved to `Controller_Params/LinearControllerData.mat`.

## 3.5 Why This Generalizes Well

State-space control with integral action is the design template that scales most cleanly to multi-DOF systems. The dual-magnet design in [§12](12_dual_magnet_levitation.md) and the 5-DOF sled-unit design in [§13](13_sled_unit_levitation.md) both reuse this same augmented-state structure, just with larger matrices. This scaling property is the central argument of [§11](11_comparative_analysis.md).