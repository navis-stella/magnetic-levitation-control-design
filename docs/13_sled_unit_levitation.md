
# 13. Sled Unit Levitation

This is the most complex system in the project: a multi-DOF magnetically levitated sled unit. It serves two purposes simultaneously — to deliver a working controller for an industrially-realistic geometry, and to validate the design recommendation of [§11.3](11_comparative_analysis.md) (state-space control as the practical default for multi-DOF magnetic bearing systems) on a system whose scale would make any of the nonlinear analytical methods prohibitive.

The chapter is organized around four threads: the mechanical configuration ([§13.1](#131-system-description)), the CAD-based multibody modeling workflow ([§13.2](#132-cad-based-multibody-modeling)–[§13.3](#133-frame-and-force-conventions)), the parameter-organization architecture that keeps the model maintainable at this scale ([§13.4](#134-parameter-organization-setup_machine_model)), and the controller and observer design ([§13.5](#135-state-space-controller-and-observer)).

## 13.1 System Description

The mechanical assembly consists of:

- A **machine stand** (`Grundgestell` / machine stand) — the stationary base frame, carrying eight armature surfaces.
- A **sled unit** (`Schlitteneinheit` / sled unit) — the mobile carriage to be levitated, carrying **eight electromagnets** distributed in two groups of four:
  - **Spindle-end magnets** — `ELO`, `ELU`, `ERO`, `ERU` (End / Left·Right / Upper·Lower).
  - **Tail-end magnets** — `SLO`, `SLU`, `SRO`, `SRU` (Sled-tail / Left·Right / Upper·Lower).
  
  Each magnet pulls toward its corresponding armature surface on the machine stand.

The system has **five actively controlled degrees of freedom**:
- Two translations: $x, y$ (perpendicular to the guideway).
- Three rotations: $\theta$ (roll), $\phi$ (pitch), $\psi$ (yaw).

The sixth DOF, translation along the guideway direction ($z$), is mechanically constrained and not part of the control problem. The eight electromagnets are therefore **over-actuated** relative to the five controlled DOFs — a redundancy the equilibrium-current allocation ([§13.5](#135-state-space-controller-and-observer), S1) exploits to absorb the asymmetry caused by an offset center of mass.

The **geometric center of the sled** is the reference for all kinematic descriptions. Two reference poses define the operating regimes:

- The **equilibrium pose** — sled geometric center aligned with the geometric center of the machine stand, all eight air gaps at their nominal value. This is the operating point about which the plant is linearized.
- The **rest pose** — sled physically rests on its supports with the lower air gaps closed to zero. This is the natural initial condition before levitation is activated.

> **Figure to be added — top-level layout of `SledUnit_Levitation_Control.slx`**
> Top-level Simulink view showing the State-Space Controller with State Estimator subsystem on the upper right, the CAD-rendered plant subsystems (Machine Frame, Spindle Actuator & Sensor, Sled Tail Actuator & Sensor, Sled Unit) in the lower half, and the visualization panels (current values, air-gap measurements, sled state) on the left. Each subsystem will be annotated.

## 13.2 CAD-Based Multibody Modeling

The mechanical geometry is imported from STEP files (in `Simplified_CAD_Model/`) directly into Simscape Multibody via **File Solid** blocks, rather than being reconstructed from primitive blocks. This is the correct choice at this scale of complexity: the CAD already encodes the geometry, inertia distributions are computed automatically from the solid and its assigned material, and any update to the mechanical design propagates into the simulation without redrawing the model.

The CAD parts retain their original German names to preserve traceability with the source mechanical drawings:

| STEP file | English description |
|---|---|
| `Grundgestell.stp` | Machine stand / base frame |
| `Magnetschiene_links.stp` | Left magnet rail |
| `Magnetschiene_rechts.stp` | Right magnet rail |
| `Schlitteneinheit.stp` | Sled unit (carriage) |

Each CAD body is wrapped in a Simulink subsystem with a graphical mask. The mask icons in `BlockMaskIcons/` (`machine_stand.png`, `sled_unit.png`, `outer_end_face.png`, etc.) give the model a clear visual identity at a glance — useful for navigation in a model that is otherwise dense with frames, joints, and force actuators.

## 13.3 Frame and Force Conventions

The multibody modeling conventions validated on the dual-magnet system ([§12.3](12_dual_magnet_levitation.md)) are applied identically here, just at higher scale:

- The z-axis of every magnet surface frame points outward from the magnet; the z-axis of every armature surface frame points inward into the armature.
- Reluctance Force Actuators connect magnet–armature pairs with consistent R/C port assignments.
- The Translational Multibody Interface bridges each magnetic force into the mechanical domain.
- Air gaps are measured by Transform Sensors (`PositionSensor`) with consistent B/F port order.

At the implementation level, each of the eight magnet–armature pairs is wrapped in an identical Simulink subsystem that bundles three blocks: a `PositionSensor` (Transform Sensor) measuring the local air gap, a Cartesian Joint defining the allowed direction of relative motion, and a `Reluctance Force Actuator` generating the magnetic force from the commanded current. This per-magnet pattern makes the model maintainable at the eight-actuator scale: each subsystem is a copy of the same template with different frame attachments and a different label, and any improvement to the pattern propagates uniformly across all eight instances.

The payoff of having established these conventions on the 1-D dual-magnet model is most visible here, where the model has many more magnet–armature pairs. A single sign error at this scale would be difficult to localize and debug; the discipline established earlier prevents it from happening in the first place.

## 13.4 Parameter Organization (`Setup_Machine_Model/`)

At this scale, parameter management is the central engineering challenge of the model — there are several hundred fixed structural and dynamical quantities, and they must remain mutually consistent across the multibody model, the controller, and the observer. The architecture in `Setup_Machine_Model/` is built around two explicit principles.

**Principle 1 — Parameters live in the *model workspace*, not the base workspace.** Polluting the base workspace with several hundred fixed values would be both messy and unsafe: any other script could accidentally overwrite a critical parameter, and naming collisions would be inevitable. The model workspace of `SledUnit_Levitation_Control.slx` is the dedicated container: it is populated by the setup chain through the model's `PreLoadFcn` and is scoped to this model alone.

**Principle 2 — Single source of truth.** The same numerical values used to build the multibody model are reused, *without copying*, by the controller and observer design scripts. Magnet positions, surface-normal directions, inertia tensors, current bounds — every quantity is defined exactly once. Modify one parameter and every downstream consumer (model, controller gain calculation, observer design) sees the change automatically. This eliminates an entire class of bugs that would otherwise be inevitable in a model this complex.

The configuration chain executes in a defined order, entered through `setup_structure_params.m`:

1. **`init_system_const.m`** — defines system-wide physical constants and constraints that are independent of the mechanical design: amplifier current limits, gravity vector, measurement-noise specifications. Loaded first so that all downstream scripts have a consistent reference.
2. **`setup_sled_dyn_params.m`** — defines the sled's kinematic and dynamic parameters:
   - *Kinematic:* the position vector of each magnet relative to the sled's geometric center, magnet dimensions, and surface normal vectors (which fix the force direction of each magnet–rail pair via the [§12.3](12_dual_magnet_levitation.md) convention).
   - *Dynamic:* the sled mass, the inertia tensor about the center of mass, the inertia tensor about the geometric center (used by the multibody solver), and the offset vector from the geometric center to the center of mass.
3. **`setup_structure_params.m`** (the entry point) — defines the stationary structure: the machine stand and magnet rails, their geometric centers, and the rigid transformations (translation + rotation) that locate each rail in the world frame. It calls `init_system_const.m` and `setup_sled_dyn_params.m`, then concludes by calling step 4.
4. **`calculate_airgap_equilibrium_and_rest.m`** — closes the loop by computing the two reference poses defined in [§13.1](#131-system-description):
   - the **equilibrium pose** (sled geometric center aligned with the rail geometric center) and its corresponding air gaps — used as the linearization point for controller and observer design;
   - the **rest pose** (a lower air gap closed to zero) and its corresponding initial sled configuration — used to set the initial state of simulation runs that start from rest.

The output of this chain fully populates the model workspace and is also available to the controller- and observer-design scripts, which is what makes Principle 2 work in practice.

Top-level simulation conditions — simulation time, external disturbance schedules, payload variations — live separately in **`setup_sim_params.m`** at the folder root, since these vary per test scenario rather than per physical design.

## 13.5 State-Space Controller and Observer (`Controller_Design/`)

The controller is a state-space controller with integral action and anti-windup, paired with a Kalman-filter observer. The structure inherits directly from the single-magnet SSC ([§3](03_state_space_control.md)) and the dual-magnet differential design ([§12.6](12_dual_magnet_levitation.md)) , but two new design challenges appear at this scale and warrant explicit discussion:

1. **Over-actuation.** Eight electromagnets serve five degrees of freedom, so the equilibrium-current allocation is a constrained optimization rather than a closed-form expression.
2. **Translation–rotation coupling.** The center of mass is offset from the sled's geometric center along $z$, which produces off-diagonal terms in the mass-inertia matrix. The controller must account for this coupling; a per-axis decentralized design cannot.

The design pipeline `state_space_control_design.m`  executes in eight stages (S1–S8 in the script).

**S1 — Equilibrium currents via constrained optimization.** The redundant allocation is solved by `fmincon` with:
- *Objective:* minimize $\sum_k (I_k - I_\text{target})^2$ — squared deviations of each current from a designer-specified target. The target is chosen to keep the operating point inside a high-sensitivity region of the linearization while leaving headroom against the amplifier limit.
- *Constraints:* six nonlinear equalities (three force-balance + three moment-balance equations about the sled geometric center) and per-magnet bounds $I_\text{min} \leq I_k \leq I_\text{max}$.

An artificial symmetric allocation would *not* satisfy equilibrium because the center of mass (CoM) is offset; the optimization absorbs the asymmetry by construction.

**S2 — Linearization of reluctance forces and moments.** For small deviations, each magnet's air-gap perturbation is

$$

\delta g_k = \mathbf{n}_k^\top \bigl(\delta\mathbf{p} + \delta\boldsymbol{\alpha} \times \mathbf{r}_k\bigr)

$$

where $\mathbf{n}_k$ is the magnet's surface-normal unit vector, $\mathbf{r}_k$ the lever arm from the sled geometric center to the magnet, $\delta\mathbf{p} = [dx, dy, 0]^\top$ the translational deviation, and $\delta\boldsymbol{\alpha} = [d\theta, d\phi, d\psi]^\top$ the rotational deviation. Symbolic differentiation of the net force and moment about the geometric center yields the stiffness Jacobians, evaluated at the equilibrium to give

$$

K_s \in \mathbb{R}^{5\times 5},\qquad K_i \in \mathbb{R}^{5\times 8}

$$

The $z$-translation row is dropped because that DOF is mechanically constrained.

**S3 — Linearized state-space model.** The 10-dimensional state $\mathbf{x} = [\mathbf{q}; \dot{\mathbf{q}}]^\top$ and 8-dimensional input $\mathbf{u} = \mathbf{i}_\text{dev}$ assemble into

$$

\ddot{\mathbf{q}} = M^{-1}\bigl(K_s\,\mathbf{q} + K_i\,\mathbf{u}\bigr)

$$

The **mass-inertia matrix $M$ contains the off-diagonal terms** that arise from the CoM offset: a translational acceleration generates an angular impulse about the geometric center, and vice versa. These cross-terms are essential for closed-loop stability and would be missed by any decentralized design. Controllability is verified explicitly via the PBH test before the gain design proceeds.

**S4 — Augmented LQR with anti-windup.** Following [§3](03_state_space_control.md), five integral states (one per controlled DOF) are appended, producing a 15-state augmented system. The LQR design uses block-diagonal weights, with *substantially larger* integrator weights on the rotational channels — necessary because angular deviations have a numerically smaller scale than translational deviations, and equal weighting would under-penalize them. The anti-windup gain $K_\text{aw}$ is computed as the pseudoinverse of the integral-state gain block, applying the back-calculation pattern of [§2](02_pid.md) generalized to the MIMO case.

**S5 — Sensor-to-DOF kinematic mapping.** The measurement model is the over-determined linear map $\mathbf{y} = H\,\mathbf{q}$, $H \in \mathbb{R}^{8 \times 5}$, where each row is the projection of position onto the corresponding magnet's local air-gap axis. The redundancy is a robustness benefit: a least-squares pseudoinverse $H^\dagger$ recovers the pose from any consistent measurement vector, with graceful degradation if a single sensor channel is corrupted.

**S6 — Observer design (pole placement *or* Kalman filter).** The observer has the standard form

$$

\dot{\hat{\mathbf{x}}} = A\,\hat{\mathbf{x}} + B\,\mathbf{u} + L_\text{obs}\bigl(\mathbf{y} - C_\text{obs}\,\hat{\mathbf{x}}\bigr)

$$

and the script implements two designs selectable via a flag:

- **Pole placement** — observer poles placed at a fixed multiple (≈6×) of the dominant controller poles. Deterministic, requires no noise model, useful for noise-free verification runs.
- **Kalman filter** — designed via the LQR-dual formulation $L_\text{kf} = \text{lqr}(A^\top, C_\text{obs}^\top, Q_\text{kf}, R_\text{kf})^\top$, with $R_\text{kf}$ from the sensor specification and $Q_\text{kf}$ tuned to balance trust between model and sensors. This is the default selection for noisy simulation and hardware deployment, consistent with the EKF philosophy of [§8](08_ekf.md).

Both options share the same observer subsystem in the Simulink model; only $L_\text{obs}$ changes between them.

**S7 — Closed-loop verification suite.** The script performs a comprehensive design-time verification before exporting any gain:

- *Pole analysis* of the open-loop plant (instability confirmed), the augmented closed loop, and the observer. Both closed-loop and observer pole sets are asserted strictly in the left half-plane; assertion failure halts the script before any unsafe gain is saved.
- *Separation principle verification* — the eigenvalues of the full 25-state closed loop (10 plant + 10 observer + 5 integrator) are compared against the union of the standalone controller and observer eigenvalues. Agreement to numerical precision confirms the classical separation result.
- *MIMO singular-value analysis* — open-loop $\sigma(L)$, sensitivity $S = (I + L)^{-1}$, complementary sensitivity $T$, and return difference $I + L$ are all plotted. Peak sensitivity $\|S\|_\infty$ is computed; the design target is $< 6$ dB.
- *Sensitivity-based MIMO robustness margins* — from $\|S\|_\infty$, the script derives the *simultaneous* gain margin $\text{GM} = \|S\|_\infty / (\|S\|_\infty - 1)$ and phase margin $\text{PM} = 2\arcsin\bigl(1 / (2\|S\|_\infty)\bigr)$ guaranteed across all input–output channels — the MIMO replacement for per-channel SISO margins, which are not meaningful in a coupled system of this size.
- *Step disturbance rejection* — a 1 N step is injected at the $y$-acceleration input through the full closed loop; the script confirms the integrator drives the steady-state position error to zero.

This verification catches design errors at script time rather than at simulation time, and the assertions ensure that the parameters exported in S8 correspond to a guaranteed-stable design.

**S8 — Parameter export.** The controller block (`ssc_params`: dimensions, equilibrium currents, gain matrices $K_\text{aug}$ and $K_\text{aw}$, nominal air gap) is saved to `Controller_Params/SSC_Params.mat`. The observer block (`obs_params`: system matrices, measurement matrix, observer gain $L_\text{obs}$) is saved to `Controller_Params/Obs_Params.mat`, with a companion `Simulink.Bus` object in `Controller_Params/obs_params_Bus.mat` so the observer subsystem in the Simulink model receives its entire configuration through a single port — the same pattern as the EKF parameter bus in [§8.4](08_ekf.md).

## 13.6 Evaluation and Results

The script `evaluation_sled_levitation.m` executes `SledUnit_Levitation_Control.slx` programmatically (without opening it), collects the air-gap responses, current commands, and sled-pose trajectories, plots them, and writes both the figures and the raw `simulation_results.mat` into `Results/`. The pattern follows `evaluation_controllers.m` from [§9](09_evaluation_framework.md) — programmatic simulation, post-processing, persistent storage — applied here to a single controller rather than a comparison batch.

The figures produced in this folder (`airgap_response.png`, `current_response.png`, `sled_pose.png`) document the closed-loop response from the rest pose to the equilibrium pose under nominal conditions; the underlying `.mat` file allows any user-defined post-processing without re-running the simulation.

### 13.6.1 Test Configuration

Closed-loop performance was evaluated over a 2-second simulation starting from the levitation equilibrium state (nominal air gap: 0.5 mm). Four external disturbances were applied sequentially to assess robustness across the controlled DOFs:

| Event | Time | Type | Amplitude |
|---|---|---|---|
| $F_y$ impulse | $t = 0.6$ s | Force pulse in $y$ (20 ms) | $-180$ N |
| $F_x$ step | $t = 1.0$ s | Force step in $x$ | $+60$ N |
| $F_y$ step | $t = 1.2$ s | Force step in $y$ | $-60$ N |
| $T_z$ step | $t = 1.4$ s | Torque step about $z$ | $+200$ N·m |

This sequence exercises both translational and rotational disturbance channels with a mix of short-duration impulses and persistent step inputs.

### 13.6.2 Sled Pose Response

All translational and rotational deviations remained at negligible levels throughout the simulation:

| DOF | Peak Deviation | Settled By | Scale |
|---|---|---|---|
| $x$ translation | $\sim 8 \times 10^{-7}$ mm | $\sim 1.5$ s | Nanometre |
| $y$ translation | $\sim 2.5 \times 10^{-5}$ mm | $\sim 0.15$ s | Nanometre |
| Roll | $\sim 1 \times 10^{-9}$ rad | $< 0.1$ s | Numerical noise floor |
| Pitch | $\sim 8 \times 10^{-11}$ rad | — | Numerical noise floor |
| Yaw | $\sim 2 \times 10^{-11}$ rad | — | Numerical noise floor |

Rotational coupling was effectively zero across all disturbance events. This confirms that the state-space controller — with the explicit translation–rotation coupling captured in the mass-inertia matrix of S3 — successfully decouples the controlled DOFs in closed loop, despite the CoM offset that makes those couplings non-trivial in the open-loop plant.

### 13.6.3 Air Gap Response

**Startup transient.** The closed loop drives all eight air gaps from their initial values (asymmetric because of the rest pose) to the common 0.5 mm setpoint:

| | Upper magnets (ELO, ERO, SLO, SRO) | Lower magnets (ELU, ERU, SLU, SRU) |
|---|---|---|
| Initial value | $\sim 0.525$ mm | $\sim 0.477$ mm |
| Setpoint | $0.500$ mm | $0.500$ mm |
| Overshoot / undershoot | $+5.0\%$ | $-4.6\%$ |
| Settling time ($\pm 1\%$) | $\sim 0.40$ s | $\sim 0.45$ s |

The initial overshoot of $\approx 5\%$ is well within the accepted range for active magnetic bearing systems (typical threshold $\leq 10$–$15\%$) and originates primarily from the Simscape mechanical solver's transient at simulation start. The four magnets within each group (upper, lower) exhibit fully symmetrical responses, confirming uniform load distribution and geometric consistency of the model.

**Steady-state performance.** After the startup transient:

| Metric | Value |
|---|---|
| Mean steady-state air gap | $0.500$ mm (on setpoint) |
| Steady-state ripple | $\pm 0.005$ mm ($\pm 1.0\%$ of setpoint) |
| Ripple source | Transform-sensor numerical noise and integrator-induced micro-oscillation |

The $\pm 1\%$ steady-state ripple does not affect levitation stability and is at the threshold of what the Simscape sensors can resolve.

**Disturbance rejection.**

| Disturbance | Peak air-gap deviation | Recovery time |
|---|---|---|
| $F_y$ impulse $-180$ N (20 ms) | $< \pm 0.005$ mm | $< 0.05$ s |
| $F_x$ step $+60$ N | $\sim -0.005$ mm | $< 0.10$ s |
| $F_y$ step $-60$ N | barely measurable | $< 0.05$ s |
| $T_z$ step $+200$ N·m | barely measurable | $< 0.05$ s |

Disturbance rejection is strong across all input channels. Even the 200 N·m torque step — the most severe test case — produced no measurable steady-state air-gap deviation, demonstrating that the integral action in all five DOFs absorbs persistent disturbances effectively.

### 13.6.4 Summary

| Performance criterion | Result | Assessment |
|---|---|---|
| Startup overshoot | $\approx 5\%$ | Acceptable |
| Settling time | $\approx 0.40$–$0.45$ s | Good |
| Steady-state ripple | $\pm 1.0\%$ | Acceptable |
| Disturbance rejection | $< 0.005$ mm deviation | Excellent |
| DOF decoupling | rotations at noise floor | Excellent |
| Air-gap symmetry | all four magnets per group overlap | Excellent |

The state-space controller demonstrates stable levitation, strong disturbance rejection, and effective multi-DOF decoupling under all tested load conditions. The system maintains the nominal 0.5 mm air gap within $\pm 1\%$ in steady state and recovers from transient disturbances within 0.1 s.

### 13.6.5 What This Demonstrates About the Recommendation

The empirical results validate the design recommendation made in [§11.3](11_comparative_analysis.md):

- **The MIMO state-space design absorbed every disturbance channel without per-channel redesign.** Whether the disturbance arrived in a translational or rotational direction, the same gain matrix handled it through the augmented integrator and the off-diagonal coupling terms in $M$.
- **The redundant actuation (8 magnets, 5 DOFs) gave the controller margin to absorb the disturbances without driving any single magnet to its current limit.** This is the practical payoff of the `fmincon`-based equilibrium allocation in S1 — by keeping every magnet near a moderate target current, the controller has bidirectional authority on every axis.
- **The design-time verification suite (S7) accurately predicted the closed-loop behavior** observed in simulation. The peak-sensitivity-based MIMO margins, the separation-principle check, and the step-disturbance simulation in S7.6 all matched the actual closed-loop performance to within the expected tolerances. This is the value of building verification into the design script rather than discovering issues only at simulation time.

These observations are the concrete sled-unit-level evidence behind the recommendation that state-space control is the practical default for multi-DOF magnetic bearing systems: extending the same workflow used at 1-DOF to a 15-state, 8-input system was structurally trivial, and the closed-loop result is excellent on every measured criterion.