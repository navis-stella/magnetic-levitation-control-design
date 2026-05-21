# Magnetic Levitation Control Design

> Simscape-based design of magnetic levitation controllers — from a single-magnet model to a 5-DOF sled with eight electromagnets.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![MATLAB R2025a](https://img.shields.io/badge/MATLAB-R2025a-blue.svg)](https://www.mathworks.com/products/matlab.html)

## About

This project develops the modeling and control infrastructure for magnetic levitation systems in MATLAB/Simulink, using Simscape Magnetic and Simscape Multibody. It was carried out as a student-assistant research project, and is structured as a layered escalation in three stages:

1. **A single-magnet 1-DOF testbed** — used as the platform for designing and benchmarking five integral-action controllers (PID, state-space, backstepping, feedback linearization, sliding mode) and a nonlinear MPC. A common Extended Kalman Filter observes the state from a noisy gap measurement, making the simulation faithful to a hardware deployment.
2. **A dual-magnet 1-DOF system** — used to validate the Simscape Multibody modeling workflow (frame conventions, force–motion coupling, CAD-based geometry) on a system simple enough to verify by hand.
3. **A 5-DOF sled with eight electromagnets** — the final, industrially-realistic system. A state-space controller with integral action and a Kalman observer levitate the sled against an over-actuated magnetic configuration, validated by a full closed-loop verification suite (separation principle, MIMO singular-value analysis, sensitivity-based robustness margins).

The single-magnet stage is exhaustive (six controllers + observer + comparison) because the goal there is *to understand which control technique fits magnetic levitation best*. The dual- and sled-unit stages then apply the chosen technique (state-space control with integral action) to systems where the choice begins to matter for real engineering reasons — primarily multi-DOF scaling.

## Repository Structure

```
.
├── 00_Shared_Library/                       Reusable Simulink subsystems
├── 01_Electromagnet_Modeling_Simscape/      Simscape Magnetic plant model
├── 02_Control_Design_SingleMagnet_Levitation/   Six controllers + EKF + evaluation
├── 03_Control_Design_DualMagnet_Levitation/     Multibody modeling validation
├── 04_Control_Design_SledUnit_Levitation/       5-DOF, 8-magnet final system
├── docs/                                    Detailed derivations and chapters
└── README.md
```

The numbered prefixes reflect the order in which the work was carried out and the order in which a new reader should explore the repository.

## Dependencies

| Component | Version |
|---|---|
| MATLAB | R2025a |
| Simulink | (matched) |
| Simscape | (matched) |
| Control System Toolbox | (matched) |
| Simulink Control Design | (matched) |
| Optimization Toolbox | (matched) |
| [acados](https://docs.acados.org/) — *optional*, only for the NMPC  | current release |

acados is required only to load and run the nonlinear MPC models in `02_*/03_Nonlinear_MPC/`. Without acados, the remaining controllers (PID, SSC, Backstepping, FBL, SMC) all work normally; the NMPC Simulink models simply will not load. Installation instructions are at the [acados documentation site](https://docs.acados.org/installation/).

## Quick Start

The controllers and observers are pre-designed and their parameters are saved in the respective `Controller_Params/`folders. To see the project work, you do not need to re-run the design scripts — just execute the two evaluation entry points:

```matlab
% From the repository root:
addpath(genpath(pwd));

% 1. Single-magnet comparison (PID, SSC, Backstepping, FBL+SSC, SMC)
cd 02_Control_Design_SingleMagnet_Levitation
run evaluation_controllers.m
%    → Results in 02_*/Results/

% 2. Sled-unit (5-DOF, 8 magnets) closed-loop test
cd ../04_Control_Design_SledUnit_Levitation
run evaluation_sled_levitation.m
%    → Results in 04_*/Results/
```

If you change any physical parameter (mass, nominal air gap, current limit, geometry), re-run the corresponding `design_*.m` script in `Controller_Design/` to regenerate the saved parameters before running the evaluation script.

## What's Inside

The repository is organized so that each system stage is self-contained. Detailed derivations, design choices, and implementation notes live in `docs/`; this README is the landing page.

### Stage 1A — Electromagnet Modeling (folder 01)

Folder `01_Electromagnet_Modeling_Simscape/` is the foundation of the project: building a Simscape Magnetic model of a single electromagnet, driving it from a supplied current source, and verifying its behavior against a mass with gravity using an ideal translational sensor. The goal of this stage is to **understand how to model an electromagnet using the Simscape Magnetic library** — what blocks to use, how the magnetic network connects to the mechanical domain, and how the resulting force matches the analytical reluctance-force expression.

Once validated, the electromagnet model is packaged as a referenced subsystem **`IntegratedElectromagnet`** and stored in `00_Shared_Library/`, so that every downstream control model in folders 02, 03, and 04 can reuse it as a single block rather than reconstructing the magnetic network each time.

**Modeling assumptions.** This electromagnet model adopts three simplifications that are standard for control-design purposes:

- **No iron saturation** — the magnetic material operates in the linear region of its B–H curve.
- **No leakage flux** — all flux generated by the coil passes through the air gap.
- **Homogeneous field distribution beneath the pole faces** — the flux density is uniform across the pole area.

These assumptions yield the analytical reluctance-force expression $F_R = K_M\,i_s^2 / x_s^2$ used throughout [§1](docs/01_system_modeling.md) and every controller derivation that follows. They are accurate for the operating range considered here and are the standard starting point for magnetic-bearing control. **Users targeting more realistic operating conditions** — large currents approaching saturation, geometry with significant fringe fields, or designs where leakage is non-negligible — can extend the Simscape model to capture those effects. The trade-off is increased simulation cost and a controller-design model that is no longer expressible in closed form.

### Stage 1B — Single-Magnet Levitation Control (folder 02)

The plant is a single electromagnet attracting a ferromagnetic mass against gravity, using the `IntegratedElectromagnet` subsystem from Stage 1A. Six controllers are designed and compared:

| Chapter | Topic |
|---|---|
| [§1 — System Modeling](docs/01_system_modeling.md) | Reluctance force, operating point, linearization |
| [§2 — PID](docs/02_pid.md) | Third-order pole placement + anti-windup |
| [§3 — State-Space with Integral Action](docs/03_state_space_control.md) | Augmented state, pole placement / LQR |
| [§4 — Backstepping](docs/04_backstepping.md) | Recursive Lyapunov design with integral state |
| [§5 — Feedback Linearization](docs/05_feedback_linearization.md) | Lie derivatives + outer state-space loop |
| [§6 — Sliding Mode Control](docs/06_sliding_mode_control.md) | Exponential reaching law + integral sliding surface |
| [§7 — Nonlinear MPC](docs/07_nonlinear_mpc.md) | acados-based formulation; offset-free MPC noted as future work |
| [§8 — Extended Kalman Filter](docs/08_ekf.md) | Universal observer, embedded in every controller |
| [§9 — Evaluation Framework](docs/09_evaluation_framework.md) | Programmatic batch comparison via `evaluation_controllers.m` |
| [§10 — Results & Discussion](docs/10_results.md) | Performance data and engineering interpretation |
| [§11 — Comparative Analysis](docs/11_comparative_analysis.md) | Choosing a controller — performance, complexity, scaling |

### Stage 2 — Dual-Magnet Levitation (folder 03)

A vertical sandwich of two electromagnets around a mobile mass, used to validate the Simscape Multibody modeling workflow (frame conventions, force–motion coupling, local-force vs. resultant-force comparison) on a system simple enough to cross-check by hand.

| Chapter | Topic |
|---|---|
| [§12 — Dual-Magnet Levitation](docs/12_dual_magnet_levitation.md) | Multibody modeling, differential drive, SSC with integral action |

### Stage 3 — Sled Unit Levitation (folder 04)

A 5-DOF magnetically levitated sled with eight electromagnets (over-actuated, with center-of-mass offset), built from imported CAD geometry. A state-space controller with integral action and anti-windup, paired with a Kalman observer, levitates the sled and is validated by a closed-loop verification suite (separation principle, MIMO singular-value analysis, sensitivity-based robustness margins).

| Chapter | Topic |
|---|---|
| [§13 — Sled Unit Levitation](docs/13_sled_unit_levitation.md) | CAD-based multibody modeling, redundant-allocation SSC, Kalman observer, MIMO verification |

## Results Highlight

The five integral-action controllers were benchmarked on the single-magnet plant under realistic measurement-noise conditions, with the same EKF feeding every controller. All five achieve sub-micrometer steady-state precision; their transient performance differs in characteristic ways.

<div align="center">
  <img src="02_Control_Design_SingleMagnet_Levitation/Results/TimeResponse_Noise_active.png" width="720" alt="Time Response"/>
</div>
> **Figure** — Air-gap time response of PID, SSC, Backstepping, FBL+SSC, and SMC under identical step + disturbance test conditions.

<div align="center">
  <img src="02_Control_Design_SingleMagnet_Levitation/Results/PerformanceCriteria_Noise_active.png" width="720" alt="Performance Criteria"/>
</div>
> **Figure** — Bar-chart comparison: overshoot, rise time, settling time, steady-state error.


Three observations stand out and are unpacked in [§10](docs/10_results.md) and [§11](docs/11_comparative_analysis.md):

1. **PID is the fastest to rise but produces the largest overshoot** — the classical SISO trade-off.
2. **FBL+SSC and Backstepping are empirically indistinguishable** — confirming their structural equivalence as a real engineering observation rather than just a theoretical curiosity.
3. **SMC produces dramatically the fastest settling time**, with competitive performance on every other metric — the practical payoff of robust control.

## Recommended Controller for Magnetic Bearing Applications

For a single-DOF demonstration or research-scale plant, any of the closed-form controllers performs well, and the choice is driven by what the project aims to demonstrate. For a general-purpose magnetic bearing system with **multiple degrees of freedom**, however, the structural argument favors a clear default:

> **State-space control with integral action** is the most practical default choice. It scales naturally and cleanly to MIMO (one larger matrix instead of a rewritten design), its tuning effort grows minimally with DOF, its real-time cost stays a matrix-vector product, and it pairs cleanly with the same EKF observer family used throughout this project. The nonlinear methods (FBL, Backstepping, SMC) remain compelling for specialized applications — large operating ranges, dominant model uncertainty, hard disturbance rejection — and should be added on top of an SSC baseline when those needs arise, rather than chosen instead of it.

This recommendation is the central engineering conclusion of the project, demonstrated concretely by the dual-magnet (§12) and sled-unit (§13) systems, where extending SSC to multi-DOF required no structural change to the design workflow.

## Status and Future Work

The single-magnet stage is complete (six controllers, EKF, evaluation, comparative analysis). The dual-magnet and sled-unit stages are complete for the documented scope. Two extensions are documented as natural next steps but are not part of the current scope:

- **Offset-free NMPC** — the standard NMPC formulation in §7 exhibits a residual steady-state error against the realistic Simscape plant. An offset-free formulation augmenting the prediction model with a disturbance state would close this gap, with implementation complexity comparable to redesigning the OCP and disturbance observer.
- **EKF on the sled-unit system** — the §13 controller uses a linear Kalman filter for measurement-noise rejection. An EKF analogous to §8, re-derived for the multi-DOF nonlinear dynamics, would extend the project's hardware-readiness story to the sled-unit case.

## Acknowledgments

This work was carried out as a student-assistant research project. The high-speed-machine system that motivates the sled-unit chapter (§13) is part of a larger research effort led by the project supervisor; the sled-unit work in this repository is a controls and modeling contribution to that broader effort.

## License

This project is released under the [MIT License](LICENSE).
