# 9. Evaluation Framework

The script `evaluation_controllers.m` is the single entry point for comparing the six EKF-based controllers under identical conditions: PID, SSC, FeedbackLin, Backstepping, SMC, and the offset-free NMPC. It runs every model headlessly, collects the same set of signals from each, computes a fixed set of metrics, and produces a bundle of figures and raw data in the `Results/` folder.

## 9.1 What It Does

1. Loads the shared simulation parameters from `setup_sim_params.m`.
2. For each controller in the model list: loads its parameter file, runs the Simulink model via `sim()`, and extracts three logged signals from `logsout`:
   - `meas_airgap` — the noisy measured air gap [mm]
   - `mass_pos` — the Simscape ground-truth mass position [mm]
   - `cmd_current` — the controller's command current [A]
3. Computes two sets of performance criteria — one for the initial transient, one for disturbance rejection — and a set of current-quality metrics.
4. Generates five comparison figures and writes them, together with the raw signal data, to `Results/`.

The `_test` variants in the project (`Maglev_SSC_test.slx`, `Maglev_SMC_block_test.slx`) are deliberately excluded from the comparison — they exist as sanity checks on their EKF counterparts, not as evaluation candidates.

## 9.2 Apples-to-Apples by Construction

Every controller is exercised against the same plant, the same EKF, the same setpoint trajectory, the same disturbance profile, and the same measurement noise — all defined once in `setup_sim_params.m`. The resulting comparison is therefore a true test of the control-design choices alone: any difference observed in the metrics is attributable to the controller, not to differences in the test conditions. This was the central design intent of the framework and remains unchanged from the earlier five-controller version.

## 9.3 Two-Phase Evaluation and Time Windows

A single simulation run exposes the controllers to two distinct events: a **reference step at $t = 0$** (the air gap must move from `airgap_init` to `airgap_soll`), and an **external-force step at $t = T_{\text{stp}}$**. These phases stress different controller properties — settling behavior versus disturbance rejection — and the evaluation reports them as two independent metric sets rather than collapsing them into a single transient.

Four time windows define the metrics consistently:

| Window | Range | Used for |
|---|---|---|
| Initial transient | $0 \leq t < T_{\text{stp}}$ | settling-process metrics |
| Unforced steady state | $0.7 T_{\text{stp}} \leq t < T_{\text{stp}}$ | residual error before disturbance |
| Disturbance window | $T_{\text{stp}} \leq t \leq T_{\text{sim}}$ | disturbance-rejection metrics |
| Final steady state | $T_{\text{stp}} + 0.9 (T_{\text{sim}} - T_{\text{stp}}) \leq t \leq T_{\text{sim}}$ | residual error under sustained disturbance |

All window boundaries are expressed as fractions of `T_stp` and `T_sim`, so changing the simulation horizon in `setup_sim_params.m` automatically rescales the metric definitions. The steady-state windows are deliberately *time-window* based rather than *sample-count* based, which keeps them robust against variable-step solvers and any future change in the model's solver configuration.

## 9.4 Metrics

**Initial transient (five metrics).**
- *Overshoot* — peak excursion past the target, normalized to the reference step.
- *Rise time* — $10\% \to 90\%$ of the reference step.
- *Settling time* — last time the trajectory exits the $\pm 2\%$ band around the target.
- *Unforced steady-state error* — mean deviation in the unforced-steady-state window.
- *Transient control energy* — $\int_0^{T_{\text{stp}}} (i_s - i_{\text{eq}})^2 dt$, with $i_{\text{eq}}$ estimated from the pre-disturbance current plateau.

**Disturbance rejection (six metrics).**
- *Peak deviation* — maximum $|x_s - x_0|$ in the disturbance window.
- *Recovery time* — disturbance onset → last exit of the $\pm 2\%$ band.
- *Final steady-state error* — mean residual in the final-steady-state window.
- *IAE (Integral of Absolute Error)* — $\int |e(t)| dt$ over the disturbance window.
- *ITAE (Integral of Time-weighted Absolute Error)* — $\int t |e(t)| dt$ over the disturbance window, with $t$ measured from disturbance onset. Penalizes slow recovery more heavily than IAE and so distinguishes "fast peak, fast recovery" from "fast peak, slow recovery" — a distinction IAE alone cannot draw.
- *Disturbance control energy* — $\int_{T_{\text{stp}}}^{T_{\text{sim}}} (i_s - i_{\text{eq}})^2 dt$.

**Current quality (three metrics, computed over the command current).**
- *Peak current* — full simulation. Sets the actuator sizing requirement.
- *RMS current* — full simulation, time-weighted ($\sqrt{\tfrac{\int i^2\,dt}{T_{\text{sim}}}}$, not sample-mean). Drives thermal load.
- *Total Variation (TV)* — $\sum |\Delta i|$ over the first 200 ms of the disturbance window. Captures true controller activity under excitation, separating it from quiescent sensor-noise tracking.

The energy metric is reported **separately** for the transient and disturbance phases rather than as a single total. A controller that uses heavy actuation during startup but recovers passively under disturbance is a structurally different design from one that does the opposite; conflating the two into one number hides the distinction.

## 9.5 Output Files

The script writes five PNG figures and one MAT file to `Results/`, all suffixed with the noise-state tag (`noise_active` if `isNoiseActive` is true, `noise_deactive` otherwise) read from `setup_sim_params.m`:

| File | Content |
|---|---|
| `TimeResponse_*.png` | Air-gap trajectories — full run, settling subplot, disturbance subplot, steady-state subplot |
| `PerformanceCriteria_*.png` | Bar chart of the five initial-transient metrics |
| `DisturbanceRejection_*.png` | Bar chart of the six disturbance-rejection metrics |
| `CurrentResponse_*.png` | Command-current trajectories in the same four-window layout as the air-gap figure |
| `CurrentQuality_*.png` | Bar chart of the three current-quality metrics |
| `simulation_results_*.mat` | Raw signals and computed metrics for user-defined post-processing |

The two current-figure outputs are produced only if the `cmd_current` signal is present in `logsout` for at least one model — controllers that do not export current still appear in the air-gap figures.

## 9.6 Plotted Coordinates

The single-magnet plant of [§1](01_system_modeling.md) is one-dimensional: the air gap $x_s$ and the mass position are related by a fixed offset, so **evaluating the air gap is equivalent to evaluating the mass position** — there is only one trajectory to look at, only the coordinate frame in which it is plotted changes.

The theoretical chapters work in the **deviation-coordinate frame** $x_1 = x_0 - x_s$, where the equilibrium sits at $x_1 = 0$ and the controllers are designed to keep the mass at that origin. This frame is the natural one for derivations because the linearization is expressed around it.

The evaluation script plots in **absolute air-gap coordinates** — the y-axis reads directly in millimetres, `airgap_soll` is drawn as a horizontal reference line, and the $\pm 2\%$ band as dotted lines around it. This frame is the natural one for visual inspection: the configured target appears on the y-axis as a recognizable physical quantity (e.g. 2.0 mm), rather than as the abstract origin of a deviation variable. Both conventions describe the same physical signal.

## 9.7 Note on the Sled-Unit Evaluation

A parallel script `evaluation_sled_levitation.m` in folder 04 plays the same role for the 5-DOF sled-unit system — executes `SledUnit_Levitation_Control.slx` programmatically, collects air-gap responses, current commands, and sled-pose trajectories, plots them, and writes the figures and raw `simulation_results.mat` to `04_*/Results/`. See [§13.6](13_sled_unit_levitation.md) for details.
