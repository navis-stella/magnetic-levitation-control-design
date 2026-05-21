# 9. Evaluation Framework

The script `evaluation_controllers.m` is the single entry point for comparing the controllers under identical conditions.

## What It Does

1. Loads the shared simulation parameters from `setup_sim_params.m`.
2. Programmatically runs each of the EKF-based Simulink models (PID, SSC, Backstepping, FBL+SSC, SMC) using `sim()` — no model needs to be opened in the editor.
3. Collects the measured air gap and the actual mass position (Simscape ground truth) from each run.
4. Plots the trajectories together for direct visual comparison and writes the figures and signals to the `Results/` folder.

## What This Enables

Because every controller uses the same plant, the same EKF, the same setpoint trajectory, and the same disturbance/noise profile (all defined in `setup_sim_params.m`), the resulting comparison is a true apples-to-apples test of the control-design choices alone. Differences in settling time, overshoot, steady-state precision, and disturbance rejection between controllers are attributable to the controller, not to differences in their test conditions.

The `_ideal` variants (`Maglev_SSC_ideal.slx`, `Maglev_SMC_block_ideal.slx`) are not included in the evaluation — they exist as sanity checks on their EKF counterparts, not as candidates for the comparison.

## Output Files

The script writes to `Results/`:
- `TimeResponse_Noise_active_.png` — overlay of air-gap trajectories with a settling-process subplot and a steady-state-region subplot.
- `PerformanceCriteria_Noise_active_.png` — bar chart comparing overshoot, rise time, settling time, and steady-state error across the five controllers.
- `simulation_results.mat` — raw signals from each run, available for any user-defined post-processing.

## Convention for Plotted Coordinates

All position trajectories are plotted in deviation coordinates ($x_1 = x_0 - x_s$). The equilibrium air gap corresponds to $x_1 = 0$; positive values indicate a smaller gap than equilibrium, negative values a larger one. This convention applies to every controller throughout the project, since all controllers are designed in these coordinates.

## Note on the Sled-Unit Evaluation

A parallel script `evaluation_sled_levitation.m` in folder 04 plays the same role for the 5-DOF sled-unit system — executes `SledUnit_Levitation_Control.slx` programmatically, collects air-gap responses, current commands, and sled-pose trajectories, plots them, and writes the figures and raw `simulation_results.mat` to `04_*/Results/`. See [§13.6](13_sled_unit_levitation.md) for details.