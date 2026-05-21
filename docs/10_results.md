# 10. Results & Discussion

The script `evaluation_controllers.m` (see [§9](09_evaluation_framework.md)) runs every EKF-based controller against the same plant, the same EKF, the same step reference (initial air gap 2.5 mm → target 2.0 mm), and the same disturbance and measurement-noise profiles defined in `setup_sim_params.m`. All five controllers are evaluated with **noise active** on the position measurement — the realistic scenario that mirrors hardware deployment.

<div align="center">
  <img src="../02_Control_Design_SingleMagnet_Levitation/Results/TimeResponse_Noise_active.png" width="720" alt="Time Response"/>
</div>

> **Figure 10.1** — Air-gap trajectories of PID, SSC, FBL+SSC, Backstepping, and SMC.
> Top: full one-second run with disturbance near t = 0.5 s.
> Bottom-left: settling subplot (0 ≤ t ≤ 0.3 s). Bottom-right: steady-state region (t > 0.8 s).

<div align="center">
  <img src="../02_Control_Design_SingleMagnet_Levitation/Results/PerformanceCriteria_Noise_active.png" width="720" alt="Performance Criteria"/>
</div>

> **Figure 10.2** — Bar-chart comparison of overshoot, rise time, settling time,
> and steady-state error across the five controllers.

## 10.1 Time Response

All five controllers stabilize the levitation gap and reject the step disturbance applied around $t = 0.5\,\text{s}$. The visible differences live almost entirely in the transient phase: PID exhibits a large, oscillatory excursion before settling; SSC produces the most damped response among the linear designs; FBL+SSC and Backstepping are visually indistinguishable from one another; SMC settles fastest and tightest.

In the steady-state region ($t > 0.8\,\text{s}$), all five trajectories occupy essentially the same noise envelope. This is by design — every controller is fed the same EKF estimate, so the residual fluctuation reflects the filter's measurement-noise pass-through rather than any difference in controller behavior. The fact that the noise band is consistent across controllers is a useful sanity check on the experiment: any quantitative difference we observe is attributable to controller design, not to a difference in test conditions.

## 10.2 Performance Metrics

Four standard metrics are computed for each controller: overshoot, rise time, settling time, and steady-state error. Three patterns emerge that connect directly back to the theoretical chapters.

**The classical PID trade-off.** PID achieves the fastest rise time of all five controllers, but at the cost of by far the largest overshoot. This is the expected behavior of an aggressively tuned 3rd-order pole-placement design ([§2.2](02_pid.md)): the gains needed to push the rise time toward the open-loop instability rate $\sqrt{2g/x_0}$ inevitably produce a poorly damped transient. SSC trades this rise time away in exchange for the lowest overshoot among the linear group — the natural payoff of moving from SISO pole placement to full-state feedback with explicit damping design ([§3](03_state_space_control.md)).

**Empirical confirmation of FBL ≡ Backstepping.** The performance numbers for FBL+SSC and Backstepping match each other to within a small fraction across every metric. [§4.3](04_backstepping.md) predicted this from the algebra — the two methods produce structurally equivalent control laws, differing only in which design knobs are exposed to the engineer. The metrics make this explicit: choosing between the two is a question of design preference (geometric/Lie-derivative perspective vs. recursive Lyapunov perspective), not a question of closed-loop performance.

**SMC's settling-time advantage.** SMC's settling time is roughly an order of magnitude lower than every other controller, while its rise time and overshoot remain competitive. This is the practical payoff of the switching-based design ([§6](06_sliding_mode_control.md)): immediately after the disturbance, the reaching law forces the trajectory back into the boundary layer, whereas the linear-feedback controllers must slowly drive the residual back inside the tolerance band through the linear closed-loop dynamics. The integral-action variant of [§6.8](06_sliding_mode_control.md) (the version benchmarked here) gives SMC the same zero-steady-state-error property as the others, so its robustness advantage is no longer paid for with residual offset.

**Steady-state error.** All controllers achieve sub-micrometer precision at steady state — the integral action across PID, SSC, Backstepping, FBL+SSC, and the integral-SMC variant is operating as designed. The small spread across controllers is comparable to the EKF measurement-noise floor and should not be interpreted as a meaningful controller-to-controller difference.

## 10.3 Why Noise Active

The evaluation deliberately runs with the position-measurement noise injected and the EKF in the loop, rather than with the ideal Simscape state. This makes the comparison representative of hardware: the controllers are judged not on how well they would handle a perfect signal, but on how well they cooperate with the same realistic observer they will face on the rig. The `_ideal` reference models (`Maglev_SSC_ideal.slx`, `Maglev_SMC_block_ideal.slx`) remain available as noiseless sanity checks, but are not part of the headline comparison for exactly this reason.

## 10.4 Takeaways for Controller Selection

- **PID** delivers the fastest rise but the worst transient — appropriate if rise time dominates the specification and the overshoot can be tolerated, otherwise dominated by the alternatives.
- **SSC** is the smoothest linear-only option — first choice when the operating range stays close to equilibrium and damping matters more than speed.
- **FBL+SSC and Backstepping** behave identically; either is appropriate when the operating range extends beyond the validity of the linearization and the model parameters are well-known.
- **SMC** is the most robust option, with the best settling time and tight disturbance rejection — preferred whenever model uncertainty or external disturbances are expected to dominate.

These takeaways are the empirical foundation for the broader engineering recommendation in [§11](11_comparative_analysis.md), which considers implementation complexity, tuning effort, real-time cost, and multi-DOF scaling in addition to the raw performance summarized above.
