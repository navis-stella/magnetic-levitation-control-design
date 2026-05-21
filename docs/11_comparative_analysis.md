# 11. Comparative Analysis: Choosing a Controller

The [§10](10_results.md) results established that all five closed-form controllers stabilize the single-magnet system with sub-micrometer steady-state precision. This chapter addresses the more practical engineering question: **given a specific application context — number of degrees of freedom, model fidelity, real-time budget, tuning expertise — which controller should you actually choose?** The comparison below covers four dimensions: performance, implementation complexity, tuning complexity, and real-time cost, ending with the scaling argument that motivates a single concrete recommendation for general-purpose magnetic bearing systems.

Only the five integral-action controllers (PID, SSC, Backstepping, FBL+SSC, SMC) are compared here, since these are directly evaluable through [§10](10_results.md). NMPC is treated separately in [§7](07_nonlinear_mpc.md) because of the residual-error caveat in [§7.5](07_nonlinear_mpc.md).

## 11.1 At-a-Glance Comparison

| | **PID** | **SSC** | **FBL+SSC** | **Backstepping** | **SMC** |
|---|---|---|---|---|---|
| **Transient performance** | Fast rise, very high overshoot | Slow rise, lowest overshoot, smooth | Moderate rise, moderate overshoot | Moderate rise, moderate overshoot | Fast rise, best settling time |
| **Steady-state precision** | sub-μm | sub-μm | sub-μm | sub-μm | sub-μm (integral variant) |
| **Robustness to model error** | Low | Moderate | Low (exact cancellation) | Low (exact cancellation) | High |
| **Implementation complexity** | Lowest | Low | Medium (Lie derivatives, sqrt, division) | Medium (recursive Lyapunov design) | Medium (sat function, sliding surface, integral state) |
| **Tuning knobs** | 3 design parameters ($\zeta, \omega_n, \alpha$) | $Q, R$ matrices (or pole locations) | Same as SSC (outer loop only) | 3 coefficients $(c_0, c_1, c_2)$ — coupled into gains | 5 parameters $(\lambda, k_i, \eta, K, \phi)$ — interacting |
| **Real-time cost (per sample)** | Trivial (a handful of multiplications) | Trivial (matrix-vector product) | Moderate (sqrt, division per step) | Moderate (similar to FBL) | Moderate (sat, sqrt, conditional logic) |
| **MIMO / multi-DOF scaling** | Difficult (decentralized; ignores coupling) | **Excellent** (just a larger matrix) | Hard (relative degree, decoupling matrix) | Hard (recursion explodes with state coupling) | Hard (one surface per output, decoupling) |

## 11.2 Where Each Controller Excels

**PID** is the right choice when implementation simplicity dominates the specification, the operating range is small, and the team has stronger SISO than MIMO design experience. For a single-DOF demonstration with a forgiving plant, it remains an excellent baseline — the [§10](10_results.md) data even gives it the fastest rise time, although at the cost of by far the largest overshoot.

**State-space control** is the right choice when the operating range stays close to equilibrium, the model is reasonably well-known, and multiple states need to be coordinated through full feedback. Its tuning effort is comparable to PID (designer chooses pole locations or LQR weights), its real-time cost is essentially identical to PID (a matrix-vector product), and its scaling to higher dimensions is dramatically better than any of the nonlinear methods ([§11.3](11_comparative_analysis.md)).

**FBL+SSC and Backstepping** are the right choice when the operating range extends beyond the linearization neighborhood and the model parameters $K_m, m$ are known to good accuracy. The [§10](10_results.md) data showed these two perform essentially identically — confirming the structural-equivalence argument of [§4.3](04_backstepping.md) — so the choice between them comes down to which design philosophy the team is more comfortable with: geometric (Lie derivatives, exact cancellation) or recursive (Lyapunov design with virtual controls). Once one is implemented, the other contributes very little new performance.

**SMC** is the right choice when robustness against parametric uncertainty or external disturbances is the dominant requirement. The [§10](10_results.md) settling-time advantage and the intrinsic insensitivity to matched disturbances are real engineering payoffs. The trade-off is tuning complexity — more parameters with stronger interactions — and the need for boundary-layer reasoning to avoid chattering. The non-obvious tuning constraint $\lambda > \sqrt{2g/x_0}$ documented in [§6.1](06_sliding_mode_control.md) also makes the design less forgiving than the others.

## 11.3 The Multi-DOF Question — The Structural Argument for SSC

The single-magnet system is 1-DOF, which obscures a major practical consideration: **how do these controllers scale when the system has multiple degrees of freedom?** Active magnetic bearings for rotating machinery typically have 4–5 DOF; general-purpose maglev systems are often 6-DOF. The five controllers scale very differently:

- **PID** scales by replication — one decentralized loop per axis. This ignores cross-axis coupling and becomes inadequate as the dynamics couple more strongly. A truly MIMO PID exists in principle but loses the implementational simplicity that justifies PID in the first place.
- **SSC scales linearly**. A 6-DOF problem produces a state vector of position + velocity + integral states (typically 18 entries); the design workflow is *structurally identical* to the 1-D case, just with larger matrices. MATLAB's `place`, `lqr`, and `kalman` all work natively in MIMO; pole placement and LQR are no more conceptually difficult in 18 dimensions than in 3. The dual-magnet design ([§12](12_dual_magnet_levitation.md)) and the sled-unit design ([§13](13_sled_unit_levitation.md)) are concrete demonstrations: extending SSC from 1-D to higher dimensions required only redefinitions of the augmented system matrix, with no structural change to the implementation.
- **Feedback linearization in MIMO** requires choosing outputs with matched relative degree, computing the decoupling matrix, and dealing with singularities of the input transformation. The complexity grows superlinearly with DOF, and the design becomes increasingly bespoke.
- **Backstepping in MIMO** loses the clean recursive structure as soon as states become coupled. Each virtual control depends on the partial derivatives of all preceding virtual controls, and the resulting symbolic expressions grow combinatorially.
- **SMC in MIMO** requires designing one sliding surface per controlled output, with explicit decoupling design between them. Workable, but rarely elegant; chattering also becomes harder to manage across multiple surfaces simultaneously.

This is the structural argument for state-space control as a default choice in magnetic bearing applications: **its complexity is essentially DOF-independent**, while the nonlinear analytical methods scale poorly enough that they become bespoke design exercises at the DOF count of practical interest.

## 11.4 Recommendation

For a single-DOF demonstration or research-scale plant, any of the five controllers is appropriate, and the choice is driven by what the project aims to demonstrate. The [§10](10_results.md) results favor SMC on raw performance metrics, but the implementation and tuning trade-offs documented above mean SMC is not automatically the "right" answer in every context.

**For a general-purpose magnetic levitation or active magnetic bearing system with multiple degrees of freedom, state-space control with integral action is the most practical default**:

- It scales naturally and cleanly to MIMO without structural redesign.
- Its tuning effort grows minimally — a designer who can apply `lqr` to a 1-D system can apply it to a 6-D system.
- Its real-time cost stays a matrix-vector product, with predictable timing across embedded hardware.
- It pairs cleanly with the same EKF observer ([§8](08_ekf.md)), re-derived once for the higher-dimensional plant, keeping the full sensing-and-control stack consistent.

The nonlinear methods (FBL, Backstepping, SMC) remain compelling for specialized applications — large operating ranges, dominant model uncertainty, hard real-time disturbance rejection — and should be added *on top of* an SSC baseline when those specific needs arise, rather than chosen *instead of* SSC. NMPC, once equipped with the offset-free disturbance model discussed in [§7.5](07_nonlinear_mpc.md), becomes attractive whenever explicit constraint handling matters more than computational simplicity, with the same multi-DOF scaling caveats as the analytical nonlinear methods plus the added cost of an external solver dependency.

The dual-magnet model in [§12](12_dual_magnet_levitation.md) and the sled-unit model in [§13](13_sled_unit_levitation.md) demonstrate this recommendation concretely: extending SSC from one magnet to two, and then to five DOFs with eight magnets, was at each stage a straightforward extension of the augmented-state matrix; redoing the equivalent extensions for FBL or Backstepping would have required reconstructing each design from scratch.
