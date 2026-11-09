# Next experiment: geometry-aware RGB–thermal motion uncertainty

**Design, 9 November 2026 · branch `ore-ms-orb-development` · base `5bea798bf`.**
**Status: experiment defined; implementation, asset recovery and execution freeze pending. No new scientific results.**

## Question and motivation

**Does propagating MAC-VO/MACThermal feature uncertainty through motion geometry provide a better reliability signal and better RGB–thermal motion combination than scalar uncertainty or fixed weighting?**

The previous angular-trace test answers a narrower question about one scalar selector. It does not settle full-covariance weighting, geometry-aware uncertainty, improved thermal representations or learned reliability. The weekly brief records progress; it is not a decision to end method development. This plan supersedes the recommendation to make reporting the next research endpoint, while preserving all previous experiment outcomes.

MAC-VO uses feature covariance for selection and optimization. MACThermal adapts the frontend to thermal sensing. MAC-I² provides the closest mechanism for this experiment: obtaining pose uncertainty from feature uncertainty and the registration Jacobian before fusing constraints. Applying that mechanism to two visual experts is a hypothesis to test, not an already validated transfer from visual–inertial fusion.

Primary references: [MAC-VO](https://arxiv.org/abs/2409.09479), [MACThermal thesis record](https://publications.ri.cmu.edu/3d-thermal-perception-for-autonomous-navigation-in-visually-degraded-unstructured-environments), and [MAC-I², §IV-A and Appendix B](https://arxiv.org/html/2609.07116).

## What changes, and what stays controlled

Keep the RGB and thermal checkpoints, image preparation, correspondences and source axes fixed. Use matched native correspondence/3D covariance snapshots. Add an offline two-frame registration replay and a pose-covariance readout; keep original native poses as a baseline. This permits a real method experiment without training a new representation first.

The local repository already contains a 3D-registration graph (`ICP_TwoframePGO`) whose residual covariance combines source-point and transformed target-point covariance. It is an implementation starting point, not proof that the existing optimizer satisfies the new covariance contract. The current native controls often use reprojection/disparity residuals and capped robust solves.

[EXP079](../../experiments/EXP-079-optimizer-factor-audit.md) reconstructed unrobust, fixed-landmark information. [EXP080](../../experiments/EXP-080-factor-proxy-comparison.md) tested residual/conditioning selectors and [EXP081](../../experiments/EXP-081-factor-geometry-audit.md) decomposed those quantities. This experiment must add both-endpoint 3D uncertainty and a verified local solve, rather than invert the old conditional matrix and rename it calibrated pose covariance.

## Method to implement

1. **Replay frozen measurements.** For each camera, obtain paired 3D points, both endpoint covariances, original feature identities, timestamps, native initialization and pose. Reconstruct from saved raw frontend outputs if sufficient; otherwise make one bounded capture per camera/recording with existing instrumentation. Never substitute model-predicted correspondences as reference truth.
2. **Solve the stated registration problem.** Use the MAC-I²-inspired weighted 3D residual, with covariance `C_i = C_source_i + R C_target_i Rᵀ`. Declare the exact rotation-dependent weighting/linearization convention and solver before execution. Save objective, gradient/step termination, rank and failure status. Do not reuse the historical raw-Huber/weighted-linearization mismatch. LM damping is a numerical device and is excluded from reported information.
3. **Propagate to motion uncertainty.** Form local information `H = sum(J_iᵀ C_i⁻¹ J_i)` in a declared SE(3) tangent convention and compute the undamped inverse only when identifiable and numerically valid. This is an approximate model covariance; calibration is an outcome, not an assumption. Preserve convergence failures and singular cases. Do not manufacture finite certainty with a pseudoinverse or outcome-selected regularization.
4. **Use one common target.** Transport means and full 6×6 covariances through the same existing nominal frame/interval mapping, including the Jacobian of the interval mapping. Preserve translation–rotation cross terms and lever-arm effects. Report translation in metres and rotation in radians/degrees separately; do not compare a mixed-unit 6D trace. Verify the covariance transport numerically before scoring.
5. **Compare combination rules on identical means.** Fuse the same replayed RGB/thermal motions with equal weights, block-isotropic pose covariance, and full pose covariance. Block-isotropic means separate isotropic translation/rotation blocks, preserving each block trace and discarding cross terms. This isolates covariance shape from a changed pose estimator.

The source/target feature errors and different feature factors can also be correlated; the local information construction initially uses the paper’s approximate factor-independence model. Save that assumption and test empirical calibration rather than asserting that propagation removes it.

The two visual estimates may have correlated errors. The primary covariance-based combination will use covariance intersection in a common local tangent, with label-free log-determinant weight selection on a fixed grid `w = 0, .05, …, 1`, transported consistently through the SE(3) iteration. The scalar-covariance arm uses the same machinery. Independence-based information fusion is not the default. Covariance intersection does not fix bias, incorrect marginal covariances or reference uncertainty; report those assumptions explicitly.

## Data and baselines

Initial development cohort: first **600 original groups, IDs 0–599**, from each of the already exposed MS² recordings:

| Condition | Recording |
|---|---|
| Day | `_2021-08-13-16-31-10` |
| Night | `_2021-08-13-22-16-02` |
| Rain | `_2021-08-06-16-19-00` |

Keep complete denominators and unavailable transitions; do not choose intervals using error, support or uncertainty. Verify actual source stems/counts in a metadata manifest before the execution freeze. If an exact requested prefix is absent, report that fact before changing the design. These recordings are development data, including the two recently evaluated ones. They cannot establish fresh generalization.

Baselines are native RGB-only/thermal-only, replay RGB-only/thermal-only, equal-weight replay fusion, block-isotropic covariance combination, and the old angular-trace selector on identical replay means. Full-covariance combination is the primary candidate. Also compare the angular rule on its original native means as a historical-system control. No retrospective oracle or recording-specific best camera is a deployable method.

Initial budget: at most **3,600 expert-frame calls** if cached fields are insufficient, plus CPU replay. No checkpoint training in this experiment. This is a first comparison chosen for attribution and speed; it is not a prohibition on subsequent learning or stronger models.

## Measurements and decision

**Primary endpoint:** relative rotation error on identical common-reference intervals, using the existing nominal/provided-reference contract. Report per recording and equal-recording mean. Translation error, joint/native/output coverage, unsupported transitions, tails, numerical failures, extra runtime and memory are mandatory companion results. Per-step results do not automatically establish trajectory VO improvement.

**Uncertainty endpoints:** rotation and translation calibration separately, proper Gaussian scores where their covariance assumptions are valid, empirical coverage, and whether predicted relative risk chooses the lower-error camera better than angular trace and simple support/context controls. Reference noise and biases limit absolute calibration claims. Lock predictions/covariances before joining reference labels.

**Development advancement rule, chosen for this new experiment:** full covariance should reduce equal-recording mean rotation error by at least 5% against both equal weighting and the strongest fixed single-camera control (native or replay), improve in at least two of the three recordings, worsen translation by no more than 5% in any recording, and introduce no additional missing delivery relative to equal weighting. Report the full-covariance versus scalar-covariance difference whether positive or negative. These are engineering usefulness criteria, not statistical significance or inherited thresholds from the stopped scalar experiment. Compare candidates on shared support and retain whole-axis coverage so abstention cannot manufacture a win.

If numerical availability differs, the comparison is not a clean accuracy win; account for every failure before interpreting common-support errors. Do not claim population confidence from three recordings or treat adjacent frames as independent trials.

## What each outcome enables

- **Useful calibration and combination gain:** execute a recording-separated replication with a frozen model; then test accumulated trajectory error and estimator integration.
- **Better risk prediction but no motion gain:** determine whether useful camera-selection headroom exists under the stated task before increasing model capacity.
- **Poor covariance scale with useful ranking:** a bounded pose-level calibration or uncertainty-head fine-tuning experiment on separate development/training recordings becomes justified. Repeated constant pixel-scale fitting is not the proposed next step.
- **Biased means or weak features dominate:** MACThermal normalization/augmentation or a pretrained shared representation becomes a concrete alternative, evaluated with fixed-mean versus changed-frontend controls.
- **Uninformative result:** retain the evidence and revise the specific mechanism; do not interpret it as failure of all multimodal fusion or launch an unbounded parameter sweep.

MAC-I²-style IMU constraints are another credible extension once synchronized streams and the intended target are verified. They are not added simultaneously here, because doing so would obscure whether the visual covariance bridge helped.

## Implementation milestones and outputs

**A. Data/solver feasibility:** recover one existing bounded packet, verify whether both endpoint covariances are recoverable, pin the solver/covariance conventions and exact input manifests. If not, specify the minimal capture required; no archive-wide recovery campaign.

**B. Implement and verify:** finite-difference residual/transport Jacobians; synthetic SE(3) Monte Carlo covariance check; covariance-intersection consistency fixtures; exact unchanged-frontend checks. Freeze code, runtime, source IDs and comparison rules before the first real outcome join.

**C. Run and decide:** execute the three-recording comparison once under that freeze, save source-axis outputs and one result table/figure, and update the weekly brief with measured gains, failures and the resulting next experiment.

Deliverables: a small opt-in replay/covariance tool, numerical verification artifacts, locked predictions, complete comparison table and a short experiment conclusion. The current task defines this experiment; it does not claim those tools or results already exist.
