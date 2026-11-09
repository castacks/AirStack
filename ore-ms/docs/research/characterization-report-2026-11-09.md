# RGB–thermal geometric uncertainty: characterization and feasibility report

2026-11-09, America/New_York. Synthesis of recorded findings through `831a4883d` on `ore-ms-orb-development`. No new inference, scoring, fitting or dataset access was performed for this report. Numerical results below are taken from the linked experiment reports; their raw artifacts were not re-audited for this synthesis.

## Research question and conclusion

Does predicted RGB/thermal geometric uncertainty identify which camera supplies more reliable measurements, and does using that signal improve visual odometry?

The completed work supports useful within-camera uncertainty ranking, reproducible baseline comparisons and a bounded engineering demonstration of availability recovery. It does not establish a generally useful cross-camera selector, calibrated cross-modal covariance, or an adaptive VO improvement. The latest unchanged-rule whole-record evaluation stopped under its predeclared coverage gate. Its matched comparisons also failed to demonstrate the required benefit over RGB.

This is a characterization and feasibility result. The current fixed-rule evaluation route is closed. Patch-gap confidence development remains closed and thermal-bias microdiagnostics remain parked under the [research decision](research-decision-2026-11-05.md) and [completed evaluation disposition](decisive-uncertainty-findings-2026-11-06.md).

## Evidence map

| Question | Finding | Interpretation and source |
|---|---|---|
| Does uncertainty rank error within a camera? | Controlled mean-L2 selective-risk median reductions of about 9–37% against matched random on the examined windows. | Useful within-camera ranking under supplied-reference assumptions; neither covariance calibration nor a camera winner. [MS² synthesis](ms2-findings-2026-10-26.md). |
| Does a pointwise cross-camera score beat simple alternatives? | Day learned selection improves over camera-scale control but loses to always RGB in every examined mean-distance case; night loses to RGB and random in every examined frame under both losses. | Comparator choice and recording dependence matter. [MS² synthesis and replication links](ms2-findings-2026-10-26.md). |
| Do pooled motion gains establish adaptation? | EXP132 angular-rule pooled rotation mean is 0.151686 degrees versus RGB 0.420089, but neither fixed uncertainty rule beats the best fixed camera within any recording. | A known RGB-collapse interval supplies 106.434% of the angular rule's net pooled rotation improvement; other intervals collectively lose to RGB. [EXP132](signal-transfer-findings-2026-11-04.md). |
| Is tracking support an accuracy measure? | Thermal ORB has higher supported-pose coverage in day/night/rain but higher translation RPE in all three. | Keep availability and error separate. All six ORB trajectories break. [EXP143](orb-full-record-findings-2026-11-05.md). |
| Can missing source continuity be repaired within a declared budget? | EXP122 retained-input catch-up restores 85/95 deliveries and all four return episodes, with 141 native pair calls versus full-read 158. | Synthetic serial-offline buffering evidence; no natural-fault, physical-accuracy or live-cadence validation. [EXP122](buffered-handover-findings-2026-11-03.md). |
| Does the final unchanged uncertainty rule pass whole-record gates? | Night joint coverage is 89.68%, below 95%; day oracle headroom is 9.04%, below 10%. Neither recording meets the 5% candidate-benefit gate. | Primary outcome is insufficient evaluation; no validated adaptive benefit. [Final evaluation](decisive-uncertainty-findings-2026-11-06.md). |

These studies reuse windows, models and reference conventions. Experiment labels, output counts and audit checks are not counts of independent scientific replications. Pixel selective risk, adjacent-motion error, trajectory RPE and delivery availability are different endpoints and are not pooled into a single performance claim.

## Final whole-record evaluation

The [execution lock](decisive-uncertainty-execution-2026-11-06.md), freeze `3ec9ed240`, assigned complete day `_2021-08-13-16-31-10` and night `_2021-08-13-22-16-02` recordings. Four continuous native runs completed on 5,695 day and 5,234 night groups, yielding 21,858 expert-frame outputs with failures/support flags retained. Predictions locked before either new RGB odometry archive was acquired. Qualification was project-unexamined within the audited lineage; unknown checkpoint training overlap prevents a model-training-held-out claim. Both recordings are now exposed.

| Recording | Expected adjacent intervals | Joint matched intervals | Joint coverage | Rotation oracle headroom |
|---|---:|---:|---:|---:|
| Day | 5,694 | 5,416 | 95.12% | 9.04% |
| Night | 5,233 | 4,693 | 89.68% | 12.20% |

The frozen contract requires at least 1,000 joint intervals and 95% coverage per recording, at least 10% rotation oracle headroom per recording, and at least 5% rotation improvement against every declared baseline with at most 5% translation increase and no coverage loss. The ordered primary disposition is **STOP_INSUFFICIENT_EVALUATION**, because night coverage fails. Day also fails the opportunity gate. Those thresholds are retained exactly.

Descriptive means on each recording's identical matched support are:

| Recording / method | Rotation error (degrees) | Translation error (m) |
|---|---:|---:|
| Day RGB | 0.046486 | 0.167096 |
| Day thermal | 0.131621 | 0.455033 |
| Day angular rule | 0.046486 | 0.167096 |
| Night RGB | 0.050481 | 0.158298 |
| Night thermal | 0.149451 | 0.603736 |
| Night angular rule | 0.051148 | 0.165873 |

The rule equals RGB on matched day intervals. At night its rotation mean is 1.32% worse and translation mean 4.79% worse than RGB. The translation safeguard alone passes that RGB comparison, but the rotation-benefit requirement does not. These descriptive comparisons do not override the coverage stop or constitute a formal rejection of all uncertainty-based methods. Source: [full evaluation findings](decisive-uncertainty-findings-2026-11-06.md).

## Defensible contributions

1. A reproducible RGB/thermal evaluation workflow with explicit source axes, missing outputs, reference qualifications, fixed baselines and prediction-before-label separation.
2. Evidence that learned uncertainty can rank within-camera supplied-reference error while cross-camera ranking and recording transfer remain unreliable for the tested rules.
3. Empirical separation of tracking availability, measurement ranking, motion accuracy and compute cost. Stronger support or faster execution does not imply lower pose error.
4. A retained-input handover mechanism under explicit request and retention budgets, plus an experimental stereo initialization repair. Their engineering findings remain distinct from the unestablished adaptive-accuracy hypothesis. See [EXP122](buffered-handover-findings-2026-11-03.md) and [EXP150](stereo-baseline-initialization-findings-2026-11-05.md).

## Limits and reporting boundaries

MS² scores use provided reference products. TD3 motion comparisons use recorded SLAM under their documented nominal transforms and interpolation assumptions. The final evaluation uses the frozen nominal common optical-frame, zero-offset/unit-rate clock and stationary-twist convention with provided RGB references. Physical synchronization, mounting/frame truth and exact shared-point identity remain unverified where stated in the source reports.

Small dependent image windows do not establish population-level weather effects. Provided-reference scores do not establish independent physical ground truth. Unknown checkpoint training membership remains disclosed. Different preparation recipes and backend/compute settings retain their original qualifications. The final completed-output selector runs both experts and demonstrates no compute saving; EXP122 call savings answer a separate synthetic scheduling question.

The repaired stereo constructor defect does not establish the cause of thermal travel shortfall. Sparse day/night native-to-projected depth ratios remain conditional observations. Patch ambiguity controls did not justify confidence-policy development. See [research decision](research-decision-2026-11-05.md), [depth replication](cross-recording-stereo-depth-findings-2026-11-06.md) and [ambiguity controls](stereo-ambiguity-controls-findings-2026-11-05.md).

## Research disposition and evidence needed to reopen

Deliver the characterization/feasibility result with the failed gates and counterexamples intact. Preserve production defaults. Do not rerun for favorable outcomes, relax gates, select another rule using these evaluation labels, or relabel the two completed recordings as fresh validation.

A future adaptive-method study requires a separately justified hypothesis, a fixed target and baselines, and new recording-level evaluation separation. New data would materially strengthen physical-accuracy claims if it adds independently qualified poses and documented sensor timing/mounting; more adjacent frames from these same exposed recordings do not resolve those limits. No upload or acquisition is required to report the present result. Any future access request should first check existing/Box coverage and specify the missing recording conditions and reference evidence.

## Evidence and recovery

The current conclusion is supported by the linked recorded findings and frozen experiment plans, including the final independent 41,248-check decision reconstruction reported in the evaluation findings. This document adds no empirical validation. Historical findings retain their dates and superseded prospective statuses.

Final evaluation artifacts are documented at `results/decisive-uncertainty-evaluation/run-001`; recovery is `ore_ms:ore-ms/backups/2026-11-06-decisive-uncertainty-evaluation`. Models/runtime remain separate recovery prerequisites. The branch source snapshot and path-scoped commit patch history through `831a4883d` are backed up at `ore_ms:ore-ms/backups/2026-11-09-branch-sync-831a4883d`.
