# EXP138: conditional common-time transport preserves its assumptions and gaps

2026-11-04 UTC. Parent EXP137 `c10eedb744`; code/protocol freeze `692f85ad0`
precedes the saved-output replay. This is reference-free development engineering.

A separate offline model can now transport completed camera-local motion onto
requested RGB intervals under an explicit stationary body-twist assumption.
Cross-camera transport additionally requires an explicit zero-offset, unit-rate
identity assumption about the provided nanosecond values. This does not establish
physical clock correspondence, physical frame calibration, accurate common-time
motion, online availability, or an adaptive selection benefit. EXP133 remains
execution-not-ready; its strict publisher and benchmark protocol are unchanged.

All 105 tests pass (23 new and 82 prior). An independent matrix-log/exponential
and source-calibration audit passes 49,118 evaluated assertions, including prefixes
of rejected mutation checks. All 30 standard semantic mutations and 13 supplemental
mutations are rejected. All 23 new source/code pins and all 2,467 parent EXP137
pins are unchanged. No new native inference, training, reference scoring, reserve
access, timestamp search, clock fit, metric scale fit or default adoption occurs.

## What is conditional

For completed native delta D = inverse(P_i) P_j and provided static camera-to-RGB
transform E, compute the common-frame delta E D inverse(E). For native [a,b] and
requested [r0,r1], the model is Exp(((r1-r0)/(b-a)) Log(E D inverse(E))). The SE3
left Jacobian preserves rotation/translation coupling, including the lever arm.
These are the same stationary body-twist equations and one-native-duration horizon
used in EXP088, with EXP133's bounded serialization roundoff normalization.
The principal logarithm assumes no unobserved rotation winding.

A shifted interval start can be ignored only if the body twist is stationary over
the relevant interval. Both requested endpoints must lie no further than one
native duration outside the native interval. Equality passes; exceeding this bound
leaves an explicit hole. This horizon is a fixed engineering guard, not a validated
bound on motion error. An accelerated synthetic trajectory x(t)=t+2t² makes that
limit concrete: native .1→.3 seconds predicts .36 m for requested .2→.4 seconds,
while constructed truth is .44 m. It passes the horizon and still has .08 m error.

All original integer endpoints, source stems, camera clock IDs, segments, provided
frame evidence and offline availability provenance remain attached. Different
clock IDs without the explicit identity assumption are refused. Nonzero offsets,
changed rates, falsely verified clocks, float/bool timestamps and malformed rigid
transforms are refused. Even numerically equal intervals in different clock domains
remain conditional. Only same-clock, identical native intervals are tagged exact.
The original exact-native publisher continues refusing cross-clock thermal motion.

## Fixed saved-output replay

Use only EXP137's saved 101-pose prefixes, with original source i→i+1 paired to
original RGB i→i+1. There is no neighbor search, source reordering or timing fit.
The provided frame convention comes from the pinned official MS² extrinsic method;
the auditor independently constructs RGB-from-NIR times inverse(thermal-from-NIR).

Four controls are declared before replay: RGB only, thermal only, a metadata
condition prior (day RGB, night thermal, rain RGB), and alternating RGB/thermal by
edge parity. The alternating control tests integration across camera changes; it
is not a quality selector. All 100 requested edges per trajectory are retained.

| Condition | RGB exact /100 | Thermal conditional /100 | Thermal horizon gaps | Prior exact/conditional/gaps | Alternating exact/conditional/gaps |
|---|---:|---:|---:|---:|---:|
| Day | 100 | 100 | 0 | 100/0/0 | 50/50/0 |
| Night | 100 | 98 | 2 | 0/98/2 | 50/48/2 |
| Rain | 100 | 98 | 2 | 100/0/0 | 50/49/1 |

Thermal gaps occur at original night stems 000023→000024 and 000027→000028,
and rain 000001→000002 and 000078→000079. They remain null; a later accepted
edge starts a separate arbitrary gauge. No missing interval is filled or bridged.
The first target pose is explicitly an arbitrary model gauge, not a measured
thermal pose at the RGB timestamp. There are 12 trajectories / 1,200 edge records,
including repeated controls, not 1,200 independent observations.

DROID's final poses use offline backend and future-image trajectory refinement.
Provenance therefore names native termination and each run's own elapsed clock;
there is no fabricated common controller deadline or per-frame output receipt.
All model trajectories remain benchmark-inadmissible, with physical frame/clock
verification and online latency availability false. Exact RGB edges inside this
engineering container do not promote the whole container to an online benchmark.

## Predicted self-consistency diagnostic

For all 99 consecutive triplets per stream (594 total), transport the predicted
coarse i→i+2 delta to its own i→i+1 interval, then compare against that stream's
predicted fine delta. Both are expressed in the provided common RGB frame.
Translation is the norm of the relative residual translation; rotation is its
principal angle. These are disagreements between predictions, not measured motion
error, ground truth, or evidence for choosing a camera. No thresholds or policies
are fitted from them.

| Stream | Translation median / p95 / max (m) | Rotation median / p95 / max (degrees) |
|---|---:|---:|
| Day RGB | .003590 / .007180 / .013292 | .054462 / .136653 / .283665 |
| Day thermal | .016804 / .034748 / .038946 | .052907 / .128582 / .207014 |
| Night RGB | .004781 / .011562 / .018286 | .024665 / .126596 / .201174 |
| Night thermal | .008958 / .032875 / .048785 | .033139 / .175401 / .313110 |
| Rain RGB | .164477 / .994272 / 1.146962 | .044089 / .192355 / .274934 |
| Rain thermal | .058822 / .238221 / 1.024070 | .030475 / .070368 / .093975 |

The large rain disagreements prevent treating the stationary model as established
accurate alignment. This diagnostic cannot distinguish model violation from pose
estimation error. Even small disagreement would not prove physical clock agreement
or accuracy. No new scientific reference values were read to make these checks.

## Validation and recovery

Synthetic tests cover coupled SE3, lever arms, pure translation/rotation, near-zero
and near-pi rotations, epoch/unit invariance, exact/conditional classification,
identity-assumption requirements, horizon boundaries, segment breaks, rigid roundoff,
and strict publisher refusal. The independent auditor uses SciPy matrix log/expm,
not the production closed form. It reconstructs source axes, camera decisions,
calibration direction, projected deltas, right-composed poses, gauge roots and nulls.
Mutations test source relabeling, float time, fitted clocks, erased model tags,
false verification/online receipts, wrong camera indices, wrong pose/segment,
erased clock assumptions, inverse direction, wrong scale and all four filled gaps.

Protocol: `experiments/common-time-model-2026-11-04.json`.
Artifacts: `results/common-time-model/run-001`.
Runtime: inherited EXP136 `sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c`;
CPU-only checks suffice here; no runtime rebuild or GPU inference occurred.
Recovery: `backups/2026-11-04-common-time-model-exp138`, with inherited runtime and
parent EXP137 image/input packet as external regeneration prerequisites.

Next work must resolve or explicitly retain the acquisition-clock uncertainty and
qualify motion-model adequacy before physically meaningful mixed-camera scoring.
Whole-recording development and native failure/availability qualification remain
open. Do not use this model-only engineering packet as an execution permit for
the frozen benchmark or as evidence that adaptive RGB–thermal selection improves
accuracy. Active test/validation assignment stays empty and final reserves closed.
