# Active perception: motion timing as the next controlled intervention

14 November 2026 UTC. `ore-ms-orb-development`, baseline `a79658f63` plus working
changes. Paper/source review and finite kinematic qualification complete.
No planner/robot execution, image rendering, neural inference or pose scoring.

## Selected question and original-paper connection

Select MACThermal T5: can the timing of an otherwise identical motion change
the usefulness of the thermal evidence acquired at a fixed deadline? This is
a separate intervention from weighting measurements, selecting points or choosing
an old reference frame. The first experiment must establish action-dependent
estimation headroom before designing a scheduling policy.

MACThermal Chapter6, printed pp62–63, already describes a working planner that
reasons about traversability cost/uncertainty. It explicitly proposes broader
coverage, resource allocation and actions reducing perceptual uncertainty. Thus
“the original system has no uncertainty-aware planning” would be false. Our
narrow question is one interpretation of that extension, not an author-reported
failure or the only original gap remaining.

## Established work and actual source depth

| Work | Relevant established mechanism | What this review permits |
|---|---|---|
| [Fisher Information Field](https://arxiv.org/html/2008.03324v1), targeted III–V | Efficient visibility/landmark-based localization-information queries for planning | Geometry-aware information planning is prior art. Released bearing-information and metric bodies inspected; no full planner run. |
| [PAMPC](https://arxiv.org/html/1804.04811v1), III-C/IV | Joint action/perception MPC, including projected location and velocity objectives | Generic image-velocity/blur-aware motion control is prior art. Paper and code version must remain distinct. |
| [Wu et al.2022](https://arxiv.org/html/2204.03134v1), III-A/B | Trajectory-wide feature geometry and anisotropic motion-blur covariance, with a short-exposure uniform-line approximation | Combining blur with predicted geometric covariance is already covered. Method read; no official implementation obtained in this pass. |

PAMPC repository pin `33263db209105f716e204acc6481834fbd9bc778`:
README, parameter loader and model-codegen file inspected. The inspected cost
vectors contain two projected-position terms; the paper describes both image
position and velocity. Do not label a wrapper around that slice an exact paper
reproduction, or conclude that other versions omit the paper objective.

FIF pin `76d7b834128bd3488d1aae3888a79f3d5216a4ab`: README and
`info_utils.h/.cpp` inspected. The released helper constructs bearing-Jacobian
information with optional visibility weighting; information metrics and Schur
marginalization are established operations. Its determinant transform has local
implementation details; copying a generic logdet does not reproduce the field.
Six external snapshot hashes are recorded in
[source review](../../results/active-motion-contract/source-review.json).
No external author-source/PDF content is included in our backup packet.

The local AirStack review is also bounded: the inspected exploration selection
body combines distance, momentum, map-processor score and other terms; the
trajectory library generates configured curves/accelerations. This does not
establish that every AirStack planner lacks perception terms or that the
inspected aerial stack is the thesis ground-vehicle planner. No production
planner, configuration or deployment interface is modified.

## Qualified matched-motion fixture

For normalized time u=t/H, use

    q_beta(u) = 10u^3 - 15u^4 + 6u^5 + beta*u^3*(1-u)^3,
    C(t) = [D*q_beta(t/H), 0, 0],
    beta in {-2,0,+2}, D=0.04m, H=0.2s.

Hold position before0 and afterH; orientation stays fixed. All three schedules
have the same monotone straight path, start/end poses, deadline, zero endpoint
velocity and acceleration. They are C2 at the stationary continuations.
The derivative factors as
`u²(1−u)²[30+3*beta*(1−2u)]`, whose final factor is at least24.
They satisfy the predeclared 0.5m/s speed and 10m/s² acceleration fixture limits.
These are kinematic camera trajectories, not demonstrated underactuated vehicle
motions or collision-free robot plans.

The two nonzero-beta schedules are time reversals in progress. Their exact
normalized squared-speed integral is the same rational number,552/385; the
beta0 value is10/7. For a fronto-parallel plane with pure lateral motion and
fixed depth, image velocity is a constant multiple of camera velocity. Therefore
the two reversed schedules also tie under the **whole-horizon squared image-
velocity integral**. Endpoint-only geometry and velocity tie as well. This does
not imply that all trajectory information metrics or blur-aware controllers tie:
fixed-time feature geometry and recent-window costs can distinguish schedules.

For one unchanged virtual exponential response tau=8ms, qualify the causal
response of a scalar linear spatial signal atH. This is a filter/mechanics
control, not a rendered image, flow label or pose error. Include a16ms uniform
recent-exposure control; it also distinguishes the schedules.

| beta | Max speed,m/s | Max acceleration,m/s² | Whole-horizon speed² integral,m²/s | Exponential lag,µm | Uniform recent lag,µm |
|---|---:|---:|---:|---:|---:|
|−2|0.376859|6.151439|0.01147013|138.3727|54.8179|
|0|0.375000|5.773503|0.01142857|119.6851|46.4159|
|+2|0.376859|6.151439|0.01147013|100.9975|38.0139|

The absolute differences are small. Do not increase speed, response time or
profile asymmetry in response to a later weak/null pose effect. A recent-box
control already captures an ordering here, so this is not evidence that an
exponential thermal model is uniquely necessary or that existing blur-aware
planning fails. These are ordinary time-filtering identities used to qualify
matched actions, not novel mathematics.

47 checks pass: exact rational endpoint/derivative/monotonicity/energy and
reversal controls, analytic polynomial critical-point extrema crosschecked on
10001 samples, and adaptive integration compared with64/128Gauss nodes to the
predeclared1e−11m bound. Source/protocol froze before computation, with no amendment.
The largest observed quadrature discrepancy is below4e−15m. Check counts are
implementation receipts, not independent experimental trials.

## Next finite estimator assay: proposed contract, not executed

Use the three existing exposed texture proxies, one4m plane, fixed native rig
and the three schedules above. Compare sharp instantaneous and exponential-
memory images only. Pose truth is always the source0 to deadlineH transform,
identical for every action. The virtual profile remains uncalibrated. No rolling,
noise, learned scheduling head, new geometry or parameter sweep is added.

This would give18three-pair dense batches and36pose fits with regenerated and
common-ID query controls. Qualify image integration and common boundary support
before an estimator freeze. All sharp images and native outputs must repeat
across schedules for each texture; the memory source images must repeat as well.
If these nulls fail, stop the implementation rather than scoring pose effects.
Keep all fit attempts and report translation/rotation separately.

The first endpoint is paired action-dependent pose error at identical progress
and target, including any negligible/null differences. Report full-horizon speed
cost, actual recent-box displacement/speed and exponential-history quantities
as separate prespecified controls. Do not use pose truth to pick an action and
then call that a deployable policy. A simulation preview generated from the known
scene is privileged map/model information and must be labeled as such. A causal
planner would additionally need an available scene estimate, candidate-response
prediction, dynamics and deadline/compute constraints.

No scheduling policy is validated by this step. If the first comparison shows
no useful action headroom, close this profile without a severity/head rescue;
that does not close T5. If headroom exists, it motivates a separately designed
ranking method compared with recent-exposure and trajectory-geometry controls,
not an automatic new-method or physical-transfer claim. Representation,
supervision, persistent estimation and the other original-paper gaps stay open.

Artifacts: [freeze](../../results/active-motion-contract/freeze.json),
[qualification](../../results/active-motion-contract/qualification.json),
[tool](../../tools/qualify_active_motion_contract.py).

Continuation15November2026UTC: the [completed image/estimator comparison](active-motion-estimator-findings-2026-11-15.md) supersedes the proposed/unexecuted status above. All36fits converge; effects small/mixed, no scheduling policy promoted.
