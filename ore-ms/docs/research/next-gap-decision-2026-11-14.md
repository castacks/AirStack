# Next original-paper gap: state continuity after interruption

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus preserved working changes.
Status: research direction recommended; implementation feasibility pending.
No new pose experiment, training or author-system reproduction.

## Decision and basis

Investigate state continuity and recovery as the next distinct paper-derived axis.
The immediate deliverable is a bounded source/asset feasibility review, before a
numerical protocol. This is a feasibility recommendation, not the only remaining
gap, an established new method or a diagnosis of our previous pose regressions.

The completed selection-learning recipe stays closed. Its positive night
pose-versus-match comparison remains useful evidence, but does not justify another
head, geometry penalty, deployment substitution or training-budget search.
See [completed comparison](selection-policy-learning-findings-2026-11-14.md).

| Original-paper family | What the completed work informs | What a further study would need |
|---|---|---|
| Persistent estimation and temporal dependence | Oracle covariance and conditional-marginal gains; tested temporal-block recipe misses gates | A distinct dependence/state mechanism, rather than a longer-window repeat |
| Thermal supervision, representation and transfer | Controlled-warp baseline succeeds; natural geometry disagreement has uncertain attribution | Defensible natural-error endpoint or a distinct geometry-preserving learning mechanism |
| Selection and posterior reliability | Selection changes pose; lower spread need not mean lower error; task objective has context-dependent benefit | Distinct set/deployment mechanism and strong matched controls; generic task training already exists |
| Continuity and recovery | Earlier buffering repairs an adjacent-input handover; scheduling transfer is mixed | Whether the estimated trajectory and map remain in a common consistent frame after interruption |
| Active perception | No new planner/source audit in this review | Planner inventory, task-level outcomes and a specific intervention |

Continuity is preferred for the next intake because it is an explicit author
limitation and differs from the completed weighting/selection recipes. This
preference does not establish a field-wide novelty gap or better achievable gains.

## Primary-source connection

Fresh targeted reread: MACThermal thesis Chapter6, printed p62 / PDF p75,
[primary thesis](https://publications.ri.cmu.edu/storage/publications/2025/12/YifeiLiu_CMU_MSR_Thesis_ThermalOffroadAutonomy.pdf).
The author identifies dependence on continuous video and stable calibration,
drift when a moving platform loses the thermal stream, explicit reinitialization,
and thermal-inertial SLAM as a future direction. Full thesis reading, figures,
released-system recovery behavior and later-work novelty were not re-audited.
The local cached PDF/text supplied the reread; web retrieval exceeded its size limit.

Our interpretation: restored output availability and restored global state
consistency are different outcomes. A bridge, reset or relocalization must state
which information carries motion across a gap and how the coordinate frame is
maintained. Merely emitting a new pose after restart cannot establish recovery.
Inertial fusion/relocalization are established methods; adding them alone is not
an original contribution.

## Existing evidence prevents repeating a solved engineering fixture

[November3 research position](research-position-2026-11-03.md) summarizes
EXP119–125. Buffering one real predecessor repairs the observed handover break;
resource-bounded variants restore return episodes. These are meaningful positive
results about inputs, scheduling and output readiness. They do not establish
physical trajectory/map accuracy, natural outages or real-time acquisition.
The unchanged night scheduling transfer does not establish adaptive superiority.
A new study must not rename these completed fixtures as state recovery.

## Finite next intake and stop rule

One question: **can the available implementation and data support a controlled
comparison of state continuity after a moving-camera interruption?** This is a
readiness question; no recovery algorithm has yet been selected.

1. Trace native graph, keyframe, pose-frame and restart/reset behavior. Distinguish
   retained state, a fresh local frame and a genuine cross-gap measurement.
2. Inventory one available sequence's timestamps, calibration, interruption
   boundaries and independent motion reference. Inspect IMU synchronization only
   if an inertial bridge is proposed. Earlier RGB/thermal endpoint mismatch means
   nominal same-ID pairing cannot silently become a synchronized sensor bridge.
3. Read relevant primary recovery/thermal-inertial methods and released code.
   Specify overlap and one possible additional mechanism before proposing code.
4. If ready, bind a single interruption condition and matched controls: uninterrupted
   context, unchanged native gap behavior, explicit restart, and an established
   bridge/relocalization baseline appropriate to the actual available sensors.
   Report post-gap error in one common pre-gap frame, recovery latency, lost
   intervals and persistent map/trajectory discontinuity. Do not realign each
   recovered segment independently and hide a frame jump.

Proceed to a numerical freeze only if coordinate semantics, cross-gap information
and evaluation reference are defensible. Otherwise record the exact missing asset
or contract and stop this intake; return to the other paper-gap families rather
than start clock searches, invent missing motion, tune outage severity or declare
continuity research impossible. No physical-failure frequency or natural-noise
claim follows from an artificially removed stream segment.

This note completes the post-synthesis direction comparison. It does not claim
that the feasibility review or recovery experiment has already been executed.
