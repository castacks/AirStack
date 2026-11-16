# Paired fixed-priority and headroom protocol

16 November 2026, America/New_York. Follows the
[paired restoration](multirecord-restoration-findings-2026-11-16.md) and
[OutdoorNight priority control](camera-priority-findings-2026-11-16.md).
Previously exposed development recordings; prior fixed-camera/RGB-first results
and documentary bounds are known. No new inference, tuning or generalization test.

## Question, fixed population and actions

Does actual thermal-first replay confirm the cross-record limits, and how much
endpoint-specific headroom remains beyond the better fixed priority? Use every
541 FlashLight and 838 SmokeRoom anchor from the completed restoration, both SIFT
and learned, with all four prediction rows per anchor. Retain frozen common-reference,
both-camera rotation≥2° and both-displacement≥1cm populations. No new windows,
clock/axis/bias fit, image labels, endpoint thresholds or eligibility changes.

Preflight complete unique prediction/input keys, successful parent audits and
prediction/decision/score/summary locks, valid native support flags, every endpoint
pointer and finite nonnegative cached call durations. Freeze source/config/protocol,
parent populations, scores and comparison pins. Reuse the existing pure
camera_priority_controls.choices and priority helper, without changing that frozen
runner. The supplied-reference adapter is necessary because the prior runner's
scoring contract is specifically raw-gyro disagreement; do not relabel it as pose.

Fixed controls: RGB, thermal, RGB-first, thermal-first. A fixed camera requests
its own matcher only when input-qualified. A priority order requests its qualified
primary and accepts status ok, otherwise tries the qualified backup. Unsupported
inputs make no call; failed qualified calls cost time. Recorded unsupported-primary
input-check time is reported separately and included in projected seconds to preserve
the original parent cost convention; it is not a matcher call. Both orders have identical
output-status unions. Choices use status alone and retain the chosen camera's
source/target/times. Lock all choices before joining reference errors. There is
no quality prediction or new pose generation.

## Fixed endpoints and descriptive analysis

Report each frontend/record separately on all declared anchors, common-reference,
paired rotation≥2°, paired translation≥1cm, and three original index thirds
restricted to common reference. Counts include failures in the denominator.
R≤1° and R≤5° require an available proposal and its own non-null nominal supplied
reference error. Joint success additionally requires frozen translation eligibility
and own directed t≤15°. Do not transfer camera translation origins or infer
metric body motion. Report output count, compared-output count, eligible population,
matcher calls and projected cached seconds for every policy/cohort.

For each endpoint compute RGB/thermal success sets; their union is a nondeployable
hindsight oracle. Report best-fixed and best-priority counts and oracle increments,
both-supported conflicting correctness, both-supported neither-correct, and
no available correct proposal. Oracle camera choices can differ by endpoint;
these ceilings are not a single achieved method. Retain source RGB-or-zero-rotation
controls descriptively to expose static-motion effects; no new identity tuning.
Compare original RGB-first summaries exactly for status/count/calls/errors, with
floating-time tolerance1e-8. Check actual learned moving thermal-first counts lie
inside previously frozen documentary bounds, without revising those bounds.

No pooled accuracy or cost across recordings/target types. OutdoorNight's
existing fixed-priority gyro results may appear alongside in a separately labelled
panel, inherited unchanged with source pins and audit. Its raw-gyro angular
agreement has no supplied translation truth and no calibrated pose interpretation.

## Verification and stop

Independent audit reconstructs routing, sensor endpoints, full cohort/score
accounting, failed-call costs, oracle conflicts and parent parity without calling
the production evaluator. Finite fixtures and semantic corruptions cover choices,
cost, oracle, joint reference eligibility and target labels; rehashed corruption
must still fail. Preserve errors rather than changing the protocol after scoring.

Save choices/locks, summaries, bound checks, source receipts, audits, figure and
verified local incremental packet. No full paired rows need duplication: use
verified restoration/OutdoorNight packet dependencies. Stop after characterizing
fixed-order tradeoffs and remaining ceilings. A fitted quality selector requires
a separate protocol, observable cue/prior-art review and recording-level splits;
these exposed records cannot be relabelled blind validation. No Box upload,
author outreach or Git publication is authorized by this replay.
