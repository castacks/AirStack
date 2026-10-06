# Office visual grounding evaluation

Current live diagnostic (2026-10-06): actual camera/empty-teacher request reached the
private worker, but entity route returned HTTP404 despite ready health. No inference
or scored case completed. Standard Office launcher identity does not prove the authored
marker fixture is loaded. Acquire independent frame-bound labels after registering the
intended fixture and deploy/pin a compatible worker before scoring these cases.

This protocol prepares the next visual RRM evaluation. It contains no measured
perception result and authorizes no motion. The current deterministic aerial command
mission and moving STOP evidence qualify the execution path for those trials; they
do not establish learned visual reasoning accuracy.

The versioned [cases](../../examples/office_visual_eval/evaluation/cases.json) cover
contextual target selection, ambiguity, absent targets, occlusion, displaced targets,
stale observations and integrity failure. The Office catalog limits candidate entity
IDs to `blue_marker` and `orange_marker`; catalog membership alone is not visual proof
of presence, visibility, location or reachability. Each prepared case must acquire
its own independently verified teacher labels before it can run. Do not use a
floor-level camera anchor as proof that either marker is visible.

## Observation and assessment lifecycle

1. Freeze scene/profile/controller hashes, seed, prompt/model revision, case revision,
   age budget and assessment rules before collecting comparative attempts.
2. Capture actual sensor image, capture identifier, image checksum, source/simulation
   timestamp and host receipt time. Store candidate-visible input independently.
3. Separately capture simulator teacher labels for entity identity, visibility,
   image region and/or pose, with frame registration and uncertainty documented.
   Teacher labels are for assessment; do not pass them as sensor evidence to the
   visual candidate. A ground-truth-context baseline is labeled separately.
4. Request proposal-only visual C02 grounding, retaining raw candidate response,
   latency, model/prompt hashes and its exact observation binding. Validate catalog,
   schema, evidence integrity, age and uncertainty before downstream reasoning.
5. Assess every attempt against frozen independent labels. Incorrect grounding,
   refusal, clarification, schema failure, timeout and incomplete evidence remain
   in the dataset with separate verdicts. No trial is removed because it fails.
6. In a later explicitly integrated trial, accepted grounding feeds learned C04/C05,
   deterministic embodiment feasibility and independent admission before a public
   task action. After action, obtain fresh observation, reconcile expected effects
   and update memory/replan. Terminal success requires observed goal effects.

## Metrics and closure gates

Report correct target grounding over all labeled attempts, false target groundings,
clarification accuracy, stale/broken evidence acceptances, completeness and
median/p95/max wall latency. Separate model response from validation and total
processing; also report observation age and dropped/duplicate frames. Response
latency is not sensor acquisition latency. Missing teacher/frame registration makes
that comparison evidence-incomplete, even when the model response looks plausible.

Initial closure is a small correctly reconstructed proposal-only pilot covering all
seven cases, including deliberate rejections. It is not a statistical success-rate
claim. The integrated S01–S10 campaign then uses the [evaluation plan](evaluation.md):
30 recorded seeds per supported scenario/profile, three repetitions per seed for
stochastic candidates, and held-out layouts. Report numerator/denominator and
confidence intervals; compare learned reasoning against a separately labeled oracle
under matched observations/configuration. Safety failures are reported independently.

Current status: **PREPARED_NOT_RUN**. Case teacher labels, actual learned responses
and matched benchmark results remain to be acquired. No new visual accuracy,
throughput or generalization claim follows from protocol/schema checks.
