# EXP-076: two-interval causal tracking-history baseline

2026-10-29 America/New_York. Branch `ore_ms`, base `19ef530c8`.

A symmetric two-interval confirmation rule avoids the isolated RGB recovery
choice in the known middle window. On the same97 retained intervals, conditional
SLAM-proxy translation RMSE is0.04282m with history versus0.11041m with the fixed
RGB preference. History matches thermal-only there. It also adds one cold-start
abstention per window and does not improve early-window matched losses. This is
a retrospective baseline, not evidence of a general quality-selector improvement.

## Frozen rule and scope

Before this scoring run, fix confirmation length2 as the smallest nontrivial
history baseline. No other lengths, thresholds, covariance scalings or alternative
rules are evaluated. Both existing windows and the EXP075 middle outlier were
already known when designing the rule. Freezing this run does not make these
conditions prospective or held-out evaluation.

At the original deadline for target pair i, an expert is confirmed only if the
original availability conditions hold for both i and immediately preceding pair
i-1 in the observed prefix: complete supported health, no execution error, and
a first camera-local motion matching that expert's exact interval. Missing,
unsupported, partial or mismatched current/previous results prevent confirmation.
Repeated stale writes do not supply a new interval. Apply the same rule to RGB
and thermal. If both confirm, retain the fixed RGB preference; if just one confirms,
choose it; otherwise abstain. Pair0 has no prior interval and always abstains.
Each window starts with empty history. There is no unconfirmed fallback.

Only target and preceding intervals contribute. Do not use newer pair-health
records even if they were observed by the one-group-lag deadline. No reference,
proxy loss, raw covariance comparison, numerical residual, pose magnitude,
whole-run outcome or fitted coefficient determines confirmation. The rule
operates on available copied event records; no body transform or reference
bridge is used in its prediction path.

Reuse all118 frozen EXP073/074 decision boundaries. Reconstruct each old advisory
result from its original prefix before predicting history, retaining exact
sensor intervals and observation stages. Neither estimator, original rule,
source journal, deadline, predictor feature contract nor default is changed.
No new inference, images, reserved recording or cross-camera trajectory fusion.

## Prediction and scoring separation

`Scripts/Setup/arbiter_history.py` has a pure history function and separate
`prepare`/`predict` commands. The immutable protocol freezes source and original
journal hashes. Its prediction input list contains no EXP075 proxy report or
reference. The prediction file is saved before scoring loads labels.
`score_arbiter_history.py` separately joins those predictions to the verified
EXP075 labels by window, pair and original decision sequence. Reference-assisted
nominal-body proxy losses remain offline evaluation data, with the same physical
body/reference limitations as EXP075. No gate is trained.

Score RGB-only, thermal-only, fixed RGB preference and history. Preserve null
unavailable results and all abstentions. Report full available populations and
matched intersections for every history/baseline comparison. Coverage loss
cannot be credited as a pose-quality improvement. Window summaries remain
separate because the captures are short, dependent and from one recording.

## Observed actions and availability

| Window | History actions | History availability | Fixed RGB-preference availability | Thermal-only availability |
|---|---|---:|---:|---:|
| Early |17 RGB,2 ABSTAIN|17/19|18/19|18/19|
| Middle |97 THERMAL,2 ABSTAIN|97/99|98/99|98/99|

The extra abstention is pair0 in each window. The original final pre-drain
abstention remains. In the middle window, pair96 changes from RGB to thermal:
current RGB motion is originally available, but its preceding pair95 was
unsupported, so the isolated recovery is unconfirmed. Thermal has consecutive
supported results and remains eligible. No future pair or loss was consulted.
There are exactly three changed actions across118 boundaries: two cold starts
and this one RGB-to-thermal choice.

## Proxy losses on matched support

Translation RMSE is conditional nominal-body SLAM-proxy error in meters, not
independent physical accuracy. Every number in a matched row uses identical
pair ordinals for the two policies.

| Window / comparison | Matched intervals | History RMSE | Baseline RMSE |
|---|---:|---:|---:|
| Early vs fixed RGB preference |17|0.010000|0.010000|
| Early vs thermal-only |17|0.010000|0.009372|
| Middle vs fixed RGB preference |97|0.042818|0.110406|
| Middle vs thermal-only |97|0.042818|0.042818|

The middle matched improvement includes the previously known1.00308m RGB proxy
residual replaced by0.04029m thermal on the same interval. It is not explained
only by dropping the cold-start interval, but remains retrospective behavior
on an already diagnosed outlier. Early history retains the original RGB choices
on its17 confirmed intervals and therefore does not improve those choices.
Thermal-only has lower aggregate early proxy loss on that support.

For completeness, full available-population translation RMSE is early history
0.010000m on17 intervals versus fixed RGB0.009891m on18, and middle history
0.042818m on97 versus fixed RGB0.109850m on98. These population sizes differ;
use the matched table for direct quality comparisons. Full thermal-only
availability exceeds history by one interval in each window. No aggregate
coverage/error scalar or tuned abstention penalty is introduced.

## Verification and artifacts

Twenty Docker CPU tests pass: cold starts, stable confirmation, symmetric
previous failures, current failures, missing/mismatched interval evidence,
missing/stale first results, no unconfirmed fallback, newer-pair invariance,
reference/loss metadata invariance and invalid target indices. A guarded replay
forbids reads of any pose-proxy path and reproduces predictions byte-for-byte.

`validate_arbiter_history.py` imports neither policy nor scorer. It independently
reconstructs original current/previous evidence from live prefixes, checks all
118 actions/deadlines and2832 decision/loss fields, rejoins original verified
labels and recomputes full and matched populations. Seven mutations are rejected
after refreshing the outer prediction hash: cold-start promotion, a future prior
pair, unconfirmed RGB recovery, changed deadline, altered loss, inflated matched
population and dropped abstention. Frozen input/source hashes remain unchanged.
These checks validate software; they are not independent scientific samples.

Artifacts are under `results/arbiter-history/`: tests, label-access guard,
semantic challenges and `run-001/{protocol.json,predictions.json,scores.json,
validation.json}`. Predictions SHA256:
`7edb45635bd7d7ad8a92b1c90001e348ed937d0c3cd05d336b7af3af7ff404de`.
Scores SHA256:
`faea724dab7db2a3524b01268218ab9e1a129e7f4883e025475413e63985ec98`.
The prediction/scoring/audit commands need standard Python only. From the
MAC-VO root, with a fresh output:

```bash
python3 -m Scripts.Setup.arbiter_history prepare --root <ore-ms-root> --output <fresh-run>
python3 -m Scripts.Setup.arbiter_history predict --root <ore-ms-root> --output <fresh-run>
python3 -m Scripts.Setup.score_arbiter_history --root <ore-ms-root> --run <fresh-run> \
  --labels <ore-ms-root>/results/arbiter-pose-proxy/run-002
python3 -m Scripts.Setup.validate_arbiter_history --root <ore-ms-root> --run <fresh-run>
```

Recovery destination: `backups/2026-10-29-exp-076`; original timeline/labels depend
on EXP073–075. Credentials, checkpoints, image data and full source archives are
excluded from this snapshot.

## Decision

Keep the frozen two-interval rule as a causal recovery baseline. It fixes the
known middle preference mistake under this conditional benchmark, at an explicit
coverage cost, and provides no early improvement or advantage over thermal-only.
Do not promote it as a generally better selector. Next replicate the unchanged
rule under a separately declared condition before tuning richer uncertainty or
history features. Preserve the already-used status of these windows and the
reserved MS² captures; a new scoring run cannot erase their development history.

Follow-up: [EXP077](EXP-077-history-day-replication.md) replicates the unchanged
rule on a separately declared100-group day recording. Both experts track; history
matches RGB on97 retained intervals and costs the same cold start. No quality
improvement over fixed RGB is established in that condition.
