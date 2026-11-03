# Geometry versus simple support: fixed additional-recording development test

2026-11-02 America/New_York. Starting point EXP112. Freeze before new inference.

## Selection and scope

Use `td3/20251027/night_thermal_so_salon2`, distinct from all three EXP112
recordings. Retain all three original 20-frame windows (manifest starts 123/619/789,
EXP013; exact input rows preserved by EXP023's experimental MSA thermal baseline).
Early RGB and thermal previously tracked; middle/late RGB failures remain in the
population. No new window/outcome filtering or scientific test replacement.
Original selection used maximum reference endpoint displacement within thirds;
these windows and recording are already exposed, not a prospective holdout.

Prior screening of the other remaining TD3 nights found mostly dark RGB or no
continuous paired window under the existing benchmark association. Do not treat
those availability failures as the requested both-healthy quality population.
This bounded replication uses the one additional source with verified usable
support and applicable existing preparation. It cannot establish broad transfer.
The two MS2 candidate reserves remain unassigned/unopened. Other supplied datasets
remain options for a later explicitly compatible benchmark.

## Fixed inputs and method

Keep original RGB/thermal weights, native MSA-prepared inputs, seed, FP32,
configuration and compiler runtime 44ff9a1e unchanged. Capture all three windows
with the geometry observer off/on. Preserve the original algorithms. Static
nominal body transforms use the same source graph and per-stream rectification
as previous TD3 experiments. Project onto the RGB interval with the same
stationary-body-twist model and one-native-duration horizon. Both experts run
on every frame; no joint graph fusion, skipped inference or live timing claim.

A separate worker wrapper can acknowledge ONLY the native terminal
`RuntimeError: No visual matches were retained`, after all declared frames,
with retained_observations=0, NO_PAIRS_SUPPORTED, complete observer and zero
optimizer submissions. Keep the original FAILED report and absence of native
motion output. This lets the shared metadata capture finish both workers without
turning zero matches into successful VO. Any other error propagates unchanged.
Both arms use exactly that wrapper. Full/partial native tracking is reported
separately from shared-controller execution completion.

## Prespecified methods and no fitting

Primary: unchanged EXP112 current-geometry admission rule: normalize common-body
conditional information with 0.1m/1degree budgets, require minimum eigenvalue>=1,
fixed RGB preference among adequate available experts, otherwise abstain.
Recovery-memory candidate remains a control, not a new primary.

Controls, using only current matching pair-health events actually received by
that decision deadline:

- RGB-only, thermal-only, original raw availability and original two-pair history.
- Higher **absolute retained point count**, ties RGB.
- Higher occupied 3x3 image-cell count, ties RGB.
- Higher retained/selected fraction, ties RGB; denominator at least 1.

Support controls choose the only available expert when one is available and abstain
when neither is. Missing required feature on an available expert is a contract
failure; no imputation. Native graph factor count and retained count are reported
separately; no assumption that they are interchangeable.

Two descriptive geometry rankings are fixed sensitivities, not replacement
candidates: larger minimum normalized information eigenvalue, and larger same
value divided by actual native graph factor count. The latter separates simple
count multiplication from per-factor geometric/noise strength. Tie RGB. They
never select an unavailable expert. No threshold search, learning, sign selection,
window replacement or adopting whichever sensitivity scores best.

## Evaluation populations and criterion

Retain all57 decision rows and114 camera slots. Before reading reference/error
labels, fix `both_supported` to both experts' valid causal model availability.
Report all-decision availability/fallback separately from the both-supported
quality population. Missing/interpolated RGB is not a relative-error label.

Primary criterion: geometry admission has coverage at least raw on the declared
population and strictly lower matched translation proxy RMSE than every fixed
camera, raw/history and three support controls on the both-supported population,
with no rotation regression on each exact matched comparison. Empty matched
populations and ties cannot pass. Report each window and the recording summary;
windows are not independent recordings. No post-outcome threshold or criterion
change. Sensitivities are descriptive, never a substitute primary.

Use fresh captured expert motions and newly computed errors against the same
supplied nominal-body recorded-SLAM reference. Lock actions before semantic
reference reads. No historical expert losses for fresh runs. Geometry information
remains conditional on fixed parent/landmarks/extrinsics and omits robust, model,
clock/map and cross-camera uncertainty. No physical accuracy or training-disjoint
claim. Unknown checkpoint membership does not veto this scoped development test.

Native preservation uses the already declared EXP112 scope: every active input,
pose, selected point/mask, native event, config and raw decision must match exactly.
If the three specific inactive-derived diagnostic field families drift, retain
all mismatches and original all-array exactness status. No numerical tolerance
passes changes to active values. Missing output is permitted only for the exact
terminal-zero-match failure in BOTH arms; preserve its original flags/status.

A positive result would motivate other separately declared recording conditions
and stronger references, not establish a method or sensor generalization. A
negative result is reported without expanding model capacity or tuning this clip.
