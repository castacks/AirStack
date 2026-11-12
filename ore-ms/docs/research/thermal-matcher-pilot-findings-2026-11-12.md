# Frozen thermal temporal matcher: translation gain, rotation regression

12 November 2026. **Stop this configuration:** replacing thermal temporal flow
with pretrained TartanMatch improves translation but worsens rotation, failing
the rule frozen before the four scientific arms. No retuning or promotion.

## Fixed comparison

Used the first 20 original groups, 000000–000019, of previously exposed rain
`_2021-08-06-16-19-00`. No training, checkpoint selection, frame search,
threshold tuning or new reference construction. All four arms complete 20/20
frames in one scientific attempt each: original thermal, passthrough thermal,
replacement thermal and original RGB.

The released matcher source is pinned to
`b3f65ffdf9723a028d82f36ab0beb8457d0def72`, with checkpoint revision
`27662ae4c0438fd901cbc80836f027b1985385f6` and actual checkpoint SHA256
`52500775d218bd2d27ddeafb52d575c193d3aef9679767cbe9b1a2dd59eaa706`.
DINOv2 architecture code is pinned to
`7764ea0f912e53c92e82eb78a2a1631e92725fc8` and supplied through a read-only
local torch.hub cache. Original replicated-gray thermal preparation is reused.
Matcher inference uses its released 420×560 internal resolution and default
CUDA FP16 autocast, returning original-resolution 256×640 pixel displacements.
See the [author implementation](https://github.com/castacks/tartanmatch/tree/b3f65ffdf9723a028d82f36ab0beb8457d0def72).

Only temporal flow changes. Native stereo outputs, covariance tensors and masks
are retained; covisibility is saved but not thresholded. Original covariance is
an experimental control, not calibrated uncertainty for the new matcher.
Changing flow can change keypoints, sampled target depths and propagated
observation covariance. This measures the total flow-replacement effect through
the existing pipeline, not a fixed-correspondence solver intervention.

## Motion accuracy

Same inherited nominal RGB-frame/constant-twist interval mapping and supplied
reference scorer as the prior fit. All metrics below use the same 18 eligible
intervals. These are local motion errors, not trajectory ATE.

| Metric | Original thermal | Replacement flow | RGB |
|---|---:|---:|---:|
| Translation mean, m | 0.960001 | 0.896703 | 0.669999 |
| Translation RMSE, m | 1.048964 | 0.972190 | 0.867443 |
| Translation p95, m | 1.500728 | 1.365458 | 1.383227 |
| Rotation mean, deg | 0.075270 | 0.087322 | 0.031285 |
| Rotation RMSE, deg | 0.095380 | 0.099270 | 0.045285 |
| Rotation p95, deg | 0.180672 | 0.164964 | 0.066400 |

Translation mean/RMSE improve 6.59%/7.32%; rotation mean/RMSE worsen
16.01%/4.08%. Both tails improve, but do not replace the predeclared primary
metrics. Nominal eligible coverage is thermal 18/19 in both configurations,
RGB 19/19. All 19 replacement flow packets are finite. Thermal's mapping gap
is retained, not interpolated or removed from availability accounting.

Advancement required strictly lower translation mean AND RMSE, no higher
rotation mean AND RMSE, nonempty common support and no coverage regression.
Both rotation conditions fail. RGB remains lower on all four mean/RMSE metrics.
The result does not show that flow endpoint error improved: no independent
temporal correspondence truth was established by the preceding preflight.

## Verification and recovery

The original MAC runtime was restored, not rebuilt: exact image
`sha256:44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`,
Torch 2.7.1+cu128 and the RTX PRO 5000 Blackwell. No packages were changed.
Base and compiler archives/member hashes pass. Checkpoints and selected input
files were restored after archive/member verification; all 80 raw image hashes
match their existing manifest. Only selected source images/assets were extracted.

A first Docker load was mistakenly started before the merged archive finished
and failed with EOF. Loading the completed, verified archive succeeded before
the smoke or any scientific arm. No scientific prediction was retried.

The one-pair interface smoke and three intervention/cleanup tests pass. The
pre-execution freeze binds 1,089 source/asset/input hashes, verified before and
after each scientific arm. Predictions were hashed and locked before scoring.
Passthrough reproduces all 19 mapped original interval slots exactly, including
the unavailable slot. Independent NumPy calculations reproduce 186 motion and
aggregate metric values, common support, coverage and the stopping decision;
all 19 flow packets pass shape/finite/probability checks. This audit does not
rerun either network or independently validate physical reference semantics.

Code: `tools/thermal_matcher_pilot.py`, `test_thermal_matcher_pilot.py`, and
`audit_thermal_matcher_pilot.py`. [Execution freeze](../../experiments/thermal-matcher-pilot-freeze-2026-11-12.json).
Local artifacts: `results/thermal-matcher-pilot/run-001/`, including flows,
native events/poses, source locks, tests, smoke, prediction lock, scores,
decision, audit and package inventory.

## Decision

Close this exact pretrained-flow/original-covariance configuration. Do not tune
covisibility, covariance, preprocessing, frames or model variants to reverse
this outcome. No original model or default changed. Separate recording-level
evidence would be needed for a broader claim; unknown training overlap and
nominal physical frame/time assumptions remain explicit.

This completes the proposed bounded comparison. A further experiment needs a
new justified question; another automatic diagnostic chain does not follow
from this mixed result.
