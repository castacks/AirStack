# Recoverable cases cluster, but image condition is not a camera-quality rule

16 November 2026, America/New_York. Completed the
[frozen failure-stratum protocol](paired-failure-strata-protocol-2026-11-16.md)
from committed base `4408b7fb8`, after the
[paired priority replay](paired-priority-findings-2026-11-16.md).
All2,758learned sensor/anchor rows and1,858endpoint/case rows are retained.
This is descriptive characterization of known exposed results. No inference,
training, feature ranking, fitted cutoff, classifier or new quality policy.

## Where the opportunities occur

A recoverable case means the opposite camera is correct when the better fixed
priority keeps an incorrect supported proposal. Best order is endpoint-specific:
RGB-first for strict rotation/joint; thermal-first for SmokeRoom coarse rotation.
These case sets are hindsight labels, not live routing inputs.

| Recording | Endpoint | Better fixed order | Recoverable cases | Original thirds: first / middle / final |
|---|---|---|---:|---:|
| FlashLight | R1 | rgb_first | 2 | 0 / 1 / 1 |
| FlashLight | R5 | rgb_first | 3 | 0 / 2 / 1 |
| FlashLight | joint | rgb_first | 6 | 2 / 2 / 2 |
| SmokeRoom | R1 | rgb_first | 23 | 0 / 1 / 22 |
| SmokeRoom | R5 | thermal_first | 22 | 0 / 11 / 11 |
| SmokeRoom | joint | rgb_first | 28 | 1 / 1 / 26 |

22of23SmokeRoom strict recoverable cases occur in the final third, at anchor
starts285.5–317.0s; the remaining case is240.5s.26of28joint opportunities also
occur in that third. Coarse opportunities beyond thermal-first differ: eleven
middle and eleven final cases. A single late-recording switch cannot be inferred
from these endpoint-specific distributions. These are adjacent recording events,
not independent experimental replications or a causal condition annotation.

## Full moving coarse strata, including missing proposals

Same frozen both-camera≥2° populations:192FlashLight and236SmokeRoom anchors.
Each anchor occurs in exactly one cell. A missing output counts as not correct;
both-supported counts separately identify two available but potentially wrong poses.

| Recording | Correctness cell at R≤5° | Cases | Both cameras output poses | Third counts: first / middle / final |
|---|---|---:|---:|---:|
| FlashLight | both_correct | 107 | 107 | 24 / 44 / 39 |
| FlashLight | rgb_only_correct | 78 | 54 | 13 / 27 / 38 |
| FlashLight | thermal_only_correct | 3 | 3 | 0 / 2 / 1 |
| FlashLight | neither_correct | 4 | 4 | 0 / 4 / 0 |
| SmokeRoom | both_correct | 154 | 154 | 21 / 78 / 55 |
| SmokeRoom | rgb_only_correct | 27 | 22 | 0 / 15 / 12 |
| SmokeRoom | thermal_only_correct | 36 | 25 | 0 / 1 / 35 |
| SmokeRoom | neither_correct | 19 | 15 | 0 / 0 / 19 |

SmokeRoom has36thermal-only-correct coarse cases, including11where RGB has no
proposal and25where RGB returns an incorrect pose. Its27RGB-only-correct cases
include5thermal gaps and22incorrect thermal poses. All19neither-correct cases
are in the final third,15with two returned poses. Status-only availability cannot
resolve those15. FlashLight has3thermal-only-correct versus78RGB-only-correct
cases (54with two poses), preserving the regression-control imbalance.

The longest SmokeRoom thermal-only-correct coarse run comprises17consecutive
half-second intervals, anchors588–604 (nominal starts294–302s). All contiguous
runs, exact case IDs and original index thirds remain in the saved summaries;
no successful window replaced the full population.

## Existing diagnostics describe failure without establishing a gate

The table reports per-camera medians over each complete correctness cell; accepted
matches and cheirality fractions are solver diagnostics, not verified match accuracy.
Fractions are null for zero matches; missingness remains explicit in full quartiles.
Cells can include missing outputs, so these medians are not a test restricted to
both-supported proposals and do not establish discrimination beyond the status gate.

| Recording | R≤5° cell | Accepted matches RGB / thermal | Inlier fraction RGB / thermal | Gray range RGB / thermal |
|---|---|---:|---:|---:|
| FlashLight | both_correct | 2375 / 977 | 54.5% / 63.3% | 201 / 89 |
| FlashLight | rgb_only_correct | 2114 / 351.5 | 57.4% / 30.1% | 203 / 93.5 |
| FlashLight | thermal_only_correct | 1792 / 1165 | 16.8% / 62.4% | 192 / 91 |
| FlashLight | neither_correct | 1807.5 / 441.5 | 11.3% / 29.5% | 199 / 93.5 |
| SmokeRoom | both_correct | 2872 / 1529.5 | 59.7% / 49.4% | 210 / 97.5 |
| SmokeRoom | rgb_only_correct | 2493 / 946 | 62.9% / 26.6% | 171 / 101 |
| SmokeRoom | thermal_only_correct | 2022 / 1347.5 | 1.2% / 64.2% | 154.5 / 50.5 |
| SmokeRoom | neither_correct | 1694 / 874 | 8.5% / 28.6% | 157 / 104 |

SmokeRoom's RGB gray-range IQR overlaps between thermal-only-correct152.75–159.75
and neither-correct153–170. Thermal inlier-fraction IQR also overlaps between
RGB-only-correct0.063–0.395 and neither-correct0.204–0.343. An apparent median
separation is not a calibrated or validated selection rule; no sign/cutoff was
tested. Thermal gray uses per-image16-bit min/max mapping, so its intensity range
is not physical temperature/exposure or directly comparable radiometry across cameras.
Fields of view and endpoints also differ. Native uncertainty was not extracted;
this study cannot conclude anything new about covariance calibration.

EXP078 already found mixed uncertainty/retained-fraction/coverage/prior-support
preferences, EXP087 closure worsened day proxy errors, and EXP112 geometry/history
failed its strict best-fixed comparisons. Those distinct conditional proxy
experiments were reviewed locally before this analysis. These descriptors do
not overturn them or become a new method just by applying them to another cache.

## Fixed visual evidence and limits

The R≤5° visual samples were locked before decoding: up to3evenly spaced original
indices in each both-supported correctness cell, for each recording. All24sampled
anchors and96source/target images pass own-record manifest SHA/time binding.
Eight contact sheets show both endpoints of both cameras, including error labels.

The fixed SmokeRoom thermal-only-correct sheet includes:

- Anchor481/240.5s: a visibly clear corridor in RGB; supplied-reference R error
  7.427°RGB versus0.681°thermal. Thermal benefit is not confined to visibly
  obscured RGB samples.
- Anchor604/302.0s: a heavily veiled RGB scene with a visible thermal person;
  errors5.779° versus0.346°. No segmentation or dominant-match-object claim.
- Anchor634/317.0s: partially veiled RGB corridor; errors12.929° versus0.699°.

The neither-correct sheet includes immediately adjacent anchor635/317.5s:
12.400°RGB/12.732°thermal. Both cameras'634targets are exactly their635sources,
and their saved gray-range minima are unchanged (149RGB/86thermal), despite
very different thermal errors. Another neither-correct sample748/374.0s shows
a visually clear textured RGB corridor, with9.094°RGB/9.097°thermal error.
Appearance alone does not explain all pose failures; no causal haze/density,
zero-lux, scene-dynamics or physical-reference claim follows from these sheets.

FlashLight's three thermal-only-correct samples are all preserved (anchors297,
298,476), alongside evenly spaced RGB-only/both/neither samples. Humans are
visible in thermal images from opposite correctness cells; visible people alone
are not established as either useful support or the source of wrong motion.

Artifacts: `results/paired-failure-strata/figures/` contains the readable contact
sheets and a full-case timeline PNG/PDF. Original render outputs remain under
`run-001/sheets/`; a separate source-pinned layout wraps long timestamp captions
without changing the locked sample, source pixels or error labels. Full paths,
times and source hashes remain in render receipts. The paper/source references
were reused; no fresh paper-body reading or author-system reproduction.

## Verification and decision

Independent audit passes5,304checks on features, numeric correctness labels,
full cohorts, status accounting, thirds/runs/linear quartiles, exact prior oracle
parity, deterministic sample IDs and rendered image hashes. All8tests pass,
including rehashed descriptor, correctness-cell, summary and sample corruptions.
Feature extraction used only cached matcher/input/image values before label join;
score-file bytes are hashed for provenance, but score values are not parsed in
that stage. No result, protocol or frozen source was changed after joining labels.

Results, logs, locks and receipts: `results/paired-failure-strata/`. Verified local
incremental packet: `/root/ore-ms-artifacts/paired-failure-strata-2026-11-16/`,
with parent restoration/priority dependency pins. Raw images, checkpoints, tokens
and local notebooks remain outside Git. No Box upload or Git push occurred.
The earlier studies were committed as4408b7fb8 before this task began.

Stop decision: characterization complete; do not fit a generic contrast/inlier
threshold or infer a causal smoke gate. The next bounded question is whether
incorrect-but-supported moving proposals collapse toward near-identity rotation.
Several inspected RGB errors are close to reference motion amount, which motivates
that hypothesis but does not prove it. Use cached rotation matrices, the same
fixed full cohorts, fixed-camera/identity controls and both recordings before
considering any new selection mechanism; keep physical-reference limits explicit.
