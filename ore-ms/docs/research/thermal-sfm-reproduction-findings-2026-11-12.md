# ThermalSfMLearner-MS: outdoor source-protocol reproduction

12 November 2026, America/New_York. Reference key: `thermal-sfm`.
**EXECUTED_CLOSE_AGREEMENT_NOT_EXACT_TABLE_MATCH.** The supplied outdoor
checkpoints run successfully on both full official ViViD night sequences.
Headline depth errors closely match the paper; the frozen all-column/pose
rounding criterion does not pass. No checkpoint or parameter search followed.

## Provenance and fixed setup

[Paper 2103.00760v3](https://arxiv.org/html/2103.00760v3);
[author source d2ab0fc5](https://github.com/UkcheolShin/ThermalSfMLearner-MS/tree/d2ab0fc5e92972e248b641572f4b595dd67af83d).
User supplied the author Drive data and model links. Four downloaded assets
match listed byte sizes; their local SHA256 values and all 46,819 regular
archive member hashes are retained. No provider checksum was exposed by the
metadata interface, so these are intake identity pins, not publisher attestations.
Indoor assets and training sequences were not used for inference.

The unchanged author raw loader/dump routine yields 989 night1 and 1,030 night2
frames, exactly 2,019 overall, matching the paper test count. It retains the
author cumulative-speed selector, frame IDs, native thermal samples, calibration,
resize and pose/depth serialization. Indoor-only preparation steps were omitted.
The model state dicts load strictly: depth epoch139, pose epoch88, no missing or
unexpected keys. Their metadata does not certify the precise T/MS loss variant
or training split; do not identify a checkpoint from the closest table row.

The [pre-outcome plan](thermal-sfm-execution-plan-2026-11-12.md) fixes both
sequences, supplied checkpoints, source preprocessing/evaluators and tolerances.
Original MAC runtime remains unchanged. A separate compatibility image
`sha256:fd7be8a3b1fe788972c54cabb5ee79433c30a53ab9951f49cf8c442f0753cf0a`
uses Torch2.7.1/CUDA12.8 and explicit legacy NumPy aliases/checkpoint loading.
Its exact reconstruction from the compiled MAC base plus a 17,625,507-byte
delta passes all archive member hashes. It is not the paper's historical
Python3.7/Torch1.5.1/CUDA10.2 runtime. No author model, loss, checkpoint or
numerical evaluation setting was changed. No training occurred.

## Depth results

Author protocol: inverse-depth resize, provided depth support in(0.001,80)m,
per-image median scaling, unweighted per-image error means. Primary comparison
pools both sequences; the predeclared sequence-macro sensitivity is also saved.
These are reference-scaled monocular metrics, not unscaled stereo-VO errors.

| Metric | Pooled result | Paper TableI MS | Within fixed ±0.0005 |
|---|---:|---:|---|
| abs_rel | 0.14570913 | 0.146 | Yes |
| sq_rel | 0.87263359 | 0.873 | Yes |
| rmse | 4.69668120 | 4.697 | Yes |
| rmse_log | 0.18429872 | 0.184 | Yes |
| a1 | 0.80049720 | 0.801 | No |
| a2 | 0.97296248 | 0.973 | Yes |
| a3 | 0.99358198 | 0.993 | No |

Five of seven rounded depth targets pass. AbsRel0.145709 and RMSE4.696681
round to the published0.146 and4.697. The a1 mismatch is only0.000502795
from0.801, just outside the prespecified tolerance; a3 is0.993582 versus0.993.
Neither tolerance nor aggregation is changed to make them pass. Sequence-macro
AbsRel/RMSE are0.145657/4.690124; this secondary result does not replace pooling.

The complete unchanged author depth evaluation scripts were also run. All16
printed per-sequence metric values agree with our reconstructed values at
their displayed precision; all2,019 saved author scaling ratios agree within
their four-decimal serialization precision.

## Pose results and source padding

Original five-frame snippet evaluation fits translation scale from references.
RE below is the source angle in radians. Original code allocates N rows while
emitting N−4 snippets, leaving four zero rows per sequence. Source-script
means/std below preserve that behavior. Actual-snippet statistics are reported
separately, without replacing the frozen primary convention.

| Sequence | Actual / allocated snippets | Source ATE mean ± std | Source RE mean ± std | Paper ATE mean ± std | Paper RE mean ± std |
|---|---:|---:|---:|---:|---:|
| outdoor_robust_night1 | 985 / 989 | 0.05613295 ± 0.03090074 | 0.02871061 ± 0.01437337 | 0.0562 ± 0.0310 | 0.0287 ± 0.0144 |
| outdoor_robust_night2 | 1026 / 1030 | 0.05951994 ± 0.03157374 | 0.02741193 ± 0.01238937 | 0.0598 ± 0.0316 | 0.0274 ± 0.0124 |

Valid-snippet ATE/RE means are0.05636089/0.02882720 for night1 and
0.05975199/0.02751881 for night2. The original eight unused rows are explicitly
retained in source predictions, never described as real evaluated snippets.
All2,011 actual snippets complete (8,044 pair-network calls). All2,019 depth
frames complete. All RE means/stds agree within the frozen four-decimal tolerance, but ATE differences make the pose paper-table criterion fail overall.

The README's night1 MS row says0.0572/0.0087, whereas paper v3 says
0.0562/0.0287. This discrepancy was documented before outcomes. Our result is
close to the paper's RE, not the README's value; that is not proof of why the
README differs. Runtime, preparation and checkpoint-version effects are not
causally separated by this reproduction.

## Verification and retained failures

The independent saved-output calculation agrees with extracted author metric
functions on16,152 depth metrics and4,022 pose metrics. Full-script rounded
metric/ratio/log controls pass another2,043 checks. All8,129 frozen pins remain
unchanged after the final launcher amendment. All four prediction files are
sealed in the prediction lock; depth scoring occurs after that lock. The
unchanged pose source computes reference scores during inference, with no
reference-dependent prediction, fitting or checkpoint selection added.

Initial freeze4d51b0beb precedes the first attempt. The depth launcher first
stopped before forward inference because the author path concatenation expected
a trailing slash. Amendment946e053f4 fixes that CLI detail before the first
successful depth forward. The pose launcher then stopped at a module import
before its first forward. Amendment18ca30511 matches direct author import-path
semantics; the completed night1 depth output was retained without rerunning.
Both failed attempts and their source/freezes are archived separately. An
initial Dockerfile base-ID resolution failure was corrected to a verified
local base tag before either attempt; build logs are retained. These are
launch/runtime issues, not scientific retuning or selective retries.

## Decision and scope

We now have an executable, audited author thermal baseline with close depth
headline agreement. Do not claim exact table or historical-runtime reproduction:
the declared all-column criterion fails and checkpoint variant metadata remains
limited. Preserve the positive agreement and precise mismatches together.

No MS² evaluation, adaptation or comparison against MAC/RGB is established.
The author loader also uses night1 for validation; this is reproduction of its
benchmark protocol, not a fresh model-generalization study. Any transfer needs
an evidenced radiometric conversion and explicit monocular scale policy; a
reference-fitted translation must not be compared silently to unscaled stereo
VO. Previous local refinement/rotation configurations remain closed.

Artifacts: `results/thermal-sfm-reproduction/` retains intake/member hashes,
preparation/build logs, failed attempts, all predictions, per-frame/snippet
metrics, source-script controls and the summary. The [living library](references/README.md)
records paper reading history, code/checkpoint pins, dataset-version distinctions
and this outcome. Full copyrighted paper text is excluded from backups.
