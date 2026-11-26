# Offline geometry-to-M08 adapter R1

26 November 2026 UTC. Base `8ac55958a02a23aa478b50d9f016771bad4ddb76`, branch `ore-ms-orb-development`. Qualification of the interface on four analytic issues passed. This is a deterministic software and synthetic-label check. No Isaac capture, full M08 campaign, new training or physical data was used. The M05–M08 files and their thresholds remain as issued. [Isaac capability audit](ISAAC_SIM_CAPABILITY_AUDIT_R1.md) · [M08 frozen plan](m08/EXPERIMENT_PLAN.md)

## Contract and implementation

- **Geometry.** The [versioned fixture](offline-geometry-m08/fixture.json) places the stationary receiver at `[0, 0, 0]` and the source at `[10, 0, 0]` in metres. The declared frame is receiver-origin, +X forward, with longitudinal distance positive toward the source. The adapter computes the 10 m Euclidean path and rejects incompatible length or axis. This is known analytic geometry with a designated unobstructed source. The fixture does not perform USD ray casting.
- **Time.** The four issues use integer ticks 0, 37, 91 and 148 at 100,000,000 ns per tick. Their targets are ticks 20, 57, 111 and 168. Each row binds `target_ns − issue_ns = 2,000,000,000` exactly. These are independent realizations with their own issue epoch and stationary draw. The tick values are schedule metadata, with no continuous trajectory across realizations. Model time is in seconds relative to each issue. The period is configurable when it divides two seconds exactly.
- **Observations.** Initial and detector seeds are explicit in the fixture and distinct from the future RNG. M05 supplies the stationary field and M08 path-average burden. The adapter projects it into the unchanged M07/M06 detector model at the matched material, range and air settings. The result is four simulated 12-bit integer codes. The predictor receives only `case,realization,split,R_code,G_code,B_code,lwir_code`. Stable realization IDs join out-of-band schedule metadata and the common evaluation mask. [M05 process](m05/environment.py) · [Detector projection](m07/forward.py) · [M08 schema](m08/contract.py)
- **Labels.** A separate routine evaluates local burden at the 16 midpoint locations. It sums segment length in metres times extinction in m⁻¹ and applies `exp(−optical_depth)` separately to 550 nm visible extinction and 11 µm LWIR absorption. The stored labels are dimensionless synthetic transmission. A separate frozen reference implementation checks the OU and field projection, and closed-form checks verify the optical-depth computation. A synthesized LWIR code includes modeled emission/reflection and path radiance. The 11 µm label is a monochromatic absorption proxy. [M06 spectral data contract](m06/DATASET_SCHEMA.md) · [M08 independent formulas](m08/reference.py)
- **Forecasts and scoring.** Joint, RGB-only and joint persistence call the unchanged M08 methods on the same serialized issue row. The RGB method excludes LWIR. Persistence holds the current joint marginal. The adapter keeps 0 and 2 s forecast rows for verification. Each issued row contains 128 equally weighted burden draws for its own horizon, plus status, flags and effective sample size. At +2 s the scorer applies the frozen 550 nm map and emits empirical CRPS, median and central 95% interval endpoints. The target is monochromatic transmission. No physical transmission reference is implied. Missing forecasts remain explicit and blank-scored, with the episode still on the common mask. The qualification fixture issued all outputs. [M08 inference](m08/predict.py) · [Target mapping](m08/dynamics.py) · [Scoring](m08/scores.py)

## Qualification gates

| Gate | Result | Evidence |
| --- | --- | --- |
| Geometry and exact time | PASS | Confirmed the 10 m +X ray and all four +2 s associations. A 250 ms period produced eight ticks. The verifier rejects an off-axis 10 m vector, a 9 m path, a non-dividing period, a negative tick and duplicate IDs. |
| Issue schema and deterministic replay | PASS | Four legal non-rail issue rows parsed through the frozen schema. All five generated CSV hashes and the run manifest matched on repeat. |
| Independent reference | PASS | Vacuum gives 1. Constant and piecewise paths with optical depth 1 give 0.3678794412, and doubled length at constant extinction gives 0.1353352832. Invalid inputs were rejected. All eight label rows are bounded and agree with the frozen independent OU/path reference and closed-form band maps within 1.12e−16. |
| Matched controls and scoring | PASS | 24 forecast rows and 12 evaluation rows. All three arms use identical geometry, issue/target ticks, reference label and mask per episode. Masked-LWIR RGB outputs agree, joint persistence preserves the joint issue marginal, and independently recomputed pairwise CRPS agrees. A rail injection followed the frozen prior-fallback behavior with explicit rail flags. |
| Future-information separation | PASS | Reran the full adapter after replacing only the future seed base from 535000 to 536000. Schedule, issues and forecasts were byte-identical. All four time-zero labels stayed identical, all four future labels changed, and only future label and CRPS columns changed in all 12 evaluation rows. The forecast function receives the issue file alone and runs before label generation. |
| Manifest consistency | PASS | Verified 122 checksum entries across the M05–M08 package and plan manifests, and verified generated artifacts against their run manifests. The qualification JSON reproduced byte-for-byte in a second invocation. |

The mean +2 s CRPS was **0.11010 joint**, **0.11108 RGB-only**, and **0.14369 joint persistence** on these four qualification issues. These values exercise scoring and control matching. Four fixed issues do not support an inference of fusion advantage. [Machine-readable qualification](offline-geometry-m08/qualification.json)

## Reproduce offline

From the AirStack root, use CPython 3.12.3 to reproduce the retained hashes. The verification needs only the standard library and the frozen local package files.

```bash
export PYTHONDONTWRITEBYTECODE=1
out="$(mktemp -d)/qualification"
python3 ore-ms/ore-ms-research/offline-geometry-m08/qualify.py "$out"
```

The [adapter](offline-geometry-m08/adapter.py) can also be run directly with a new output path and optionally `--fixture`. Each run writes `schedule.csv`, `issues.csv`, `forecasts.csv`, `truth.csv`, `evaluation.csv` and `manifest.json`. The manifest records fixture, adapter and frozen manifest hashes plus per-artifact hashes and seeds. The [qualification runner](offline-geometry-m08/qualify.py) retains baseline, repeat and future-counterfactual runs outside Git. The version-controlled JSON contains the verification summary and hashes.

**Smallest remaining dependency.** Obtain one separately authorized static Isaac geometry receipt that binds a stage-local camera ray to the same known first hit and simulation clock. Require a pinned stage and asset manifest, resolved metre scale and optical-axis convention, independent range check at the 10 m source, and exact issue-to-target tick association. Verify those fields against the analytic fixture before replacing its geometry input. The current audit makes no Isaac geometry, hardware or M12 acceptance claim. [Existing fixture protocol](isaac-readiness/GEOMETRY_FIXTURE_PROTOCOL.md) · [Isaac entry gates](isaac-readiness/ISAAC_ENTRY_CRITERIA.md)
