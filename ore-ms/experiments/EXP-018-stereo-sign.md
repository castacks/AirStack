# EXP-018: Signed stereo flow, saved-depth controls and camera-topic ordering

Date: 2026-10-20 (America/New_York). Branch `ore_ms`, base `75222af3e` plus the frozen EXP-018 source. **Completed: computational topic ordering and signed correspondence controls pass; physical stereo calibration remains unresolved. No defaults changed.**

## Findings and decision

The existing frontend computes depth from `abs(horizontal_flow)` and the provisional TD3 adapter uses the absolute projection baseline. Its saved positive depths cannot identify the original model flow sign. Bounded native stereo inference now retains that missing signed output. All **12 reconstructed depth arrays are bitwise identical**, including dtype and bytes, to their saved first-depth arrays. Strict original checkpoint hashes and active-state loading pass, including the previously audited RGB migration. No VO trajectory was rerun or replaced.

On the six fixed thermal stereo pairs, the learned horizontal flow follows observed signed feature displacements closely. Left-minus-right disparity is **negative model horizontal flow**. The observed day medians are positive and night window-start medians are negative; this is reproduced by the model, rather than introduced solely by a learned-flow sign error. Each image still contains a distribution of signed displacements; these medians are not universal signs for a recording.

| Thermal first frame | Fitted inliers | Observed disparity median, px | Model disparity median, px | Median signed horizontal error, px | Sign agreement, eligible matches |
|---|---:|---:|---:|---:|---:|
| day original | 163 | +5.820 | +6.023 | 0.259 | 157/158 |
| day early | 309 | +10.631 | +10.599 | 0.288 | 302/303 |
| day late | 138 | +4.361 | +4.677 | 0.262 | 135/135 |
| night early | 163 | -4.667 | -4.662 | 0.160 | 156/157 |
| night middle | 78 | -1.005 | -1.267 | 0.364 | 34/34 |
| night late | 54 | -3.462 | -3.419 | 0.169 | 53/53 |

Sign eligibility requires both observed and model disparity magnitudes >1 px; horizontal-error summaries include all finite fitted inliers. These are correspondence-consistency diagnostics, not verified feature ground truth or depth accuracy. The fixed vertical-residual sensitivity (<=1.5 px) retains thermal sign agreement 99.07–100%, with only 12 sign-eligible matches in night middle. All match coordinates, RANSAC masks and model outputs are retained.

Feature selection matters: the dense model field has **60.5–67.2% positive disparity pixels across all six thermal inputs**, even where fitted-feature medians are negative. Do not extrapolate feature agreement to the whole image or treat the dense model's majority sign as truth. Dense sign statistics include all pixels, with no texture/visibility/uncertainty validity criterion beyond finite output.

All **12 timestamp-associated raw-to-processed thermal images reproduce pixel-exactly** with the archived producer under OpenCV4.10.0.84, as labeled. Swapping raw left/right produces **zero exact image matches**. Day and night raw topic frame IDs, raw camera-info, processed camera-info and R/P matrices are identical; thermal projection maximum day/night difference is exactly zero. Thus these samples provide no evidence of a *computational raw-to-processed topic swap*. Metadata supplies labels, not authoritative physical camera identities or mounting history. This result cannot exclude a physical reassignment upstream of those labels.

The supplied thermal P matrices still have a negative baseline, while many day correspondences have positive disparity even after the vertical-residual restriction. Positive-depth projection-sign agreement in the restricted day samples is only 15.7%, 12.1%, and 35.7% (original/early/late), versus 100%, 91.7%, and 100% at the selected night starts. A blanket sign flip cannot reconcile both recordings or the mixed signs within images. The previous body/stereo transform discrepancy also remains. **Keep defaults; do not adopt corrected metric depth, a sign flip, or cross-modal fusion from this audit.**

## Fixed inputs and reproducibility

The fixed population is the first stereo pair of day original/early/late and night early/middle/late, for RGB and thermal. Exact images and timestamps come from `report.json/input_rows[0]`, not visual or error-based reselection. Day manifest starts are 348/306/1132; night starts are 123/619/789. The source arrays are `results/uncertainty/run-001/{original,early,late}/` and `results/separate-clip/night-run-001/baseline/{early,middle,late}/`.

Missing images and camera metadata were extracted from the original day/night Box ZIPs by authenticated byte ranges. Python ZIP reads validate selected-member CRCs; new extracted-file hashes and range metadata are recorded. The whole 8.86GB/7.71GB archives were not downloaded or rehashed. Raw thermal association uses unique timestamps within 2ms, preserving original missing timestamp indices; all 12 selected raw/processed timestamp differences are zero. The original model ZIP was restored independently, SHA256 `77ad168672019f94d8e48ebb087cb109b938bcf8784e3df480070cd5f437f00c`, with ZIP CRC validation.

Matching remains EXP-017's fixed SIFT cap2000, bidirectional ratio0.7 mutual matches, fundamental RANSAC1.5px/confidence0.999/seed0. A fitted fundamental matrix can accept correspondences inconsistent with horizontal rectification; both all-inlier and <=1.5px vertical populations are reported. Reconstruct magnitude maps with execution `fx*abs(baseline)/depth`, then bilinearly sample at left feature coordinates. Principal-point offsets are retained in the separate signed-P diagnostic. Model and feature signs are compared in the same left-minus-right convention.

RGB day controls have 659/853/643 fitted inliers, median signed horizontal errors 0.229/0.230/0.274px and sign agreement 100%/99.88%/100% on eligible matches. All **three night RGB inputs lack a usable fixed-protocol SIFT fit**, including early RGB (insufficient matches; middle/late have no descriptors). They remain empty populations, not successful validations. Their model execution and saved-depth reproduction still pass. Temporal VO tracking in earlier experiments is a different measurement from this first-frame stereo feature fit.

GPU inference uses Torch2.7.1+cu128 on RTX PRO5000 Blackwell, native supplied uint8 PNGs, BGR-to-RGB, CPU float32 /255, no resize, fp32 encoder/decoder and seed0. Models strictly load the original RGB/thermal checkpoints and decoder-depth settings. GPU image ID is `sha256:ffce9f0f1142b71cb518382328a4196f87f7fc6f4bde1a0e36069c612e69cad9`; source Dockerfile, exact package inventories and output versions are retained. Geometry/preprocessing runs separately in `Dockerfile.geometry`, pinned OpenCV4.10.0.84/NumPy2.2.6. Inference consumes supplied PNGs directly; its OpenCV5 decoder produces unchanged model depths.

## Validation and recovery

13 numeric/regression tests pass. All 112 CPU-plan and 174 signed-plan file hashes pass before/after their runs and independent validation. All 12 signed attempts pass finite flow/positive covariance and depth controls; maximum relative depth difference is exactly zero, stronger than the preregistered1e-4 gate. An independent SciPy interpolation implementation reproduces sampled model flows and signed-error/sign-count statistics; saved depth dtype/bytes match. All source calibration, model files, inherited depth arrays and EXP-017 outputs remain unchanged. PNG/PDF figure visually inspected.

Primary artifacts are `results/stereo-sign/{inputs,run-001,signed-run-001,staging}/`, `validation.json`, and `signed-correspondences.png/.pdf`. The raw topic-order diagnostic and CPU matching results are independent of the signed inference results. CPU and signed plans are separately frozen; no failed primary analysis run is replaced. Initial setup issues (pytest's unavailable jaxtyping addopts, and validation's Python module search path) were corrected before successful checks and recorded in staging metadata.

Box snapshot: `backups/2026-10-20-exp-018`. It contains this experiment's new input samples, raw signed outputs, full match arrays, plans, validation, figures, runtime inventories, source/tests and reports. It excludes credentials, checkpoints, disposable ZIP-range cache blocks and full dataset archives. The original code tree and previously documented snapshots provide inherited first-depth/control inputs; model recovery still uses the original shared checkpoint source.

## Next bounded step

Obtain authoritative physical thermal camera identity/mounting and per-recording stereo/body calibration provenance, specifically whether the same optical-frame labels/calibration apply to both recordings. Software replay and model sign agreement do not supply that information. In parallel, the preserved signed fields permit a preregistered fixed-grid spatial sign/vertical-consistency audit to separate feature selection from whole-image geometry. Neither step requires fitting a fusion rule; independent geometric reference is still needed for metric uncertainty calibration.
