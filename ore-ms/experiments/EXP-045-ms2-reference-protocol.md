# EXP-045: MS² reference audit and bounded evaluation protocol

Date:2026-10-25 America/New_York. Branch `ore_ms`; starting commit `d4a163977`.
**A conditional provided-pair depth diagnostic is defined. Physical temporal/cross-modal error evaluation remains ineligible.** No model inference, error scoring, direction selection, clock correction, fitted calibration, VO or fusion was performed.

## Source evidence and intended pose convention

Re-read and archive the official [data format](https://sites.google.com/view/multi-spectral-stereo-dataset/data-format), [setup](https://sites.google.com/view/multi-spectral-stereo-dataset/setup) and [original loader at 0bb0fc1a](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset/blob/0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284/dataloader/MS2_dataset.py). Source URLs, retrieval date, byte counts and SHA256 are in `results/ms2-reference-protocol/sources/registry.json`. No new author communication was sent.

The loader associates image and pose files by identical stem and returns absolute poses relative to the first image. Its commented normalization rotates translated position columns by the inverse first rotation. This is consistent with an intended camera-to-initial pose convention, **an interpretation of loader comments rather than an exposure/export implementation specification**. The supplied webpage identifies left-camera coordinate references derived from GNSS/IMU and sequence-relative initialization. Neither source documents the full projection/rectification/exposure association needed to certify physical reprojection labels. Do not silently populate adapter `gt_pose`.

The navigation format identifies position, quaternion, NED velocity, angular rate and three filter-status fields. Those raw navigation samples are absent from this bounded pilot; their schema is available but their values/quality are not verified. This is a missing-input limitation, not a statement that the original full archive omits them.

## Frozen reference and timing audit

Recover only missing pilot/metadata artifacts from EXP041 after verifying its archive and every member hash. Keep EXP041–044 inputs and outputs unchanged. Audit the original first20 IDs000000–000019, not a motion-selected interval. Read raw calibration with the existing numeric-array-only unpickler and retain recorded integerns timestamps. No arbitrary pickle loading or rotation projection.

| Reference quantity | RGB | Thermal |
|---|---:|---:|
| Initial pose max absolute deviation from identity |3.12e-17|1.11e-16|
| Left-camera pilot duration (s) |1.933097728|1.945087607|
| Endpoint translation under camera-to-initial interpretation (m)|0.023911163|0.023726863|
| Path length, camera-to-initial interpretation (m)|0.027975719|0.027554515|
| Path length, inverse interpretation (m)|0.027972106|0.027552225|
| Endpoint rotation, camera-to-initial interpretation (rad)|0.000514830|0.000525958|
| Full-sequence recorded stereo pairs within inherited2ms gate|10130/10131|0/10131|

All40 pilot poses pass the retained1e-5 rigidity screen. Inversion gives almost identical motion magnitudes here: this near-static interval cannot select a transform convention from these statistics. Both interpretations are exported and neither is selected using model outputs. Their similarity is not a convention-validation result.

Raw calibration gives identity stereo rotation and horizontal right-camera translations−0.2991842053m RGB/−0.3038995142m thermal. Intrinsics and baseline match the unchanged adapter manifest. Under this **provided rectified-camera model**, define positive left disparity `d = u_left-u_right = fx*B/Z`; expected forward left-to-right horizontal flow is `−d`, vertical flow0. This mathematical convention does not establish physical simultaneity or depth accuracy.

All4 timestamp series are strictly increasing,10,131 samples each. Every pilot time equals its original source row. The existing2ms stereo/30ms left-camera cross gate admits0/20 pilot groups; it is unchanged. Keep all four recorded timestamps per ID. Do not fit a latency or re-pair images to improve agreement.

## Protocol frozen before error scoring

The next bounded diagnostic may reuse EXP044's20 saved stereo outputs per modality with precisely these source IDs, explicit thermal recipe and original checkpoint hashes. It requires no new inference, no temporal flow, no pose-direction choice and no new dataset acquisition.

1. **Primary label:** supplied `depth`, single-scan LiDAR projection, uint16/256meters; zero invalid. It remains a projected reference proxy with timing/occlusion/calibration uncertainty. Use `depth_multi` and `depth_filtered` as separately labelled sensitivities, never silently merge them. Multi-scan products depend on navigation/ICP; filtering also depends on a stereo network.
2. **Primary population:** every nonzero depth pixel whose `u_right=u_left−fx*B/Z` is in `[0,width−1)`, allowing both horizontal interpolation neighbours. Require no model confidence, feature retention, residual or output-based selection. Retain counts and exclusions. This mask establishes geometry coverage only; it does not verify visibility or reject dynamic objects.
3. **Target:** supplied-pair stereo flow residual against `(-fx*B/Z,0)` at the original left pixel. Preserve signed horizontal and vertical components. Report per-modality descriptive error and uncertainty association; document proxy noise. A physical covariance-calibration claim or Gaussian NLL consistency claim is not justified without modelling reference uncertainty.
4. **Units:** report pixels and normalized image-plane residuals `(du/fx,dv/fy)`; transform the model's diagonal covariance by `diag(1/fx,1/fy)`. These are normalized image-plane quantities, not full3D or pose covariance. Do not compare raw pixel variance between resolutions or call normalized coordinates equal physical scene populations.
5. **Aggregation:** preserve each image's counts and summaries, then report the20 per-image summaries. Do not pool millions of spatially dependent pixels as independent evidence or infer generalization from this previously inspected pilot. RGB and thermal have different depth support/FOV/visibility and exposure assumptions; no direct winner-selection/fusion claim is permitted from these separate populations.
6. **Sensitivity:** retain the same source IDs and formula for both alternative depth variants. Report differing support counts and, if comparing variants, explicitly report their common support separately. Do not choose the best variant, threshold or pose convention after inspecting errors.
7. **Gates:** hash-check all source pixels, manifest, calibration, checkpoint and saved flow/covariance files. Require finite output arrays and native geometry. Preserve missing/nonfinite outputs as failures; do not repair or interpolate them. Report the unchanged failed timing diagnostic alongside every result.

Coverage established before scoring:

| Source label | RGB nonzero / right-in-bounds pixels | Thermal nonzero / right-in-bounds pixels |
|---|---:|---:|
| Single scan `depth` |142933 /141472|187027 /184792|
|32-scan `depth_multi`|387880 /385381|283565 /280636|
|Network-filtered `depth_filtered`|295889 /295751|219444 /219300|

These counts are coverage, not independent sample sizes or reliable-correspondence counts. No error values or covariance rankings have been computed in this experiment. The primary diagnostic can characterize agreement with supplied projected references under the provided-pair protocol. It cannot establish relative actual geometric reliability across modalities.

## Remaining physical evaluation gates

Temporal reference reprojection requires explicit transform action/direction, applicable rectified-camera frames, camera/pose time association, navigation quality and treatment of dynamic/occluded points. Physical stereo/cross-modal comparisons additionally require exposure/timestamp semantics or a justified bound on their effect under the evaluated motion. Training recipe/split provenance remains unknown; this pilot is not verified training-held-out. Do not infer physical error bounds from recorded receive-time offsets or the small navigation motion alone.

Keep EXP042's prepared timestamp question. Add: do exported odometry matrices map current rectified-left-camera coordinates to the initial rectified-left frame; what equations and camera/INS timestamp association produce them; are rectification rotations included; which nav-status/accuracy fields qualify the exported poses? No question was sent. Clarification or appropriate acquisition/export code would materially strengthen physical evaluation; more copies of the same near-static20 images will not resolve it.

## Reproduction and validation

In `macvo:ms2-smoke`, from `MAC-VO-main` with AirStack mounted at `/root/AirStack`:

```bash
python -m Scripts.Setup.ms2_reference_protocol \
 --root /root/AirStack/ore-ms \
 --manifest /root/AirStack/ore-ms/results/ms2-timestamp-semantics/source-protocol-input-manifest.json \
 --output /path/to/fresh-audit.json
```

Output paths must be fresh. The audit freezes166 calibration/manifest/pose/timestamp/depth inputs and verifies them unchanged. Independent validation uses scalar Gauss-Jordan inversion and matrix multiplication for both pose hypotheses, a division-free alternative support inequality for all120 depth maps, integer timing controls and350 historical source/result hash checks. All958 controls pass. Runtime is the recovered smoke image, NumPy2.2.6/OpenCV4.10.0; startup driver diagnostics are retained. No GPU/model inference required.

Artifact path `results/ms2-reference-protocol/` contains source snapshots/registry, audit JSON, independent validator and validation JSON. Recovery target Box `backups/2026-10-25-exp-045`; EXP043+EXP044-complete remain prerequisites for original pixels/outputs. The new recovery snapshot includes the additional EXP041 calibration/readme/timestamp metadata, without full dataset archives, packages, tokens or model weights.
