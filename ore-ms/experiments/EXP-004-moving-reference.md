# EXP-004: First moving-window comparison to recorded SLAM

Date: October 18, 2026 (America/New_York). Branch `ore_ms`, base commit
`af553955e844d42abb164ce5218e6be06cf19c8c` with the current uncommitted reference
analysis and start-offset runner. Earlier setup/data provenance: EXP-001–003.

## Measured result

Both modalities complete 20 frames with no interpolation fallback, finite
trajectories, and finite retained 3D covariances with positive minimum eigenvalues.
Against the recorded SLAM proxy, conditional anchored translation RMS is 0.1277 m
for RGB and 1.5171 m for thermal on this one moving window. No scale fitting is
applied. This is evidence of different disagreement on this particular test,
not a benchmark, statistical modality ranking, covariance calibration result,
or independent ground-truth accuracy claim.

| Modality | Frames | Matches | Reference displacement (m) | VO displacement (m) | Anchored translation RMS (m) | Anchored rotation RMS (deg) | Consecutive translation RMS (m) | Consecutive rotation RMS (deg) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| RGB | 20 | 3734 | 8.872 | 8.996 | 0.1277 | 0.3108 | 0.0309 | 0.1038 |
| THERMAL | 20 | 3325 | 8.857 | 11.377 | 1.5171 | 5.6879 | 0.1415 | 0.7869 |

The thermal trajectory travels farther than the proxy over this window. Calibration,
processed-image/training mismatch, stereo/depth estimation, and reference error
are possible contributors; their contributions have not been isolated.

## Why this reference is interpretable

`super_odometry/odometry/info.yaml` explicitly identifies poses of `vehicle` in
`sensor_init`. The 17,410 high-rate pose samples use xyz and xyzw quaternion
ordering. Their interpolation at archived dynamic TF times matches TF numeric
values to numerical precision: median translation residual approximately
8.9e-16 m and rotation residual approximately 1.57e-15 degrees. Inverting the
TF matrices instead yields median translation disagreement approximately 31.78 m.
This establishes the recorded direction and confirms both exports represent the
same source. It does not independently validate the source trajectory.

Reference translation is linearly interpolated; rotation uses SLERP at each
left-camera acquisition timestamp. Nanosecond epochs remain int64 until converted
to small relative seconds for interpolation. Extrapolation, duplicate/reference
non-monotonic timestamps, nonfinite poses, and invalid quaternions are rejected.

## Selection and coordinate protocol

Selection uses body-reference motion only, without inspecting VO estimates.
Among 20-frame matched windows with adjacent RGB acquisition gaps no greater
than 150 ms, the selector chooses greatest reference endpoint displacement.
Result: manifest rows 348–367, original source image indices 452–471 in all four
streams. Duration is approximately 1.899992 s; body-reference displacement
approximately 8.873249 m. This is a high-motion diagnostic window, not an
unbiased or representative evaluation split.

Camera reference is formed as:

`T_world_vehicle(t) @ T_vehicle_optical @ H(R_rect.T) @ H(R_optical_from_NED)`

The static vehicle/camera chain comes from the supplied YAML; `R_rect` is the
left processed-camera rectification rotation. The optical-from-NED basis maps
NED `(forward,right,down)` to optical `(right,down,forward)`.
Both estimated and reference trajectories are individually expressed relative
to their first pose. No multi-pose best-fit SE(3), timestamp-offset fit, or scale
alignment is performed. Consecutive relative-pose residuals use
`inverse(reference_delta) @ estimated_delta`.

RGB and thermal references are sampled at their respective actual times, not a
shared fabricated time. Identity T_BS and absolute projection baseline remain
explicit execution choices from EXP-003. Thermal signed calibration is still
unresolved, so the reference comparison is conditional on the supplied
extrinsics/rectification interpretation. No reference error bars are known.

## Validation and artifacts

20 focused tests pass, including interpolation of known translation/rotation,
extrapolation and duplicate-time rejection, camera lever-arm composition,
optical/NED basis, identity and known-scale-error metrics, reference-only window
selection, adapter input behavior, and earlier checkpoint/pairing checks.
A SciPy test-fixture angle array needed explicit `(N,1)` shape in this runtime;
the final tests pass. Both native-resolution moving runs complete with no
interpolated frames. The optional torch.compile fallback remains unchanged.

Local outputs:

- `results/moving-vo/run-001/{rgb,thermal}/`: complete runner reports, source
  pairs, effective configs, depth, graph observations/covariances, trajectories,
  and exported `proxy-reference.npz` camera-to-world pose matrices.
- `results/reference/reference-audit.json`: reference and window audit.
- `results/reference/moving-comparison.json`: numeric metrics and protocol.
- `results/reference/moving-comparison.png`: trajectory and disagreement plots.
- `results/reference/run-meta.json`: source hashes, base commit, image ID.

The plot is generated by local `results/reference/plot_moving.py`. Each modality
is plotted in its own initial camera NED frame; neither plot implies vehicle/world
alignment or a common camera origin.

## Reproduction

Use the existing `macvo:blackwell` project mount. From `MAC-VO-main/`, first run
`Scripts/Setup/td3_reference.py` as documented in `Scripts/Setup/README.md` to
confirm selection. Then run each modality into a fresh output directory:

```bash
python Scripts/Setup/run_td3_execution.py \
  --dataset ../data/tartandrive3/20251027/1_salon_mac_cov_day \
  --manifest ../results/data-access/paired-manifest.json \
  --output ../results/moving-vo/new-run \
  --modality rgb --start 348 --frames 20 --acknowledge-provisional-geometry
```

Repeat for `thermal`. Add `--trajectory-root ../results/moving-vo/new-run` to the
reference analysis command to compute the comparison. Run tests with:

```bash
python -m pytest Scripts/UnitTest/test_research_setup.py \
  Scripts/UnitTest/test_td3_sequence.py Scripts/UnitTest/test_td3_reference.py -q
```

## Next measurement

Use the retained per-feature observations to define a geometric consistency
residual against this proxy motion, explicitly accounting for reference and
calibration uncertainty. Audit thermal scale/preprocessing sensitivity before
interpreting its larger trajectory disagreement as covariance miscalibration.
Repeat on additional predeclared windows before claiming predictive relationships
between RGB/thermal uncertainty and relative geometric reliability.
