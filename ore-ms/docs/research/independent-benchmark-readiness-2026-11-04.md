# EXP133: independent benchmark specification and readiness

2026-11-04, branch `ore_ms`, parent `21ab29c61`. Specification freeze `7a24ae69c`; strict timestamp amendment `731a5a62d`.

**The benchmark specification and common scorer are prepared; benchmark execution is not ready. No reserve payload, new inference, reference scoring, external submission or claimed SOTA result occurred.** The two reported MS² reserves remain closed and unassigned to active validation/test. This work defines what a fair comparison needs and identifies the concrete missing implementations.

MS² is the primary candidate because its official sources provide synchronized RGB/thermal stereo and camera-frame odometry. It supports a qualified provided-reference comparison, not an automatic physical-ground-truth or official VO leaderboard claim. The existing public-data access scripts were previously supplied by the user. Official depth splits are not silently treated as a VO protocol. [Official repository](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset) · [Data format](https://sites.google.com/view/multi-spectral-stereo-dataset/data-format).

| Comparison role | Fixed modes | Readiness |
|---|---|---|
| Matched-inference selector track | RGB, thermal, RGB preference, metadata-condition prior, raw uncertainty, angular-proxy uncertainty | Local experts/weights exist; full trajectory publisher and test commands/proofs pending. Both experts execute. |
| Published stereo context | MAC-VO RGB; ORB-SLAM3 RGB/thermal with loop closing off; DROID-SLAM stereo RGB/thermal | Pin official source revisions; separate clean baseline builds/assets/adapters and development smoke required. |
| Project thermal context | Current fixed MACT checkpoint/backend | Label as project checkpoint/configuration; training overlap and recipe equivalence unknown. |
| Diagnostics | Six frozen EXP131 ridge models | No retraining, averaging or outcome-selected model; not an adopted candidate. |
| Separate/optional | DPVO monocular; TartanVO stereo | DPVO does not belong in primary metric-stereo ranking; local TartanVO pose/depth assets absent. |

The machine-readable matrix has12 primary method modes. Selector rows consume four camera streams, while individual stereo-system rows consume two. They are not sensor/compute-matched claims. Standalone systems use their documented backend scope; DROID final refinement and causal received-output selection are not a common live-latency mode. No method is declared the latest leaderboard leader merely because it is a representative published baseline.

Pinned upstream sources, retrieved from official repositories before benchmark outcomes:

| Source | Revision |
|---|---|
| [ms2](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset) | `0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284` |
| [macvo](https://github.com/MAC-VO/MAC-VO) | `8683b532d6f6e8981f6b1a646fb5f52f02c1ea61` |
| [orbslam3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` |
| [droid](https://github.com/princeton-vl/DROID-SLAM) | `2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99` |

Two source details affect comparability. ORB-SLAM3 reads `loopClosing` from settings, so the stereo VO-context mode can explicitly disable it. [Pinned implementation](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/System.cc). DROID’s EuRoC example multiplies estimated translation by1.10 and invokes scale-corrected alignment; its geometric stereo path uses an internal0.1baseline. A new adapter must derive metric conversion from actual supplied calibration and verify all relevant code paths. The EuRoC multiplier may encode that dataset’s calibration; it must not be copied into MS² or replaced by a reference-fitted factor. [Pinned evaluator](https://github.com/princeton-vl/DROID-SLAM/blob/2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99/evaluation_scripts/test_euroc.py) · [Pinned geometry](https://github.com/princeton-vl/DROID-SLAM/blob/2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99/droid_slam/geom/projective_ops.py).

Local inventory confirms the two existing74MB project checkpoints, with hashes frozen in the contract. At the configured benchmark locations, ORB source/library/vocabulary and DROID source/checkpoint are absent; the pinned scientific runtime has no installed DROID backend or DPVO module. The inventory is scoped to configured paths/imports, not a global filesystem census. Current MAC execution is a modified project checkout; a clean upstream reproduction is a separate pending adapter. Existing bounded MS² adapter validation does not prove full-recording benchmark execution.

The most immediate missing component for our selector is a reference-free selected-motion trajectory publisher. Existing studies lock interval choices and motions; they have not verified a full trajectory with explicit breaks, source timestamps and common-camera frame semantics. Interval oracle improvements cannot be presented as ATE improvements. Public baseline builds and scale/frame adapters are the other concrete prerequisite.

The frozen evaluation unit is the full recording. Complete chronological source stems and missing inputs must be retained, without outcome-selected excerpts. The condition prior is fixed from recording metadata: dayRGB/nightthermal/rainRGB, with sole-available fallback. Data preparation retains official rectification, exact source nanosecond times, the existing named thermal recipe and source calibration unit conversion; any model-specific resizing must adjust intrinsics explicitly. Method/config/runtime/assets/seeds0/1/2 must be fixed before final execution. No LiDAR/reference depth/IMU/GPS is fed to these visual-only rows.

The common project-defined scorer reports scale-preserving SE3-aligned and origin-aligned translation ATE, fixed approximately1second relative translation/rotation error, pose/reference/RPE coverage, segment/reset counts and longest contiguous output. It forbids timestamp searching and gap bridging. Invalid reference rotations are retained as explicit exclusions using the existing MS² serialization tolerance; gross invalid matrices are not repaired. Degenerate positional alignment is returned as unidentifiable. Engineering success requires terminal success, no reset and >=95% pose and reference-qualified RPE coverage; this is an explicitly chosen project threshold. Equal recording weighting is primary; interval pooling is secondary. Accuracy on short failed fragments cannot establish a complete-recording win. [evo metric definitions and alignment distinction](https://github.com/MichaelGrupp/evo/wiki/Metrics).

Analytic validation passes **41 checks and 22 mutations**, using no real benchmark data. Cases include rigid-frame invariance, analytic angular drift, surviving scale error, missing/reset/invalid-reference coverage, source-time gaps, degenerate alignment, failed-estimator status, and invalid partition/scale/modality/claim/readiness changes. Evidence must point to hash-checked passing artifacts; readiness strings cannot unlock execution. Final active test and validation lists remain empty.

The first frozen scorer passed its38 checks. Review found a missing estimator timestamp-type check: Python numerical equality allowed equal-valued floats and False/0. The separate v2 amendment adds strict integer validation and reruns the original suite plus regressions proving that the old scorer accepts those representations and v2 rejects them. Original source/results remain untouched. Future execution must bind `independent_benchmark_metrics_v2.py` and the amendment. No metric formula, threshold, alignment or sampled population changed, and no real reference outcomes informed the fix. [Amendment](independent-benchmark-metrics-amendment-2026-11-04.md).

Access and split boundary: seven inspected MS² recordings and all nine TD3 recordings remain development. `_2021-08-13-16-31-10` and `_2021-08-13-22-16-02` remain conditional reserve candidates with no active assignment. Their prior payload-exposure provenance must be established before a project-heldout claim; checkpoint-training overlap remains unknown and separately disclosed. These two day/night records provide no independent rain test. The specification cannot be treated as permission to bypass a missing execution freeze or consume the reserves for debugging.

The following concrete work order is now recorded:

1. Build the pinned ORB-SLAM3 stereo baseline in an isolated runtime; freeze assets/configuration and a canonical MS² input/output adapter.
2. Validate source timestamps, rectification, pose direction, stereo scale and failure output on already exposed development captures. Complete the reference-free selector trajectory publisher and composition/break tests.
3. Build the pinned DROID stereo adapter, verify calibration-derived baseline conversion and remove evaluation-time scale fitting from the common primary scoring path. Report its offline backend scope.
4. Run development-only end-to-end comparisons and full-recording readiness checks. Then create a separate execution freeze with complete evidence, source hashes and explicit test assignment before any final-reserve payload is opened.

No user clarification is needed to begin these reversible development tasks. The remaining items are implementation/provenance requirements; no new data-access request or external message was sent. STheReO’s prior access/reference issues remain a secondary audit question, not an unannounced substitute dataset.

Artifacts: `experiments/independent-benchmark-2026-11-04.json`, the metric-v2 amendment, and `results/independent-benchmark-preparation/{sources-001,run-001}/`. They include pinned public-source snapshots, configured local inventory, readiness report, both analytic audits and source hashes. [Full protocol](independent-benchmark-protocol-2026-11-04.md).
