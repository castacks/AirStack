# EXP134: ORB stereo/MS² development adapter

2026-11-04, America/New_York. Parent EXP133 / `d5d5a7e28`.
Initial freeze `eda9ac5c8`; final compatibility freeze `3d29197e9`.

The pinned ORB-SLAM3 stereo adapter completes six exposed-development runs:
RGB and thermal on the previously inspected day, night and rain openings. Every
run processes all 20 original source frames and returns 20 tracking-OK poses in
one segment. All 27 adapter tests and 2,628 independent input/output checks pass.
This clears a bounded native-build/input/output smoke prerequisite. It does not
complete a full-recording benchmark or establish pose accuracy.

## Frozen inputs and output contract

The unchanged first 20 archived stems, `000000`–`000019`, come from:

| Condition | Previously exposed MS² recording |
|---|---|
| Day | `_2021-08-06-11-23-45` |
| Night | `_2021-08-13-21-18-04` |
| Rain | `_2021-08-06-16-19-00` |

Input and calibration recovery uses the hash-verified EXP041/042/043/048/054
packets. Only selected sync-image/time/calibration members and original source
manifests are extracted. No new depth or odometry member is extracted or scored.
No reserved recording is downloaded, opened or assigned to a test partition.

RGB PNG bytes are copied unchanged. Thermal is processed by the existing
`thermal_producer_replay` function extracted from the unchanged MS² loader,
using `td3-firestereo-v2-clahe-bilateral-v1`. An independent order-statistic/OpenCV
implementation reproduces every prepared thermal image exactly in this runtime.
The original uint16 PNGs remain unchanged. OpenCV 4.6.0/NumPy 1.26.4 are recorded;
this is not a claim of bitwise equality to every historical OpenCV build.
No resizing, re-rectification, temporal re-pairing or source-row exclusion occurs.

Rectified pinhole settings use provided `K`, and `baseline_m=-T_right_x/1000`.
The native settings parser confirms both `b` and `bf=fx*b` within the predeclared
float32 tolerance. Native tracking receives left time relative to the first source
time; integer subtraction precedes floating conversion. Original left and right
nanosecond timestamps remain exact integers in input/output journals. This
preserves provided timestamps and pairing, without asserting exposure simultaneity.

The runner has no viewer or IMU and explicitly disables loop closing. Local
mapping remains native. Each returned pose is the received `TrackStereo` result,
not a post-shutdown reoptimized trajectory. Tracking failures remain null at their
original source slots; missing tails, map changes and source gaps end segments.
Native `T_camera_from_world` is inverted to camera-local `T_world_from_camera`
in metres. The thermal-to-common-RGB frame is explicitly **unverified**; no fitted
extrinsic, time offset, scale, reference bridge or missing-pose interpolation is
introduced. This output cannot yet be substituted for the common-frame benchmark
trajectory contract.

## Build and retained failures

Official ORB source is pinned to
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`; Pangolin v0.6 is pinned to
`dd801d244db3a8e27b7fe8020cd751404aa818fd`. Ubuntu 24.04 uses the digest in
`environment/Dockerfile.orb-ms2`. Package versions, vocabulary, shared libraries,
runner and runtime image hashes are retained.

The final native source has one explicit patch: skip the diagnostic print of
uninitialized `originalCalib2_` for rectified stereo. It changes the settings
printer only. Pangolin requires an explicit `cstdint` compiler include on this
toolchain. ORB, standalone g2o and the wrapper use C++11 and `-march=native`.
The wrapper rejects missing/mismatched flags before inference.

| Attempt | Outcome retained without overwriting |
|---|---|
| `run-001` | All six processes SIGSEGV before frame 0 in the rectified settings printer. All source output slots retained as null. |
| `run-002` | Logging guard permits startup; all six processes abort after frame 0 with heap corruption. |
| `run-003` | Matching wrapper `-march=native` alone does not repair the heap failure; all six abort after frame 0 again. |
| `run-004` | Pinning standalone g2o and wrapper to C++11 completes all six runs. |

The core backtrace points to `g2o::HyperGraph::clear` during pose optimization.
Standalone g2o had inherited GCC 13's C++17 default, while ORB compiled C++11.
Compiler probes record Eigen's C++17 over-aligned allocation flag 1 versus 0, and
default versus native SIMD alignment 16 versus 64 bytes. Matching SIMD flags was
necessary for consistent wrapper types but insufficient to fix graph deletion;
the subsequent C++11 rebuild fixes the observed failure. Each change has a separate
pre-execution amendment, image digest and output namespace. No images, calibration,
ORB settings, seed, request budget or scientific threshold was tuned on outcomes.
Build failures and all three failed native attempts remain in the recovery packet.

## Completed bounded smoke

All results below are `run-004`, seed 0, first 20 archived source frames. Native
processing is the sum of `TrackStereo` call durations. Process wall time includes
startup and shutdown; neither is an acquisition-deadline or real-time guarantee.

| Condition / modality | Returned poses / source | Segments | Terminal success | Native processing (s) | Process wall (s) |
|---|---:|---:|---|---:|---:|
| Day RGB | 20/20 | 1 | yes | 0.398 | 3.973 |
| Day thermal | 20/20 | 1 | yes | 0.326 | 3.621 |
| Night RGB | 20/20 | 1 | yes | 0.366 | 3.872 |
| Night thermal | 20/20 | 1 | yes | 0.366 | 3.772 |
| Rain RGB | 20/20 | 1 | yes | 0.332 | 3.822 |
| Rain thermal | 20/20 | 1 | yes | 0.345 | 3.621 |

The independent auditor verifies all frozen sources and source/calibration pins,
RGB encoded bytes, thermal decoded pixels, both clocks, all source slots, native
tracking states, matrix inversion, segment rules and native calibration. All
2,628 checks pass. The 27 analytic tests cover strict float/bool timestamp rejection,
metric displacement and inversion, crashes/missing tails, segment breaks,
reserve/path/hash/calibration rejection and compiler-ABI mismatch rejection.

## Reproduction and remaining gates

Final execution manifest:
`experiments/orb-ms2-development-2026-11-04-v4.json`.
It pins source hashes, every original input manifest/calibration, seed, modality,
output namespace and exact runtime image:
`sha256:1542a9a2fe4f645aa7b3db1d4d002c0a0b5941a91db105c5587db62b68bd8445`.

From `ore-ms`, the per-run commands inside that image are:

```bash
python3 tools/orb_ms2_development.py prepare --root /workspace \
  --manifest /workspace/results/ms2-timestamp-semantics/source-protocol-input-manifest.json \
  --modality rgb --seed 0 --output /workspace/results/orb-ms2-development/NEW/night-rgb
python3 tools/orb_ms2_development.py run \
  --directory /workspace/results/orb-ms2-development/NEW/night-rgb
```

Mount `ore-ms` at `/workspace`; use a fresh destination. Original commands,
stdout, per-frame journals and execution locks are retained. The audit CLI accepts
`--root`, `--protocol` and a fresh `--output`; original result files are never
overwritten. The saved image is the exact execution runtime; rebuilding against
moving Ubuntu package mirrors is not promised to reproduce its bytes.

EXP133's original structural/access contract remains valid and execution-not-ready,
with active test/validation empty. Its historical model-asset verification was not
rerun here. Next: reference-free selected-motion publisher, source-defined common
camera/time proofs, DROID metric-stereo adapter and complete-recording development
validation. Those precede any separate final execution assignment. No accuracy,
generalization, heldout-network, SOTA, training or production-adoption result is
claimed by this engineering smoke.

Artifacts: `results/orb-ms2-development/run-{001,002,003,004}`.
Recovery: `backups/2026-11-04-orb-ms2-development-exp134`.
