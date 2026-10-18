# OSMO + Box staging

## Retained-feature proxy-motion consistency

After the moving-window run and reference export:

```bash
python Scripts/Setup/td3_feature_consistency.py \
  --trajectory-root ../results/moving-vo/run-001 \
  --output ../results/feature-consistency/new-run --max-depths-m 10 20 50 100
```

This reconstructs the original NED observations from saved pixels/depth and uses
the graph's explicit frame endpoints. The residual is `p1 - (R_proxy p0 + t_proxy)`;
approximate covariance is `C1 + R_proxy C0 R_proxy.T`. This omits temporal/depth/
flow cross-correlations and reference/calibration uncertainty. Normalized residuals
are diagnostics, not a chi-square calibration test. Static-scene behavior is
assumed; retained features may include dynamic or vehicle-attached objects.

Outputs include all retained observations, per-frame-pair and depth-range rank
associations, an approximate depth-partial rank association, and diagnostic scalar
fits. Covariances include configured priors, local depth variance, and floors;
they are not the raw learned flow uncertainty. Range sweeps are exploratory
sensitivity checks, not an evaluation split. The optional trajectory scalar fit
is reported both in-sample and as a first-half/second-half diagnostic within the
same segment. No VO poses, input depths, covariances, or model weights are corrected.
These results do not establish relative RGB/thermal reliability or justify fusion.

## Moving-segment proxy reference

The recorded `super_odometry/odometry/info.yaml` identifies vehicle poses in
`sensor_init`. The high-rate `interp_data.txt` and `interp_timestamps.txt` are
checked against dynamic TF. They are the same recorded SLAM source, not an
independent ground-truth measurement.

```bash
python Scripts/Setup/td3_reference.py \
  --dataset ../data/tartandrive3/20251027/1_salon_mac_cov_day \
  --manifest ../results/data-access/paired-manifest.json \
  --calibration ../data/calibration/yamaha.yaml \
  --frames 20 --output ../results/reference/reference-audit.json
```

The selector maximizes recorded body-reference endpoint displacement over a
20-frame manifest window with acquisition gaps at most 150 ms. It does not use
VO results for selection. For this clip it selects manifest start 348, source
indices 452–471. Pass `--start 348 --frames 20` to the bounded runner for both
modalities, with a fresh output directory. The runner's `--start` refers to
manifest rows, not original source indices.

After the two runs, add `--trajectory-root ../results/moving-vo/run-001` to the
reference command to export reference poses and conditional trajectory residuals.
Translation is linearly interpolated and rotation uses SLERP at each modality's
actual timestamps, without extrapolation. Camera reference uses supplied YAML
extrinsics, inverse rectification rotation, and the optical-from-NED basis.
Both trajectories are anchored at their first pose only; no fitted scale or
best-fit SE(3) alignment is applied. Thermal signed geometry and body extrinsics
remain provisional, so these are proxy-reference diagnostics, not benchmarks.

## Bounded camera-local TD3 odometry

The provisional adapter uses manifest acquisition timestamps, each rectified
camera's P intrinsics, and the absolute projection baseline. Grayscale RGB-right
is replicated into three channels; raw uint16 thermal is rejected. Exported poses
are in each rectified left camera's NED frame, with no body-frame extrinsics or
ground-truth evaluation. Thermal signed calibration remains unresolved.

From `MAC-VO-main/` inside the GPU runtime, run each modality separately into a
new output directory (replace `rgb` with `thermal` for the second run):

```bash
python Scripts/Setup/run_td3_execution.py \
  --dataset ../data/tartandrive3/20251027/1_salon_mac_cov_day \
  --manifest ../results/data-access/paired-manifest.json \
  --output ../results/bounded-vo/new-run \
  --modality rgb --frames 10 --acknowledge-provisional-geometry
```

The explicit acknowledgement applies to this execution experiment, not to
calibration validity. The runner uses MACVO_Performant settings with regular
fp32 frontend, mapping disabled, and synchronous CPU optimization. It retains
the exact effective config, input pair rows, signed and execution baselines,
first depth/covariance maps, graph observations, and camera-local trajectory
with int64 nanosecond timestamps. Existing modality output directories are not
overwritten. Partial/nonfinite trajectories and missing visual matches fail.

Both 10-frame RGB and thermal runs are documented in the project experiment
notes. Performance timings are not benchmarks: this image lacks the C compiler
needed by the optional torch.compile paths, which fall back to eager execution.

Adapter and setup regression checks:

```bash
python -m pytest Scripts/UnitTest/test_research_setup.py \
  Scripts/UnitTest/test_td3_sequence.py -q
```

## Released RGB checkpoint and TD3 preparation

The recovered FlowFormer removed seven inactive state entries in upstream
`39f63e657ccf2085977aad717f849842e18c77dd` and registered the fixed sampling
grid in `4bcccbd153ca50dc86332e6db7cb59f48c4c239b`. The released RGB
`MACVO_FrontendCov.pth` predates these changes. The covariance frontend now
accepts exactly that legacy signature, reports the migration, and strictly
checks all active state. Unknown extra state or missing learned parameters
remain errors. MACT needs no migration.

```bash
python Scripts/Setup/verify_mact_checkpoint.py \
  --checkpoint ../models/macslam/MACVO_FrontendCov.pth --allow-legacy-rgb --forward
```

Without `--allow-legacy-rgb`, the verifier still requires an exact state match.
The frontend's normal checkpoint loader applies the audited migration with a
warning when needed. Both RGB and thermal have passed real-image inference;
this does not validate VO accuracy or thermal training preprocessing.

Prepare an acquisition-time manifest (limits below are exploratory):

```bash
python Scripts/Setup/prepare_td3_sample.py \
  --dataset ../data/tartandrive3/20251027/1_salon_mac_cov_day \
  --output ../results/data-access/paired-manifest.json \
  --stereo-max-ms 2 --cross-max-ms 30
python Scripts/Setup/audit_td3_geometry.py \
  --dataset ../data/tartandrive3/20251027/1_salon_mac_cov_day \
  --calibration ../data/calibration/yamaha.yaml \
  --output ../results/data-access/geometry-audit.json
python -m pytest Scripts/UnitTest/test_research_setup.py -q
```

Pairing retains original sparse indices and exact decimal timestamps as integer
nanoseconds; it drops missing images and duplicate acquisitions and never reuses
or reverses acquisition times. It greedily picks the nearest available sample,
breaking ties toward earlier times; it does not optimize global match count.
Every accepted RGB/thermal pair is within the specified cross-modal limit.
The manifest only references source images; it does not modify them.

The geometry audit supports both archived TF schemas and reports both possible
serialized transform directions. It does not select a convention, rewrite TF,
or certify calibration. This clip's thermal TF/projection discrepancy remains
unresolved. The supplied `yamaha.yaml` is now under `data/calibration/` and all
13 values match archived transforms with reversed frame-pair labels. Do not
invert values solely because the labels are reversed. The optional
`--thermal-config ../data/calibration/upstream/get_thermal_data.yaml` compares
the published processing recipe with the saved R/P matrices; reproducing them
does not independently validate calibration. See `docs/calibration.md` at the
project root for the comparison and source snapshot provenance.
See the project experiment notes for measured results.

## Blackwell GPU environment

Build and run from the MAC-VO repository root:

```bash
docker build --progress=plain --network=host -t macvo:blackwell \
  -f Docker/Dockerfile.blackwell .
docker run --rm --gpus all \
  -v /root/AirStack/ore-ms:/workspace -w /workspace/MAC-VO-main \
  macvo:blackwell python Scripts/Setup/verify_mact_checkpoint.py --forward
```

The image uses Torch 2.7.1 / torchvision 0.22.1 with CUDA 12.8 and native
PyTorch attention. Do not install the legacy `requirements.txt` in this image:
its xFormers pin would replace this framework with a Blackwell-incompatible
version. The local Docker image must be rebuilt or published to an accessible
registry to use it in a future OSMO workflow; its tag alone does not persist.

Verified on RTX PRO 5000 Blackwell: strict MACT checkpoint loading and GPU
forward passes at 128x128 and 480x640, with finite flow and positive covariance.
The 480x640 synthetic forward took 0.404 seconds including first-use overhead;
this is a smoke test, not a dataset accuracy result or a throughput benchmark.
`python MACVO.py --help` also passes in this image. Full odometry and DPVO
remain untested.

## Verify the recovered MACT frontend

From the MAC-VO repository root, in an environment with the model dependencies:

```bash
python3 Scripts/Setup/verify_mact_checkpoint.py
python3 Scripts/Setup/verify_mact_checkpoint.py --forward --device cpu
```

Both passed for the extracted `../models/macslam/MACT_Offroad.pth`: a strict load of
428 entries and finite synthetic flow/covariance outputs. Use `--forward` for
the GPU test. The current `macvo:box` image uses Torch 2.4 and cannot execute
GPU kernels on this session's Blackwell GPU; a CUDA 12.8 / Torch 2.7+ build
is required. Device discovery alone does not validate GPU execution.

FlowFormer was recovered at `c836f66e597620a2ffaaffb60693201050b93b7a`;
DPVO at `f7266f7b0b3c212001afe260d391539376df8283`. DPVO compiled extensions
have not been built. The archive lacks the parent Git metadata needed to
verify the original submodule revision pins.

OSMO workflow storage is disposable. This setup installs the `rclone` command
in the project Docker image and configures it at workflow startup from an
OSMO-managed secret. The Box token is never committed to this repository.

## One-time Box authorization

1. Start an OSMO workflow with the project image, then run `rclone config`.
2. Create a remote named `ore_ms` with storage type `box`; leave the client ID,
   client secret, app config, and access-token prompts blank; select the
   `user` account type.
3. Because OSMO is headless, select `n` when asked to use a browser. Run the
   printed `rclone authorize ...` command on your own browser-capable computer,
   complete the CMU Box sign-in, then paste the resulting token back into the
   OSMO terminal. Do not send the token in chat.
4. Still in that OSMO workflow, encode its newly created config:

   ```bash
   base64 -w 0 ~/.config/rclone/rclone.conf
   ```

5. On your **laptop** (where `osmo login` is configured), save the single-line
   output as an OSMO `GENERIC` credential. This avoids placing it in shell
   history:

   ```bash
   IFS= read -rs BOX_RCLONE_CONFIG_B64  # paste the base64 config, then press Enter
   osmo credential set ore-ms-box --type GENERIC \
     --payload "rclone_config_b64=${BOX_RCLONE_CONFIG_B64}"
   unset BOX_RCLONE_CONFIG_B64
   ```

   Do not save the value in this repository, Box, or a shell-history file.
   CMU Box SSO may require a Box password configured under Settings → Account
   → Authentication before OAuth succeeds.

6. Add this reference to the `credentials:` section of the OSMO task that
   runs MAC-VO (for AirStack's `osmo/workflows/airstack-dev.yaml`, it sits next
   to `airlab-nucleus` and `airlab-docker-login`):

   ```yaml
   ore-ms-box:
     RCLONE_CONFIG_B64: rclone_config_b64
   ```

   The left side is the environment variable available in the task; the right
   side is the field in the OSMO credential. Use `RCLONE_REMOTE: ore_ms` in
   the task's `environment:` section if the default is overridden.

Only use this remote from OSMO. Copying the same Box refresh token to multiple
machines can invalidate it.

## Every workflow

Expose `RCLONE_CONFIG_B64` and `RCLONE_REMOTE=ore_ms` to the workflow, then run:

```bash
bash Scripts/Setup/bootstrap_osmo.sh
```

Stage an active input subset with `copy`, not `sync`:

```bash
rclone copyto ore_ms:ore-ms/macslam.zip ../data/archives/macslam.zip --progress
python -m zipfile -e ../data/archives/macslam.zip ../models
rclone copy ore_ms:ore-ms/tartandrive3/20251027 ./data/tartandrive3/20251027 \
  --progress --transfers 8 --checkers 16 --retries 10
```

Before ending the workflow, copy experiment outputs back to Box. Keep the
AirLab NAS as the canonical source of raw data and use Box only for selected
inputs, model weights, manifests, and experiment artifacts.

## Raw uncertainty and frozen-scale checks

`run_td3_execution.py --record-uncertainty` optionally saves a fixed interior
16-pixel grid before selection, selected candidates, inbound/outlier masks,
and the covariance model's values before/after its floors. Learned temporal
variance is in pixel squared units; endpoint-zero pixel variance is a fixed
prior, and local depth variance is constructed from depth patches. Recorded
arrays are copies; the default runner does not enable recording.

`td3_uncertainty_check.py prepare` writes a plan before new inference, selecting
20-frame maximum-reference-displacement windows in the first/last manifest
quarters. It freezes translation scales fitted on the first half of the prior
moving window. Run the bounded runner for both modalities at the starts in
`plan.json`, under `OUTPUT/{early,late}/`, and replay the original moving window
under `OUTPUT/original/`, all with recording enabled. Then run `evaluate` with
the same arguments. Example shared arguments:

```bash
python Scripts/Setup/td3_uncertainty_check.py prepare \
  --dataset ../data/tartandrive3/20251027/1_salon_mac_cov_day \
  --manifest ../results/data-access/paired-manifest.json \
  --calibration ../data/calibration/yamaha.yaml \
  --training ../results/moving-vo/run-001 \
  --output ../results/uncertainty/run-001
```

Replace `prepare` with `evaluate` after execution. Evaluation preserves original
trajectories, writes `analysis.json` and `scale-check.png`, compares the replayed
graphs to the original, and reports raw-variance rank associations at both
endpoint depths ≤50 m. This is a same-clip proxy consistency diagnostic, not
independent validation or uncertainty calibration. See EXP-006 for results.

## Thermal failure: reprojection and disparity diagnostics

Reuse the six EXP-006 runs without further inference or corrections:

```bash
python Scripts/Setup/td3_reprojection.py \
  --source ../results/uncertainty/run-001 \
  --dataset ../data/tartandrive3/20251027/1_salon_mac_cov_day \
  --calibration ../data/calibration/yamaha.yaml \
  --output ../results/thermal-failure/run-002
```

The output includes per-observation arrays, forward/reverse reprojection,
depth-independent Sampson epipolar distances, stereo disparity residuals,
range/region summaries and representative images. Endpoint depths ≤50 m define
the primary population; complete inbound arrays remain saved. Grid disparity
is derived from depth and execution baseline; retained disparity is checked
against the saved graph. `analysis.json` records source and input hashes.
Existing output directories are refused.

The configured `disp` backend weights image and disparity residuals with pixel
and disparity covariance. Projected 3D covariance is not its direct pose weight.
The exported inverse-variance residual energies omit Huber robustification and
are not optimizer losses. See EXP-007 for methods, measured limitations and the
next controlled ablation.

## Fixed-match backend ablation

Replay both analytic objectives against identical saved graph inputs:

```bash
python Scripts/Setup/td3_backend_ablation.py \
  --source ../results/uncertainty/run-001 \
  --dataset ../data/tartandrive3/20251027/1_salon_mac_cov_day \
  --calibration ../data/calibration/yamaha.yaml \
  --output ../results/backend-ablation/run-002 --threads 1
```

Each pair uses saved world points, observations/covariances and the saved prior
pose as a common static initializer. Both arms retain native LM, Huber,
weighting and stopping settings; only `graph_type` changes. Independent pair
increments are composed for diagnosis. This is not a new end-to-end VO run.
No model inference, feature selection or scale fitting occurs.

`analysis.json` contains input fingerprints, loss histories, local errors,
composed-trajectory metrics and source/input hashes. NPZ files store motion
increments and composed poses. Disparity replay must reproduce original local
motion within 1 mm / 0.01 degrees; otherwise the script raises after saving
results. Conditioning is computed at fitted poses using inverse-variance
weighted local projection Jacobians (meters/radians), before Huber; it is not
the optimizer Hessian. See EXP-008 for results and limits.

## Fixed covariance weight ablation

Create the plan before evaluating any weight arm:

```bash
python Scripts/Setup/td3_weight_ablation.py prepare \
  --source ../results/uncertainty/run-001 \
  --dataset ../data/tartandrive3/20251027/1_salon_mac_cov_day \
  --calibration ../data/calibration/yamaha.yaml \
  --output ../results/weight-ablation/run-001 --threads 1
```

Then run the same command with `evaluate` instead of `prepare`. The plan freezes
per-modality medians of active u/v/disparity variance from original frames0–9,
in pixel squared units. Arms are `learned`, `fixed_uv`, `fixed_disp`, and
`fixed_both`; all use the disparity backend and identical nonweight inputs.
No reference data enters constant selection. Source hashes and execution
settings must match the plan before evaluation.

Outputs include `plan.json`, `analysis.json`, `weight-comparison.png`, and
motion/trajectory NPZ files per arm/window/modality. Learned replay controls
must reproduce original local motions within 1 mm / 0.01 degrees. Geometry
fingerprints match between arms and source inputs remain unchanged. EXP-009
reports both translation and rotation effects; fixed covariance values are
experimental settings, not calibrated uncertainty or an adopted replacement.
