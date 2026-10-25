# EXP-044: MS² pretrained frontend execution smoke test

Date: 2026-10-24 America/New_York. Branch `ore_ms`; starting commit `ef6ca6f57`.
**RGB passes all39 native-resolution calls. Thermal execution is pending access to the original checkpoint.** Runtime compatibility only; no physical accuracy, uncertainty calibration or stereo simultaneity claim.

## Scope and recovery

Exactly EXP041–043's first20 pilot IDs000000–000019 from MS² night `_2021-08-13-21-18-04`. Per modality:20 same-ID stereo and19 consecutive left temporal pairs. No resizing, clock shift, re-pairing, reference scoring, VO, fitting or fusion. Thermal requires explicit `td3-firestereo-v2-clahe-bilateral-v1`; training-recipe equivalence remains unverified.

EXP043's self-contained archive SHA256 `5658f69f6b2b186453003728a5a56e78b65cc5b523797c2ec204100f0e657fd9` verifies, as do all277 member hashes. It contains the full bounded pilot; no separate EXP041/042 downloads or full dataset archives needed. All prior files remain unchanged.

Original checkpoint folder303805616544 and file2037543542268 return404 with the restored Box account. The project Box folder has no model archive. User was asked for the shared Box URL or `macslam.zip` in the project folder. Thermal inference has not run. Original archive must match SHA256 `77ad168672019f94d8e48ebb087cb109b938bcf8784e3df480070cd5f437f00c`.

RGB recovered from the [official MAC-VO release](https://github.com/MAC-VO/MAC-VO/releases/download/model/MACVO_FrontendCov.pth), SHA256 `bec6edd7e195bab863132f1e9659cdd26e6eaeae7cfd24a626828de294cf5b3a`. Existing audited migration loads434 entries: seven unused upstream keys removed, deterministic sampling-grid buffer initialized, all active state loaded strictly. No learned state synthesized.

## Runtime and input validation

Rebuilt committed Blackwell Dockerfile; new `Docker/Dockerfile.ms2-smoke` pins audited input libraries. Image `sha256:71c3a376d4ed502ea705877a694abea1ba643b7826012b602219a6b2a1c23140`. Python3.11.13; Torch2.7.1+cu128; torchvision0.22.1+cu128; PyPose0.7.3; NumPy2.2.6; OpenCV4.10.0.84; RTX PRO5000 Blackwell. This differs from EXP043's CPU Python3.12/Torch2.9.1 runtime; all14 adapter tests pass here. One upstream PyPose docstring warning remains. Startup driver-init diagnostics are retained; CUDA matrix multiplication and all reported network calls execute successfully.

Independent original-producer AST replay matches all80 RGB/thermal model-input tensors exactly; four saved EXP043 first-input tensors also match exactly. Source IDs, separate left/right integer timestamps, raw image hashes and unset ground-truth fields verify. No original outputs are overwritten.

FP32 evaluation/inference mode; seed0; one CPU thread; no autocast. Matmul TF32 false; cuDNN TF32 true; cuDNN benchmarking false. Runtime fingerprint and effective configuration saved.

## Measured RGB result

| Check | Result |
|---|---|
| Stereo / temporal calls |20/20 and19/19 pass|
| Flow and covariance shape/dtype |1×2×384×1224 / float32|
| Finite flow and finite positive covariance |39/39 pass|
| Stereo covariance range, pixel² |0.0182612–16091.8389|
| Temporal covariance range, pixel² |0.00100908–113.9892|
| Median stereo / temporal forward time |0.14763s /0.14758s|
| Maximum allocated CUDA memory |4.665GiB|
| First stereo / temporal repeated inference |Both flow/covariance bitwise identical|
| Manifest+80 source images unchanged |81/81 hashes pass|
| Independent saved-array audit |39/39 files pass|

Times include synchronization around the model call and exclude image loading, GPU transfer and compressed writing. Descriptive measurements, not a throughput benchmark. Retain large covariance values; positivity does not establish calibration or accuracy.

Save complete flow/covariance fields, source IDs, inference input times and all four associated camera times in compressed NPZ files. Independent validation re-reads every array, checks shape/dtype/finiteness/positivity, file/checkpoint hashes, source IDs/times, and complete expected pair coverage. It independently regenerates all input tensors using original producer bodies and checks recorded inference-input hashes.

## Reproduction

From `ore-ms/MAC-VO-main`:

```bash
docker build --network=host -t macvo:blackwell -f Docker/Dockerfile.blackwell .
docker build --network=host -t macvo:ms2-smoke -f Docker/Dockerfile.ms2-smoke .
```

Inside `macvo:ms2-smoke`, mount AirStack at `/root/AirStack`, work in `/root/AirStack/ore-ms/MAC-VO-main`, and run:

```bash
python Scripts/Setup/ms2_frontend_smoke.py \
  --root /root/AirStack/ore-ms \
  --manifest /root/AirStack/ore-ms/results/ms2-timestamp-semantics/source-protocol-input-manifest.json \
  --rgb-checkpoint /root/AirStack/ore-ms/models/macslam/MACVO_FrontendCov.pth \
  --thermal-checkpoint /root/AirStack/ore-ms/models/macslam/MACT_Offroad.pth \
  --modalities rgb \
  --output /root/AirStack/ore-ms/results/ms2-frontend-smoke/rgb-run-001

python Scripts/Setup/validate_ms2_frontend_smoke.py \
  --root /root/AirStack/ore-ms \
  --manifest /root/AirStack/ore-ms/results/ms2-timestamp-semantics/source-protocol-input-manifest.json \
  --run /root/AirStack/ore-ms/results/ms2-frontend-smoke/rgb-run-001 \
  --output /root/AirStack/ore-ms/results/ms2-frontend-smoke/rgb-run-001/independent-validation.json
```

Fresh paths required: existing outputs are rejected. After restoring original thermal weights, run `--modalities thr` into a fresh directory and validate it. Omitting `--modalities` runs both; validator checks coverage for the explicitly requested modalities. No existing runner, dataset registration, calibration or defaults change.

## Remaining work and preservation

Restore/hash-verify original thermal checkpoint; execute and independently audit its39 fixed calls at native256×640. Timing/reference/training-provenance limits remain regardless of runtime success. Recovery target `backups/2026-10-24-exp-044` preserves bounded pilot/manifest, preprocessing producer, RGB outputs, logs and code. Model weights/full dataset archives excluded; recover from official release/original shared checkpoint source.
