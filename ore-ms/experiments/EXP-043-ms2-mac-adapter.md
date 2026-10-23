# EXP-043: MS² source-protocol MAC input adapter

Date:2026-10-23 America/New_York;branch `ore_ms`;starting commit `e926ff417`. **Implemented and exercised a real MAC sequence adapter on all20 fixed MS² pilot IDs:80 images,40 camera frames and120 reference depth maps.14 tests and1,087 independent controls pass. Thermal preprocessing is an explicit named replay of the previously audited converter;its exact relationship to the released checkpoint's training recipe remains unverified. No model inference or timing correction.**

## Adapter behavior

New `DataLoader/Dataset/MS2.py` provides `MS2ManifestSequence` for EXP042's source-protocol input manifest. The dataset root is `ore-ms/`;the manifest retains original paths,hashes,source IDs,cameraK/baselines,all four recorded camera times,and reference provenance. Read PNG bytes using `IMREAD_UNCHANGED`,verify the hash of the decoded payload,and reject declared/actual dtype or shape discrepancies. Reject paths leaving the root,incorrect source-image stems,non-increasing/non-integer recorded times,duplicate IDs,invalid or variable geometry,and unknown thermal transforms.

`raw_pair()` returns native source arrays. Thermal remains uint16;there is no implicit8-bit cast or intensity division. RGB model frames explicitly convert source uint8 BGR to RGB float32 in[0,1]. Every model image has shape1×3×H×W. Source cameraK is float32 as required by the existing MAC interface;baseline is preserved in float64 meters. No image resize,crop,rectification or camera calibration change is applied by this adapter.

`MS2StereoData` extends the actual `StereoData` interface with `right_time_ns`,`source_stem`,`input_transform` and `recorded_time_semantics`. Existing `time_ns` and outer frame time contain the exact left source integerns;right recorded time remains separate. `source_record()` exposes an independent copy of the complete row,including the other modality's source times. Sequence clipping preserves the original source-ID/time association while local frame indices follow `SequenceBase`. Real collated batches and the existing resize transform retain the added metadata.

Identity `T_BS` denotes camera-local execution;it is not a claimed physical vehicle extrinsic. Do not populate `gt_pose` from references whose direction is unverified. Do not choose a reference depth automatically: `depth(index,variant)` requires `depth`,`depth_multi` or `depth_filtered` and returns float32 meters plus a separate nonzero validity mask. Source label variants and their provenance remain in the manifest. Frame `gt_depth` stays unset.

## Explicit thermal model-input option

The [MACThermal thesis](https://publications.ri.cmu.edu/storage/publications/2025/12/YifeiLiu_CMU_MSR_Thesis_ThermalOffroadAutonomy.pdf),Sec.2.1 and3.1/3.3,describes percentile binding,CLAHE,bilateral filtering and preprocessed thermal network inputs. Its augmentation is a training procedure,not an inference default. The recovered archived TD3 producer had already reproduced provided processed images exactly in EXP017;use its fixed operation order as a named provisional input recipe rather than inventing a display normalization.

Thermal model frames require the caller to explicitly name:

```
td3-firestereo-v2-clahe-bilateral-v1
```

That recipe retains the archived1%/99% order-statistic indices,clips a copy of the raw array,truncates the scaled result to uint8,applies CLAHE clip2/tile8×8,then bilateral diameter5/sigmaColor20/sigmaSpace15. Replicate the output across three channels and divide by255 into float32. Reject degenerate percentile ranges. Do not apply another geometric rectification to already rectified MS² pixels. Omitting the thermal recipe permits raw access but deliberately produces no model frame.

Producer SHA256 `9e50052737bcddc97db6d5150b565352f05186ba6e1fa556b3dea1eaba58bd5c`;source-function AST SHA256 `1dc2bebc6c064aa7fd456ef795371098cb80ff49c3dad3983e5e1e7f42456ee0`. The archived source/config hashes and complete prior recovery archive hash verify before use. The official thesis was re-downloaded and matches its earlier17,420,576-byte SHA256 `91d680477b21d2292e81877df8ed05dfef0dbe829784857baa1581add645d6a4`. Preserve its source metadata,not another full copyrighted PDF/text copy.

This verifies a specific preprocessing implementation and a compatible float tensor range;it does not verify the released checkpoint's training distribution,exact recipe or MS² training membership. The first thermal-left source retains619 native values;the explicit processed input has254 levels,versus5 from the unintended generic COLOR read. That contrast is a precision/input-contract finding,not model quality evidence.

## Replay usage

Run in the normal project Python environment from `MAC-VO-main`:

```python
from DataLoader.Dataset.MS2 import MS2ManifestSequence, THERMAL_RECIPE

rgb = MS2ManifestSequence(root, manifest, "rgb")
thermal = MS2ManifestSequence(root, manifest, "thr",
                              thermal_transform=THERMAL_RECIPE)
raw_left, raw_right = thermal.raw_pair(0)
frame = thermal[0]
right_recorded_ns = frame.stereo.right_time_ns[0]
depth_m, valid = thermal.depth(0, "depth")
```

Direct explicit import leaves existing dataset configurations/runners unchanged. No global registration or default selection is altered. This is a source-protocol adapter,not a new VO method or an approval of physical exposure simultaneity. EXP041/042's failed inherited timing diagnostic remains unchanged;no re-pairing,clock fit or fitted offset is introduced.

## Validation

Real CPU runtime:Python3.12.3,PyTorch2.9.1+cpu,torchvision0.24.1+cpu,PyPose0.7.3,NumPy2.2.6,OpenCV4.10.0. Single-thread/seed0 pilot inspection. Isolated CPU packages avoid modifying the system/project environment;no checkpoint or GPU is needed. An initial check found the available AirStack Docker image lacks Torch;CPU validation subsequently used genuine installed libraries rather than mocks.

14 focused tests pass under the project's pytest configuration:exact RGB color/time handling,raw thermal immutability and explicit transform selection,clipping/batching/source metadata,explicit masked metric depth,rejection of corrupt paths/hashes/times/dtypes/shapes/baselines/IDs,decoded-schema validation,unknown transforms/degenerate data,and metadata preservation through existing resizing. The first pytest launch failed because the configured jaxtyping plugin was absent;install it and preserve the failure log. Upstream/plugin instrumentation warnings remain in the passing log;the explicit test assertions execute,and no claim of complete runtime type instrumentation is made.

Independent full-pilot validation executes the original producer's two function AST bodies with the pinned OpenCV/NumPy runtime and compares complete arrays against adapter model tensors. All40 thermal images match exactly;all40 RGB images match independent decode/channel/float conversion. Check each camera timestamp,K/baseline,raw array after model conversion,and each of120 decoded depth maps/validity masks. All1,087 controls pass.205 frozen input hashes remain unchanged,including all200 source image/depth files and prior timing/schema outputs. Store first RGB/thermal model tensors with original left/right times in NPZ artifacts for review.

Tests: `Scripts/UnitTest/test_ms2_adapter.py`. Full pilot/source validation,source recovery/installation logs,runtime fingerprint,raw inputs/manifest references,model-input NPZ and frozen registries live under `results/ms2-mac-adapter/`. No source PNG,reference file,earlier model output,weight,calibration or timing gate changes.

## Outcome and next step

The input adapter is computationally validated and ready for a bounded frontend execution check using the original pretrained checkpoints. Such a check would establish runtime compatibility only. Strong physical cross-modal error claims still require the unresolved timing/reference/training-provenance assumptions to be addressed explicitly.

Recovery target:Box `backups/2026-10-23-exp-043`,with adapter/tests/report/notebook,archived producer functions,all bounded replay inputs and an incrementalGit bundle after EXP042. Packages,weights,full dataset archives and the full thesis PDF are excluded. Recreate the recorded CPU environment to replay tests/array validation;no inference or GPU is part of this result.
