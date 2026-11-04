# EXP135: selected-motion trajectory publication and source frame/time checks

2026-11-04, America/New_York. Parent EXP134 / `7071ee4d2`.
Publisher/replay freeze `c7a6714d1` precedes all new publication outputs.

The reference-free exact-interval publisher is implemented. Six saved exposed
day/night/rain RGB/thermal trajectories reconstruct 20/20 poses on their own
source clocks. Fixed missing/late/reset controls retain two holes and four
segments per recording. All 57 thermal motions requested on RGB intervals are
correctly refused. The provided numeric thermal-to-RGB frame convention is
source-backed; cross-stream acquisition-clock equivalence remains unverified.

Validation passes 23 new publisher tests, 27 unchanged ORB adapter tests,
2,505 independent checks and 24 semantic corruption rejections. Two additional
analytic publisher/common-scorer integration tests pass. No new native inference,
training, real-data reference scoring, reserve payload access or default change occurs.

## Publisher contract

`tools/selected_motion_trajectory.py` accepts an explicit source timestamp/ID axis,
already selected camera actions, exact native intervals, received-output times,
original decision deadlines, sensor/controller clock IDs, native segment IDs and
static target-from-camera frame contracts. It has no camera selector, preference
fallback, reference reader, nearest-timestamp search or temporal transport model.
Evidence identifiers are caller/source declarations and never promoted to physical
verification. A future payload is not inspected before its receipt deadline.

For a declared static camera transform `E = T_target_from_camera`, it forms
`D_target = E D_camera inverse(E)` and right-composes
`P_next = P_current D_target`. This includes the extrinsic translation lever arm;
rotating only the delta translation is insufficient. Relative native motion cancels
each expert's arbitrary world origin, so switching native gauges does not introduce
an absolute-pose splice. Analytic camera-switch tests use noncommuting rotations
and translations. They do not claim a real synchronized cross-modal switching run.

Every expected source frame keeps one pose or null and a segment ID or null.
Initialization must be explicit; an empty/uninitialized stream receives no automatic
identity pose. Missing, late or rejected motion ends the segment. A subsequent valid
motion starts a new gauge at that motion's original source start time, leaving the
past missing slot null. The gauge root is recorded as unpublished in that new
segment. Reset and source-gap boundaries are explicit. No prior output is repaired,
carried across a hole or interpolated. Correctly ordered SO3 serialization roundoff
uses the existing EXP133 `1e-5` rigidity screen and records its bounded correction;
gross nonrigid matrices are rejected.

Exact-native mode requires both native endpoints to equal the requested interval
and the sensor-clock identity to match. A stationary-twist projection, though present
elsewhere in this project, is not silently inserted here. Any future projected-motion
adapter needs its own explicit model/provenance contract and development validation.

## Source frame qualification

The [pinned official MS² loader](https://github.com/UkcheolShin/MS2-MultiSpectralStereoDataset/blob/0bb0fc1a544d8ad58c7fda5bc025cb0cbb400284/dataloader/MS2_dataset.py)
defines the transform as:

```text
T_RGB_from_thermal = T_RGB_from_NIR × inverse(T_thermal_from_NIR)
translation_metres = translation_millimetres × 0.001
```

The exact `set_extrinsics` method is re-executed with a numeric-array/tensor shim
and hash-pinned calibration JSON; no object pickle, image, navigation or pose file
is loaded by it. Independent block rotation/translation algebra agrees to `1e-14`.
The left rectification rotations are explicitly checked as identity for these
three calibrations; a different rig/rectification requires additional proof.
Provided translation of RGB-from-thermal is approximately
`[-0.008279, -0.423435, -0.502393]` metres in all three captures.
This establishes the supplied source convention, not surveyed physical accuracy.

Transforming thermal motion into the provided RGB-oriented coordinate frame at
thermal timestamps does not make those timestamps RGB acquisition times.

| Exposed opening | Equal RGB/thermal left timestamps /20 | Equal adjacent intervals /19 | Thermal-minus-RGB recorded offset range (ms) |
|---|---:|---:|---:|
| Night `_2021-08-13-21-18-04` | 0 | 0 | −11.756 to +60.564 |
| Day `_2021-08-06-11-23-45` | 0 | 0 | −36.503 to +33.335 |
| Rain `_2021-08-06-16-19-00` | 0 | 0 | −10.908 to +61.537 |

No offset, drift, common physical clock or exposure correction is fitted. Numeric
timestamp equality alone would also be insufficient to prove clock equivalence.
The publisher's analytic tests reject different clock IDs even at equal endpoints.

## Locked replay results

Inputs are only saved EXP134 predicted poses/receipts, original source timestamp
manifests and provided calibration from the same three development openings.
No new source recording or reference outcome is consumed. Each modality retains
its original separate process receipt clock; no common controller time is invented.

| Replay mode, repeated for all three openings | Poses /20 | Accepted motions /19 | Segments |
|---|---:|---:|---:|
| RGB on its own clock | 20 | 19 | 1 |
| Thermal in provided RGB-oriented frame, on thermal clock | 20 | 19 | 1 |
| Thermal requested on RGB clock/intervals | 0 | 0 | 0 |
| RGB with fixed missing/late/reset controls | 18 | 17 | 4 |

Break controls are frozen before replay: edge 6 abstains, edge 10 arrives one
controller nanosecond late, and edge 14 declares a reset. These are synthetic
publisher controls, not newly observed estimator failures or an adaptive policy.
Both missing target poses remain null. The strict thermal/RGB control supplies
no invented initialization, so all 20 expected output slots remain null.

The independent auditor reconstructs each segment directly from saved absolute
native poses under its declared gauge, rather than calling publisher helpers or
recursively reproducing its implementation. It checks source IDs/times, all nulls,
lineage, original deadlines, frame conjugation and reset roots, with the pre-frozen
`2e-5` matrix tolerance. All 2,505 checks pass. It rejects 24 mutations including
float timestamps, inverse-direction poses, filled holes, resumed old segments and
silent thermal fallback. All original source/input hashes remain unchanged.

The 50 primary tests run in EXP134's exact runtime
`sha256:1542a9a2fe4f645aa7b3db1d4d002c0a0b5941a91db105c5587db62b68bd8445`.
A separate pre-test hash declaration binds two analytic common-scorer compatibility
tests, using the existing local AirStack runtime with SciPy. The untouched v2 scorer
accepts exact source clocks/pose direction on an analytic curved trajectory and
retains the missing-frame/reset penalty after a hole. Those are mathematical
fixtures, not real-data accuracy scores. The original 41-case historical suite and
historical model-asset verification are not claimed rerun here.

## Readiness and next work

The publisher component, serialization/segment tests and provided numeric frame
convention are now available. Common-time cross-modal motion remains unqualified:
the strict publisher cannot join these unequal intervals without an explicitly
validated reference-free temporal adapter and source clock evidence. This is an
identified contract boundary, not permission to relabel or fit the source times.

EXP133 remains valid and execution-not-ready, with active validation/test lists
empty and final reserves closed. Full-recording development and the DROID metric
stereo adapter remain independent engineering work. No selector ATE, physical
accuracy, adaptive superiority, live deadline or benchmark benefit is claimed.

Artifacts: `results/selected-motion-publisher/{source-001,run-001}`.
Primary manifest: `experiments/selected-motion-publisher-2026-11-04.json`.
Recovery: `backups/2026-11-04-selected-motion-publisher-exp135`.
