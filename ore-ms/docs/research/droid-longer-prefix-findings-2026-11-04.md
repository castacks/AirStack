# EXP137: longer exposed prefixes resolve DROID's bounded startup limit

2026-11-04, America/New_York. Parent EXP136 / `a81d8f306`.
Source intervals are selected before recovery payload inspection; native freeze
`50c9819f6` precedes all new model runs.

All six RGB/thermal day/night/rain runs initialize on the fixed original
`000000–000100` prefixes and return 101 finite admitted offline poses each.
The five cases that did not initialize in EXP136's 20-frame openings first
initialize at source indices 21–60. All first-20 actual input tensors, timestamps,
intrinsics and categorical frontend traces match EXP136 exactly. No motion or
warmup threshold is changed. The 82 tests and 13,874 independent checks/30 semantic
mutation rejections pass; all 2,467 frozen source/input hashes remain unchanged.

This establishes a short-prefix initialization limit for these exposed runs,
under the fixed baseline settings. It does not establish cross-modal selection
benefit, pose accuracy, generalization, or a completed whole-recording benchmark.

## Source selection and preservation

Choose all three previously exposed MS² recordings before restoring camera
payloads: day `_2021-08-06-11-23-45`, night `_2021-08-13-21-18-04`, rain
`_2021-08-06-16-19-00`. Use exactly original stems `000000–000100`, with no
motion/error/support-based sampling, substituted interval or source renumbering.

EXP114's existing Box packet contains its first 100 early stems `000001–000100`.
Source 0 is already restored from EXP134. The recovery archive's size/SHA256 and
all 12,916 member hashes are checked, without parsing scientific reference values.
Only 1,200 required camera PNGs are extracted. No reference-depth, odometry or
reserved-recording member is extracted. Original source timestamps come from
the frozen EXP114 metadata plus the existing source-0 manifest.

The original EXP134 OpenCV 4.6 producer runtime prepares the canonical RGB/thermal
PNGs. All 240 PNG files in the six 20-frame overlaps match the original canonical
bytes exactly. New source manifests contain visual inputs and metadata only, with
no reference paths. The prior named thermal producer remains unchanged.

## Only allocation and prefix length change

The native model, released checkpoint, metric baseline conversion, image-area cap,
crop/intrinsic rule, seed, native time units and every motion/initialization setting
remain exactly EXP136's amended configuration. Warmup stays eight keyframes and
the motion filter stays 2.4. No learned parameter or pose-reference value is used
to tune the extension.

The buffer allocation increases from 64 to 128 slots, enough for the maximum
101 input states, a 16-frame native filler batch and next state. A process-local
wrapper permits only this source-count-derived allocation change, refuses other
setting changes and restores the original module setting even on failure.
This is a bounded-capacity change, not keyframe selection or a motion-policy change.

The exact EXP136 runtime is reused:
`sha256:d3cf8a41dd8090c8939cd840b4735d526cec9a54e48ac36d506f91d13ee56e1c`.
There is no new native build or compatibility patch. The prior CPU/CUDA geometry
controls remain a hash-bound prerequisite, not a newly claimed rerun.

## Initialization and complete output populations

Elapsed times below are differences of the original recorded timestamps on each
camera's own source clock, not measured acquisition-to-controller latency.
No common RGB/thermal physical clock is asserted.

| Case | First initialized source index | Recorded source elapsed (s) | Final keyframes | Returned source calls | Admitted offline poses |
|---|---:|---:|---:|---:|---:|
| Day RGB | 21 | 2.133 | 24 | 101/101 | 101/101 |
| Day thermal | 18 | 1.803 | 27 | 101/101 | 101/101 |
| Night RGB | 56 | 5.666 | 13 | 101/101 | 101/101 |
| Night thermal | 60 | 6.079 | 13 | 101/101 | 101/101 |
| Rain RGB | 58 | 5.866 | 11 | 101/101 | 101/101 |
| Rain thermal | 29 | 2.934 | 18 | 101/101 | 101/101 |

All six finish successfully on their first declared attempt. Previously admitted
short-prefix output was 20/120 poses, from day thermal only. The long runs provide
606/606 finite offline poses over their larger, repeated development population;
these counts are not a matched-frame error comparison or an adaptive benefit.

The source/input preservation controls are stronger than matching only filenames.
For all 120 overlapping tracking calls, actual tensor bytes, intrinsics, native
float times, original integer stereo timestamps and source IDs match EXP136.
Keyframe counts and initialization flags have zero categorical prefix divergences.
The extra observations permit the previously incomplete startup; a new image
recipe, weight, threshold, timestamp relabel or chosen easier window does not
explain the difference in those first-20 traces.

Native DROID output still uses offline global backend optimization and image-refined
trajectory filling with an interpolated initialization. Future images can change
the returned poses at earlier source times. Earlier uninitialized online states
are not retroactively called on-time successes; EXP136's nulls and failures remain
unchanged. DROID does not expose a per-frame tracking-failure status in this adapter,
and finite output does not establish physical accuracy or continuous correct tracking.

## Independent validation

The 82 tests comprise ten new allocation/restore/forbidden-setting cases and the
72 existing DROID/ORB/publisher/scorer cases. The source preparation and all six
native runs succeed on the first declared attempt; no retry or outcome-based
interval/threshold adjustment occurs.

The auditor uses the previously frozen independent quaternion/matrix/scale
checker, original calibration and source metadata, and its own image-resize/tensor
reconstruction. It verifies all 101 source slots, input hashes, native clocks,
metric translations, finite/quaternion/segment gates, actual tracking/filler inputs,
original first-20 inputs and frontend-state traces. All 13,874 checks pass and
30 corruptions are rejected, including wrong scale, floating source timestamps,
invented online latency, hidden scale fitting and altered metric poses.

No actual reference pose/depth value is scored, no network is trained, and no
reserved recording is opened or assigned. The original benchmark structural/access
contract remains valid and execution-not-ready, with active validation/test empty.
Historical model-asset verification is not newly claimed.

## What is ready next

The bounded DROID baseline now initializes under unchanged settings on all three
exposed conditions. The next independent issue is explicit common-target-time
qualification before comparing mixed-camera trajectories. Supplied clock semantics
and a reference-free temporal adapter must remain explicit; source intervals cannot
simply be relabelled. Recording-level evaluation and provided-reference qualification
still precede any accuracy or adaptive-superiority claim. These roughly ten-second
development prefixes do not replace the full-recording benchmark unit.

Artifacts: `results/droid-longer-prefix/{source-001,run-001}`.
Selection: `experiments/droid-longer-prefix-selection-2026-11-04.json`.
Native manifest: `experiments/droid-longer-prefix-2026-11-04.json`.
Recovery: `backups/2026-11-04-droid-longer-prefix-exp137`, using the saved EXP136
runtime as an external prerequisite.
