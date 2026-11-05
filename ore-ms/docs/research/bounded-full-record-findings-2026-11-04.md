# EXP141: bounded DROID execution on complete provided-pair recordings

Started 2026-11-04 America/New_York. Parent EXP140 `5c472df43`.
Intake freeze `16a6586e9`; corrected guard `efde01dc6`; original full-record
freeze `fecabee33`; one resource-retry declaration `19129fb8e`.

The latest declared attempts complete **6/6 streams**, returning **55,618/55,618 finite offline poses**. All six original attempts, two capacity retries and two same-capacity feature-layout corrections are retained. A finite offline pose does not establish tracking correctness or online availability.

This is camera-local offline engineering under MS²'s published same-ID stereo
pairing convention. Physical exposure clocks/calibration, actual trajectory
accuracy, online availability and adaptive RGB–thermal benefit remain unverified.
EXP133 is not promoted to an executable scientific benchmark.

## Complete inputs, not selected windows

All original stems are retained: day 5,810, night 10,131 and rain 11,868 source
groups, totalling 27,809 groups / 111,236 RGB/thermal PNGs. No source selection is
based on motion, errors, output support or resource survival. The source group is
the published four-camera ID convention, not a physically verified common exposure.

Recovered 3,612 distinct prior PNGs before fetching another 107,624 camera members.
The previous recovery archive and all selected member hashes are checked; 1,200
resident/prior overlaps agree. New requests validate exact HTTP206 range bounds
and historical object lengths, bzip2 block indexes, actual selected TAR member
names/sizes, PNG schemas and every PNG chunk CRC. Only RGB/thermal camera images
are extracted. No odometry/depth reference values or reserved-record payloads are
extracted. New visual exposure is declared prospectively in its own ledger.

The independent source audit passes **10,675,581 scalar/CRC/hash checks**, including
all 111,236 actual PNG hashes and 8,705 cached compressed ranges. Raw camera PNG
payloads total **37,767,920,837 bytes**. Current full sync-archive SHA verification
is not claimed: new remote TAR-header checks cover newly fetched camera members;
restored members are bound to their checked historical archive/member hashes.

The exact original OpenCV4.6 producer image prepares all six complete streams.
RGB PNG bytes are copied unchanged; the thermal recipe remains
`td3-firestereo-v2-clahe-bilateral-v1`, with original uint16 inputs retained.
All 1,212 PNGs in the first 101 overlap match EXP137 byte for byte. Input clocks,
provided calibration, resize/crop and pre-crop intrinsic scaling remain unchanged.
There is no nearest-time search, clock fit, scale fit, new model or weight update.

## Bounded storage and explicit capacity failure

The original DROID adapter and native source are unchanged. The initial wrapper changes
allocation and checks capacity before calling native track; the later fixed-capacity
feature-layout correction is described below. The 17 reserved
slots cover the pinned 16-frame filler batch plus the next frontend state.
There is no pruning added by the wrapper, source skipping, threshold adjustment,
reset, chunk stitching, alternate-camera fallback or unfinished-pose salvage.
The native model's original motion filter/keyframe removal remain active.

Initial capacity is the largest power of two within one quarter of the recorded
CUDA device memory divided by worst-case per-slot state bytes: 2,048 slots, with
conservative stop at 2,031 native states. Persistent storage is bounded; complete
backend/correlation/filler workspace is not guaranteed. If a capacity guard stops
tracking, the original adapter leaves its entire unfinished offline pose axis null.
The sidecar retains every requested source ID, the blocked source, unattempted
suffix, returned-call flags and final-pose availability as separate facts.

Night RGB reaches that guard after 7,876/10,131 calls, with 2,031 states and
11.105 GiB peak Torch-allocated memory. That observed resource failure motivates an
explicit amendment: one 4,096-slot retry, derived from a half-device-memory state
budget, with the same 17-slot reserve / 4,079-state guard. Only original failures
whose cause is `NativeCapacityExceeded` are eligible; all six original attempts
continue and are retained. There is no selection by pose quality or reference
error, no retry of successful cases, and no further automatic capacity increase.
The source, weights, geometry and motion settings are identical. Both larger-buffer
attempts finish tracking but fail backend initialization because the native 32-bit
correlation accessor receives the entire allocated feature tensor, including unused
rows. RGB 4096-slot storage has 3,185,573,888 elements and thermal 2,684,354,560, exceeding
the 2,147,483,647 exclusive accessor limit. All those failures remain in the outputs.

A separate fixed-capacity compatibility correction clones only the feature prefix
containing every active keyframe plus the unchanged 17-row filler/next-state reserve
after all tracking and before original offline termination. It checks bitwise int16
equality of every retained half feature, preserves the counter and other state,
and refuses if retained elements still exceed the native index limit. No active
keyframe, factor, source image or optimization step is removed. No native rebuild,
64-bit kernel substitution or allocation increase occurs. Only the never-indexed
unused tail is discarded.

A synthetic CUDA contract uses the actual 31×98/32×80 feature grids, 12 active
keyframes and 17 reserved rows, compacting 64 allocated slots to 29. Native
correlation outputs for every active/reserved fixture index are bitwise identical
before/after compaction (maximum difference zero). This is a kernel fixture, not
a comparison of two completed full-record optimizations.
Failed 4096-slot attempts are retained and both night cases are
rerun once with the corrected storage representation at the same 4096-slot cap.

## Original attempts and resource retries

Initial fixed 2048-slot attempts:

| Case | Slots | Track calls / requested | Finite offline poses | Outcome | Peak allocated GiB | Host seconds |
|---|---:|---:|---:|---|---:|---:|
| night-rgb | 2048 | 7876/10131 | 0 | capacity stop | 11.105 | 420.20 |
| night-thermal | 2048 | 7202/10131 | 0 | capacity stop | 8.973 | 288.39 |
| day-rgb | 2048 | 5810/5810 | 5810 | complete | 23.075 | 543.30 |
| day-thermal | 2048 | 5810/5810 | 5810 | complete | 19.264 | 388.55 |
| rain-rgb | 2048 | 11868/11868 | 11868 | complete | 24.931 | 890.84 |
| rain-thermal | 2048 | 11868/11868 | 11868 | complete | 21.872 | 603.12 |

One fixed 4096-slot resource retry for eligible capacity aborts:

| Case | Slots | Track calls / requested | Finite offline poses | Outcome | Peak allocated GiB | Host seconds |
|---|---:|---:|---:|---|---:|---:|
| night-rgb | 4096 | 10131/10131 | 0 | native/output failure | 37.551 | 613.78 |
| night-thermal | 4096 | 10131/10131 | 0 | native/output failure | 33.394 | 500.66 |

Fixed 4096-slot unused-feature-layout correction:

| Case | Slots | Track calls / requested | Finite offline poses | Outcome | Peak allocated GiB | Host seconds |
|---|---:|---:|---:|---|---:|---:|
| night-rgb | 4096 | 10131/10131 | 10131 | complete | 38.353 | 1059.20 |
| night-thermal | 4096 | 10131/10131 | 10131 | complete | 34.730 | 798.19 |

Final complete cases 6/6; finite offline poses 55618/55618.


Returned tracking calls are not a tracking-correctness or online-pose-availability
metric. Final finite poses come from offline native backend and image-refined
trajectory filling, including interpolated initialization and future images.
They do not retroactively turn earlier warmup states into on-time successes.
A failed finalization does not publish unfinished frontend poses as final outputs.

Peak memory is this process's Torch-allocated peak, not total device VRAM or allocator
reservation. Adapter wall time covers construction, tracking and finalization;
host invocation time in the table also includes startup and input-hash preflight.
These are engineering observations, not a controlled runtime ranking. CPU intake,
preparation/auditing and recovery work can overlap; GPU model cases run sequentially.

## Control failures, repeatability and independent validation

The first bounded prefix attempt fails before tracking because replacing DROID's
class name disrupts its upstream named-super constructor. That capture is retained.
The corrected wrapper temporarily preserves the original module-global class during
its original constructor, restoring bindings/settings even on exceptions. A later
canonical invocation also fails before producing images because this producer image
has `python3`, not `python`; the corrected invocation uses the exact same image/code.
Both are integration failures, not silently retried model-tracking outcomes.

**139 tests pass**: 105 inherited cases and 34 capacity/restore/boundary/type/constructor/compaction
cases. Six corrected 101-source controls each return 101 finite offline poses.
The declared 32-slot day RGB control stops before source 55 with 55 returned calls;
all 101 requested slots remain, with null offline poses. Its blocked frame and
unattempted suffix are independently verified.

All successful prefix input reads match saved EXP137 exactly. Four categorical
frontend traces match; rain RGB has 29 changed count fields and rain thermal 9.
Repeating the unchanged original 128-slot rain runs also changes 32/38 fields,
respectively, with identical input reads. The initial strict equality auditor
failure is retained. Amended validation reports every difference and requires the
unchanged-reference control; it does not assert that allocation has no numerical
effect. Offline pose matrix entries differ even when count traces agree, so exact
numerical repeatability is unestablished. These differences are not measured motion
error, alignment quality or a reason to select a camera.

The corrected prefix audit passes 18,998 checks and rejects 21 metric mutations.
The six original full attempts pass **1,030,620 checks / 24 metric mutations**. The failed resource retries pass **324,274 checks / 8 mutations**; corrected layout attempts pass **425,612 checks / 8 mutations**. A supplemental audit ties each compacted active count to the final tracking receipt, guard and native estimate.

Full-run validation independently reconstructs source time/stereo identity,
provided-baseline metric conversion, xyzw quaternion matrices, finite/null segments,
actual native input tensors/intrinsics, per-source status, reserve bounds and phase
ordering. Actual tracking and filler tensor hashes are checked against decoded
canonical inputs. Capacity-aborted suffixes cannot become published poses. No
scientific reference trajectory is read by these auditors.

## Limits, artifacts and recovery

No new scientific reference scoring, training, clock/scale fitting, default adoption,
validation/test assignment or final-reserve access occurs. Provided same-ID stereo
is not physical exposure proof. Original native tracking-failure status and causal
per-frame final-pose receipts remain unavailable. The strict selected-motion
publisher and benchmark protocol remain unchanged.

Protocols and declaration: `experiments/full-record-intake-2026-11-04.json`,
`droid-bounded-full-record-2026-11-04.json`,
`droid-bounded-capacity-retry-declaration-2026-11-04.json` and
`droid-bounded-resource-retries-2026-11-04.json`,
`droid-feature-layout-correction-declaration-2026-11-04.json` and
`droid-bounded-feature-layout-retries-2026-11-04.json`.
Results: `results/bounded-full-record`, including all original attempts, retries,
source/pose/status audits and `final-summary.json`.
The corrected layout protocols and native correlation contract are also retained.
Recovery: `backups/2026-11-04-bounded-full-record-exp141`.

All 111,236 raw PNGs are preserved in 39 approximately 1 GB verified USTAR shards.
Metadata, checkpoint, source/locks, receipts, failures, tests, audits and incremental
Git bundle are separate. Derived canonical PNGs can be regenerated with the pinned
producer; compressed range binaries are omitted while receipts and actual source
PNGs remain. Recovery instructions distinguish these prerequisites and archive
scope; no credentials or actual reference-pose/depth payloads are included.
An isolated directory replay checks all protocol source hashes and historical
first-101 aliases using hash-verified raw/canonical cache hardlinks. Its source
audit passes 10,666,876 checks with all 8,705 compressed cache binaries explicitly
omitted. This is not a fresh canonical regeneration or a replay of compressed
range hashes.

The exact existing EXP136 model runtime and EXP134 producer runtime are inherited.

These runs establish their recorded execution/availability outcomes under a
qualified input convention. A meaningful accuracy or selection claim still needs
a separately frozen evaluation and justified frame/time/reference semantics.
