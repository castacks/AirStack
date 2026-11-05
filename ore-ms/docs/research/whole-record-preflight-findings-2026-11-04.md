# EXP140: complete source inventory and the unchanged DROID allocation limit

2026-11-04 America/New_York. Parent EXP139 `826e633b9`; preflight freeze
`b3602c826`, auditor amendment `3eb6b1c46`.

The three complete development recordings contain 27,809 original source groups /
111,236 RGB/thermal stereo PNG members. Current replay inputs cover 303 groups;
prior exposed-data recovery can raise this to 903. Another 26,906 complete groups
still require camera acquisition. All three records and both modalities remain in
the target population; no easier recording is selected after capacity inspection.

The unchanged EXP137 source-count-derived DROID allocation requires 16,384 slots
for night and rain. Persistent native state alone then needs 68.608 GiB for RGB or
57.813 GiB for thermal, exceeding this runtime's reported 47.267 GiB CUDA device
capacity. This is an allocation limitation of the current conservative wrapper,
not proof that DROID cannot process these recordings with another memory strategy.
No full-record tracking, model inference, accuracy scoring or benchmark execution
occurs in this preflight. Short windows and allocation probes are not whole runs.

## Exact source population and recovery

Restore only 21 existing metadata members from the day EXP048, night EXP041 and
rain EXP054 packets: each sync TAR member index, compressed bzip2 block index,
archive size/hash registry and four original camera timestamp text files. Verify
all three packet hashes and selected-member hashes, rejecting conflicting files.
No reference payload member, camera image, calibration object or model is extracted.
The existing EXP114 broader packet's complete SHA256 is also checked; its member
hash manifest identifies previously exposed recoverable windows without decoding
reference values or camera pixels.

Four indexed PNG stem sets per recording match exactly and are complete ordinal
axes starting at original `000000`. All four timestamp arrays have the same counts
and strictly increasing integer nanoseconds. There are no unmatched camera stems,
renumbering, nearest-time matches, substitutes or outcome-selected windows.
The full axis and indexed PNG byte sizes are saved in `source-axis.csv`.

| Condition | Complete source groups | Four-camera PNGs | Resident complete groups | Resident or prior-recoverable groups | Still missing groups |
|---|---:|---:|---:|---:|---:|
| Day | 5,810 | 23,240 | 101 | 301 | 5,509 |
| Night | 10,131 | 40,524 | 101 | 301 | 9,830 |
| Rain | 11,868 | 47,472 | 101 | 301 | 11,567 |
| Total | 27,809 | 111,236 | 303 | 903 | 26,906 |

Resident coverage is hash-qualified using all 1,212 original raw PNG paths in the
EXP137 prefixes. Existing EXP114 contains 300 previously exposed groups per record
in early/middle/late windows; original source 0 supplies the additional group.
The resident early prefix overlaps that recovery population, so the counts are
unions, not sums. All 1,200 overlapping camera hashes agree. Recovery eligibility
is bound to the checked existing packet and its recorded member hashes; this
phase does not extract or newly run those middle/late windows.

| Condition | Indexed RGB/thermal PNG bytes | Still missing PNG bytes after prior recovery | Historical complete sync archive bytes |
|---|---:|---:|---:|
| Day | 9,515,745,011 | 9,061,869,350 | 14,795,109,417 |
| Night | 11,613,003,409 | 11,260,259,133 | 19,721,804,466 |
| Rain | 16,639,172,417 | 16,194,502,610 | 24,906,100,204 |
| Total | 37,767,920,837 | 36,516,631,093 | 59,423,014,087 |

PNG bytes are indexed member payload sizes, excluding filesystem/canonical output
costs. Complete sync archives also contain other sensors. Archive sizes/hashes are
historical source metadata; no fresh remote content or complete sync archive is
fetched or verified here. Range acquisition can reuse the restored block indexes,
but any new remote response must validate current size, range bounds and member
identity. A remote archive cannot be assumed unchanged because an old index exists.

## Exact native state storage and measured CUDA probes

Use the unchanged EXP136 runtime and its pinned official native `DepthVideo` source.
Do not load weights, read images, call tracking or run the backend/filler. Allocate
only native stereo state at buffers 64 and 128 for RGB 248×784 and thermal 256×640.
Both geometries retain the prior crop/intrinsics contract; no threshold or model
setting is modified. All 12 persistent tensors are inventoried from actual objects.

For height h, width w, feature cells q=(h/8)(w/8), and b slots, exact tensor storage is:

`b * (7*h*w + 1032*q + 50)` bytes.

The full-resolution terms are uint8 RGB images and float32 upsampled disparity.
Feature terms include two float32 disparity fields, stereo half-precision feature
maps and two half-precision recurrent-state tensors. Small fields include time,
flags, poses and intrinsics. Stereo has two feature-map views even though its image
storage keeps the native three-channel image buffer. No persistent tensor is omitted.

| Camera | Bytes per slot | 64-slot tensor bytes / measured CUDA allocation delta | 128-slot tensor bytes / measured CUDA allocation delta |
|---|---:|---:|---:|
| RGB | 4,496,290 | 287,762,560 / 289,852,928 | 575,525,120 / 576,492,544 |
| Thermal | 3,788,850 | 242,486,400 / 242,487,808 | 484,972,800 / 484,973,568 |

Actual tensor counts exactly match the independent equation and linear slot slope.
Measured allocator totals can exceed tensor payload bytes. The first auditor
incorrectly imposed an unsupported 512-byte-per-tensor rounding upper bound; RGB
allocations violate it. That failed check and original auditor are retained.
Separate v2 removes only the unsupported upper bound and still checks exact tensor
storage, slot slope and that measured allocation contains the persistent storage.
No tensor formula, native source, input, allocation measurement or capacity decision
is adjusted or rerun to fit that auditor. CUDA/container startup emitted nonfatal
initialization diagnostics; native imports and all four CUDA allocations completed.

The frozen wrapper uses `max(64,next_power_of_two(source_frames+16+1))`, reserving
source-count capacity plus a native 16-state filler batch and one next state.
This source-count bound is deliberately conservative: actual native storage is
keyframe-based. Observed short-prefix keyframe fractions do not prove full-record
capacity and are not extrapolated to select a smaller cap.

| Condition | Frames | Unchanged allocation slots | RGB state alone (GiB) | Thermal state alone (GiB) | Decision |
|---|---:|---:|---:|---:|---|
| Day | 5,810 | 8,192 | 34.304 | 28.907 | State fits; full-run peak unproven |
| Night | 10,131 | 16,384 | 68.608 | 57.813 | Both exceed total CUDA device memory |
| Rain | 11,868 | 16,384 | 68.608 | 57.813 | Both exceed total CUDA device memory |

These are lower bounds, excluding model parameters, correlation, graph/backend and
filler workspaces and other runtime allocations. Day is not admitted as a complete
feasible run merely because state alone fits. No impossible allocation is attempted,
no GPU out-of-memory crash is induced and no record is shortened, split or reset.

## Independent checks and limits

The v2 auditor passes 252,888 checks over original full source axes, all timestamp
and byte-size CSV fields, resident/prior recovery unions, overlap hashes, exact
source and runtime identity, all native tensor counts and the six capacity cases.
Six copied-output mutations reject missing rows, wrong stems, false input coverage,
wrong tensor element size, falsely loaded weights and underreported allocation.
All 1,250 initial preflight pins and 28 parent EXP139 pins remain unchanged.
The amendment separately pins the corrected auditor and original protocol hash.

Published same-ID pairing is the engineering convention. EXP139's unresolved
exposure/export event, clock domain/mapping, stereo correspondence and physical
calibration limitations remain. These source/capacity checks do not establish
accurate simultaneous stereo geometry, physical common-time motion, adaptive
benefit or causal availability of DROID's offline final trajectories. EXP133 stays
execution-not-ready, active test/validation assignment empty and final reserves closed.

## Concrete continuation

The source intake target is now explicit: all original RGB/thermal members for all
three complete recordings, with current remote/member validation before use.
Previously archived images should be recovered before new requests. Full-record
canonical preparation must preserve the already frozen producer and prefix bytes.

Before unchanged DROID can run that entire target on this GPU, declare and qualify
a different capacity strategy with explicit overflow/failure accounting. A bounded
keyframe allocation must not hide exhausted capacity, alter motion thresholds,
truncate records or join reset trajectories into one continuous gauge. State-only
memory estimates cannot establish backend/filler peak feasibility. ORB's existing
CPU path has no demonstrated version of this particular GPU-state blocker, but its
full-record runtime/memory and source intake still need qualification.

This preflight is complete; whole-record execution is pending those concrete input
and capacity steps. No small-window result is promoted to a whole-record result.

Artifacts: `results/whole-record-preflight/{source-001,run-001}`.
Protocol: `experiments/whole-record-preflight-2026-11-04.json`.
Audit amendment: `experiments/whole-record-preflight-audit-amendment-2026-11-04.json`.
Recovery: `backups/2026-11-04-whole-record-preflight-exp140`.
