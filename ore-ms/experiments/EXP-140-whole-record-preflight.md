# EXP140: complete source inventory and DROID allocation preflight

2026-11-04 America/New_York. Parent EXP139; freeze `b3602c826`, independent
auditor amendment `3eb6b1c46`.
[Findings](../docs/research/whole-record-preflight-findings-2026-11-04.md).

All three complete exposed records: day5,810/night10,131/rain11,868 groups,
27,809total/111,236camera PNG members. Restore21visual/index metadata files only;
packet/selected-member hashes verified.303groups resident;903available after prior
exposed-window recovery;26,906still missing. Current remote archive hashes unverified;
historical complete sync archives total59,423,014,087bytes. No reference values read.

Actual pinned native stereo DepthVideo allocations at64/128slots confirm all12
persistent tensors:RGB4,496,290bytes/slot,thermal3,788,850. Unchanged full-source
wrapper needs8192slots day,16384night/rain: statealone RGB34.304/68.608GiB,
thermal28.907/57.813GiB vs47.267GiB CUDAdevice. Night/rain both exceed totalmemory;
day statefits but model/backend/workspace feasibility unknown. Not a general DROID
impossibility; no impossible allocation or full-record tracking attempted.

Initial auditor's unsupported allocator-rounding upper bound fails; separate v2
removes only that bound, original failure retained. Exact tensor formula and measured
probe unchanged.252,888independent checks/6mutation rejections pass;1,250preflight/
28parentpins unchanged. No weights/images/tracking/backend in GPUprobes. No new
images/native inference/scoring/training/reserves/default adoption. Physical clocks/
calibration and offline availability limitations remain; EXP133unready.

Next validated complete-source intake and separately declared bounded-capacity
execution with explicit overflow/failure accounting; no truncation/chunk joins or
threshold tuning. Allthree records/bothmodalities retained. Recovery
`backups/2026-11-04-whole-record-preflight-exp140`.
