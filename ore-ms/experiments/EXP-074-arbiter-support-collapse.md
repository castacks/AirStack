# EXP-074: advisory thermal fallback on known support collapse

2026-10-29 America/New_York. Branch `ore_ms`, base `cae2a062a`.

## Predeclared scope

Use the complete existing 100-group EXP032 middle window, manifest start0
(original paired start700), already evaluated in EXP033. Historical support
was 97/99 unsupported RGB pairs and zero unsupported thermal pairs. This is
a retrospective availability diagnostic deliberately exercising known collapse;
it is not held-out evaluation or evidence of generalization. No reserved MS²
recording or new scene is used. No alternate start, threshold or policy is chosen
from the new outputs.

Restore only the 405 prepared dataset files from the EXP032 recovery archive.
The complete compressed archive SHA256 and each restored member SHA256 must
match its existing recovery manifest. Keep exact camera timestamps, images and
provisional geometry. Record the immutable restoration manifest and freeze all
source/input/checkpoint hashes and commands before inference.

## Fixed method and comparisons

Use unchanged EXP073 scheduler, event recorder, v2/v3 runners and estimator
settings. Run fresh standalone v2 RGB and thermal controls, then one shared
100-group replay. All use the same pinned Blackwell runtime, seed0, FP32,
synchronous optimizer and raw uncertainty recording. The scheduler serializes
frame calls by sensor timestamp and records live event prefixes on a shared
monotonic controller clock. This remains offline serial execution.

The advisory rule requires complete supported health and a valid first
camera-local motion matching the exact camera interval. Prefer RGB when both
experts are eligible; choose thermal when only thermal is eligible; otherwise
abstain. The decision for pair i follows completion of group i+2. Evaluate the
last pair at the final frame boundary before drain. Do not backfill that decision
from shutdown results. Preserve unsupported pairs, repeated writes and interval
mismatches as observed, without treating filled trajectories as fresh motion.

Compare RGB-only, thermal-only and the fixed advisory rule at those identical
boundaries. Primary diagnostic is available versus abstained intervals; also
count unsupported choices. These are camera-local availability measurements,
not pose accuracy, covariance calibration or a stitched cross-camera trajectory.
Thermal-only is a necessary baseline: recovering RGB unavailability need not
outperform simply using thermal throughout this known condition.

## Validation plan

New versioned auditors retain the original EXP071/073 auditors unchanged.
`validate_arbiter_event_replay_v2.py` checks bitwise control preservation and
raw-array health while allowing sparse optimization submissions, repeated old
writes and no new final-frame solution. It does not assume a solved last pair.
`validate_shared_arbiter_replay_v2.py` independently reconstructs event-prefix
availability, exact deadlines, advice and the three fixed coverage comparisons.
It is locked to this 100-group middle manifest and frozen plan. It imports
neither scheduler nor policy code. Recompute timeline hashes in semantic
mutation tests so those tests exercise audit logic as well as integrity checks.

Existing policy, pairing and recorder regression tests run in Docker. Results,
commands, source hashes, independent proofs and logs are retained under
`results/arbiter-support-collapse/`; authoritative inputs remain under
`results/long-night-validation/prepared-001/datasets/middle/`.

## Results

The unchanged advisory rule produces **97 THERMAL, 1 RGB and 1 ABSTAIN** decisions.
It makes zero unsupported choices. The full-capture original support pattern
matches the historical result: RGB97/99 unsupported pairs, thermal0/99.

| Policy at identical declared deadlines | Available intervals | Abstentions |
|---|---:|---:|
| RGB-only | 1/99 | 98/99 |
| Thermal-only | 98/99 | 1/99 |
| Advisory, fixed RGB preference | 98/99 | 1/99 |

The advisory rule recovers availability relative to RGB-only and matches
thermal-only. This condition does **not** demonstrate an availability or accuracy
advantage over thermal-only. All 98 ordinary boundaries have thermal available;
RGB is also eligible for just one of them. The last boundary has neither current
result committed and abstains, exactly as specified before replay.

RGB's supported target frames are97 and99. The first normal result for frame97
is committed while processing frame98. An additional write of that old result
occurs while processing frame99; it does not become another eligible interval.
The new frame99 result arrives only in termination, after the final decision.
Thus two supported RGB pairs after full capture yield only one eligible RGB
result by the declared deadlines. The final thermal result likewise arrives in
termination. No advice is revised after drain, and filled/refined RGB poses are
not substituted for missing results.

| Check | RGB | Thermal |
|---|---:|---:|
| Input stereo frames | 100 | 100 |
| Original unsupported pairs | 97/99 | 0/99 |
| First motion writes after complete capture | 2 | 99 |
| Repeated writes, excluded as new measurements | 1 | 0 |
| Streamed events, exactly matching saved journal | 207 | 497 |
| Bitwise-equal arrays versus fresh standalone v2 | 2613 | 2613 |
| Compared NPZ files | 103 | 103 |
| Observer errors | 0 | 0 |

These are fresh same-runtime preservation comparisons, not claims of bitwise
parity with the historical EXP033 environment. The two extra arrays relative
to EXP033's2611-array comparisons are the v2 tracking-flag arrays now present in
both compared outputs. Original configuration/report fields and trajectories,
first-depth fields, full tensor maps and all99 raw uncertainty files match.
Raw covariance/health counts, support flags, graph intervals, first/stale write
semantics and derived camera increments pass independent source-array audit.

The independent shared audit passes4158 decision-field checks across200
scheduled frame calls and99 decisions, including all fixed-policy counts.
Seven semantic mutations are rejected after updating the outer timeline hash:
wrong fallback action, future-prefix length, altered sensor order, early deadline,
shutdown backfill, treating a stale result as new, and unsupported RGB eligibility.
All43 existing policy/pairing/recorder regression tests pass in Docker. Test counts
and audit fields are software checks, not independent scientific samples.
All405 inputs, both checkpoint hashes and frozen sources remain unchanged.
No estimator/scheduler/advisory/default code was modified in this experiment.

The compressed EXP032 archive is878919049 bytes; its SHA256 and restored file
hashes are recorded in `input-restoration.json`. Runtime image:
`sha256:700f33b2f919364dfd8aefe486568ff69a7832cc5942149f03eef5b561230239`.
Timeline SHA256:
`2225bc7830860516f074028f0c94e7f56d30d41778fba4cb41b0730957d586cc`.
The capture plan freezes exact commands, input paths, source/checkpoint hashes
and method before all three runs. Recovery destination:
`backups/2026-10-29-exp-074`, dependent on EXP032 for prepared images and on
EXP071–073 for prior code/runtime/checkpoint recovery. Credentials, weights and
the full original data/source archives are excluded from this new snapshot.

## Reproduction and decision

From the MAC-VO root in the pinned Docker image, use the same v2 control commands
as EXP073 with the middle dataset,100 frames,start0. The frozen capture plan
contains all three actual Docker commands and thread flags. Shared replay:

```bash
python -m Scripts.Setup.shared_arbiter_replay \
  --dataset ../results/long-night-validation/prepared-001/datasets/middle \
  --manifest ../results/long-night-validation/prepared-001/datasets/middle/manifest.json \
  --frames 100 --start 0 --acknowledge-provisional-geometry \
  --output <fresh-output>
```

Run `validate_arbiter_event_replay_v2` once per expert against its fresh standalone
v2 control, saving `<camera>-preservation.json` beside the capture plan. The
shared independent auditor is executable with standard host Python:

```bash
python3 Scripts/Setup/validate_shared_arbiter_replay_v2.py \
  --root /root/AirStack/ore-ms \
  --run /root/AirStack/ore-ms/results/arbiter-support-collapse/run-001
```

The availability baseline has now been exercised on both a supported window
(EXP073) and a known collapse window. The next useful step is a declared
pose-comparison contract: transport to a verified common body/interval target,
choose an applicable independent reference and fixed loss, then evaluate whether
uncertainty/history can improve selection over both fixed experts. These
camera-local advisory records are not a fused trajectory, pose-quality labels,
a trained gate, or evidence of VO accuracy improvement. No new data upload was
needed for this test.
