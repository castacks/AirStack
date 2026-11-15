# Continuity feasibility: retained tracking, restart and observable recovery

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus preserved working sources.
Bounded source/asset intake complete. No neural inference, trajectory scoring,
new implementation, rendering, clock fitting or training.

## Finding and disposition

The current source supports attempting a visual match from the retained keyframe
to the next received image. It does not establish global recovery after arbitrary
loss of overlap. An explicit restart must state a new frame or supply a measured
connection to the old frame. Restoring adjacent outputs, interpolation and true
cross-gap localization are distinct outcomes.

**Natural physical recovery comparison: NOT EXECUTION-READY.** The inspected
prefix lacks a restored full stereo input bundle at declared paths, a qualified
cross-gap inertial bridge and independently verified thermal pose/timing truth.
These are readiness limits, not evidence that recovery or thermal-inertial SLAM
cannot work. Provided-reference development and exact synthetic mechanism tests
remain possible with separately qualified inputs and an explicit target.

No new recovery mechanism or novelty is selected. Established visual
relocalization/IMU bridging/map merging must be controls or overlap, not renamed
contributions. A method proposal needs a specific thermal failure and mechanism
beyond those established operations. Do not launch another scheduling fixture,
recover large archives, or substitute a nominal RGB reference for physical thermal
truth merely to obtain a numerical result.

## Native source trace

Inspected working files are SHA-bound in
`results/continuity-feasibility/report.json`; this is the local version, not a
claim about every upstream version/configuration.

| Stage | Source behavior | Consequence for a recovery test |
|---|---|---|
| Initialization | `Odometry/MACVO.py:initialize` estimates depth, obtains a motion-model pose and stores the keyframe | Initial frame depends on configured motion model; inspect the actual config before running |
| Frame receipt | `run` passes the retained `prev_keyframe` to `run_pair`; it has no explicit lost/relocalization state machine in the inspected path | Removing arrivals may cause cross-gap image matching; do not assume every interruption forces a reset |
| Matching/pose | `run_pair` estimates depth/flow, registers world points from the previous pose and optimizes the new pose | A successful cross-gap visual match can carry the old coordinate frame; this has not been numerically tested here |
| Low match count | Fewer than10accepted matches marks the new pose `need_interp`; tracking context was already advanced | Later adjacent matches can resume without establishing correction of the earlier global error |
| Finalization | `terminate` invokes configured `MapRefiner`; pose/motion interpolation uses flagged entries and neighboring frame indices | Offline interpolation may use later poses; save raw online states separately and never score interpolated availability as causal recovery |
| Motion initialization | `MotionModel.py` includes static, learned and GT-with-noise alternatives | Presence of a GT model does not mean our chosen run uses it; a recovery experiment must prohibit reference-derived initialization/bridging |
| Data interface | The inspected MS2 adapter emits `StereoFrame`, not synchronized inertial packets | IMU bridging is not a drop-in assumption for this source/data path |

`TwoFramePGO` constructs local observations/points and optimizes a new pose with
map points fixed in the inspected ICP path. No place-recognition database,
cross-map merge or general relocalization call was found along this inspected
MACVO processing path. This targeted path review is not an exhaustive audit of
all AirStack estimators. Calling `initialize` on an existing object is not a
qualified reset: graph, motion model and optimizer state must all be specified.

## One-prefix asset check

Population fixed to the already-exposed night `_2021-08-13-21-18-04`, stems
000000–000019. Read the recovered historical manifest, not another recording.
No pixels decoded, reference matrices parsed or timestamp offsets refitted.

- The historical manifest is present in the recovered persistent-window packet.
  None of its40thermal stereo files exists at either the working-tree or recovered
  declared relative paths. This is a path-bundle readiness result, not a claim
  that the images are globally unavailable or absent from backups.
- All20raw left thermal frames remain available in the prior warp-assay folder.
  Four prepared first-pair views remain in the qualified frontend inputs. Neither
  constitutes the complete20-frame raw stereo sequence.
- All20supplied RGB-camera reference files are present and match their recorded
  SHA256 values. Bytes were checked; no new motion/accuracy values were read.
- Their inherited use for thermal evaluation requires nominal rig/temporal
  assumptions. The prior clock/input-contract findings remain authoritative;
  exact exported-clock disagreement is not proof of unsynchronized exposures.
- No capture-specific synchronized IMU stream/extrinsics/noise contract was
  qualified for this prefix. This is not a global search asserting that no IMU
  data exists. The MS2 loader inspected here carries no IMU packet.

The [full-record reference review](provided-reference-full-record-findings-2026-11-05.md)
and [input contract](shared-motion-input-contract-findings-2026-11-12.md) already
establish these reference/temporal limitations. Missing runtime paths were found
and documented; archived paths must not be silently presented as live inputs.

## Established recovery mechanisms and source depth

[ORB-SLAM3 v2](https://arxiv.org/html/2007.11898v2), targeted III, IV-A, V-D and
VI-A reading: relocalization associates current observations to existing map
geometry; inertial propagation supports short visual losses; disconnected maps
can later be related through verified shared observations. Restarting a local
map alone does not provide its transformation to the old map. Those mechanisms
are prior art. Thresholds/branches in the released code are configuration and
sensor-mode specific, not universal outage limits.

Author repository pinned to
`4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`:
[released code](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4).
Targeted tracking-loss/timestamp/relocalization/map-creation branches inspected in
`src/Tracking.cc`, reset entry points searched in `src/System.cc`, state declarations
searched in `include/Tracking.h`, repository README inspected. Four file hashes
retained in the report. Read-only snapshots remain outside the repository; no
build, thermal integration, benchmark reproduction or author-code execution.

[TP-TIO](https://arxiv.org/abs/2012.03455) abstract recheck confirms thermal-inertial
optimization is existing work. No fresh full-method/code recovery audit.
[Self-TIO author dataset repository](https://github.com/junwoonlee/self-tio-dataset)
README recheck describes camera/IMU calibration and GNSS references, and states
no public network/code release is planned. It is an external data lead, not a
locally qualified recovery baseline or newly acquired dataset.

## What a defensible next experiment must separate

There are three measurable questions, and a single resumed-output count cannot
answer all of them:

1. **Retained tracking:** can actual pre/post-gap image evidence estimate the
   missing relative transform while preserving the old frame?
2. **Local restart:** does adjacent estimation resume in a declared new local
   frame, with the unknown connection explicitly retained?
3. **Global recovery:** can verified map observations or a qualified sensor
   estimate that connection, and how much global error remains afterward?

For a future protocol use one fixed interruption, save unprocessed online poses
and state IDs, and score in one pre-gap coordinate frame. Retain uninterrupted
context, unchanged-native gap processing, explicit restart and an established
appropriate bridge/relocalization control. Report missing poses, false recovery,
latency and post-gap error; keep finalized/interpolated results secondary. Do not
align each segment independently or use reference motion as an estimator input.

With no visual overlap, no map association and no other measurement across an
interval, arbitrary intervening motion is unobserved. A prior may predict it but
cannot turn it into measured recovery. This identifiability boundary is useful
for evaluation, not new SLAM mathematics or an impossibility result for recovery
when other measurements exist.

The finite intake ends here. The next justified work is a specific mechanism and
input/target choice: qualified synthetic sequence for a causal mechanism test,
restored supplied-reference stereo for explicitly nominal development, or a
qualified thermal-inertial/reference dataset for physical accuracy. These support
different claims and are not interchangeable. The other original-paper gaps
remain available; this review does not require a sensor/data acquisition detour.
