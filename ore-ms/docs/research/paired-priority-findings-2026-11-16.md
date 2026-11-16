# Fixed camera priority has recording, frontend and endpoint tradeoffs

16 November 2026, America/New_York. Completed the
[paired-priority protocol](paired-priority-protocol-2026-11-16.md) on the
[restored paired evidence](multirecord-restoration-findings-2026-11-16.md).
Actual thermal-first replay confirms the earlier cross-record limits and resolves
SmokeRoom's previously indeterminate coarse ordering. Thermal-first gains three
moving coarse rotations there, while losing strict rotation and joint pose success.
A universal thermal-priority replacement is not supported. No new quality gate,
inference, training, threshold/clock/axis fit or model adoption was performed.

## Actual learned moving comparison

Same frozen both-camera≥2° populations; failures stay in each denominator.
Targets are nominal supplied-reference errors, not independent physical truth.

| Recording | Policy | Eligible anchors | Outputs | R≤1° | R≤5° |
|---|---|---:|---:|---:|---:|
| FlashLight | RGB | 192 | 192 | 176 | 185 |
| FlashLight | Thermal | 192 | 168 | 57 | 110 |
| FlashLight | RGB-first | 192 | 192 | 176 | 185 |
| FlashLight | Thermal-first | 192 | 192 | 79 | 134 |
| SmokeRoom | RGB | 236 | 221 | 151 | 181 |
| SmokeRoom | Thermal | 236 | 231 | 116 | 190 |
| SmokeRoom | RGB-first | 236 | 236 | 155 | 192 |
| SmokeRoom | Thermal-first | 236 | 236 | 121 | 195 |

FlashLight thermal-first loses 97 strict and 51 coarse successes relative to
RGB-first despite identical output availability. SmokeRoom thermal-first loses
34 strict successes while gaining three coarse successes. Among both-supported
SmokeRoom moving cases, 57 favor RGB versus 23 thermal at 1°; at 5°, 25 favor
thermal versus 22 RGB. Availability alone cannot resolve these incorrect poses.

Actual learned moving thermal-first counts lie inside the frozen documentary
bounds: FlashLight79 in65–81 and134 in127–134; SmokeRoom121 in116–121 and195
in190–195. The prior SmokeRoom coarse bound allowed either ordering; the actual
pairing now establishes195 versus192. Bounds were not revised after this replay.

## Translation changes the decision

Joint means own-camera R≤5° and directed translation error≤15° on the frozen
both-displacements≥1cm cohort. This is a pair proposal diagnostic, not metric body
translation or a stateful VO trajectory.

| Recording | Learned policy | Translation cohort | Joint successes | Rate |
|---|---|---:|---:|---:|
| FlashLight | RGB-first | 519 | 452 | 87.1% |
| FlashLight | Thermal-first | 519 | 305 | 58.8% |
| SmokeRoom | RGB-first | 483 | 356 | 73.7% |
| SmokeRoom | Thermal-first | 483 | 256 | 53.0% |

The three additional coarse SmokeRoom rotations do not establish a broader pose
gain: thermal-first loses100 joint successes on483 translation anchors. On the
236 moving anchors alone, joint success also falls154→104. FlashLight loses147
joint successes on its full translation cohort (151→79 on its moving subset).

## Computation and frontend dependence

| Full recording | Learned policy | Outputs | Matcher calls | Projected cached seconds |
|---|---|---:|---:|---:|
| FlashLight | RGB-first | 527 | 558 | 153.298 |
| FlashLight | Thermal-first | 527 | 591 | 166.336 |
| SmokeRoom | RGB-first | 551 | 1,175 | 327.109 |
| SmokeRoom | Thermal-first | 551 | 1,181 | 333.843 |

Thermal-first adds33calls onFlashLight and6 onSmokeRoom. Whole-record projected
time rises about8.5% and2.1%, respectively. SmokeRoom's moving subset instead
changes251→241calls and69.070→67.842s; that local reduction does not apply to its
whole recording. Durations come from the same cached parent capture; they are
not live lazy-execution latency, energy or accuracy-matched compute benchmarks.
Unsupported primaries make no matcher call. Their tiny recorded input-check time
is separately reported and retained in projected seconds to match the parent's
accounting convention. Failed qualified calls retain their full cost.

SIFT behaves differently on the same moving populations:

| Recording | RGB-first R≤1° / R≤5° | Thermal-first R≤1° / R≤5° | Outputs for either order |
|---|---:|---:|---:|
| FlashLight | 14 / 73 | 32 / 118 | 157 |
| SmokeRoom | 30 / 107 | 64 / 133 | 198 |

SIFT thermal-first helps both moving rotation endpoints on both records. That is
not evidence to adopt the weaker frontend. Whole translation-cohort SIFT joint
success changes206→199 onFlashLight and167→200 onSmokeRoom: even within this
frontend the coarse rotation and joint conclusions differ. All seven cohorts,
both frontends, fixed-camera controls and inherited RGB-or-zero controls remain
in the machine-readable summaries. Static/small-motion counts are not presented
as moving accuracy, and the preserved identity controls remain strong at coarse
thresholds.

## How much can camera selection possibly recover?

The endpoint-specific oracle uses reference labels and counts an anchor whenever
either available proposal is correct. It is not deployable and can choose different
cameras at different endpoints. Best-priority is the better of the two fixed
orders for that particular endpoint, not a single policy achieving all columns.

| Recording | Learned endpoint | Eligible | Best fixed camera | Better fixed order | Oracle | Oracle beyond better order |
|---|---|---:|---:|---:|---:|---:|
| FlashLight | Moving R≤1° | 192 | 176 | 176 | 178 | 2 |
| FlashLight | Moving R≤5° | 192 | 185 | 185 | 188 | 3 |
| FlashLight | Joint on translation cohort | 519 | 452 | 452 | 458 | 6 |
| SmokeRoom | Moving R≤1° | 236 | 151 | 155 | 178 | 23 |
| SmokeRoom | Moving R≤5° | 236 | 190 | 195 | 217 | 22 |
| SmokeRoom | Joint on translation cohort | 483 | 349 | 356 | 384 | 28 |

FlashLight offers only2/192 strict or3/192 coarse moving successes beyond RGB-first
(about1.0/1.6 percentage points). SmokeRoom offers23/236 strict or22/236 coarse
beyond the respective better order (9.7/9.3 points), and28/483 joint successes
(5.8points). These are ceilings on exposed data, not evidence an observable cue
can recover them without trading away currently correct proposals. Even perfect
selection cannot repair the19SmokeRoom or4FlashLight moving anchors with no
available≤5°-correct proposal. Whole translation-cohort learned oracle misses
99SmokeRoom and61FlashLight anchors; those require better proposals or another
mechanism, not a different choice between the same two outputs.

## OutdoorNight remains a separate target tier

The [existing nominal gyro replay](camera-priority-findings-2026-11-16.md) is
inherited unchanged: on242moving anchors, learned RGB-first125/180 versus
thermal-first164/218 within1°/5° of gyro; oracle188/228 leaves24/10 beyond the
better priority. It has no supplied translation truth. Its thermal-first cost
reduction is a result for that recording, not transferred toSmokeRoom/FlashLight.
No pooled accuracy or timing average is computed across these target types.

Figure (PNG/PDF): `results/paired-priority-controls/figures/paired-priority-comparison.*`.
The supplied-reference and gyro panels are explicitly labelled separately.

## Verification, retained attempts and decision

All1,379anchors and5,516parent prediction rows remain fixed;2,758 status-only
choice rows were locked before joining errors. Original RGB/thermal/RGB-first
summaries and zero-rotation controls pass476parity comparisons per recording,
952total. Independent auditors pass28,441FlashLight and43,291SmokeRoom checks,
71,732total, reconstructing routing, own sensor endpoints, reference eligibility,
cost, all cohorts, numeric endpoint labels, oracle conflicts and historical bounds.

Six finite tests and six semantic-audit tests pass in the verified suite. Rehashed
choice, cost, oracle, joint-eligibility and target relabelling corruptions fail as
required; the valid full replay passes. Original attempts remain visible: an
initial finite fixture understated zero-rotation success at5°; preflight then
caught an incorrect zero-duration assumption for unsupported inputs, before a
freeze or choice. These were corrected before the successful source freeze.
The frozen original camera-corruption test later proved ineffective because it
assigned RGB to an already-RGB initial choice. Its failed log/source remain;
`source/verified-test-suite.py` changes that fixture to the opposite camera and
runs the same twelve tests. The tracked command is
`PYTHONPATH=/tmp/ore-ms-packages python3 tools/verify_paired_priority_tests.py`
from `ore-ms/` (with equivalent installed dependencies). No production policy, audit, result or frozen hash
was changed to fix that post-freeze test. This is documented fixture maintenance,
not a changed scientific endpoint or a discarded recording failure.

The supplied references can depend on IMU; physical clock/exposure/reference truth
and training overlap remain qualified. Prior SubT-MRS/MAC-VO/thermal reading was
reused; no new paper-body reading, author-system reproduction or novelty claim.
Full evidence/figures/logs are under `results/paired-priority-controls/`. A verified
incremental packet with dependency pins is stored under
`/root/ore-ms-artifacts/paired-priority-controls-2026-11-16/`.
No Box upload, author outreach or Git publication occurred.

Stop decision: retain both fixed orders as controls; do not adopt universal
thermal-first or fit a quality gate from these outcomes. The next bounded task
is failure characterization of SmokeRoom's small, explicitly enumerated recoverable
strata versus wrong-both cases, with FlashLight as a regression control. Use fixed
sampling and existing diagnostic machinery; review previous uncertainty/closure/
geometry null results before proposing a cue. Any subsequent selector needs a
separate protocol and recording-level splits, with exposed status preserved.
