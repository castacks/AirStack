# Contribution decision: stop fixed-weight extensions; qualify a real failure before a new method

15 November2026. This is an inventory/prior-art decision, not a new accuracy
experiment. The objective remains a useful VO contribution, including directions
beyond uncertainty weighting. No new method is selected for implementation.

## Decision

Stop the fixed-weight fusion/extra-optimization experiment ladder. Current evidence
supports scoped engineering findings and a useful negative attribution result,
but does not establish a compelling new-method claim. Do not manufacture one by
renaming a gate, changing a weight, selecting successful windows or counting more
audit checks. Arithmetic integrity is necessary but does not establish novelty or
scientific importance. Existing positive results remain valid under their scopes.

Prioritize investigating **natural within-recording degradation and recovery**
as the next application question. This is a direction for baseline assessment,
not a claim that recovery is new or an authorization to train a recovery model.
A specific unmet failure must first survive comparison with current published
multispectral systems and the strongest matched-sensor simple controls.

## 16November follow-up: existing learned matching largely closes the synthetic gap

The [full-population learned control](smokeseer-learned-findings-2026-11-16.md)
reaches97.92%smokyRGBand98.53%thermal rotation success with the unchanged
monocular pose backend. In nine identical-camera scenes, both-camera rotation
failures shrink217→2relative to SIFT. The large SIFTgap does not establish an
unmet problem beyond this existing frontend control. CleanFicusregression and
all other cases remain reported. This closes the bounded synthetic-SIFT route
as a new-method justification, not physical smoke or broader thermal research.
No model training, additional tuning, full author-system or real-smoke reproduction
occurred. Existing real day/night controls retain their distinct conclusions.

## Natural transition readiness: 16November

The next finite task was source/asset readiness for natural degradation, not
another synthetic stress test. Earlier
[continuity feasibility](continuity-feasibility-findings-2026-11-14.md) and
[controlled continuity](continuity-sequence-findings-2026-11-14.md) already cover
retained tracking versus restart on artificial gaps; no repeat was performed.
[Dynamic thermal evidence](dynamic-thermal-evidence-findings-2026-11-15.md)
already records local spatial/temporal effects under nominal interval assumptions,
not an independently qualified physical transition benchmark.

**Current disposition:** published-release metadata and small sidecars are
accessible, but the inspected public folder does not supply the intended adverse
sequences. This closes the bounded readiness intake; no accuracy run or new
mechanism is selected. Do not repeat the same access/search loop without a new
mirror, publication update or actual recording.

The [pinned release](https://github.com/seungsang07/cross-spectral-stereo-inertial-odometry/tree/fd4d81d7dd61cd50bda8571b38da0f6e8f90d5b6)
links separate public dataset and matcher folders. Initial HTTP403and anonymous
API401did not establish that these shares were private or broken. A subsequent
anonymous browser loaded their normal listing responses without sign-in. Current
root listings contain3dataset items and1matcher item, no folders or next-page
link, below the30item page limit. This is an observed public-view inventory,
not a claim about the owner's other storage or globally unavailable data.

| Publicly listed item | Bytes | Qualification in this pass |
|---|---:|---|
| `calibration.yaml` |2246|Downloaded, exact listing size and SHA256verified; four cameras and LiDAR transform structurally checked. |
| `ground_truth_well-lit.txt` |314576|Downloaded, exact listing size and SHA256verified;2358finite8-column rows, increasing first column and unit-norm last-four tuples. |
| `well_lit_day_out.bag` |26823027384|Listing only; complete bytes/hash/topics not verified. A content-HEADrequest returned401, which is not proof that normal GETcannot download it. |
| `roma_traced_448_672.pt` |1055079444|Listing only; bytes/hash/TorchScript interface not qualified. |

The inspected dataset listing contains no INOUT-TRANS,TIR-NOISE-IN,NIGHT-OUT
or GLARE-OUTbag. The [paper IV-A/TableI](https://arxiv.org/html/2606.29757v1)
identifies natural transition/noise conditions and LiDAR-based reference generation;
that does not establish that all corresponding exports are present in this folder,
or certify this supplied reference's frame, time units or uncertainty. The
well-lit filename is a candidate control label, not newly qualified physical
condition provenance. No sensor image was decoded or new pose estimated.

### Calibration and replay contract

Repository defaults bind RGB-left and thermal-right,640×512RadTan, with two
camera-to-IMU matrices. Both pass finite/homogeneous/SO3structure checks. The
**downloaded sidecar adds RGB-right and thermal-left**, plus LiDAR extrinsics.
All four camera matrices and the LiDAR rotation pass structural checks. Preserve
thermal-left skew0.642882; do not silently zero it or substitute the default
thermal calibration when switching a control to RGB-right. This resolves the
initial two-camera inventory limitation; actual bag/sequence association,
rectification and physical accuracy remain unqualified. No calibration was fitted.

The NUC source uses `ros::Time::now()` in callbacks/timers,25fps and1.5missed
periods (60ms threshold),3s warmup and10RGBbuffer slots. The provided launch does
not set `use_sim_time`; the README requests normal-rate playback. Future tests
must bind playback rate, clock mode, delivery/queue delays and header timestamps.
Changing playback speed or starving callbacks must not be mislabeled physical
NUC. This is a replay requirement inferred from source, not an observed defect
or a new recovery mechanism. The inspected host and existing ore-msGPUimage lack
`rosbag`,`roslaunch` and`rospy`; no ROS1odometry build/run was attempted.

### Coverage and concrete next input

A bounded local filename check found no matching release/transition names in
`ore-ms/data` or the artifact directory. Box's276directory-name entries at depth2
identify no such release; archive payloads were not re-inspected, so absence
from all backups is not established. Earlier local availability/fixed-camera
findings remain authoritative for their different populations.

Connector discovery surfaced an uninstalled SharePoint plugin, without verified
consumer-OneDrive support; none was suggested, installed or granted permissions.
The anonymous browser fallback resolved listing/small-file access. Its tools were
isolated under `/tmp` and throwaway containers, leaving the exact research image
unchanged. Failed HTTP/browser attempts remain preserved rather than rewritten
as an owner-authentication requirement. No login or author outreach occurred.

The user was asked for an existing local/Box copy or mirror. The useful missing
inputs are the natural adverse-condition bags and their matching references and
metadata, especially INOUT-TRANS andTIR-NOISE-IN. Once supplied, inventory all
sequences/topics/reference bindings, qualify an isolated ROS1runtime and the
published multispectral/single-modality controls, and freeze the whole selected
population before inference. Do not download/rebenchmark the large day control
alone as evidence of a new transition failure, synthesize a missing condition,
or train a model to fill this data gap.

Raw public listings, small files/hashes, source pins, structural audits, runtime
checks and access attempts are under `results/natural-transition-readiness/`.
Verified packet: `/root/ore-ms-artifacts/natural-transition-readiness-2026-11-16/`.
No new inference, full bag/weight download, Git publication or Box upload occurred.

## What the local failure inventory actually supports

| Candidate | Measured local evidence | Contribution decision |
|---|---|---|
| More fixed thermal weighting or decoupled RGB refinement | Extra RGB optimization explains the large position gain; longer-day thermal increment is0.140%against strongestRGB with rotation/CPU costs; longer-night increment regresses. | Close this branch as a new-method search. Keep short-day6.4%thermal gain as supporting evidence, not a universal negative. |
| Another adaptive camera-choice gate on the same windows | EXP127 reports zero natural availability headroom over the better fixed camera in18windows/7recordings. EXP129 also has zero conditional headroom in its4provided-export windows. | No new availability-gate fit on those cohorts. These counts do not constrain all data, pose accuracy or a changed estimator. |
| Recovery from the known RGB collapse | EXP132 pair96: RGB rotation discrepancy50.187674deg versus thermal1.337599deg. Existing simple rules already select thermal. A recording-wise hindsight fixed choice explains98.431%of pooled oracle rotation headroom. | Large failure is real under the supplied reference, but not an unsolved failure of the strongest simple controls. Do not rediscover it as a learned recovery success. |
| Learned task-aware point selection | Prior pose-reward policy improves night against match-reward, but loses the full advancement bar and does not beat pose-weighting; native full pool is more accurate in all3families. | Keep the positive ablation; do not repeat another small selector/head as a new direction. Full-pool context is not compute-matched. |
| Cross-match covariance prediction | Large empirical dependence under the specified half-LSB generator; predictive transfer is mixed and changes no pose estimate. | Interesting modeling evidence; not yet a demonstrated physical uncertainty failure or downstream pose benefit. Another covariance fit needs a genuinely different mechanism/target. |
| Thermal supervision/representation or cross-sensor adaptation | Controlled U²Flow assay succeeds; natural provided-geometry targets retain visibility/timing ambiguity; sensor-transfer inventory does not qualify physical cross-sensor claims. | Broad directions remain open. Neither missing dense labels nor failed local recipes rule out self-supervision; no specific unmet failure is established here to justify immediate training. |

Sources: [availability inventory](exposed-data-adequacy-findings-2026-11-03.md),
[provided-export survey](provided-export-survey-findings-2026-11-04.md),
[signal transfer](signal-transfer-findings-2026-11-04.md),
[query-policy learning](selection-policy-learning-findings-2026-11-14.md),
[covariance transfer](covariance-transfer-findings-2026-11-15.md),
[thermal target diagnostic](natural-flow-geometry-findings-2026-11-13.md),
[sensor evidence](sensor-transfer-readiness-2026-11-14.md), and
[full-span day fallback](long-day-fallback-findings-2026-11-15.md).
These populations and targets differ; do not pool them or infer one common cause.

## Relevant prior work and reading limits

A newly inspected primary source is [Cross-Spectral Stereo Inertial Odometry,
arXiv2606.29757v1](https://arxiv.org/html/2606.29757v1). Targeted reading covers
III and IV, not a full-paper/figure review or reproduction. It already combines
cross-spectral matching, reliability weighting, asynchronous estimation and a
thermal-dropout policy. Its evaluation includes transition and thermal-noise
conditions; its own results retain tradeoffs against RGB. Those generic mechanisms
cannot be proposed here as unexplored. Its metric/sensor/evaluation setup differs
from our cached visual-only studies; raw percentages are not comparable.

The [released repository](https://github.com/seungsang07/cross-spectral-stereo-inertial-odometry/tree/fd4d81d7dd61cd50bda8571b38da0f6e8f90d5b6)
is pinned at `fd4d81d7dd61cd50bda8571b38da0f6e8f90d5b6`.
README, NucMonitor source/header and targeted VioBackend/FrameIngestor paths were
inspected read-only. Code uses arrival-gap monitoring and buffered RGB ingestion;
VioBackend selects one active camera during NUC. A cached thermal image appearing
in ingestion is not by itself evidence that stale thermal factors remain active.
No implementation defect, runtime success or unresolved recovery mechanism is
established by this bounded source inspection. README links matcher weights and
ROS1bags; downloads, reference payloads, runtime and metric adapters are unverified.
README reports a2026RA-L publication; publisher metadata was not independently checked.

Earlier [multispectral failure recovery](https://open.metu.edu.tr/handle/11511/117644)
also covers inertial support during visual loss. This pass reads its institutional
abstract/metadata only, not full methods/code. Prior MSO Chapter5 and MAC-VO/DNLS
reviews remain relevant; no new full reading or reproduction is asserted for them.

## One finite gate before a new method

The next useful task is **baseline and evidence readiness for natural transitions**,
not an accuracy run or a new gate implementation. The new release's transition
and thermal-noise sequences are candidate sources, not an established new gap.

1. Confirm actual assets, calibration, timestamps, reference generation and sensor
   inputs. Inspect metadata first. Do not assume a README download link proves
   access, independent ground truth, licensing scope or benchmark readiness.
2. Pin and qualify the published multispectral implementation and its single-modality
   controls before a method proposal. Treat RGB/thermal/IMU and visual-only bundles
   separately; do not credit inertial robustness to a thermal innovation.
3. Identify an existing, repeatable failure on complete natural sequences. Require
   benefit beyond best fixed-camera, simple support/fallback, extra-optimization and
   applicable published fusion controls. Report continuity and recovery latency as
   well as translation,rotation,coverage and actual end-to-end cost.
4. Only then state one mechanism that differs from inspected prior art, with a
   deployable input signal, independent evaluation population and predeclared
   practical endpoint. Freeze numerical thresholds before evaluation, accounting
   for reference uncertainty and run variability rather than choosing a paper-sized
   percentage after outcomes.

Stop this direction if the strong published/simple controls solve the candidate
failure, the data contain no useful complementary transitions, or the required
reference/runtime evidence cannot be qualified. That closes this candidate,
not the broader original-paper gaps. Revisit representation, supervision, sensor
adaptation or persistent uncertainty only with a distinct evidenced question;
do not funnel every direction back into another weighting experiment.

## Environmental coverage qualification

The [smoke/darkness intake](smoke-darkness-readiness-2026-11-15.md) makes an important
limit explicit: our smoke evaluation was absent and night captures did not certify
zero illumination. The small daylight gains do not settle thermal utility under
smoke or darkness. Real-smoke thermal-stereo operational assessment is the current
priority, with supplied-reference and sensor-layout limits kept explicit. This
extends environmental coverage without claiming a new fusion mechanism or using
estimated poses as independent truth.

## Current deliverable and preservation

No immediate new-method go-ahead is justified. The concrete result is this
contribution matrix, newly pinned prior-art source and an explicit baseline-first
entry gate. A characterization report can preserve existing findings, but its
sufficiency for a thesis/publication is not assumed and it is not the sole research
objective. The [original-paper gap map](original-paper-gap-map-2026-11-13.md)
remains the broader scope reference.

Primary-page snapshots, five code-file hashes and local evidence pins are kept
outside Git under `results/contribution-decision/`. Paper registry reading events
retain exact depth. No estimator was built/run, no new model trained, no new data
or protected recording inspected, and no numerical experiment launched here.
