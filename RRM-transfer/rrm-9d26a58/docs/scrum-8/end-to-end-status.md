# Goal-to-finish performance status

As of 2026-10-13. This page summarizes measured scope; [HANDOFF.md](../../HANDOFF.md)
retains the detailed dated evidence. The six stages in [WORK_PHASES.md](../../WORK_PHASES.md)
are a revisable roadmap toward RRM's overall purpose, not a separate requirements baseline.

The 2026-10-09 runtime checkpoint: Office's opt-in requestable writer acquires actual
drone-camera RGB/segmentation pairs; one native pair independently labels both
markers VISIBLE and passes the assessment pairing gate. Initial native detach crash
is retained; post-render lifecycle recovery captures/detaches three frames, with
healthy readiness/GUI and disarmed inactive robot afterward. Same-render bridge clock
now gives exact ROS stamp/RGB correspondence for three captures without equating
ReferenceTime. Camera-frame reconciliation corrects a live tilt mistakenly cached as
authored body rotation and updates stereo offsets to the pinned ZED asset. Latest
three-frame mounting residual is ~2µm/0.00053°; the mount check passes. Physical-base
versus ROS map-frame base discrepancy is8.44–8.47cm/1.05–1.06° in this episode;
world-camera consistency still fails unchanged3cm/2° bounds (~8.4cm/1.06°). Map
binding remains blocked, console UNKNOWN and seven Office cases PREPARED_NOT_RUN. Same pinned image;
no learned inference or flight. Native teacher pairing is not learned goal success.
The [world/map binding checkpoint](world-map-binding.md) records a read-only source
and live-state audit. An approved grounded Office capture in a new episode found
8.47–8.54 cm rendered-versus-ROS world-camera translation residual in three frames,
while camera mounting remained within its existing limit. A separate 77-pair
exact-stamp grounded base comparison spanned 7.06–9.14 cm. Neither result establishes
a world/map transform or permits console admission.
In a second fresh Office boot, three native frames still exceeded the frozen 3 cm
world-camera translation gate (6.20–6.26 cm), while physical camera mounting
remained about 2 µm. Its early and late grounded base traces retained 64 and 63
exact-stamp pairs with 5.50–8.66 cm and 5.37–7.44 cm translation residuals.
The changing residual across boots and within the second boot is consistent with
an estimated local-pose offset, but no causal estimator diagnosis or multi-pose
world/map calibration exists. GUI frames advanced and showed both markers; console
binding remains UNKNOWN, mission IDLE, and all seven cases PREPARED_NOT_RUN.
An approved disarmed offset-spawn check then compared the origin with fresh
X=0.75 m and Y=0.75 m Office boots. Physical displacement was about 0.75 m on
each chosen axis, while ROS local displacement was +0.000095 m on X and -0.029117 m
on Y. Rendered-versus-ROS camera translation residuals became 73.34–73.36 cm
and 76.30–76.31 cm. The declared identity ROS world/map TF cannot be used to map
the simulator's marker coordinates to the estimator-local pose across these boots.
Separate stationary episodes do not establish a motion-stable transform. No
flight, target binding, learned inference or console admission followed.
The [proposed frame contract](world-map-frame-contract.md) treats ROS `world` as
the current local-map alias and Isaac stage positions as privileged assessment
data. No operational transform is implemented. The later target-provenance gate
holds static manifest waypoints; the visual v4 contract excludes localization
claims from 2-D image evidence. Neither creates an operational map target source.

## Demonstrated scope and remaining gaps

| Area | Retained evidence | Limit |
| --- | --- | --- |
| Recovery capture | The 2026-10-06 STOP/fresh-state/separate LAND checkpoint covers the 11.619 s LAND with 99 physical samples and all 18 control streams; final grounding/disarm is verified. | Earlier descents have capture gaps. A later 1.046 s gap lies outside the covered LAND. Assess the complete lifecycle with these boundaries visible; stable hold and broader capture/control reliability remain open. |
| Visual grounding | Real GUI checks pass. Latest frozen v3 prompt check on the same color-qualified development frames identifies both markers (6.564 s, 2/2 complete identities); blue-absent frame produces valid clarification (3.490 s). Separate teachers and exact prompt/source/raw binding retained; earlier v2 failures remain recorded. | Absent-blue: 0/1 visible identities; no false blue-presence claim, but clarification is not successful orange identification or absence verification. Two development frames and simplified emissive lighting do not establish generalization, Office appearance, uncertainty calibration, localization or live freshness. Temporary services stopped; default worker remains old. Office binding UNKNOWN; seven cases PREPARED_NOT_RUN. Model/dependency/OCI provenance unresolved. |
| Learned execution | Historical Cosmos proposal/import/public-task integration exists; a learned Office navigation attempt timed out and recovered by landing. Latest frozen INFERRED C02→qualitative inspection probe returns a learned blue-target navigation plan (7.621 s). Capability router reports unsupported INSPECT; adapter holds for missing map binding. | Parser acceptance is not goal/effect correctness: NAVIGATE_TO alone does not implement inspection. No physical feasibility or safety admission, execution or qualitative learned target achievement demonstrated. Frozen replay is not fresh live state. Deterministic movement-parser successes are separate evidence. |
| Observation/replanning | Fresh-state reacquisition, inter-action CONTINUE/SKIP_SATISFIED/HALT and bounded stop/recovery paths have narrow trial evidence. | General learned semantic replanning under changed targets and execution failures remains unverified. |
| Comparative evaluation | Mock acceptance, strict replay and matched same-implementation reports exist. Latest retained mock exports each match 34/34 expectations, with 32/34 complete attempts; intentional trace loss stays UNKNOWN. | These are component/instrument results. Matched learned/oracle/baseline/ablation campaigns and defensible integrated distributions remain outstanding. |
| Embodiment coverage | Neutral contracts/profile routing and isolated Kuka-Allegro calibration, gateway and bounded joint-action evidence exist. | No shared-core semantic GRASP/PLACE task has completed through live reasoning, admission, actuation and verified effects. Other profile declarations are preliminary. |

## Control and measurement questions

The retained pre-LAND height excursion remains unresolved. A tracking TF rejection
reset PID integrals and emitted idle thrust before the dip; the TF cause and
counterfactual physical causation are not established. The source has bounded TF
lookup diagnostics; their current deployment must be checked before drawing new
controller conclusions. Fresh external authority alone does not establish
uninterrupted internal controller admission.

Earlier failed takeoffs, nominal replays, STOP trials and clock/pose measurements
remain in the [handoff](../../HANDOFF.md). Their source/image/epoch and raw-capture
limits matter: receipt timestamps do not prove sensor acquisition alignment, and
odometry is not independent physical truth. Passing dependency/readiness checks
does not establish scene physics or flight robustness.

## Meaning of finish and performance claims

An action-server SUCCEEDED response is insufficient. Successful task completion
requires fresh independent evidence of the requested effects. Landing after a
failed goal can verify recovery while the requested task remains failed.
Missing, stale or contradictory evidence remains UNKNOWN and blocks unsupported
success claims.

A performance claim requires frozen scenarios, source/model/scene/controller identity,
seeds, complete exported evidence and independent assessment. Include every failed
or incomplete attempt; distinguish deterministic parsing, learned reasoning,
route planning and physical execution. Report success/recovery/grounding outcomes
and latency distributions with their actual coverage and uncertainty.

Use the [evaluation protocol](evaluation.md), [visual protocol](vision-evaluation-protocol.md)
and [contract allocation](architecture.md) for the detailed criteria. Current scope
is strong component coverage and a narrow partially demonstrated aerial loop;
integrated learned reasoning and cross-embodiment performance remain open.
