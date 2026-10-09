# Office visual grounding evaluation

This supports stage 2 of the revisable [working roadmap](../../WORK_PHASES.md).
Visual evaluation requires independently labeled target identification, uncertainty
and unsupported/stale-evidence rejection; camera/API checks alone are insufficient.

Current evidence and API diagnostics live in [HANDOFF.md](../../HANDOFF.md) and
[measured status](end-to-end-status.md). Camera refresh and entity API availability
are demonstrated; v3 identifies both markers in one isolated development frame,
not general or Office grounding. The Office marker launcher is now active with fresh
actual stage observations and visible markers in the GUI camera. Console binding
remains UNKNOWN. Native Office camera labels and exact ROS clock/raster correspondence
are acquired in a bounded diagnostic. Camera mounting now passes its check; world/map
pose binding remains blocked. All seven cases remain PREPARED_NOT_RUN.

The opt-in frozen pairing gate in [visual_evaluation.py](../../rrm/visual_evaluation.py)
and [offline CLI](../../scripts/rrm_validate_visual_pair.py) checks supplied fixture
manifest/launcher/exporter identity, epoch/revision, exact camera/frame stamp/hash,
PNG header dimensions, complete entity/prim labels, visibility/regions and same-epoch
age. BOUND_FOR_ASSESSMENT means matching supplied records only, not label correctness,
decoded pixels, model provenance, score or execution authority. The caller must trust
the acquisition/exporter and clock; checksums are not authentication. This source-only
gate does not retrofit existing scorers. The active opt-in Office camera writer now
captures RGB/semantic pixels/camera parameters/ReferenceTime on the actual existing
drone-left render product. One native pair passes BOUND_FOR_ASSESSMENT with both
markers VISIBLE. It uses a native-render topic/clock, not an admitted ROS/GUI frame:
the exporter now additionally retains the bridge's same-render simulation-time
annotator. Three captures exactly match ROS stamps/RGB without rebasing ReferenceTime.
Authored body-frame correction and pinned ZED stereo offsets reduce the latest
mounting residual to~2µm/0.00053°. Physical-base versus ROS map-frame base discrepancy
is8.44–8.47cm/1.05–1.06° in this episode; world-camera consistency still fails
frozen3cm/2° bounds (~8.4cm/1.06°). Do not treat this as a calibrated origin or a
cross-episode estimator-error distribution. Physical reads are callback-phase,
not historical render state.
No alignment fitting, learned score or motion. An actual GUI frame remains BLOCKED
without its own independently valid registration/time/map binding.

The isolated [render probe](../../simulation/render_teacher_probe.py) and
[converter](../../rrm/render_teacher.py) acquired one actual static 480×300 marker
image with two independent semantic-pixel regions on 2026-10-08. One writer callback
binds RGB, semantic pixels and rational render-reference time; replay passes the pairing
gate. This uses a separate fixture/topic/epoch, not the Office drone camera. Missing
pixels yield UNKNOWN visibility, not assumed occlusion. The timing boundary follows [NVIDIA's ReferenceTime/writer API](https://docs.omniverse.nvidia.com/kit/docs/omni.replicator/1.12.27/source/extensions/omni.replicator.core/docs/API.html).

The teacher-free [frozen pilot boundary](../../rrm/visual_pilot.py) separately assesses
identity/presence after exact local raw-output re-parsing; pixel regions never verify
physical localization. The `--absent-entity blue_marker` variant independently
measures the omitted prim ABSENT while orange remains VISIBLE; missing pixels alone
stay UNKNOWN. Positive presence against measured ABSENT is counted separately.

Earlier malformed and v2 missing-kind/empty-output trials remain in the handoff.
A cached-processor replay shows distinct image tensors, not original attention or
failure causation. Predeclared semantic-region color gates pass on both emissive-v1
variants, yet v2 remains incomplete (0/2 positive, 0/1 absent-case identities).
Neither semantic labels nor fixture-color qualification establish learned grounding
or Office lighting robustness. Full historical timings belong in the evidence log.

The next frozen `visual-claims/v3` prompt uses explicit paired exists/kind instructions
and a generic fictional example without changing schema, parser or qualification.
One request per same development frame yields 2/2 complete teacher-supported
identities in the positive case (6.564 s), and valid NEEDS_CLARIFICATION with no
snapshot in the blue-absent case (3.490 s, 0/1 visible identities). The latter is safe
non-admission, not successful orange identification or learned absence verification.
No retries or teacher facts sent. These two reused frames are not held-out accuracy,
uncertainty calibration or readiness for motion. Default deployed service stays old.

The shared [identity qualifier](../../rrm/visual_world_builder.py) requires fresh
explicit exists and exact-kind TRUE facts without completing them from the catalog.
Pilot reports candidate status, evidence acceptance, candidate completeness and
teacher-supported identities; its legacy `schema_accepted` flag means ACCEPTED
evidence, so it is false for a valid NEEDS_CLARIFICATION response.
[Live-client source](../../rrm/cosmos_entity_verifier_client.py) additionally
requires its existing localized claim and matching task/episode/revision; this change
is not deployed in the console. These are evidence-contract checks, not independent
physical localization, unique target selection or sensor-freshness verification.
Synthetic absence/uncertainty checks do not count as learned Office cases.

This protocol prepares the Office evaluation and authorizes no motion. The current deterministic aerial command
mission and moving STOP evidence qualify the execution path for those trials; they
do not establish learned visual reasoning accuracy.

The versioned [cases](../../examples/office_visual_eval/evaluation/cases.json) cover
contextual target selection, ambiguity, absent targets, occlusion, displaced targets,
stale observations and integrity failure. The Office catalog limits candidate entity
IDs to `blue_marker` and `orange_marker`; catalog membership alone is not visual proof
of presence, visibility, location or reachability. Each prepared case must acquire
its own independently verified teacher labels before it can run. Do not use a
floor-level camera anchor as proof that either marker is visible.

## Observation and assessment lifecycle

1. Freeze scene/profile/controller hashes, seed, prompt/model revision, case revision,
   age budget and assessment rules before collecting comparative attempts.
2. Capture actual sensor image, capture identifier, image checksum, source/simulation
   timestamp and host receipt time. Store candidate-visible input independently.
3. Separately capture simulator teacher labels for entity identity, visibility,
   image region and/or pose, with frame registration and uncertainty documented.
   Teacher labels are for assessment; do not pass them as sensor evidence to the
   visual candidate. A ground-truth-context baseline is labeled separately.
4. Request proposal-only visual C02 grounding, retaining raw candidate response,
   latency, model/prompt hashes and its exact observation binding. Validate catalog,
   schema, evidence integrity, age and uncertainty before downstream reasoning.
5. Assess every attempt against frozen independent labels. Incorrect grounding,
   refusal, clarification, schema failure, timeout and incomplete evidence remain
   in the dataset with separate verdicts. No trial is removed because it fails.
6. In a later explicitly integrated trial, accepted grounding feeds learned C04/C05,
   deterministic embodiment feasibility and independent admission before a public
   task action. After action, obtain fresh observation, reconcile expected effects
   and update memory/replan. Terminal success requires observed goal effects.

## Metrics and closure gates

Report correct target grounding over all labeled attempts, false target groundings,
clarification accuracy, stale/broken evidence acceptances, completeness and
median/p95/max wall latency. Separate model response from validation and total
processing; also report observation age and dropped/duplicate frames. Response
latency is not sensor acquisition latency. Missing teacher/frame registration makes
that comparison evidence-incomplete, even when the model response looks plausible.

Initial closure is a small correctly reconstructed proposal-only pilot covering all
seven cases, including deliberate rejections. It is not a statistical success-rate
claim. The integrated S01–S10 campaign then uses the [evaluation plan](evaluation.md):
30 recorded seeds per supported scenario/profile, three repetitions per seed for
stochastic candidates, and held-out layouts. Report numerator/denominator and
confidence intervals; compare learned reasoning against a separately labeled oracle
under matched observations/configuration. Safety failures are reported independently.

Current status: **PREPARED_NOT_RUN**. Case teacher labels, actual learned responses
and matched benchmark results remain to be acquired. No new visual accuracy,
throughput or generalization claim follows from protocol/schema checks.
