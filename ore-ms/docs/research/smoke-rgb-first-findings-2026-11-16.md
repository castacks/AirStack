# SmokeRoom fixed RGB-first fallback findings

16 November 2026. Completed [frozen protocol](smoke-rgb-first-protocol-2026-11-16.md).
Existing-baseline characterization on exposed cached predictions; zero new
inference, tuning or reference-informed switching. Prior source reading reused;
no new paper reading or author-system reproduction.

## Result and deployment meaning

Thermal provides useful backup during moving RGB outages, mostly at the coarse
rotation bound. It does not turn this simple fallback into a uniformly accurate
or cheap system. On the same 236 anchors where both cameras' reference rotation
is ≥2°, the learned frontend gives:

| Policy | Valid proposal /236 | Rotation error ≤1° /236 | Rotation error ≤5° /236 |
|---|---:|---:|---:|
| RGB | 221 (93.6%) | 151 (64.0%) | 181 (76.7%) |
| Thermal | 231 (97.9%) | 116 (49.2%) | 190 (80.5%) |
| RGB-first, thermal on missing RGB | 236 (100%) | 155 (65.7%) | 192 (81.4%) |

All 15 requested thermal attempts in this cohort produce a proposal: four meet
1° and eleven meet 5°. Thus eleven additions exceed 1°, and four exceed 5°.
Conditional median rotation error changes from RGB's 0.2290° to 0.2585° for
fallback. Filling every output gap does not imply filling it accurately.
Unconditional success cannot decrease because accepted RGB is never replaced;
a new wrong proposal could harm a persistent estimator more than abstention.
This replay does not measure that risk or full VO recovery.

All 15 moving fallback events lie at nominal 290.5–306.5 seconds. The first,
middle and last event images show obscured RGB views while thermal retains scene
structure. Images were chosen by event time, without error or appearance selection;
the cameras have different fields of view. Every gate-triggered moving event is
retained in `moving-fallback-events.json`. Motion and obscuration co-occur; these
are not density labels or a causal isolation of smoke from motion/dynamics.
Four coarse failures occur at 290.5, 291.5, 292.0 and 293.5 seconds. Thermal is
useful in this section, but is not immune to failure.

A separately declared **post-replay rotation-only diagnostic** retains accepted
RGB, otherwise emits zero rotation. It adds zero successes at either bound on
these 236 moving anchors: all 15 missing RGB intervals have reference rotation
above 5°. Thermal's four/eleven additions therefore exceed this cheap static
assumption in this moving cohort. On all 837 common-reference anchors, RGB-or-zero
gets 734/782 successes at 1°/5°, versus actual thermal fallback's 452/507.
This exposes the static/small-motion mixture in whole-recording averages. Zero
rotation supplies no translation proposal and is not a main policy or complete
pose control; it was not inserted into the frozen replay.

## Whole population, translation and negative cases

On all 838 declared anchors, learned RGB/thermal/fallback emit 500/494/551
proposals. Their ≤1° success counts are 412/348/452 and ≤5° counts 460/453/507.
The first anchor has no common reference and no thermal input; reporting it
preserves the original population. On the 837 common-reference anchors, output
and success counts are identical. Of 337 requested thermal attempts, 51 return
valid proposals and **286 return none**. Among the 51 additions, 40 meet 1° and
47 meet 5°. The 287 total no-proposal anchors include the unsupported first input.

On the fixed 483 paired translation≥1cm anchors, nominal joint rotation≤5° and
directed translation error≤15° succeeds for RGB 349/483 (72.3%), thermal 250/483
(51.8%), fallback 356/483 (73.7%). Of 29 thermal attempts, 21 return proposals:
seven pass joint bounds and fourteen fail. Conditional translation-direction
median changes from 7.1745° to 7.3303°. Do not classify translation-ineligible
additions in the full population as translation errors.

In the fixed final third (279 common-reference anchors), RGB/thermal/fallback
≤5° successes are 169/193/186. Accepted wrong RGB outputs keep fallback below
fixed thermal here. Thirds are time bins, not smoke-density labels. On the 236
moving anchors, the hindsight per-endpoint union is 178 at 1° and 217 at 5°;
fallback achieves 155 and 192. This gap is an oracle bound, not an achieved
selection gain or permission to tune a new gate on these exposed outcomes.

Frontend dependence remains: SIFT RGB/fallback valid outputs are 162/198 on the
236 moving anchors. At 1°, RGB/fallback/thermal succeed 24/30/60 times; at 5°,
85/107/100. RGB-first is substantially worse than fixed thermal at the strict
bound with this weaker RGB frontend. All SIFT strata and failures remain recorded.

## Matching budget

Exact counts and projected time tell different parts of the deployment cost:

| Learned population | RGB calls | Extra thermal calls | RGB seconds | Fallback seconds | Projected increase |
|---|---:|---:|---:|---:|---:|
| All 838 original anchors | 838 | 337 | 233.82 | 333.74 | 42.7% |
| Same 236 moving anchors | 236 | 15 | 65.87 | 70.20 | 6.6% |
| Same 483 translation anchors | 483 | 29 | 134.56 | 143.06 | 6.3% |

One learned call includes both temporal matching directions. Failed attempts
still cost calls. The moving subset's small overhead is not the full-recording
budget: most thermal attempts in the full recording produce no valid proposal.
Times sum recorded parent per-pair seconds, not a live lazy-execution benchmark.
They omit model startup/residency, routing, memory, sensor power and stateful VO.
SIFT cache reuse additionally changes with a lazy schedule; its 12.6% moving and
26.2% common-reference projected increases are especially approximate.

## Reproducibility and limits

Runner: `tools/smoke_rgb_first_fallback.py`; independent auditor:
`tools/audit_smoke_rgb_first_fallback.py`. Existing pure fixed-preference helper
is source-pinned and reused unchanged. Qualification covers 21 synthetic selector
checks and 3,352 sensor rows; selection locks 1,676 decisions before score join.
Independent main audit passes 18,565 checks; separate identity/event/pin audit
passes 59 checks. Runtime is exact CPU image `ore-ms:geometry-compiler`, image
SHA256 `44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`.

Evidence is ignored under `results/smoke-rgb-first-fallback/`: freeze, qualification,
decision lock, all decisions/strata, summary/completion, audits/logs, diagnostic
plans/scripts/outputs and static figures. The incremental packet lives under
`/root/ore-ms-artifacts/smoke-rgb-first-fallback-2026-11-16/`; its manifest records
archive/member hashes and full readback verification. Parent raw data/checkpoints
are retained separately. No Box upload or Git publication.

Selected rotation is re-expressed in IMU axes by published camera→IMU rotation;
selected sensor and actual image-header timestamps remain explicit. Unit
translation remains in that camera's target frame: unknown monocular scale cannot
supply the lever-arm correction to the IMU origin. No mixed-frame metric trajectory
is constructed. The [nominal reference qualification](smoke-reference-pose-findings-2026-11-16.md)
and its fused-reference, exposure, origin, mostly-yaw gyro and training-overlap
limits all apply. No new algorithm or held-out generalization is established.

## Next finite step

Stop this replay without selector rescue. Transfer the same status-only rule to
an independent real illumination recording after whole-span camera/calibration/
reference qualification. The extracted SubT-MRS FlashLight release is a candidate;
its March timestamps overlap its supplied reference, whereas the raw ZIP appears
to contain a different July recording. Qualify its own calibration and inputs
before freezing matching; do not reuse SmokeRoom RC7 extrinsics or claim zero lux
from the recording name. Keep NGCLAB's unresolved mocap chain separate.

The independent-recording input stage is now [qualified with explicit limitations](flashlight-qualification-findings-2026-11-16.md); its [unchanged matching transfer is complete](flashlight-transfer-findings-2026-11-16.md), with no learned moving-cohort gain.
