# Unchanged FlashLight matcher and RGB-first transfer protocol

16 November 2026, before FlashLight matching predictions. Follow the completed
[input qualification](flashlight-qualification-findings-2026-11-16.md) and reuse
prior source reading. This tests deployment characteristics of existing baselines,
not a new selector or author-system reproduction. No new paper reading required.

## Locked population and hypothesis

Run every declared541 anchor /1,082 sensor pair, both SIFT and learned frontends:
2,164 rows including unsupported inputs. Preserve all failures, three startup
reference exclusions and the initial unsupported thermal input. The pre-existing
538 common-reference,192 paired rotation≥2° and519 paired displacement≥1cm
cohorts are fixed before this study. Keep all anchors and fixed thirds by original
index (floor(index*3/541)); do not select successful prefixes or darker windows.

Hypothesis: thermal can add accurate proposals where RGB returns no valid output,
with regime-dependent benefit and added matching cost on a separate recording.
No direction of sensor ranking is required. Inspecting qualification images showed
lit samples, so this is not a certified zero-illumination experiment. Generalization
beyond this recording and training overlap are qualified.

## Fixed computation and decisions

Use native grayscale inputs and own SP1 camera calibration from the qualification
freeze. Models: existing `models/macslam/MACVO_FrontendCov.pth` for RGB and
`models/macslam/MACT_Offroad.pth` for thermal, no weight changes. Reuse frozen
`deployment_lighting` native-size inference,4,096-point grid and≤1px bidirectional
cycle gate; SIFT creation and mutual0.75ratio matching from `smokeseer_sift`.
Use the same calibrated MEI/pinhole ray lifting to a virtual focal400 camera;
RANSAC1px/0.999probability/1,000iterations, fixed RNG0, ≥8 cheirality inliers
for status `ok`. No keypoint, flow, confidence, pose or model threshold changes.

Request RGB first; retain any `ok` proposal. If absent, request thermal only when
its input is supported, retain it iff status `ok`, otherwise abstain. Reuse pure
`smoke_rgb_first_fallback.choose`, not reference values. Fixed RGB and fixed thermal
are controls. No replacement of accepted RGB, identity insertion or new gate.
Lock full predictions first, then sensor choices before pose-error scoring.
Scoring-only reference data must not enter inference or selection. Every selected
proposal keeps sensor, source/target paths and own exact export stamps. Conjugate
rotation into IMU axes with its camera→IMU rotation; keep translation unit direction
in selected target camera. Unknown scale prevents metric lever-arm transport.
No mixed-frame SE(3) trajectory or stateful VO recovery is claimed.

Primary endpoint: R≤1° success on the fixed192 moving anchors, missing outputs
count as failures. Secondary: R≤5° there and joint R≤5°/directed-t≤15° on519
translation-qualified anchors; report same endpoints on all/common/fixed thirds,
with eligible denominators explicit. Retain conditional errors and every addition:
thermal attempts, valid outputs, correct/incorrect additions and no-output attempts.
A hindsight either-correct union is a bound only. RGB-first success is monotonic
against RGB by construction; wrong added estimates may harm a persistent estimator.

Prespecify rotation-only RGB-or-zero diagnostic: retain accepted RGB rotation,
otherwise zero rotation evaluated at RGB stamps. No translation proposal or extra
matcher calls. Report1°/5° success, including moving/common cohorts. It is not a
fourth full-pose fallback or a substitute for abstention safety evaluation.

Cost primary: exact pair matcher calls, learned call includes both temporal
directions. All failed thermal attempts cost a call; unsupported inputs do not.
Secondary: sum recorded RGB pair seconds plus thermal seconds when requested,
as a warm cached projection. Explicitly synchronize CUDA timing boundaries.
No live lazy-schedule latency, resident memory, startup, sensor energy or real-time
VO claim. SIFT cache reuse differs under lazy scheduling, so time projection is
especially approximate. The two full control arms are computed for evaluation,
not claimed as an actual lazy execution run.

## Qualification, audit and stop

Use exact image `ore-ms:geometry-compiler` SHA256
`44ff9a1e6fa95d220437714e5be9b94f0f73c8eaebd4fcae3f843ceea14bef09`.
Before execution verify all14,715 gray hashes, qualification completion/audit,
calibration, model and consumed source/protocol pins. Preflight all1,082 intervals;
reference-free matcher-input rows omit reference poses/cohort labels. Qualify both
native image sizes with bitwise repeated flow and unchanged model state; qualify
camera roundtrips against unchanged SmokeRoom lifting at its own calibration and
known synthetic rays at FlashLight calibration, plus selector truth table.
Freeze qualified runtime/source before full inference. Use deterministic cuBLAS
`:4096:8`; preserve failed attempts instead of overwriting evidence.

The generic export comparison adapter adds configurable calibration, independent
matcher-only inputs and unified prediction/choice/score locks; it composes unchanged
primitives rather than copying frozen hard-coded SmokeRoom runners to change paths.
An independent auditor reconstructs pose errors, decisions, frame/time bindings,
cohorts, cost counts, all summaries and identity control without importing runner
scoring/selection. Stop after this transfer and preserve positive/null/negative
outcomes. No tuning, axis/clock fit, model search or extra budget follows.
