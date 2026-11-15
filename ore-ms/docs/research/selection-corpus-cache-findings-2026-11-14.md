# Policy corpus and native cache qualified; training frozen

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus source-pinned working changes.

## Completed preparation

The [600-scene design](selection-policy-contract-findings-2026-11-14.md) is now
materialized, independently audited and cached through the unchanged native MACT
model. All candidate packets, reference rewards and training-only normalizations
are available. Weight-only finite-difference derivatives and execution/scoring
interfaces pass qualification. A source/input-bound training freeze is ready.
**No utility adapter has been fitted and no policy accuracy benefit is claimed.**

|Preparation|Completed evidence|
|---|---|
|Corpus|600 exact planned scenes; 60 existing textures; all three 240-train/80-evaluation folds preserved|
|Reference audit|22,934 checks; 393,216,000 view rays and 294,912,000 flow/visibility pixels|
|Native inference|600 primary three-pair batches plus one bitwise null repeat; 1,803 neural pair evaluations|
|Candidate cache|600/600 available, zero invalid packets; 62,765 candidates; 58–188 per scene, all above the matched 32-query budget|
|Cache/labels/statistics audit|12,701 checks; all 600 label packets and all three training-only normalizations|
|Weight derivatives|237 checks; six fixed fold/theta probes; 198/198 solves converge; maximum absolute implicit/finite-difference gradient difference 1.374e−6|
|Runner/scoring checks|Zero-utility parity and policy-shape smoke, zero optimizer updates; deliberate known-gain, lost-availability and zero-headroom fixtures behave as specified|

These counts describe reference and implementation verification, not learned
performance. The 600 scenes remain artificial developmental data using already
exposed image atlases; recording/motion/geometry exclusion is not a fresh physical
thermal benchmark. The broader original-paper directions remain open.

## Render and reference contract

Four rig views are saved per scene: source left/right and target left/right.
The full rotation-vector poses and foreground half-planes are exactly those in
the design. The right target center is t+R[baseline,0,0]. Compact NPZs contain
uint8 images, surface/atlas masks, exact pose/cameras and three dense source-to-
target flow/visibility maps. World points are reconstructed analytically rather
than stored redundantly. Bilinear sampling and 8bit rounding remain fixed.

The independent audit reconstructs closest permitted surfaces, camera geometry,
texture sampling and all three visibility maps. A projected point is eligible
only when it is in the canvas and agrees with the nearest visible surface along
the target ray, with the fixed reference-support convention. Occluded points do
not become correct match labels merely because their projection is in bounds.

The first partial render is retained. Independent preflight found 220 target-
stereo visibility disagreements on the first scene: equivalent SciPy/OpenCV
rotations rounded projected canvas-edge pixels to opposite sides of strict
bounds. Before any native inference, the reference projection was amended to
snap coordinates within 1e−9 pixel of the canvas boundary to that exact boundary,
matching the existing atlas convention. All 512 completed partial-attempt images,
surface IDs and atlas masks repeat bitwise in complete attempt002. Geometry,
textures, camera motion, model, features and training choices were unchanged.
The original source/freeze, partial artifacts, failed probe and amendment remain.

Completed corpus: `results/selection-corpus/run-002/`.
Render freeze: `experiments/selection-corpus-render-freeze-002-2026-11-14.json`.

## Native cache and label separation

The recovered MACT checkpoint loads strictly without migration and its model
state is unchanged. Native inference uses the existing prepared grayscale values
replicated across channels and the same L0/R0, L1/R1 and L0/L1 batch. Native
NoDepth requested200 selection, range/covariance admission and optical covariance
packet construction are unchanged. Canonical source(v,u) order fixes adapter ties.

Native capture opens image arrays, not pose/flow/visibility reference arrays.
Known rendering/manifest metadata are available; this is not a claim of blind
reference provenance. None of those labels or profile IDs is used to filter the
candidate set or construct the eight utility features. Predictions, packets and
raw features are locked before reward-label derivation. Reference-support masks
live in separate label files; statistics are independently recomputed from each
fold's successful training candidate packets only, never evaluation packets.

The first cache audit is retained as failed. Its target-UV backprojection used
NumPy float32 weak-scalar subtraction, while the producer explicitly converts
UV coordinates to float64. One of 363 coordinate components failed the original
rtol3e−5/atol1e−8 check near an optical axis. A separate auditor corrects that UV
cast; the original auditor remains frozen for reference auditing. All producer
outputs, labels, statistics and numerical tolerances are unchanged. Corrected
cache auditing then passes all 12,701 checks.

Caches: `results/selection-native-cache/run-001/` and
`results/selection-derived-cache/run-001/`.
The audit-only amendment is bound in
`experiments/selection-native-cache-audit-002-2026-11-14.json`.

## Weight-only derivative qualification

The first canonical training scene in each fold, the first deterministic uniform
32-point subset and two predeclared theta values are used. Two folds reuse the
same physical scene with different training-only feature normalizations; these
are six numerical probes, not six independent real-world examples. All probes,
parameter perturbations and solve receipts are retained.

Central differences at1e−3 and5e−4 are compared with independent implicit
differentiation of the actual IRLS fixed-point score. Its pose Jacobian includes
changes in residual Jacobians and rotation-dependent covariance; it is not merely
an inverse Gauss–Newton Hessian shortcut. Torch matrix exponentials differentiate
pose retractions. A convergent matrix-log series is checked against the analytic
SE(3) Log before differentiating the bounded pose loss.

All 198 solves converge under the original stopping rule. Both finite-difference
steps and the implicit derivative agree under the predeclared rtol0.03/atol1e−5,
with maximum absolute discrepancy1.374e−6. No solver tolerance, feature, loss,
probe selection or weight range was tuned after these outcomes. This is local
fixed-probe qualification, not proof that every future training perturbation
will converge. Training must keep every later failure with its declared loss.

Receipts: `results/selection-weight-derivatives/run-001/`.
Source/input freeze: `experiments/selection-weight-derivative-freeze-2026-11-14.json`.

## Concrete frozen next execution

`experiments/selection-policy-training-freeze-2026-11-14.json` binds 1,828 input/
source hashes, the completed qualification and these executables:

- `tools/train_selection_policy.py`: the three learned controls, 64 updates,
  eight parameters, identical scene schedules, fixed policy samples or weight
  finite differences; final theta only. Dense predictions remain cached/frozen.
- The same runner captures all evaluation predictions without reading evaluation
  reference arrays. Primary deterministic top-k and four stochastic contexts
  remain separate; no best-seed choice.
- `tools/score_selection_policy.py`: scoring only after a global prediction lock,
  with pairwise common-success primary comparisons, positive control-error
  headroom and no lost control-success scene. The original five-percent/no-
  rotation-or-availability-regression developmental gate remains fixed.

Native, uniform, information, task-weight-only, match-reward and pose-reward arms
remain in the comparison. Native full-pool and stochastic contexts are not silently
converted to matched-budget or best-seed results. A gate pass would advance this
restricted learning-target hypothesis; it would not by itself establish native
system superiority, physical calibration, natural transfer or algorithm novelty.

Status: **READY_FOR_FROZEN_TRAINING; ZERO_UTILITY_OPTIMIZER_UPDATES**. The next
work is the single fixed three-fold learning/evaluation run, retaining every
training/evaluation failure. No geometry, data, features, query budget, objective,
optimizer, checkpoint or seed search follows from its outcomes. Do not reopen
previous completed recipes or treat a failure of this one as closure of the
original papers' broader gaps.
