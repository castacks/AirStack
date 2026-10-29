# EXP-081: saved factor objective / geometry disagreement audit

2026-10-29 America/New_York. Branch `ore_ms`, starting commit `22465ee54`.

Lower residual energy can coexist with a worse pose proxy even within the same
day window. Decomposing covariance-weighted residuals, depth/spatial support and
conditional leverage does not isolate a single cause or a validated separating
rule. Keep these as diagnostics and preserve the unchanged history baseline.

## Declaration and scope

Before this decomposition, freeze all217 original deadlines/434 camera slots,
all331 deadline-available first motions, and all117 both-available intervals.
Do not choose the largest-error examples or drop successful selections. Use
four fixed target-forward-depth bins `[0,5),[5,20),[20,80),[80,infinity)` and all
nine row-major3x3 cells of the target observed image. Keep all factors and all
extreme inferred depths; no clipping, normalization fit, reoptimization, neural
inference, new images/reference or reserved data is used. EXP080 outcomes and
EXP079 summaries were already examined; this is a declared retrospective audit,
not unseen-data preregistration or causal identification.

Materialize geometry without opening outcome/reference/model files. Separately
join the frozen, independently validated EXP080 qualified nominal-body,
reference-assisted SLAM-proxy losses. For each original four diagnostic choices
and each separate translation/rotation loss, classify the chosen camera as
better/worse/tied versus the other expert using the inherited1e-12 tolerance.
All24 window×diagnostic×loss groups, all72 class groups including empty groups,
all IDs and936 classification records are preserved. Examples are always the
first chronological member of each nonempty class, not an outcome-selected
extremum. The same recording/reference/body-frame limitations still apply.

## What is decomposed

Reconstruct the exact saved reprojection/disparity factors at the native global
optimizer pose before the first writer, with the same fixed landmarks as
EXP079. Covariance retains its full2x2 UV cross term and separate disparity
variance. Whitened image residual energy is `r_uv.T C_uv^-1 r_uv`; disparity
energy is `r_d^2 / variance_d`. No missing correlation or calibration is invented.
Their sums reproduce all EXP079 whitened sums of squares for the eligible
motions. Factor identity/provenance and exact original motion-event/deadline
bounds are inherited and rechecked. Six terminal writes remain excluded.

Compute the whitened camera-right-tangent Jacobian `A` (translation m, rotation
rad). For the unrobust, undamped, fixed-landmark objective, leverage is the sum
of each factor's three diagonal entries in the column-space projector of `A`.
The producer uses its thin SVD; the independent validator uses finite-difference
forward perturbations and QR. Leverage sums to numerical rank6 in every real
motion. This conditions on exact landmarks, ignores nuisance/cross-observation
uncertainty and is not posterior pose confidence or convergence telemetry.
Translation and rotation information block traces are kept separately in
m^-2 and rad^-2; no mixed-unit trace is used as a score.

Every depth/cell partition retains counts, component energy, leverage and both
information-trace sums/shares. Zero-total shares are null. Fixed tail budgets
sum the largest `ceil(0.01*N)` factors, at least one, separately for energy and
leverage; factors are never removed. For the sparse18-factor RGB survivor the
one-factor budget is5.56%, so this nominal1% concentration is not directly
comparable as an identical sample fraction to200-factor cases. Equal-value
boundary ties do not change the sum; no tail-selected inference is performed.

## Descriptive camera populations

The table reports medians of per-motion descriptors, not pooled-factor medians
or shares of pooled sums. Motion/factor counts are exact. The single RGB middle
survivor is a selected sparse population; it is not comparable as99 independent
observations to the thermal expert. The full spatial partitions and separate
variance/information descriptors are preserved in the artifacts.

| Window / camera | Motions / factors | Median depth m | UV energy/factor | Disparity energy/factor | Near5m leverage share | Far80m leverage share | Top nominal1% energy share |
|---|---:|---:|---:|---:|---:|---:|---:|
| early/rgb |18 / 3598|3.9189|0.2492|0.8147|0.7127|0.0232|0.1217|
| early/thermal |18 / 3432|5.4562|0.0354|0.0467|0.5850|0.0526|0.1057|
| middle/rgb |1 / 18|1.2452|20.4375|7.9808|1.0000|0.0000|0.1241|
| middle/thermal |98 / 17541|9.5629|0.2534|0.0515|0.4311|0.1155|0.1312|
| day-middle/rgb |98 / 18391|12.6454|1.6788|0.9337|0.5203|0.1940|0.1597|
| day-middle/thermal |98 / 13656|9.1853|0.7921|0.1943|0.5118|0.1101|0.1795|

There are56636 factor blocks available at original deadlines; EXP079's57594
included958 additional terminal-write factors. The103 unavailable camera slots
are retained as null:97 unsupported/unwritten RGB intervals plus6 late terminal
motions. There are no empty or non-SPD eligible factor blocks silently dropped.

## Concrete chronological checks

For the first worse early-night reprojection preference (pair1), thermal has
median reprojection0.02905px versus RGB0.30268px. Its image/disparity weighted
energy per factor is0.02284/0.05264 versus RGB0.24785/1.58687, yet its translation
proxy error is0.003663m versus RGB0.003079m. Lower energies are therefore not
sufficient to order the proxy even within early night.

For the first worse day reprojection preference (pair0), thermal has median
reprojection0.22637px versus RGB0.51263px and much lower image/disparity weighted
energy per factor0.45393/0.13344 versus125.52413/15.03532. Its translation proxy
error is nevertheless0.04493m versus RGB0.02651m. The immediately following
first better day case (pair1) also prefers thermal with lower component energies,
but thermal's proxy error0.02746m is better than RGB0.04551m. The low-energy
ordering occurs in both classes; this is not repaired just by separating UV
from disparity normalization.

The middle-night both-available case is pair96: the sparse RGB estimate has
18 retained factors, all its conditional leverage below5m, and per-factor image/
disparity energies20.43754/7.98080. Thermal's are0.25922/0.02871. The known proxy
errors are RGB1.00308m versus thermal0.04029m. This is a useful diagnostic for an
already known failure, not evidence of a universal threshold.

For day reprojection selections, better/worse translation groups have40/58
intervals. Selected versus other camera median near5m leverage shares are
0.5113/0.5161 in the better class and0.5118/0.5239 in the worse class. Selected
median image/disparity energies are lower in both classes. For whitened-score
selection, the better/worse counts are33/65; selected median near5m leverage
shares0.5210/0.5047 and nominal-top1% energy shares0.1279/0.1595 do not establish
an independently validated separator. All other diagnostics, both losses,
empty/tied classes, descriptors and spatial/depth partitions remain in the
complete machine report; no direction or threshold is fitted from these groups.

## Interpretation and limits

The source builds each expert's landmarks from its own predicted source depth
and uses its own predicted correspondences/disparity as target observations.
A low fitted residual measures self-consistency with those same inputs. It does
not directly test an independent landmark position or motion truth. The
conditional leverage/information reconstruction also treats those model-derived
landmarks as exact. That source contract explains why objective fit cannot be
assumed to be physical confidence; this audit does not identify whether shared
model errors, reference errors, calibration, association or optimizer behavior
caused a particular proxy discrepancy.

Covariance scaling fixtures show that multiplying all covariances by4 divides
whitened energies and information traces by4 while leaving normalized leverage
unchanged. Actual cameras differ in both residuals and covariance; no claim is
made that a variance-scale change caused the observed camera preference. Saved
inferred depths remain unverified physical geometry, and selection-conditioned
summaries are not a causal decomposition of error. Native robust Huber/FastTriggs
behavior, solver damping and iteration history are outside this unrobust audit.
The observed descriptor overlap is not a proof that no richer predictor exists.

## Validation and reproduction

17 CPU tests pass: fixed bin/cell boundaries, conserved sums/shares, correlated
UV covariance, covariance scaling, independent QR/SVD leverage, full/deficient/
zero rank, zero residual/null shares, tail budgets and invalid depths/pixels/
covariance rejection. The separate validator imports neither producer nor outcome
analyzer. It reconstructs331 motions/56636 factors via independent forward
projection and finite-difference Jacobians, checks QR projector leverage and all
partitions/descriptors, and reconstructs all936 outcome records,24 groups,
class summaries and chronological examples. All checks pass.

A runtime guard blocks outcome/label/reference/model reads during geometry
extraction and produces identical geometry rows and frozen-input associations.
Nine semantic mutations with refreshed outer geometry digest are rejected:
depth count, leverage total, tail budget, descriptor, outcome class,
chronological example, missing interval, invented unavailable expert and cell
share. No historical artifact is changed. Protocol/source/input and outcome
hashes are checked separately, before and after geometry extraction.

Same CPU image as EXP079/080:
`sha256:ae59352813af040d1a60000f2b13eed990b8364f8a416f5b87e385d4496aade7`,
Python3.12, NumPy1.26.4, SciPy1.17.1, `OPENBLAS_NUM_THREADS=1`, `runc`,
`NVIDIA_VISIBLE_DEVICES=void`. No GPU/neural runtime or native solver is run.
Standalone pytest disables the repository's jaxtyping addopts.

From MAC-VO root, ore-ms mounted at `/workspace`, use a fresh output directory:

```bash
python3 -m pytest -o addopts= Scripts/UnitTest/test_factor_geometry_audit.py -q
python3 -m Scripts.Setup.factor_geometry_audit prepare --root /workspace --output <fresh-run>
python3 -m Scripts.Setup.factor_geometry_audit geometry --root /workspace --output <fresh-run>
python3 -m Scripts.Setup.factor_geometry_audit outcome --root /workspace --output <fresh-run>
python3 -m Scripts.Setup.validate_factor_geometry --root /workspace --output <fresh-run>
```

Authoritative output: `results/factor-geometry-audit/run-001/`, with protocol,
geometry, outcomes, independent validation, guard and mutation evidence,
`all-factor-partitions.csv`, `class-descriptors.csv` and
`chronological-examples.csv`. Recovery target `backups/2026-10-29-exp-081`;
dependencies are EXP079 factor audit, EXP080 predictions/outcomes and their
EXP073–078 raw expert/label/history provenance. Credentials, images and model
weights are excluded.

## Decision

The audit documents objective/geometry differences and concrete low-residual /
higher-proxy-error counterexamples, without isolating one failure cause. Retain
all diagnostics and the unchanged history control; do not tune a separator on
these windows or assert a physical-confidence improvement. Before another
selector, the next bounded check is the actual robust/weighted objective and
solver contract against these same saved factors, since the current summaries
are explicitly unrobust and condition on fixed landmarks. Missing convergence
telemetry must remain missing, and a native replay would require a separately
declared numerical-preservation check rather than an unqualified convergence claim.
