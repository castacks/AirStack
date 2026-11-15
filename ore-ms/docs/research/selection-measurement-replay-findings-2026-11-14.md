# Query selection changes pose even with dense measurements held fixed

14 November 2026, America/New_York. Branch `ore-ms-orb-development`, baseline
`a79658f6341194b0954852226f200313ccdf5408` plus frozen working sources.

The [paired frontend assessment](paired-frontend-assessment-findings-2026-11-14.md)
measured increased variability when native query selection is regenerated. This
follow-up identifies what happens when those saved query sets are applied to the
**same unperturbed dense fields**. Selection alone produces substantial spread;
rain additionally has substantial within-query measurement sensitivity and
interaction. This is mechanism evidence within three exposed pairs and the
existing controlled half-LSB ensemble, not a new selection or covariance method.

## One crossed replay

Let M0 be baseline dense flow/covariance fields and St the native source-query
set saved for perturbed draw t. The original study already supplies T(M0,S0),
T(Mt,S0) and T(Mt,St). We add only T(M0,St), with the same packet construction,
covariance propagation, identity initialization, IRLS solver and stopping rules.
No neural inference, raw-image perturbation, new frame, reference-pose reading,
training, noise sweep, selector change or fitted scale is involved.

The query intervention includes identity and count changes. Native downstream
range/covariance-sanity checks are recomputed against M0. Thus it is not a
count-matched identity-only test or a clean isolation of NMS versus thresholding.
The dense fields are held fixed everywhere; the selected means/covariances still
change because the queried locations change.

All 390 added fits converge: 128 draws plus baseline/null for each condition.
Every original draw remains in the complete matched population. All six
baseline/null packets, poses and covariance readouts reproduce the original
baseline bitwise. Selected point ranges remain day 132–155, night 51–72,
rain 36–49. No solve failure or censoring explains the result.

## Result

|Case|Measurement changes, original queries: mean translation shift mm|Query changes, original measurements: mean translation shift mm|Both changes: mean translation shift mm|
|---|---:|---:|---:|
|day|0.665|2.115|2.293|
|night|1.313|1.698|2.117|
|rain|256.332|208.978|366.780|

Shifts are norms of baseline-relative left SE(3) Log translation components.
The baseline is an estimator output, not true camera motion. Conditional draws
from one pair are not independent recordings. Selection-only mean rotation
shifts are 0.004821°, 0.003569° and 0.137891° respectively.

![Crossed replay](../../results/selection-measurement-replay/run-002/selection-measurement-replay.png)

In a single baseline-relative Log tangent, define A as the measurement-only
vector, B as selection-only, Y as both changes and I=Y−A−B. This gives an exact
coordinate decomposition, including finite-motion/nonlinear effects in I. It
is not a distribution-free additive causal attribution on SE(3).

Using the same original baseline information for whitening and ddof=1 centering:

|Case|Var(A) trace|Var(B) trace|Var(I) trace|A/B cross trace|A/I cross trace|B/I cross trace|Var(Y) trace|
|---|---:|---:|---:|---:|---:|---:|---:|
|day|0.021032|0.110404|0.000880|0.003792|−0.002103|−0.000637|0.133369|
|night|0.028607|0.053434|0.001424|0.001509|−0.001596|0.001272|0.084649|
|rain|3.652078|2.659085|0.626965|−0.101906|0.085503|0.531426|7.453151|

Each cross term is Cov(U,V)+Cov(V,U). The six terms sum to Var(Y); relative
matrix reconstruction errors are below 3.5e−16. Selection-only/full trace ratios
are 0.828, 0.631 and 0.357. **These are variance-magnitude comparisons, not
percentages of explained variance:** cross terms are correlated and can be
negative. Full arrays and all six covariance matrices are retained.

Day/night show a large standalone query-policy effect with relatively small
interaction. Rain is different: measurement-only spread is already large, and
interaction is appreciable (mean interaction translation-coordinate norm 90.97
mm). A single statement that selection explains all rain sensitivity would be
unsupported. A scalar inflation fitted to these three cases is not justified.

## Verification and retained runtime correction

The independent audit passes **7,575 checks** across all 390 fits. It checks
saved-set provenance, baseline-field sampling, range admission, independent
NumPy kernel/3D covariance arithmetic, stationarity on exact serialized inputs,
normal-information inverse, analytic SE(3) Log values, all decomposition terms,
trial availability and original measurement/full variance reproduction. Packet
and stationary-fit tolerances are unchanged from the preceding study. It does
not rerun or reaudit the original perturbed network outputs; those inherit the
preceding retained audit.

The first partial replay was interrupted before scoring because its wrapper
omitted the original deterministic helper/four Torch CPU threads. Its partial
journal, source and freeze remain retained. Complete attempt002 restores that
original runtime call. It changes no data, native arithmetic, query set, solver
or scientific threshold. All complete-attempt source/input pins were frozen
before those fits; no audit amendment was needed for attempt002.

Protocol: `experiments/selection-measurement-replay-freeze-2026-11-14.json`.
Raw artifacts: `results/selection-measurement-replay/`, complete `run-002`.

## Relation to the original-paper gap and stopping decision

This refines V3/I2/I4 into a measured question about the uncertainty target of an
estimator with data-dependent query selection. It does not establish a missing
mathematical term or an undisclosed assumption in MAC-I²: its [Appendix B](https://arxiv.org/html/2609.07116v1)
uses a stationary local Gaussian/Gauss–Newton approximation. Generic rematching
sensitivity is already discussed by [Bonnabel, Barczyk and Goulette](https://arxiv.org/pdf/1410.7632);
their ICP conclusions do not transfer as a theorem about this learned frontend.
This source pass rechecked those specific passages, not full papers or systems.

The contribution here is a reproducible mechanism assessment: under unchanged
native dense predictions, data-dependent queries alone can materially change
this thermal pose harness. The result is stronger than a correlation between
query turnover and pose spread. It remains narrower than a complete native VO
or MAC-I² reproduction, physical sensor calibration or evidence of accuracy loss.

Stop this finite attribution assessment here. Do not automatically sweep NMS,
match counts or jitter, train a stability head, or treat the original-paper gap
as solved. Before selecting a correction, a separate contribution decision must
specify its deployment objective and evaluation target, distinguish identity,
count and geometry if claimed, and establish a specific prior-art difference.
Other original-paper directions—including thermal supervision/transfer,
continuity/recovery and persistent estimation—remain open.
