# Shared image-error modes: partial development transfer beyond a scalar

15 November2026UTC. Branch `ore-ms-orb-development`, baseline `a79658f63` plus
frozen working sources. Three recording-excluded development folds complete.
All six structured fits report optimizer convergence; all30predictive covariance
matrices are available. No neural inference, pose optimization or pose correction.

## Result and interpretation

The five-parameter shared-mode model improves Gaussian covariance scores over
the train-fitted scalar and same-capacity independent-mode control in five of
six scenes. It beats all three fitted predictive controls, including the pooled
full-shape control, in two scenes. At recording level, it beats all three on
day and rain averages, but loses on night. These positive and mixed cases are
both retained; there is no universal improvement claim.

This is a predictive extension of the
[cross-match covariance attribution](cross-match-covariance-findings-2026-11-15.md).
The earlier empirical dependence is not merely assumed to be a usable predictor:
models fit other recordings, then predict using only baseline geometry. The
increment is modest, bounded evidence that this structured family can add value
beyond train-only scalar scaling in some cases. It is not a new covariance
principle, physical calibration result, reliable general transfer or improved VO.

A test-fitted scalar oracle beats the common-mode model on all three recording
averages. Common-mode baseline-relative95%containment is only60.16% on day and
99.22% on night/rain. Thus the model's limited shape improvements do not establish
that it predicts the aggregate variance scale or tails reliably across recordings.
No mode, rank, bound, initialization or training-budget rescue follows.

## Frozen question, data and targets

The [design](covariance-transfer-design-2026-11-15.md) predates fitting and new
scores. Use the same six artificial known-pose scenes and128saved fixed-ID draws
per scene. Leave out one entire day/night/rain texture recording and both its
geometries; fit on the other four scenes. These are **previously exposed
recording-excluded development folds**, not a fresh/blind test, a network-training
holdout or a sensor-transfer benchmark. Each scene's draws share the same image
pair and do not represent independent physical recordings.

The target is
`y=leftLog(T_draw inverse(T_unperturbed))`, in the baseline translation-first
tangent. All models fix the mean to zero. Training and scoring use the uncentered
second moment around this unperturbed estimate; no evaluation mean is subtracted
or fitted. This differs explicitly from the earlier centered-covariance attribution.
Noise-induced empirical mean shifts account for1.12–2.56% of the observed
native-normalized second moment across scenes. Original true-pose bias/accuracy
is a separate quantity, not the target being calibrated here.

The input generator remains independent half-LSB prepared-pixel noise, clipping,
channel replication and view reuse from the old freeze. It is not a measured
camera noise model or the checkpoint's learned predictive-error distribution.
Covariance scaling for this perturbation target cannot be promoted to a correction
of the native published uncertainty or a physical fusion weight.

## Geometry-dependent model and controls

From the unperturbed packet and its frozen pose, define four residual derivatives
for constant temporal flowu,v and source/target positive disparity magnitudes:

    B_i = [R e_x*z1/fx, R e_y*z1/fy, X0/d0, -R X1/d1].

Use the qualified pose influence G_i. For each modej, the common basis is
`(sum_i G_i B_ij)(sum_i G_i B_ij)^T`; the independent-mode basis instead sums
`(G_i B_ij)(G_i B_ij)^T` over points. Both predict
`C=a P_native+sum_j q_j M_j`, with five positive coefficients. These are frozen
lookup/depth derivatives, not a full network or changing-association derivative.
Pixel/disparity units label the parameterization, not inferred physical sensor noise.

Baseline-only feature construction does not consume perturbed residuals or
empirical covariances. Whiten pose labels and bases with a fixed Cholesky factor
of baseline P_native. The five predictive controls are:

| Control | Fitted quantities and information |
|---|---|
| Native | Unchanged baseline P_native; no fit |
| Scalar | One train-only Gaussian MLE multiplier of P_native |
| Independent modes | Five fitted positive coefficients; no cross-match mode sharing |
| Common modes | Five fitted positive coefficients; four shared image-error modes |
| Pooled shape | Train-pooled full6×6native-whitened second moment,21independent entries; no geometry-mode model |

The last control tests whether a generic transferred covariance shape suffices.
It is not a Deeper-DICE neural reproduction. A sixth **oracle test scalar** fits
aggregate native-normalized second moment from the evaluation labels after the
prediction lock. It is unavailable as a predictor and is never used to select
parameters or replace a primary control.

Each structured fit uses one fixed L-BFGS-B initialization and200iteration cap,
positive coefficients via log bounds[−30,10], analytic gradients and fixed
ftol/gtol. All six return success in19–79iterations. The day/rain independent-mode
fits touch lower coefficient bounds; all common-mode fits remain off the bounds.
Some fits terminate by relative objective change with gradients above gtol;
optimizer success is not proof of a global optimum or unique parameters.
These bounds are model constraints, not measured noise floors. No restart occurs.

## Complete recording-level likelihood results

Score is the mean Gaussian NLL in the fixed native-whitened tangent, with the
shared6log2pi constant omitted. Lower is better. Average the two scenes equally
within each excluded recording; negative NLL is valid for this continuous-density
score. Differences are nats, not percentage pose improvements.

| Excluded recording | Native | Train scalar | Independent modes | Common modes | Pooled shape | Oracle test scalar |
|---|---:|---:|---:|---:|---:|---:|
| day |0.001692|−18.304531|−18.196955|−18.415476|−18.297025|−19.545297|
| night |0.000835|−21.368683|−21.349685|−21.324737|−21.380695|−21.564127|
| rain |0.000747|−21.543966|−21.534438|−21.642869|−21.627585|−21.974534|

Most of the change from native to fitted scores is scale adaptation to the small
imposed perturbation distribution. The scientific comparison is against fitted
controls, not that large native-versus-fitted difference. Common minus scalar
is−0.110945/+0.043947/−0.098903nats for day/night/rain. Common minus pooled shape
is−0.118450/+0.055958/−0.015284nats. No pooled-only win is promoted.

## All scene outcomes and containment

| Scene | Train scalar NLL | Independent modes | Common modes | Pooled shape | Common baseline-relative95%containment |
|---|---:|---:|---:|---:|---:|
| day-plane4m |−16.640061|−16.391584|−16.818818|−16.409282|46.09%|
| day-two-depths |−19.969001|−20.002327|−20.012133|−20.184768|74.22%|
| night-plane4m |−21.245168|−21.207768|−21.265030|−21.325072|98.44%|
| night-two-depths |−21.492199|−21.491602|−21.384443|−21.436317|100.00%|
| rain-plane4m |−21.141157|−21.110528|−21.221639|−21.189174|98.44%|
| rain-two-depths |−21.946776|−21.958347|−22.064100|−22.065996|100.00%|

Day-plane and rain-plane are positive against all three fitted predictive controls.
Day-two-depths and rain-two-depths improve over scalar/independent modes but lose
the pooled-shape control; both night geometries also lose that shape control.
Night-plane still improves over scalar and independent modes. Do not replace
these comparisons with “learning failed” or “correlation universally helps.”

![Recording comparisons and all-scene containment](../../results/covariance-transfer/covariance-transfer.png)

Containment uses the fixed zero-mean perturbation distribution around the
unperturbed estimate, not ground-truth pose coverage. Native containment is100%
on all scenes; that does not make it the best-scoring or physically calibrated
model. The oracle scalar's recording-average containment is96.48%/95.31%/95.31%,
which is a test-informed diagnostic rather than an available uncertainty estimate.
The fitted common model's low day containment remains visible despite its NLL
improvement over the predictive controls.

All predicted/observed translation and rotation spread, mean shifts, covariance
matrices and sample denominators are in the CSV and JSON. All768saved pose
estimates and their original accuracy/availability remain unchanged. There is
no new pose-error reduction, state fusion or task-level outcome in this experiment.

## Verification and paper connection

117pre-fit mechanics checks pass: mode derivatives versus finite differences
(max2.7481e−11m per pixel derivative discrepancy), likelihood gradients versus
finite differences(max5.5690e−10), common-mode covariance identities and disjoint
recording assignments. The30pin execution freeze binds sources/features/labels
and qualifications before parameter fitting. All three folds and30predictive
covariances lock before evaluation scoring. Evaluation labels enter only the
scorer for their own fold; cross-validation naturally uses each recording as
training material in the other folds. No score is used to change the recipe.

The independent audit reconstructs baseline geometry from original packets,
labels from original pose receipts via matrix logarithms, every saved optimizer
function call/objective/gradient, recording exclusions, predictive matrices,
NLL/containment/oracle and all denominators. It passes2,108checks on its first
execution, with no method/input/tolerance amendment. All30predictive matrices
are positive definite; all36primary/oracle scene-method slots are retained.

Censi/Bonnabel already cover correlated propagation; Deeper-DICE II-C/III
already covers Gaussian covariance learning and the distinction between spread
and mean error. Fresh targeted Deeper-DICE reading includes its covariance
parameterizations and relevant setup/evaluation context; no released author
implementation was obtained from the inspected paper links or executed. The
native local covariance/packet/pose source and our fixed linear influence were
already source-pinned and audited. The new hypothesis is this restricted geometry-
mode transfer comparison, not generic NLL, Cholesky covariance or shared noise
as a claimed innovation. Reference-bank reading depths remain explicit.

## Stopping decision and artifacts

Close the exact four-mode/five-parameter recipe. Retain its modest positive
comparisons and the night/pooled-shape/containment limitations. This result does
not validate a generally transferable dependence predictor or physical posterior
correction. It does not close structured covariance learning, thermal supervision,
representation or the other original-paper gaps. No rank/mode/scale/optimizer
rescue or automatic fusion experiment is selected.

Sources:`tools/covariance_transfer.py`, `tools/audit_covariance_transfer.py`.
Design/freeze:`experiments/covariance-transfer-{design,freeze}-2026-11-15.json`.
[Predictions and model parameters](../../results/covariance-transfer/training-001/predictions.json),
[all scores](../../results/covariance-transfer/scoring-001/scores.json),
[all outcomes CSV](../../results/covariance-transfer/all-outcomes.csv),
[audit](../../results/covariance-transfer/audit-001.json).
