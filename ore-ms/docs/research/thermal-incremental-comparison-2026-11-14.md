# Thermal image/temporal features versus incremental pose utility

14 November2026 America/New_York. ore-ms-orb-development baseline53afa19d1 plus
hash-recorded working source. Fixed exploratory comparison completed, no tuning.

## Finding

Thermal has useful reference-informed translation fusion opportunity in the day
and night windows, but this small image/temporal model does not predict it across
recordings. Added gradient anisotropy and bidirectional tracking summaries do not
improve pose accuracy over the summary-only baseline. This does not establish
that sensor-conditioned relationships are unlearnable or entropy is excessive.
The experiment uses global low-dimensional features, not local learned cross-
modal representations, physical sensor noise, thermal material semantics or a
full correspondence/depth-conditioned information analysis.

## Corrected source binding

The preceding60frame intake uses day_2021-08-06-11-23-45 and
night_2021-08-13-21-18-04. The600frame translation-fusion study uses different
day/night records. We did not join them. Instead we used matching100group
nominal-joint-motion records and first100raw left images of each camera:
600PNGs total,297expected increments. Every image SHA matches the archived raw
manifest and original source manifest. Every left-camera timestamp matches the
saved motion binding.596locked packet/source entries preserved. Six1GB raw
archive parts have full historical SHA checks; only selected images extracted.
The22.98MB nominal packet was downloaded from its previously documented Box
location; original packet NPZ hashes match its saved binding lock, but a separate
historical whole-archive hash was not available beside that remote file.

All three source-manifest hashes match the existing thermal restoration source
hashes. This is a same-ID nominal benchmark: original camera timestamps differ,
and physical exposure synchrony is unverified. Supplied RGB pose products remain
qualified references, not independent ground truth. These recordings have been
examined previously; this is exploratory recording-excluded development.

## Frozen comparison

Protocol written before feature extraction/fitting in notebook002. Same ridge
translation-weight objective, lambda1, unpenalized intercept and training-only
normalization; train on two recordings and apply to the third. Allfold predictions
locked before new scores. Hold native RGB orientation exactly. Native RGB/thermal
point means remain unchanged, no new neural inference/native pose optimization.

Summary8: log ratios of factor count, covariance trace, median depth and depthIQR;
translation/rotation disagreement and both translation norms. This is a newly
specified summary control on this cohort, NOT a rerun of the older600frame8cue
model: depth/covariance roles and timing convention differ. Depth statistic uses
point distance in the saved conditioned packet coordinates; it is not an
independently calibrated disparity-quality estimate.

Augmented adds four features per camera: contrast-normalized gradient RMS,
structure-tensor minor fraction, forward/backward KLT survival and8x4source-grid
coverage. Thermal tracker input uses fixed p5/p95 clipping to8bit; RGB grayscale
gets the same normalization. Native inference preprocessing is unchanged.
ShiTomasi200/.01/5; LK21x21/3levels;1pxforward-backward tolerance. Temporal
survival is a diagnostic tracker output, not verified true correspondence.
Whole-image features do not establish that RGB and thermal track the same surfaces.

Bothmodels minimize training translation-vector squared error with the same
regularizer, but augmented has more coefficients. Training is not compute-matched;
model selection/capacity/tolerance/threshold sweeps were not performed.

## Paired-support results

All values are relative translation errors in metres. Allrotation matrices remain
exactly RGB; error statistics have the same rotation population as RGB.

| Recording | Paired / expected | RGB mean / RMSE | Summary mean / RMSE | Augmented mean / RMSE |
|---|---:|---:|---:|---:|
|day|99/99|0.046390 / 0.052889|0.046390 / 0.052889|0.046390 / 0.052889|
|night|99/99|0.022281 / 0.035981|0.022281 / 0.035981|0.022281 / 0.035981|
|rain|98/99|0.271722 / 0.504434|0.285229 / 0.510908|0.287170 / 0.511637|

Both learned models clip to zero thermal weight on all day/night evaluation
intervals. Mean thermal weights on rain: constant0.062608, summary0.055740,
augmented0.061092. Rain augmented RMSE is1.43%higher than RGB. No positive
accuracy result. All297feature slots retained; one rain interval lacks paired
support and is explicitly excluded from matched-error comparison. This pilot
has no new final-output publisher/fallback or availability improvement claim.
Constant and equal blend, thermal-translation context, p95 and complete fold
models/weights remain in machine-readable artifacts. Thermal-translation context
retains RGB rotation; do not label it full thermal-only pose evaluation.

### Post-scoring explanatory oracle

After seeing the locked comparison, computed the best per-interval convex blend
using reference translations. This was NOT a predeclared success endpoint and is
not an available predictor. No model/threshold changed as a result.

| Recording | RGB mean | Oracle mean | Mean reduction |
|day|0.046390|0.036045|22.30%|
|night|0.022281|0.015008|32.64%|
|rain|0.271722|0.268228|1.29%|

A useful thermal correction need not make thermal alone the best expert.
Conversely, positive oracle opportunity can include bias cancellation and nominal
reference/timing effects. It does not guarantee learnability from the captured
inputs. The small rain opportunity differs from older600prefix results because
these are distinct populations and timing contracts; do not pool their gains.

## Validation and stopping

600image hashes and original source IDs/times verified. Independent standalone
saved-output auditor imports no producer;6260checks reconstruct blend arithmetic,
translation mean/RMSE and exact rotation copies. Independent augmented least-
squares reconstructions agree with all six ridge fits to1.22e-16maximum coefficient
difference. Feature computation reads no reference files; later fitting reads
only training-recording targets; allfold predictions lock before evaluation.
Historical scores were inspected during source intake, so this is not blind
validation. All297expected motion slots and296paired outputs retained with
their explicit denominators. Feature stage7.853seconds on this host is overhead,
not a processing saving or a live-cadence measurement. Isolated NumPy2.2.6,
Pillow11.3.0,OpenCV4.10.0.84; CPU threads1.

Close this feature/model configuration without retuning. Do not repeat another
global-summary score as a new hypothesis. The next distinct representation question
would examine local persistent thermal observations against RGB-covered geometry
and pose sensitivity, with actual correspondences/depth and applicable prior
baselines. Such a study needs a separate mechanism/source review and execution
contract; this result does not select a new trained model. A larger model alone
is not justified by the failed small model. Broader sensor-conditioned learning
remains open; pose/VO accuracy or matched-accuracy compute remains decisive.

Prior thermal-specific representation overlap remains TP-TIO/ETIO per the linked
thermal-structure-intake-2026-11-14.md. No fresh paper claim or author reproduction
is introduced here. Data, source hashes, models, all weights/errors and validation:
results/thermal-incremental-comparison/{recovery,run-001}.
