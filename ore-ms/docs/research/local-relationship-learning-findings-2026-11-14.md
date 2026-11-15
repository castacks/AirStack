# Local RGB-conditioned thermal gate: completed learning comparison

**Interpretation update:** [Fixed-group diagnosis](fixed-group-diagnosis-findings-2026-11-15.md)
quantifies sensitivity to mismatched recorded RGB/thermal intervals. The same-ID
nominal results below remain unchanged; they do not isolate physical sensorquality
or environmental contribution. No corrected fusion result is claimed.


14 November2026 America/New_York; ore-ms-orb-development baseline53afa19d1 plus
source-hash-frozen implementation.18foldmodels,720Adamupdates completed. No failed
solve or nonfinite training gradient. All1782learned evaluation poses locked before
new evaluation scoring. Exact recipe complete; no optimizer/model/input rescue.

## Finding and practical decision

The full local+RGBcontext gate improves same-backend RGB translation in day/night
but not rain. It does not consistently improve simpler constant/affine thermal
weights or own-local-input models. Keep the scoped joint-estimator translation
gains; the experiment does NOT establish that the richer relationship model adds
value. Lower-dimensional affine gating has lower translation RMSE than the full
MLP in all3folds. Do not attribute the observed improvement specifically to its
RGB-relative inputs, nonlinear capacity, or learned relationship.

A fixed0.1thermal precision baseline already gives almost the same gain. This is
a useful application-level candidate/control, not verified novelty or a deployed
full-VO solution. No universal-success gate is imposed: positive day/night and
negative rain/rotation tradeoffs are reported separately.

## Methods and controls

[Qualified input contract](local-relationship-layer-contract-2026-11-14.md):24inputs,
16tanhhidden units,1sigmoid per accepted thermal factor,417parameters. RGB
precision fixed; thermal precision multiplied by gate in(0,1). Frozen native
means/depth/covariance, source landmarks conditioned on, original12GaussNewton
steps. Output is taskutility, not calibrated covariance. RGB-relative context
comes from the originalRGB-cov fit; its construction cost belongs to the model.
No direct cross-camera pixel/surface correspondence or long-track identity assumed.

Six learning arms per excluded recording: constant1parameter; boundedaffine6;
MLPlocal16,context8,all24,andRGB-only24. AllMLPs share24→16→1architecture with
unused inputs zeroed after training-fold normalization. InitialhiddenXavier seed0;
outputweight0,biaslogit(.1). Affine weight0,biaslogit(.1). Adam40updates,lr.001,
betas.9/.999,eps1e-8; batch6,threeintervals from each trainingrecording, same
seeded cyclic99interval permutations for every arm. Input statistics computed
only from relevant camera factors in two trainingrecordings. Translation loss
squared/.1m plus rotationmatrixFrobenius squared/(sqrt2*.01rad). Allrandomness,
source/cache/qualification pins and schedule fixed before training.

Training40updates is a bounded pilot, not convergence or a sufficient-capacity
claim. First/last minibatch losses are different batches, so their change is NOT
reported as a whole-training-objective improvement. No early stopping, checkpoint
selection, extra epochs or alternate seeds. Sameupdates does not establish equal
compute across heads. Earlier original6parameter sharedhead used a different
precisionrange/gauge and fullbatch schedule; its completed results remain
historical context. The new boundedaffine is a distinct matched control, not a
silent reproduction of that original head.

## Primary results

All learned arms and same-backendRGB/fixedjoint have99/99outputs per recording.
One rain interval lacks thermal input and explicitly falls back toRGB. Thermal-
only context has98/99rain; do not compare its whole-population aggregate as if it
had the same population. Relative increment errors, not accumulated trajectoryATE.

| Recording | Method | Translation mean / RMSE m | Rotation mean / RMSE deg |
|---|---|---:|---:|
|day|rgb_cov|0.045538 / 0.051786|0.059202 / 0.063692|
|day|fixed_point1|0.044241 / 0.050209|0.059332 / 0.063604|
|day|constant|0.044258 / 0.050232|0.059329 / 0.063605|
|day|affine|0.044190 / 0.050133|0.059328 / 0.063590|
|day|local|0.044213 / 0.050169|0.059327 / 0.063598|
|day|context|0.044318 / 0.050304|0.059336 / 0.063626|
|day|all|0.044225 / 0.050191|0.059323 / 0.063595|
|day|rgb_only|0.045553 / 0.051781|0.059554 / 0.064043|
|night|rgb_cov|0.020738 / 0.034281|0.036297 / 0.053587|
|night|fixed_point1|0.020693 / 0.033742|0.036517 / 0.054037|
|night|constant|0.020692 / 0.033746|0.036513 / 0.054028|
|night|affine|0.020688 / 0.033744|0.036510 / 0.054026|
|night|local|0.020678 / 0.033745|0.036491 / 0.053995|
|night|context|0.020693 / 0.033782|0.036495 / 0.053985|
|night|all|0.020692 / 0.033770|0.036503 / 0.054005|
|night|rgb_only|0.020834 / 0.034446|0.036242 / 0.053413|
|rain|rgb_cov|0.191728 / 0.371328|0.040476 / 0.046590|
|rain|fixed_point1|0.192101 / 0.371938|0.040488 / 0.046608|
|rain|constant|0.192110 / 0.371951|0.040488 / 0.046608|
|rain|affine|0.192081 / 0.371904|0.040488 / 0.046607|
|rain|local|0.192092 / 0.371924|0.040489 / 0.046608|
|rain|context|0.192126 / 0.371969|0.040490 / 0.046610|
|rain|all|0.192101 / 0.371930|0.040489 / 0.046609|
|rain|rgb_only|0.200482 / 0.378984|0.040925 / 0.047063|

FullMLP versus same-backendRGB: translationmean/RMSE day−2.88%/−3.08%,night
−0.22%/−1.49%,rain+0.19%/+0.16%. Rotationmean/RMSE day+0.21%/−0.15%,night
+0.57%/+0.78%,rain+0.03%/+0.04%. Do not bury the favorable translation cases;
rotation remains a separate endpoint. FullMLP improves allfour aggregateerrors
against equally trainedRGB-only in day/rain, but rain's trainedRGB-only worsens
the unchangedRGB control. Beating a degraded learned control is insufficient.

FullMLP meanthermal gate day/night/rain0.10039/0.09345/0.09733. Fixed0.1baseline
and trainedconstant are essentially competitive. FullMLP versusfixed0.1 T-RMSE
−0.036%day,+0.084%night,−0.002%rain: no consistent meaningful increment. Against
affine, fullMLPT-RMSE is+0.116%/+0.078%/+0.007%worse. Local-only also has lower
translationmean/RMSE than fullMLP in all3folds, except distinctions remain tiny.
These tiny differences are descriptive; no significance claim or error-bar bound.

NativeRGB is retained as context in scores.json. Same-backendRGB differs from
nativeMAC-VO; thus the above percentages must not be relabeled full nativeVO gains.
NativeRGB rotationRMSE0.051366/0.052225/0.042346deg is lower thanfullMLP in all3.
The geometric backend itself explains some large apparent native-to-joint changes.

## Solver/learning qualification and independent checks

Float64CPU Torch2.7.1. Original fixed12step NumPy solver retained as independent
reference. Parity atgate0,.1,1on firstpair in eachrecord passes1e-8bound. Six
model-parameter directional finite differences use a fixed smooth poseprobe,
zero and deterministicnonzero outputweights, h1e-5,atol1e-6,rtol1e-3; allpass.
Originalpositive-depth,rank1e-10andfinalobjective-increase checks implemented
before training. Initialpre-domain-guard qualification retained; finalqualification
repeated after guards, before source/trainingfreeze. No scientificdata/tolerance
amendment or learned-run retry. Finite-solver differentiation, not an assumed
stationary implicitgradient.

Each training process opens only its two trainingrecording score/reference files;
heldrecordingreference files are used only after global39filepredictionlock.
Sources/cache alreadyexposed, so this is recording-excluded developmental testing,
not blindphysicalgeneralization. Cache code never opened referencepose files.
All18saved NumPy network-forward reconstructions reproduce held gate mean/min/max
on allavailable camera rows. Independent originalNumPy solver with savedperfactor
weights matches firstheldpose for everymodel, maxtransformdifference2.11e-15.
Allprediction/source/cachehashes unchanged; scalartranslation metrics crosschecked.
Allpredictions,statuses,perintervalT/Rlosses,models,normalizers,training minibatches,
losses and auditreceipts retained.

## Cost and scope

Perfit40update training about3.4–4.0s and99interval evaluation about0.4–0.5s
on this host withCPUthreads1. Foldworkers ran concurrently; these are observed
processwall times, not an isolated benchmark. They exclude sourceimage/native
frontend generation and cache preparation(11.70sfor3records). No speed/energy,
compute-saving or livecadence claim. Fullfeature extraction plus originalRGBfit
is additional work; a smallgate alone does not establish economical deployment.

Same-ID nominal sourceaxis and actualtimestamps preserved. Physical exposure
synchrony and independent thermal-body reference remain unverified; reference
is suppliedRGBposeproduct. Sourcepoint/covariance biases and omitted landmark
uncertainty remain model assumptions. No IMU,newdataset,frontendfinetune,
referenceoffset/scale fit,source/depth correction or checkpointselection.

## Stopping decision

Close this exact24→16→1/40update recipe as tested, without capacity/epoch/loss/
feature/mode rescue. The useful positive baseline is conservative thermal
precision in the joint estimator; the richer learnedrelationship is not advanced
on currentevidence. This does not close RGB–thermal representation learning or
show thermal entropy is excessive. Any nextmethod needs a distinct measured
mechanism and priorwork comparison; do not rerun the completedlearning as pending.

Supporting sources and priorart unchanged from the inputcontract: DNLS already
trains correspondence uncertainty through pose optimization; Deeper-DICE learns
VOerror/correction models; MSO directly addresses multispectralfusion. No new
reading,author-system reproduction or algorithmnovelty established by this run.
Artifacts: results/local-relationship-learning/run-001. Prerequisites: verified
local-relationship-contract and thermal-incremental-comparison Box backups.
